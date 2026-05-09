#include "shoot_private.h"

/**
 * @brief 检测拨盘电机是否堵转
 * @return 1 表示当前处于堵转状态（高电流+低转速），0 表示正常
 */
static uint8_t IsLoaderStalled(void)
{
    int16_t current_abs = (loader->measure.real_current > 0) ?
                              loader->measure.real_current :
                              -loader->measure.real_current;
    float speed_abs = (loader->measure.speed_aps > 0.0f) ?
                          loader->measure.speed_aps :
                          -loader->measure.speed_aps;
    uint8_t is_stalled = (current_abs > STALL_CURRENT_THRESHOLD) &&
                         (speed_abs < STALL_SPEED_THRESHOLD);
    StallDebug_s *p = ShootDebug_GetStallPtr();

    // 堵转调试信息只保留电流、速度和是否堵转这三个直接观测量，目的是状态机阶段由 `HandleLoaderStall` 单独回写，避免两处互相覆盖。
    p->current_abs = current_abs;
    p->speed_abs = speed_abs;
    p->is_stalled = is_stalled;

    return is_stalled;
}

/**
 * @brief 堵转检测与非单发自动反转处理状态机
 * @param current_mode 当前的发射模式
 * @return 经过堵转处理后的发射模式
 */
loader_mode_e HandleLoaderStall(loader_mode_e current_mode)
{
    float current_time = DWT_GetTimeline_ms();
    uint8_t single_fire_is_feeding = (current_mode == LOAD_1_BULLET) &&
                                     (single_fire.state == SF_FEEDING);
    uint8_t is_shooting = (single_fire_is_feeding != 0u) ||
                          (current_mode == LOAD_2_BULLET) ||
                          (current_mode == LOAD_3_BULLET) ||
                          (current_mode == LOAD_BURSTFIRE);

// 返回前统一更新调试状态，目的是堵转状态机会在多个 case 间跳转，集中收口可以避免某条返回路径忘记同步调试状态。
#define RETURN_WITH_DEBUG(mode)                                \
    do {                                                       \
        ShootDebug_GetStallPtr()->state = stall_handler.state; \
        return (mode);                                         \
    } while (0)

    switch (stall_handler.state) {
    case STALL_NORMAL:
        if (single_fire_is_feeding != 0u) {
            // 单发送弹阶段改为强推到目标位，堵转判据只写入调试观测；这里不能进入自动反转消抖，否则限制位变紧后一次正常顶弹就会被状态机提前退回。
            (void)IsLoaderStalled();
            RETURN_WITH_DEBUG(current_mode);
        }
        if (is_shooting && IsLoaderStalled()) {
            // 先进入消抖阶段而不是立即反转，目的是单拍卡顿或瞬时电流尖峰不应直接触发反转动作。
            stall_handler.state = STALL_DETECTING;
            stall_handler.detect_start_time = current_time;
            stall_handler.saved_mode = current_mode;
        }
        if (current_mode == LOAD_REVERSE || current_mode == LOAD_STOP) {
            // 主动反转或停止时清空连续反转计数，目的是这两种模式不应继续沿用上一轮自动解卡的失败累计。
            stall_handler.reverse_count = 0;
        }
        RETURN_WITH_DEBUG(current_mode);

    case STALL_DETECTING:
        if (single_fire_is_feeding != 0u) {
            // 兼容旧状态残留：若单发已经进入强推策略，就清掉上一拍的堵转消抖上下文，避免旧状态继续走向反转阶段覆盖本次强推目标。
            stall_handler.state = STALL_NORMAL;
            (void)IsLoaderStalled();
            RETURN_WITH_DEBUG(current_mode);
        }
        if (!IsLoaderStalled()) {
            // 消抖期间若堵转特征消失就直接恢复正常，目的是这说明刚才只是短暂扰动，不应进入自动反转。
            stall_handler.state = STALL_NORMAL;
            RETURN_WITH_DEBUG(current_mode);
        }
        if ((current_time - stall_handler.detect_start_time) >= STALL_DETECT_TIME) {
            StallDebug_s *p_stall = ShootDebug_GetStallPtr();

            // 持续堵转超过门槛后进入反转阶段，目的是只有确认卡死时才让拨盘主动退回，避免瞬时电流尖峰打断正常送弹。
            stall_handler.state = STALL_REVERSING;
            stall_handler.reverse_start_time = current_time;
            if (stall_handler.saved_mode != LOAD_1_BULLET) {
                // 非单发模式没有可靠的上一弹位缓存，暂时沿用短反转兜底，避免本轮修改把 2/3 发和连发的行为一起扩大。
                stall_handler.reverse_target_angle = loader->measure.total_angle -
                                                     LoaderOutputAngleToMotorAngle(REVERSE_ANGLE);
            }
            // 非单发以外的历史状态只保留兜底目标写入，目的是防止异常状态进入反转时没有明确角度参考。
            single_fire.lock_target_angle = stall_handler.reverse_target_angle;
            stall_handler.reverse_count++;
            p_stall->reverse_count = stall_handler.reverse_count;
            p_stall->reverse_target_angle = stall_handler.reverse_target_angle;
            LoaderSetAngleRef(stall_handler.reverse_target_angle);
        }
        RETURN_WITH_DEBUG(current_mode);

    case STALL_REVERSING:
        if ((current_time - stall_handler.reverse_start_time) >= REVERSE_TIME) {
            // 反转持续到固定时长后进入恢复等待，目的是当前策略以时间窗收口，比再加一套位置完成判据更稳更简单。
            stall_handler.state = STALL_RECOVERY;
            stall_handler.recovery_start_time = current_time;
        }
        // 反转阶段统一返回 STOP，目的是正常发射逻辑必须暂时让位给堵转状态机自己控制拨盘。
        RETURN_WITH_DEBUG(LOAD_STOP);

    case STALL_RECOVERY:
        if ((current_time - stall_handler.recovery_start_time) >= RECOVERY_TIME) {
            if (stall_handler.reverse_count >= MAX_REVERSE_COUNT) {
                // 连续自动解卡达到上限后直接停火，目的是继续自动重试只会反复卷弹，必须交给人工处理。
                stall_handler.state = STALL_NORMAL;
                stall_handler.reverse_count = 0;
                RETURN_WITH_DEBUG(LOAD_STOP);
            }
            // 恢复等待结束后停在回退位置等待下一次明确指令，目的是堵转后的自动动作只负责解压，不再继续重试卷弹。
            stall_handler.state = STALL_NORMAL;
            RETURN_WITH_DEBUG(LOAD_STOP);
        }
        RETURN_WITH_DEBUG(LOAD_STOP);

    default:
        stall_handler.state = STALL_NORMAL;
        RETURN_WITH_DEBUG(current_mode);
    }

#undef RETURN_WITH_DEBUG
}
