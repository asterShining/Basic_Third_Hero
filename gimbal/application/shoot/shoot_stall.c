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
 * @brief 堵转检测与自动反转处理状态机
 * @param current_mode 当前的发射模式
 * @return 经过堵转处理后的发射模式
 */
loader_mode_e HandleLoaderStall(loader_mode_e current_mode)
{
    float current_time = DWT_GetTimeline_ms();
    uint8_t is_shooting = (current_mode == LOAD_1_BULLET) ||
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
        if (!IsLoaderStalled()) {
            // 消抖期间若堵转特征消失就直接恢复正常，目的是这说明刚才只是短暂扰动，不应进入自动反转。
            stall_handler.state = STALL_NORMAL;
            RETURN_WITH_DEBUG(current_mode);
        }
        if ((current_time - stall_handler.detect_start_time) >= STALL_DETECT_TIME) {
            StallDebug_s *p_stall = ShootDebug_GetStallPtr();

            // 持续堵转超过门槛后进入反转阶段，目的是只有确认卡死时才值得让拨盘主动回退半发节距。
            stall_handler.state = STALL_REVERSING;
            stall_handler.reverse_start_time = current_time;
            stall_handler.reverse_target_angle = loader->measure.total_angle -
                                                 LoaderOutputAngleToMotorAngle(REVERSE_ANGLE);
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
            // 恢复等待结束后回到原来的发射模式，目的是用户原始意图仍应继续生效，除非已经超过自动解卡上限。
            stall_handler.state = STALL_NORMAL;
            RETURN_WITH_DEBUG(stall_handler.saved_mode);
        }
        RETURN_WITH_DEBUG(LOAD_STOP);

    default:
        stall_handler.state = STALL_NORMAL;
        RETURN_WITH_DEBUG(current_mode);
    }

#undef RETURN_WITH_DEBUG
}
