#include "video_link_motor.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dji_motor.h"
#include "motor_def.h"
#include <math.h>
#include <stddef.h>

// Pitch 超过该阈值时图传机构需要切到避让高位，低于该阈值时回到低位贴限位保持。
#define PITCH_ANGLE_THRESHOLD 26.0f
// 切换两端限位时只跑速度环，速度给得比原寻零更高，让机构不再用位置环慢慢追固定偏移。
#define VIDEO_LINK_LIMIT_SEEK_SPEED_DPS 5000.0f
// 速度环贴限位阶段需要足够输出克服减速箱静摩擦，但仍保留低于正常满输出的限幅来控制撞限位冲击。
#define VIDEO_LINK_LIMIT_SEEK_CURRENT_LIMIT_RAW 9000.0f
// 速度已经降到这个窗口内时，才可能认为机构被机械限位挡住，而不是仍在正常高速移动。
#define VIDEO_LINK_LIMIT_STALL_SPEED_DPS 140.0f
// 贴限位时必须看到足够电流，避免仅凭速度滤波抖动把中途减速误判成已经到端。
#define VIDEO_LINK_LIMIT_STALL_CURRENT_RAW 450.0f
// 每拍总角度变化足够小时才允许累计到位时间，用来过滤减速箱弹性回弹和反馈噪声。
#define VIDEO_LINK_LIMIT_STALL_DELTA_DEG 5.0f
// 限位判据连续成立一小段时间后再锁位置，既保留速度切换的响应，又避免碰一下就误锁。
#define VIDEO_LINK_LIMIT_STALL_HOLD_MS 80u
// 两端切换的期望行程明显短于该时间，超时后停机可以挡住装配卡滞、限位缺失或反馈异常。
#define VIDEO_LINK_LIMIT_SEEK_TIMEOUT_MS 3500u

typedef enum {
    VIDEO_LINK_STATE_BOOT_WAIT = 0,
    VIDEO_LINK_STATE_SEEK_LOW_LIMIT,
    VIDEO_LINK_STATE_HOLD_LOW_LIMIT,
    VIDEO_LINK_STATE_SEEK_HIGH_LIMIT,
    VIDEO_LINK_STATE_HOLD_HIGH_LIMIT,
    VIDEO_LINK_STATE_FAILED,
} VideoLinkMotorState_e;

static DJIMotorInstance *video_link_motor = NULL;
static uint8_t is_video_link_enabled = 0u;
static VideoLinkMotorState_e video_link_state = VIDEO_LINK_STATE_BOOT_WAIT;
static float video_link_normal_speed_maxout = 10000.0f;
static uint32_t video_link_seek_start_ms = 0u;
static uint32_t video_link_stall_start_ms = 0u;
static float video_link_last_total_angle = 0.0f;
static float video_link_low_limit_angle = 0.0f;
static float video_link_high_limit_angle = 0.0f;
static uint8_t video_link_low_limit_valid = 0u;
static uint8_t video_link_high_limit_valid = 0u;
static uint8_t video_link_last_online = 0u;

/**
 * @brief 清空 PID 的运行态但保留参数
 *
 * 速度环撞限位后会积累较大的误差和积分，直接切回位置环可能把上一阶段的残留输出带到保持阶段。
 * 这里只清运行态字段，不碰 Kp/Ki/Kd/MaxOut 等配置，保证模式切换干净且不破坏初始化参数。
 */
static void VideoLinkClearPIDRuntime(PIDInstance *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
    pid->ERRORHandler.ERRORCount = 0u;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
    DWT_GetDeltaT(&pid->DWT_CNT);
}

/**
 * @brief 把图传电机切到速度环贴限位模式
 *
 * 该模式只负责快速去机械端点，不使用位置环和固定角度偏移，避免位置目标不准时在中途拖慢动作。
 */
static void VideoLinkUseSpeedLimitSeek(float speed_ref)
{
    video_link_motor->motor_settings.outer_loop_type = SPEED_LOOP;
    video_link_motor->motor_settings.close_loop_type = SPEED_LOOP;
    video_link_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
    video_link_motor->motor_controller.speed_PID.MaxOut = VIDEO_LINK_LIMIT_SEEK_CURRENT_LIMIT_RAW;
    DJIMotorSetRef(video_link_motor, speed_ref);
}

/**
 * @brief 把图传电机切回位置环保持模式
 *
 * 位置环目标使用刚贴到限位时的 total_angle，后续只负责抵抗重力和振动，不再继续向限位硬顶。
 */
static void VideoLinkUsePositionHold(float angle_ref)
{
    video_link_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
    video_link_motor->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
    video_link_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
    video_link_motor->motor_controller.speed_PID.MaxOut = video_link_normal_speed_maxout;
    DJIMotorSetRef(video_link_motor, angle_ref);
}

/**
 * @brief 清空限位学习结果并恢复等待状态
 *
 * 电机掉线或异常状态恢复时，旧的 total_angle 与真实机构位置不再可信，必须丢弃两端限位记录。
 */
static void VideoLinkInvalidateLimits(void)
{
    video_link_low_limit_valid = 0u;
    video_link_high_limit_valid = 0u;
    video_link_low_limit_angle = 0.0f;
    video_link_high_limit_angle = 0.0f;
    video_link_seek_start_ms = 0u;
    video_link_stall_start_ms = 0u;
    video_link_last_total_angle = 0.0f;
    video_link_state = VIDEO_LINK_STATE_BOOT_WAIT;
}

/**
 * @brief 从当前位置开始向指定机械限位快速运动
 *
 * 每次进入速度环都重置堵转证据和 PID 运行态，保证反向切换、掉线恢复和首次启动都从同一套干净条件开始。
 */
static void VideoLinkStartLimitSeek(VideoLinkMotorState_e state,
                                    float speed_ref,
                                    uint32_t now_ms,
                                    float current_total_angle_deg)
{
    DJIMotorEnable(video_link_motor);
    VideoLinkClearPIDRuntime(&video_link_motor->motor_controller.speed_PID);
    VideoLinkClearPIDRuntime(&video_link_motor->motor_controller.angle_PID);
    VideoLinkUseSpeedLimitSeek(speed_ref);
    video_link_seek_start_ms = now_ms;
    video_link_stall_start_ms = 0u;
    video_link_last_total_angle = current_total_angle_deg;
    video_link_state = state;
}

/**
 * @brief 根据速度、电流和角度增量判断是否已经贴住机械限位
 *
 * 三个条件同时满足并持续一小段时间才算到端，可以在速度切换足够快的同时保留基本的误判保护。
 */
static uint8_t VideoLinkLimitReached(uint32_t now_ms, float current_total_angle_deg)
{
    uint8_t is_stalled;

    is_stalled = (uint8_t)((fabsf(video_link_motor->measure.speed_aps) < VIDEO_LINK_LIMIT_STALL_SPEED_DPS) &&
                           (fabsf((float)video_link_motor->measure.real_current) > VIDEO_LINK_LIMIT_STALL_CURRENT_RAW) &&
                           (fabsf(current_total_angle_deg - video_link_last_total_angle) < VIDEO_LINK_LIMIT_STALL_DELTA_DEG));

    if (is_stalled != 0u) {
        if (video_link_stall_start_ms == 0u) {
            video_link_stall_start_ms = now_ms;
        }
    } else {
        video_link_stall_start_ms = 0u;
    }

    video_link_last_total_angle = current_total_angle_deg;

    if (video_link_stall_start_ms == 0u) {
        return 0u;
    }

    return (uint8_t)((now_ms - video_link_stall_start_ms) >= VIDEO_LINK_LIMIT_STALL_HOLD_MS);
}

/**
 * @brief 进入失败态并停掉图传电机
 *
 * 找不到限位时继续给速度参考只会增加机构风险，因此失败态直接停机，等待电机掉线复位或整机重新初始化。
 */
static void VideoLinkFailLimitSeek(const char *limit_name)
{
    VideoLinkUsePositionHold(video_link_motor->measure.total_angle);
    DJIMotorStop(video_link_motor);
    video_link_state = VIDEO_LINK_STATE_FAILED;
    LOGWARNING("[video_link] %s limit seek timeout", limit_name);
}

void VideoLinkMotorInit(void)
{
    // 初始化 M2006 图传固定电机，CAN ID 和 PID 参数仍沿用原模块配置，只改变运行时的限位切换策略。
    Motor_Init_Config_s video_link_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 7,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 3.2f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = 13000.0f,
            },
            .speed_PID = {
                .Kp = 2.4f,
                .Ki = 0.5f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 3000.0f,
                .MaxOut = 12000.0f,
            },
            .current_feedforward_ptr = NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006
    };

    video_link_motor = DJIMotorInit(&video_link_config);
    if (video_link_motor != NULL) {
        video_link_normal_speed_maxout = video_link_motor->motor_controller.speed_PID.MaxOut;
    }
}

void VideoLinkMotorEnable(void)
{
    if (video_link_motor != NULL) {
        // 云台退出零力后只恢复驱动侧使能，实际跑速度环还是位置环由 VideoLinkMotorTask 的状态机决定。
        DJIMotorEnable(video_link_motor);
        is_video_link_enabled = 1u;
    }
}

void VideoLinkMotorDisable(void)
{
    if (video_link_motor != NULL) {
        // 零力模式下把保持目标先收在当前位置再切断输出，避免下次使能时沿用速度环参考或远端位置目标。
        VideoLinkUsePositionHold(video_link_motor->measure.total_angle);
        DJIMotorStop(video_link_motor);
        is_video_link_enabled = 0u;

        if (video_link_state == VIDEO_LINK_STATE_SEEK_LOW_LIMIT ||
            video_link_state == VIDEO_LINK_STATE_SEEK_HIGH_LIMIT ||
            video_link_state == VIDEO_LINK_STATE_FAILED) {
            VideoLinkInvalidateLimits();
        }
    }
}

void VideoLinkMotorTask(float pitch_angle_deg)
{
    uint32_t now_ms;
    float current_total_angle_deg;
    uint8_t motor_online;

    if (video_link_motor == NULL) {
        return;
    }

    now_ms = (uint32_t)DWT_GetTimeline_ms();
    current_total_angle_deg = video_link_motor->measure.total_angle;
    motor_online = (video_link_motor->daemon != NULL && DaemonIsOnline(video_link_motor->daemon) != 0u) ? 1u : 0u;

    if (motor_online != video_link_last_online) {
        if (motor_online == 0u) {
            // 图传电机掉线后 total_angle 连续性不再可信，必须废弃两端限位并等待重新上线后再速度环找端点。
            VideoLinkInvalidateLimits();
            VideoLinkUsePositionHold(current_total_angle_deg);
            DJIMotorStop(video_link_motor);
            LOGWARNING("[video_link] motor offline, invalidate limits");
        } else {
            // 复活后的第一帧只作为新的角度比较起点，真实限位仍要靠后续速度环重新贴到机械端确认。
            video_link_last_total_angle = current_total_angle_deg;
            LOGINFO("[video_link] motor online, limit seek pending");
        }
        video_link_last_online = motor_online;
    }

    if (!is_video_link_enabled || motor_online == 0u) {
        return;
    }

    switch (video_link_state) {
    case VIDEO_LINK_STATE_BOOT_WAIT:
        if (pitch_angle_deg >= PITCH_ANGLE_THRESHOLD) {
            // 首次启动时直接朝当前需求端运动，高 Pitch 不再先回低位绕一圈，响应会更快。
            VideoLinkStartLimitSeek(VIDEO_LINK_STATE_SEEK_HIGH_LIMIT,
                                    VIDEO_LINK_LIMIT_SEEK_SPEED_DPS,
                                    now_ms,
                                    current_total_angle_deg);
            LOGINFO("[video_link] seek high limit start");
        } else {
            // 低 Pitch 时直接向低端限位贴靠，到端后记录当前位置并切位置环保持。
            VideoLinkStartLimitSeek(VIDEO_LINK_STATE_SEEK_LOW_LIMIT,
                                    -VIDEO_LINK_LIMIT_SEEK_SPEED_DPS,
                                    now_ms,
                                    current_total_angle_deg);
            LOGINFO("[video_link] seek low limit start");
        }
        return;

    case VIDEO_LINK_STATE_SEEK_LOW_LIMIT:
        if (pitch_angle_deg >= PITCH_ANGLE_THRESHOLD) {
            // 用户在回低位途中又抬高 Pitch 时立即反向去高位，避免继续跑向已经不需要的端点。
            VideoLinkStartLimitSeek(VIDEO_LINK_STATE_SEEK_HIGH_LIMIT,
                                    VIDEO_LINK_LIMIT_SEEK_SPEED_DPS,
                                    now_ms,
                                    current_total_angle_deg);
            LOGINFO("[video_link] switch to high limit seek");
            return;
        }

        VideoLinkUseSpeedLimitSeek(-VIDEO_LINK_LIMIT_SEEK_SPEED_DPS);
        if ((now_ms - video_link_seek_start_ms) >= VIDEO_LINK_LIMIT_SEEK_TIMEOUT_MS) {
            VideoLinkFailLimitSeek("low");
            return;
        }

        if (VideoLinkLimitReached(now_ms, current_total_angle_deg) != 0u) {
            // 到低端限位后用当前 total_angle 作为保持目标，避免继续速度环硬顶机械限位。
            video_link_low_limit_angle = current_total_angle_deg;
            video_link_low_limit_valid = 1u;
            VideoLinkClearPIDRuntime(&video_link_motor->motor_controller.speed_PID);
            VideoLinkClearPIDRuntime(&video_link_motor->motor_controller.angle_PID);
            VideoLinkUsePositionHold(video_link_low_limit_angle);
            video_link_state = VIDEO_LINK_STATE_HOLD_LOW_LIMIT;
            LOGINFO("[video_link] low limit reached");
        }
        return;

    case VIDEO_LINK_STATE_HOLD_LOW_LIMIT:
        if (video_link_low_limit_valid == 0u) {
            VideoLinkStartLimitSeek(VIDEO_LINK_STATE_SEEK_LOW_LIMIT,
                                    -VIDEO_LINK_LIMIT_SEEK_SPEED_DPS,
                                    now_ms,
                                    current_total_angle_deg);
            LOGINFO("[video_link] low limit invalid, seek again");
            return;
        }

        if (pitch_angle_deg >= PITCH_ANGLE_THRESHOLD) {
            // 需要切到高位时不再给一个固定角度目标，而是直接速度环冲到高端限位。
            VideoLinkStartLimitSeek(VIDEO_LINK_STATE_SEEK_HIGH_LIMIT,
                                    VIDEO_LINK_LIMIT_SEEK_SPEED_DPS,
                                    now_ms,
                                    current_total_angle_deg);
            LOGINFO("[video_link] seek high limit start");
            return;
        }

        VideoLinkUsePositionHold(video_link_low_limit_angle);
        return;

    case VIDEO_LINK_STATE_SEEK_HIGH_LIMIT:
        if (pitch_angle_deg < PITCH_ANGLE_THRESHOLD) {
            // 用户在去高位途中放低 Pitch 时立即反向回低端，速度环目标始终跟随当前避让需求。
            VideoLinkStartLimitSeek(VIDEO_LINK_STATE_SEEK_LOW_LIMIT,
                                    -VIDEO_LINK_LIMIT_SEEK_SPEED_DPS,
                                    now_ms,
                                    current_total_angle_deg);
            LOGINFO("[video_link] switch to low limit seek");
            return;
        }

        VideoLinkUseSpeedLimitSeek(VIDEO_LINK_LIMIT_SEEK_SPEED_DPS);
        if ((now_ms - video_link_seek_start_ms) >= VIDEO_LINK_LIMIT_SEEK_TIMEOUT_MS) {
            VideoLinkFailLimitSeek("high");
            return;
        }

        if (VideoLinkLimitReached(now_ms, current_total_angle_deg) != 0u) {
            // 到高端限位后同样只锁住当前角度，后续保持依赖位置环而不是持续顶限位。
            video_link_high_limit_angle = current_total_angle_deg;
            video_link_high_limit_valid = 1u;
            VideoLinkClearPIDRuntime(&video_link_motor->motor_controller.speed_PID);
            VideoLinkClearPIDRuntime(&video_link_motor->motor_controller.angle_PID);
            VideoLinkUsePositionHold(video_link_high_limit_angle);
            video_link_state = VIDEO_LINK_STATE_HOLD_HIGH_LIMIT;
            LOGINFO("[video_link] high limit reached");
        }
        return;

    case VIDEO_LINK_STATE_HOLD_HIGH_LIMIT:
        if (video_link_high_limit_valid == 0u) {
            VideoLinkStartLimitSeek(VIDEO_LINK_STATE_SEEK_HIGH_LIMIT,
                                    VIDEO_LINK_LIMIT_SEEK_SPEED_DPS,
                                    now_ms,
                                    current_total_angle_deg);
            LOGINFO("[video_link] high limit invalid, seek again");
            return;
        }

        if (pitch_angle_deg < PITCH_ANGLE_THRESHOLD) {
            // 需要回低位时也直接速度环去低端限位，到端后再切位置环保持。
            VideoLinkStartLimitSeek(VIDEO_LINK_STATE_SEEK_LOW_LIMIT,
                                    -VIDEO_LINK_LIMIT_SEEK_SPEED_DPS,
                                    now_ms,
                                    current_total_angle_deg);
            LOGINFO("[video_link] seek low limit start");
            return;
        }

        VideoLinkUsePositionHold(video_link_high_limit_angle);
        return;

    case VIDEO_LINK_STATE_FAILED:
        DJIMotorStop(video_link_motor);
        return;

    default:
        // 状态值异常时回到等待入口，下一拍会按当前 Pitch 重新选择要贴的机械端点。
        VideoLinkInvalidateLimits();
        return;
    }
}
