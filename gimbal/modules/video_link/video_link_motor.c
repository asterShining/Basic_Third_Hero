#include "video_link_motor.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dji_motor.h"
#include "motor_def.h"
#include <math.h>
#include <stddef.h>

// What: 保留图传电机抬起位相对基准零点的总转子角位移。
// Why: M2006 带 36:1 减速箱，真实机构从平放到抬起会跨越很多电机转子角，继续只看单圈角会天然丢圈。
#define VIDEO_LINK_ROTATION_DEG 1650.0f
// What: 保留现有 Pitch 触发图传切位的阈值。
// Why: 本次修改只解决开机寻零冲击问题，不顺手改动已经联调过的机构切换阈值。
#define PITCH_ANGLE_THRESHOLD 26.0f
// What: 定义寻零阶段向下限位贴靠的目标速度，单位 deg/s。
// Why: 用户反馈当前寻零速度偏慢且“看起来没力”，因此把贴限位速度再抬高一档，让机构动作更明显。
#define VIDEO_LINK_HOMING_SPEED_DPS (-1000.0f)
// What: 定义寻零阶段速度环最大输出上限，量纲与 DJI 电流原始控制量一致。
// Why: 当前找零速度已经很高，但用户反馈“还是没力”，因此继续放宽临时输出上限，让电机更容易真正顶住限位。
#define VIDEO_LINK_HOMING_CURRENT_LIMIT_RAW 5000.0f
// What: 定义判定“已经贴住限位”时允许的最大残余速度。
// Why: 当前用户主要问题是“找零时间太长”，因此继续放宽速度窗口，让高速贴限位后的减速阶段更快进入到位判定。
#define VIDEO_LINK_HOMING_STALL_SPEED_DPS 100.0f
// What: 定义判定“已经贴住限位”时要求达到的最小电流幅值。
// Why: 降低电流门槛后，电机只要开始明显顶住限位就更容易被识别出来，不必等到电流抬得很高才算到位。
#define VIDEO_LINK_HOMING_STALL_CURRENT_RAW 300.0f
// What: 定义判定“已经贴住限位”时每拍允许的最大角度变化量。
// Why: 高速找零时减速箱弹性和机构回弹更明显，继续放宽角度微动窗口可以减少到位判定被反复打断。
#define VIDEO_LINK_HOMING_STALL_DELTA_DEG 4.0f
// What: 定义贴限位后需要持续满足堵转判据的保持时间，单位 ms。
// Why: 用户当前最在意的是“尽快结束寻零”，因此把保持时间再压低，让接触限位后更快宣布成功。
#define VIDEO_LINK_HOMING_STALL_HOLD_MS 120u
// What: 定义整次自动寻零的最长允许时间，单位 ms。
// Why: 继续给更宽松的总超时余量，避免长行程或短时总线波动把本来能完成的找零提前判成失败。
#define VIDEO_LINK_HOMING_TIMEOUT_MS 7000u

// What: 定义图传电机内部生命周期状态。
// Why: 这次需求本质上是“正常控制前先跑一次寻零”，必须把等待反馈、寻零、正常工作和失败拆成显式状态。
typedef enum {
    VIDEO_LINK_STATE_BOOT_WAIT = 0,
    VIDEO_LINK_STATE_HOMING,
    VIDEO_LINK_STATE_READY,
    VIDEO_LINK_STATE_FAILED,
} VideoLinkMotorState_e;

// What: 保存图传电机实例指针。
// Why: 图传电机模块的全部控制都围绕这一颗 M2006 展开，内部必须长期持有实例地址。
static DJIMotorInstance *video_link_motor = NULL;
// What: 保存图传电机当前是否允许工作。
// Why: 云台零力态下不应该继续给图传电机发控制参考，需要一个独立的使能门。
static uint8_t is_video_link_enabled = 0;
// What: 标记是否已经记录到本次上电的机构基准零点。
// Why: 只有寻零成功后，后续的角度切位逻辑才有可靠的“低点基准”可用。
static uint8_t is_base_recorded = 0;
// What: 保存寻零得到的基准总角度。
// Why: 现有图传切位逻辑是“基准角 + 抬起偏移”，因此基准零点必须跨周期保留下来。
static float base_total_angle = 0.0f;
// What: 保存图传电机当前内部状态。
// Why: 需要在模块内部区分“还没拿到反馈”“正在温柔寻零”“已经可正常控制”“本次上电寻零失败”四种处理路径。
static VideoLinkMotorState_e video_link_state = VIDEO_LINK_STATE_BOOT_WAIT;
// What: 保存图传电机正常工作态的速度环最大输出。
// Why: 寻零期间会临时压低 speed_PID.MaxOut，完成后必须无歧义地恢复到原始控制强度。
static float video_link_normal_speed_maxout = 10000.0f;
// What: 保存本次寻零开始时刻。
// Why: 需要做整次寻零的超时保护，避免异常情况下无限期顶住下限位。
static uint32_t video_link_homing_start_ms = 0u;
// What: 保存堵转判据首次连续成立的起始时刻。
// Why: 到位判定需要“连续满足一段时间”而不是单拍命中，所以必须记住保持计时起点。
static uint32_t video_link_stall_start_ms = 0u;
// What: 保存上一拍的总角度。
// Why: 堵转判定要确认“几乎不再前进”，必须比较前后两拍的总角度增量。
static float video_link_last_total_angle = 0.0f;
// What: 标记寻零结束后是否需要先稳一拍再恢复正常切位。
// Why: 刚记完零的当拍先把参考钉在基准位，可以避免机构在恢复 READY 的第一拍立刻跳去另一个目标。
static uint8_t video_link_ready_settle_pending = 0u;
// What: 标记本次寻零完成后是否还禁止直接回到高位。
// Why: 高点掉电恢复后如果立刻按照 `pitch>=阈值` 回高位，肉眼几乎看不到重新校准动作，因此需要先强制卡在低位直到重新满足上升沿条件。
static uint8_t video_link_high_position_rearm_pending = 0u;
// What: 标记本次重新允许高位之前是否已经观测到 Pitch 低于阈值。
// Why: 用户要的是“先低于阈值一次，再重新上穿阈值才允许回高位”，所以必须显式记录这次低位经过事件。
static uint8_t video_link_high_position_low_seen = 0u;
// What: 保存图传电机上一拍的在线状态。
// Why: 只有识别出“离线/复活”的边沿，模块才能在 M2006 单独掉电后自动判定零点失效并重新寻零。
static uint8_t video_link_last_online = 0u;

void VideoLinkMotorInit(void)
{
    // What: 初始化M2006图传电机的配置参数
    // Why: 为M2006电机注册CAN2 ID7发送节点，以及角度和速度双闭环
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
                .Kp = 2.4f, // 显著增大速度环P，克服36:1变速箱静摩擦
                .Ki = 0.5f, // 加入极少量积分抵消重力
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

    // What: 注册图传电机并保存实例。
    // Why: 后续的自动寻零、正常切位和失败保护都需要直接访问同一个电机对象。
    video_link_motor = DJIMotorInit(&video_link_config);

    // What: 在初始化后缓存正常态的速度环输出上限。
    // Why: 后续寻零会临时压低这个值，缓存原值后恢复才不会把“温柔限流”误带入正常工作态。
    if (video_link_motor != NULL) {
        video_link_normal_speed_maxout = video_link_motor->motor_controller.speed_PID.MaxOut;
    }
}

void VideoLinkMotorEnable(void)
{
    // What: 恢复使能电机
    // Why: 云台模式为陀螺仪等稳定模式时恢复闭环调节
    if (video_link_motor != NULL) {
        // What: 重新允许电机响应控制参考。
        // Why: 图传电机在零力态可能被显式停掉，重新进入工作态时必须先恢复驱动侧使能。
        DJIMotorEnable(video_link_motor);
        // What: 打开模块级工作使能门。
        // Why: `VideoLinkMotorTask()` 需要依赖这一位决定是否继续推进寻零或正常切位状态机。
        is_video_link_enabled = 1;
    }
}

void VideoLinkMotorDisable(void)
{
    // What: 取消使能电机
    // Why: 处于零力状态时切断电流输出，保证安全
    if (video_link_motor != NULL) {
        // What: 零力模式下立即停止给电机发有效电流。
        // Why: 无论当前处于寻零还是正常控制，进入零力后都不应继续拉拽图传机构。
        DJIMotorStop(video_link_motor);
        // What: 关闭模块级工作使能门。
        // Why: 状态机只有在云台允许图传电机工作时才应继续运行。
        is_video_link_enabled = 0;
        // What: 若本次上电还没完成寻零，则把状态退回等待反馈。
        // Why: 零力中断寻零后，下一次重新使能时应从干净的自动寻零入口重新开始，而不是沿用半截计时状态。
        if (is_base_recorded == 0u) {
            video_link_state = VIDEO_LINK_STATE_BOOT_WAIT;
            // What: 清空整次寻零起始时间。
            // Why: 旧的超时计时不应跨零力状态残留，否则重新使能后会立刻被误判超时。
            video_link_homing_start_ms = 0u;
            // What: 清空连续堵转保持计时。
            // Why: 中断后的重新使能必须重新观察“持续贴住限位”，不能复用上一次剩余保持时间。
            video_link_stall_start_ms = 0u;
            // What: 清空 READY 首拍稳定标记。
            // Why: 尚未成功寻零就不存在“寻零完成后的稳定一拍”，保留旧标记只会污染后续状态流转。
            video_link_ready_settle_pending = 0u;
            // What: 恢复正常态控制配置。
            // Why: 寻零阶段会把外环和限幅改成临时参数，中断后先恢复干净配置，避免下次进入时读到脏状态。
            video_link_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
            video_link_motor->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
            video_link_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
            video_link_motor->motor_controller.speed_PID.MaxOut = video_link_normal_speed_maxout;
        }
    }
}

void VideoLinkMotorTask(float pitch_angle_deg)
{
    uint32_t now_ms;
    float current_total_angle_deg;
    float target_ref_angle_deg;
    uint8_t motor_online;

    // What: 先判断电机实例是否存在。
    // Why: 后续无论是在线状态边沿监测还是真实控制命令下发，都必须建立在图传电机实例已经完成初始化的前提上。
    if (video_link_motor == NULL) {
        return;
    }

    // What: 获取当前系统毫秒时间轴。
    // Why: 自动寻零的超时和堵转保持时间都应按真实时间计算，不能再依赖任务拍数硬估周期。
    now_ms = (uint32_t)DWT_GetTimeline_ms();
    // What: 缓存当前总角度反馈。
    // Why: 寻零堵转判定和正常切位目标都依赖总角度，提前取出可避免后面多次重复访问结构体。
    current_total_angle_deg = video_link_motor->measure.total_angle;
    // What: 读取图传电机当前在线状态。
    // Why: 如果 M2006 单独掉电或电调复位，必须第一时间让零点失效并触发重新寻零，不能继续拿旧基准角工作。
    motor_online = (video_link_motor->daemon != NULL && DaemonIsOnline(video_link_motor->daemon) != 0u) ? 1u : 0u;

    // What: 处理图传电机离线/复活边沿。
    // Why: 主控不重启而 M2006 单独掉电时，旧的基准零点和运行态都不再可信，必须显式退回自动寻零入口。
    if (motor_online != video_link_last_online) {
        if (motor_online == 0u) {
            // What: 电机离线后立即使当前零点失效。
            // Why: 只要图传电机失去反馈，就无法再保证之前记录的 total_angle 基准仍然和当前物理机构严格一致。
            is_base_recorded = 0u;
            // What: 同步清空保存的基准总角度。
            // Why: 既然零点已经被判定失效，就不应再保留任何可能被后续 READY 逻辑误用的旧基准值。
            base_total_angle = 0.0f;
            // What: 把状态机退回等待反馈入口。
            // Why: 电机重新上线后应该先重新观察有效反馈，再从头执行温柔寻零，而不是直接回到 READY。
            video_link_state = VIDEO_LINK_STATE_BOOT_WAIT;
            // What: 清空整次寻零超时计时。
            // Why: 掉电前的寻零起始时刻对重新上电后的状态机已经没有意义，继续沿用只会造成误判超时。
            video_link_homing_start_ms = 0u;
            // What: 清空堵转保持计时。
            // Why: 重新上电后的限位接触证据必须重新采样，不能复用掉电前的“连续贴住限位”计时结果。
            video_link_stall_start_ms = 0u;
            // What: 清空 READY 稳定过渡标记。
            // Why: 零点已经失效后，不存在“寻零完成后的第一拍稳态过渡”，保留旧标记只会污染恢复流程。
            video_link_ready_settle_pending = 0u;
            // What: 清空上一拍角度缓存。
            // Why: 电机复活后的第一帧角度要作为新的比较起点，不能再和掉电前的旧角度做增量判断。
            video_link_last_total_angle = 0.0f;
            // What: 清空“允许回高位”的重置门控。
            // Why: 零点已经失效后，之前关于高位回切的历史状态也全部不可信，必须随离线事件一并作废。
            video_link_high_position_rearm_pending = 0u;
            // What: 清空“已经见过 Pitch 低于阈值”的标记。
            // Why: 这次掉电恢复后的重新校准要重新观察一次完整的低于阈值事件，不能沿用掉电前的旧记录。
            video_link_high_position_low_seen = 0u;
            // What: 恢复正常态的控制参数模板。
            // Why: 掉电可能发生在寻零中途，先把临时限流和速度外环配置撤掉，复活后状态机会再重新设置正确的找零参数。
            video_link_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
            video_link_motor->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
            video_link_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
            video_link_motor->motor_controller.speed_PID.MaxOut = video_link_normal_speed_maxout;
            // What: 将电机置为停止状态。
            // Why: 离线期间继续维持旧 stop_flag 没有意义，先明确停机，等重新进入自动寻零时再由模块主动使能。
            DJIMotorStop(video_link_motor);
            // What: 记录一次零点失效日志。
            // Why: 现场若出现“图传位姿突然不可信”，日志里需要明确说明是电机离线导致模块主动废弃了旧零点。
            LOGWARNING("[video_link] motor offline, invalidate homing");
        } else {
            // What: 电机复活后先刷新角度缓存。
            // Why: 自动寻零恢复时要从复活后的第一帧反馈重新开始计算角度增量，避免把复活前后的跳变当成堵转证据。
            video_link_last_total_angle = current_total_angle_deg;
            // What: 输出一次复活日志。
            // Why: 联调时需要能明确看到“掉电恢复后模块已准备重新自动寻零”的时间点。
            LOGINFO("[video_link] motor online, re-home pending");
        }
        // What: 无论离线还是复活，都更新上一拍在线状态缓存。
        // Why: 只有把这次边沿吃掉，下一拍状态机才能回到稳定的电平处理逻辑。
        video_link_last_online = motor_online;
    }

    // What: 边沿监测完成后，再判断当前是否允许图传电机执行实际控制动作。
    // Why: 用户提到“电机重新上电没有明显重新校准”，根因之一就是旧逻辑在失能期间直接 return，错过了掉电/复活边沿；现在要保留监测，但仍然禁止在零力态下发控制命令。
    if (!is_video_link_enabled) {
        return;
    }

    switch (video_link_state) {
    case VIDEO_LINK_STATE_BOOT_WAIT:
        // What: 在真正开始寻零前先等待电机反馈上线。
        // Why: 没有有效反馈时启动低速贴限位既无法做堵转判定，也无法安全记录零点。
        if (motor_online == 0u) {
            return;
        }

        // What: 在重新进入自动寻零前显式恢复驱动侧使能。
        // Why: 若上一拍经历过离线边沿，模块会主动把 stop_flag 拉成 STOP；复活后必须先重新使能，找零参考才会真正生效。
        DJIMotorEnable(video_link_motor);

        // What: 寻零入口只保留速度环。
        // Why: 当前最小改动方案是让电机以低速持续向下限位贴靠，位置环在这个阶段反而会妨碍稳定限流找零。
        video_link_motor->motor_settings.outer_loop_type = SPEED_LOOP;
        // What: 寻零阶段仅启用速度闭环。
        // Why: 这颗 M2006 当前没有配置可用的电流环参数，若贸然打开 CURRENT_LOOP，未配置的内环会把输出链路吃掉。
        video_link_motor->motor_settings.close_loop_type = SPEED_LOOP;
        // What: 显式关闭寻零阶段的前馈叠加。
        // Why: 找零动作只需要一条简单、可预期的低速贴靠控制，任何额外前馈都会放大接触下限位时的不确定性。
        video_link_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
        // What: 临时压低速度环最大输出。
        // Why: 在当前控制框架里，这一项就是最直接的“温柔限流”手段，能把贴限位时的冲击显著压下来。
        video_link_motor->motor_controller.speed_PID.MaxOut = VIDEO_LINK_HOMING_CURRENT_LIMIT_RAW;
        // What: 给寻零动作下发固定的低速下探参考。
        // Why: 让电机始终以统一、可复现的速度慢慢向下限位贴靠，便于堵转判定和后续上车复现。
        DJIMotorSetRef(video_link_motor, VIDEO_LINK_HOMING_SPEED_DPS);
        // What: 记录整次寻零的起始时间。
        // Why: 后续若迟迟找不到下限位，需要依据这个时间做全流程超时退出。
        video_link_homing_start_ms = now_ms;
        // What: 清空连续堵转保持计时。
        // Why: 刚开始找零时还没有任何“贴住限位”的证据，保持计时必须从零开始。
        video_link_stall_start_ms = 0u;
        // What: 保存进入寻零前的当前总角度。
        // Why: 下一拍开始就要依赖前后两拍总角增量判断机构是否还在继续缓慢爬行。
        video_link_last_total_angle = current_total_angle_deg;
        // What: 清空 READY 首拍稳定标记。
        // Why: 重新进入寻零入口意味着上一次 READY 结束态已经失效，不能保留旧的一拍稳定状态。
        video_link_ready_settle_pending = 0u;
        // What: 将内部状态推进到 HOMING。
        // Why: 反馈和限流参数都准备好后，后续每拍都应按“正在自动寻零”的逻辑处理。
        video_link_state = VIDEO_LINK_STATE_HOMING;
        // What: 输出一次状态切换日志。
        // Why: 联调时需要明确知道模块何时真正开始自动找零，方便把日志与机构动作对齐。
        LOGINFO("[video_link] auto homing start");
        return;

    case VIDEO_LINK_STATE_HOMING:
        // What: 寻零过程中持续维持速度外环和温柔限流配置。
        // Why: 其他模块或上次状态残留不应把图传电机从“低速贴限位”模式拉回正常工作参数。
        video_link_motor->motor_settings.outer_loop_type = SPEED_LOOP;
        video_link_motor->motor_settings.close_loop_type = SPEED_LOOP;
        video_link_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
        video_link_motor->motor_controller.speed_PID.MaxOut = VIDEO_LINK_HOMING_CURRENT_LIMIT_RAW;
        // What: 每拍继续给电机下发固定低速参考。
        // Why: 就算任务调度抖动或上层短暂没改参考，寻零态也必须始终把电机钉在“慢速向下找零”的动作上。
        DJIMotorSetRef(video_link_motor, VIDEO_LINK_HOMING_SPEED_DPS);

        // What: 先做整次寻零的超时保护。
        // Why: 若下限位始终没有被可靠识别，继续顶下去只会平白增加机构和供电风险。
        if ((now_ms - video_link_homing_start_ms) >= VIDEO_LINK_HOMING_TIMEOUT_MS) {
            // What: 到达超时后立即停机。
            // Why: 失败态下继续给速度参考没有意义，最安全的做法就是切断这颗图传电机的输出。
            DJIMotorStop(video_link_motor);
            // What: 恢复正常态控制配置。
            // Why: 即使这次寻零失败，模块内部配置也不应永远卡在临时限流找零参数上。
            video_link_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
            video_link_motor->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
            video_link_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
            video_link_motor->motor_controller.speed_PID.MaxOut = video_link_normal_speed_maxout;
            // What: 切到 FAILED 状态。
            // Why: 本次上电不再自动重试，避免在异常装配或故障工况下反复贴限位。
            video_link_state = VIDEO_LINK_STATE_FAILED;
            // What: 打出失败日志。
            // Why: 现场看到图传电机没完成初始化时，需要能从日志里直接区分是“未触发”还是“已超时失败”。
            LOGWARNING("[video_link] auto homing timeout");
            return;
        }

        // What: 只有在速度很低、电流足够大且角度几乎不再变化时才认为进入了疑似堵转到位状态。
        // Why: 三个条件同时成立才能较稳地排除“只是慢速爬行”或“短时通信抖动”造成的假到位。
        if (fabsf(video_link_motor->measure.speed_aps) < VIDEO_LINK_HOMING_STALL_SPEED_DPS &&
            fabsf((float)video_link_motor->measure.real_current) > VIDEO_LINK_HOMING_STALL_CURRENT_RAW &&
            fabsf(current_total_angle_deg - video_link_last_total_angle) < VIDEO_LINK_HOMING_STALL_DELTA_DEG) {
            // What: 第一次命中堵转到位判据时记录保持计时起点。
            // Why: 后续要判断它是否连续稳定贴住下限位达到了足够长的时间。
            if (video_link_stall_start_ms == 0u) {
                video_link_stall_start_ms = now_ms;
            }
            // What: 连续保持足够久后才真正记零并切回正常控制。
            // Why: 这样可以避开限位接触瞬间的反弹和反馈抖动，减少“刚碰一下就误记零”的概率。
            if ((now_ms - video_link_stall_start_ms) >= VIDEO_LINK_HOMING_STALL_HOLD_MS) {
                // What: 将当前总角度记为本次上电的机构零点基准。
                // Why: 现有正常态目标都是围绕低点基准角做相对偏移，寻零成功后首先要把这个基准固定下来。
                base_total_angle = current_total_angle_deg;
                // What: 标记零点已经有效。
                // Why: 后续 READY 状态和重新使能路径都需要依赖这一位判断是否还要再次自动寻零。
                is_base_recorded = 1u;
                // What: 恢复正常态的位置外环和速度内环配置。
                // Why: 自动找零已经结束，后续就应该回到原本的“基准角 + 抬起偏移”位置控制模式。
                video_link_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
                video_link_motor->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
                video_link_motor->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
                video_link_motor->motor_controller.speed_PID.MaxOut = video_link_normal_speed_maxout;
                // What: 先把参考值钉在刚记录下来的基准零点。
                // Why: 刚完成寻零的这一拍先稳住当前位置，能避免状态切回 READY 后立刻因为外部阈值条件跳到另一个目标。
                DJIMotorSetRef(video_link_motor, base_total_angle);
                // What: 标记 READY 第一拍需要仅做稳态过渡。
                // Why: 这样下一拍再恢复正常目标切换逻辑，机构过渡会更平顺。
                video_link_ready_settle_pending = 1u;
                // What: 寻零成功后先禁止直接回高位。
                // Why: 无论是冷启动还是高点掉电恢复，若当前 Pitch 恰好还在阈值以上，直接回高位会把这次低位校准动作完全吃掉。
                video_link_high_position_rearm_pending = 1u;
                // What: 同时清空“已见过低于阈值”的标记。
                // Why: 后续必须重新观察到一次 Pitch 低于阈值，才能认定高位回切条件被真正重新武装。
                video_link_high_position_low_seen = 0u;
                // What: 切到 READY 状态。
                // Why: 自动寻零成功后，后续任务逻辑就该完全交回正常图传切位控制。
                video_link_state = VIDEO_LINK_STATE_READY;
                // What: 输出一次寻零成功日志。
                // Why: 方便现场确认这次开机已经可靠完成了图传电机自动找零。
                LOGINFO("[video_link] auto homing done");
                return;
            }
        } else {
            // What: 任一堵转判据失效时清空保持计时。
            // Why: 到位确认要求“连续成立”，一旦中途恢复运动或电流掉下去，就必须重新开始计时。
            video_link_stall_start_ms = 0u;
        }

        // What: 更新上一拍总角度缓存。
        // Why: 下一拍还要继续用它判断“当前这一拍究竟有没有继续往下爬行”。
        video_link_last_total_angle = current_total_angle_deg;
        return;

    case VIDEO_LINK_STATE_READY:
        // What: 寻零完成后的第一拍仅清掉稳态过渡标记。
        // Why: 上一拍已经把参考钉在基准零点，这一拍直接恢复阈值切换容易让机构刚记零就立刻抽一下。
        if (video_link_ready_settle_pending != 0u) {
            video_link_ready_settle_pending = 0u;
            return;
        }

        // What: 默认让图传电机维持在低点基准位。
        // Why: 当前设计仍然是“只有 Pitch 抬高到阈值以上时，图传机构才需要切到高位”。
        target_ref_angle_deg = base_total_angle;

        // What: 寻零完成后先等待一次“低于阈值”事件。
        // Why: 只有真的看见 Pitch 回到低阈值以下，后续再上穿阈值才有资格被视为新的高位切换请求。
        if (video_link_high_position_rearm_pending != 0u && pitch_angle_deg < PITCH_ANGLE_THRESHOLD) {
            video_link_high_position_low_seen = 1u;
        }
        // What: 在已经见过低于阈值之后，允许下一次上穿阈值解除高位门控。
        // Why: 这条边界正是为了解决“高点掉电恢复后校准动作不明显”，必须强制经历一次新的阈值上升沿才放行高位。
        if (video_link_high_position_rearm_pending != 0u &&
            video_link_high_position_low_seen != 0u &&
            pitch_angle_deg >= PITCH_ANGLE_THRESHOLD) {
            video_link_high_position_rearm_pending = 0u;
        }

        // What: 只有在高位门控解除后，才允许根据 Pitch 阈值切到抬起位目标。
        // Why: 这样可以确保高点掉电恢复后的那次重新校准在机构上是可见的，而不是校完立刻又回到高位。
        if (video_link_high_position_rearm_pending == 0u && pitch_angle_deg >= PITCH_ANGLE_THRESHOLD) {
            target_ref_angle_deg += VIDEO_LINK_ROTATION_DEG;
        }

        // What: 将正常态目标角下发给电机。
        // Why: READY 以后图传电机就回到原有的角度切位职责，不再参与自动寻零流程。
        DJIMotorSetRef(video_link_motor, target_ref_angle_deg);
        return;

    case VIDEO_LINK_STATE_FAILED:
        // What: 失败态持续保持停机。
        // Why: 本次上电既然已经确认自动寻零失败，就不应该继续让图传电机在未知零点下工作。
        DJIMotorStop(video_link_motor);
        return;

    default:
        // What: 对异常状态值做兜底复位。
        // Why: 即使状态变量被意外写坏，也要优先回到最安全的“等待有效反馈再启动自动寻零”路径。
        video_link_state = VIDEO_LINK_STATE_BOOT_WAIT;
        // What: 清空整次寻零计时。
        // Why: 兜底恢复时不应把未知状态下的旧时间戳带入新的状态机周期。
        video_link_homing_start_ms = 0u;
        // What: 清空堵转保持计时。
        // Why: 旧的连续到位证据在状态异常后已经不可信，必须重新采样。
        video_link_stall_start_ms = 0u;
        // What: 清空 READY 首拍稳态标记。
        // Why: 异常恢复后第一要务是重新找回合法状态，不应直接跳过正常过渡逻辑。
        video_link_ready_settle_pending = 0u;
        return;
    }
}
