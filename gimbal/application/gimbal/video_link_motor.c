/**
 * @file video_link_motor.c
 * @brief 图传固定电机模块实现
 *        M2006电机(CAN2, ID7)用于图传镜头角度固定:
 *        位置环控制向下运动 → 堵转检测确认到位 → 停止发力
 */

#include "video_link_motor.h"
#include "dji_motor.h"
#include "motor_def.h"
#include "robot_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "general_def.h"

/* ========================= 可调参数宏定义 ========================= */

// 目标角度(电机转子total_angle, 单位deg), 负值表示向下转动方向
// Why: M2006减速比36:1, -500°约为输出端-13.9°, 需根据实际安装调整
#define VIDEO_LINK_TARGET_ANGLE (-500.0f)

// 堵转检测: 电流绝对值阈值 (M2006反馈电流范围约±10000)
// Why: 用户指定2000, 2006电机堵转时电流会迅速上升, 该值需根据负载实测微调
#define VL_STALL_CURRENT_THRESHOLD 2000

// 堵转检测: 速度绝对值阈值 (deg/s), 低于此值认为电机已停转
// Why: 30 deg/s足够区分正常运动与堵转, 避免减速过程误触发
#define VL_STALL_SPEED_THRESHOLD 30.0f

// 堵转消抖时间 (ms), 持续满足堵转条件超过此时间才确认堵转
// Why: 100ms消抖可过滤瞬态冲击和启动阶段的假堵转信号
#define VL_STALL_DEBOUNCE_TIME 100.0f

/* ========================= 状态机枚举 ========================= */

// 图传电机工作状态
// Why: 三态状态机简洁且覆盖全部工作场景(待机/运动/锁定)
typedef enum {
    VL_IDLE = 0, // 待机: 电机停止, 等待使能信号
    VL_MOVING, // 运动: 位置环驱动, 向目标角度运动中
    VL_LOCKED, // 锁定: 堵转检测触发后停止发力, 已到位
} VideoLinkState_e;

/* ========================= 模块内部变量 ========================= */

// 电机实例指针
static DJIMotorInstance *vl_motor = NULL;

// 当前状态机状态
static VideoLinkState_e vl_state = VL_IDLE;

// 堵转消抖计时起点 (ms)
static float stall_detect_start_time = 0.0f;

// 堵转消抖中标志: 1=正在消抖计时, 0=未检测到堵转
static uint8_t stall_detecting = 0;

// 使能请求标志: 由外部Enable/Disable接口设置
static uint8_t vl_enable_flag = 0;

/* ========================= 函数实现 ========================= */

/**
 * @brief 初始化图传固定电机
 *        注册M2006到CAN2, ID7, 配置位置环+速度环串级PID
 */
void VideoLinkMotorInit(void)
{
    // M2006电机配置: CAN2, ID 7 (属于0x1FF发送组)
    // Why: CAN2 ID 1-6已被摩擦轮占用, 7为下一个可用ID
    Motor_Init_Config_s vl_config = {
        .can_init_config = {
            .can_handle = &hcan2, // 使用CAN2总线
            .tx_id = 7, // 电调拨码开关设为ID 7
        },
        .controller_param_init_config = {
            // 位置环PID: 外环, 输出作为速度环的目标值
            // Why: Kp=10适中响应, MaxOut=8000限幅防止速度过快冲击机械结构
            .angle_PID = {
                .Kp = 10.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5000,
                .MaxOut = 8000,
            },
            // 速度环PID: 内环, 输出直接作为电流控制量
            // Why: Kp=10 Ki=1提供稳态精度, MaxOut=10000为M2006最大电流的安全限幅
            .speed_PID = {
                .Kp = 10.0f,
                .Ki = 1.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, // 使用电机编码器反馈角度
            .speed_feedback_source = MOTOR_FEED, // 使用电机编码器反馈速度
            .outer_loop_type = ANGLE_LOOP, // 外层为位置环
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP, // 位置+速度串级
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 根据安装方向调整
        },
        .motor_type = M2006, // M2006电机类型
    };

    // vl_motor = DJIMotorInit(&vl_config);

    if (vl_motor == NULL) {
        LOGERROR("[video_link] M2006 init failed!"); // 初始化失败记录日志
        return;
    }

    // 初始化时停止电机, 等待云台使能后再启动
    // Why: 防止上电瞬间电机意外转动
    DJIMotorStop(vl_motor);

    // 初始化状态机为IDLE
    vl_state = VL_IDLE;
    stall_detecting = 0;
    vl_enable_flag = 0;

    LOGINFO("[video_link] M2006 on CAN2 ID7 init OK"); // 初始化成功日志
}

/**
 * @brief 检测M2006是否堵转 (高电流 + 低转速)
 * @return 1=当前满足堵转条件, 0=正常运行
 */
static uint8_t IsVideoLinkStalled(void)
{
    if (vl_motor == NULL)
        return 0;

    // 取电流和速度的绝对值
    // Why: 电流和速度可正可负, 堵转判断只关心幅值
    int16_t current_abs = (vl_motor->measure.real_current > 0) ? vl_motor->measure.real_current : -vl_motor->measure.real_current;
    float speed_abs = (vl_motor->measure.speed_aps > 0.0f) ? vl_motor->measure.speed_aps : -vl_motor->measure.speed_aps;

    // 同时满足高电流和低速度才认为堵转
    // Why: 单一条件容易误判(如启动瞬间电流大但速度正在上升)
    return (current_abs > VL_STALL_CURRENT_THRESHOLD) &&
           (speed_abs < VL_STALL_SPEED_THRESHOLD);
}

/**
 * @brief 图传固定电机状态机, 周期性调用
 *        IDLE → MOVING → LOCKED
 */
void VideoLinkMotorTask(void)
{
    if (vl_motor == NULL)
        return; // 电机未初始化, 跳过

    float current_time = DWT_GetTimeline_ms(); // 获取当前系统时间

    switch (vl_state) {
    case VL_IDLE:
        // 待机状态: 等待使能信号
        // Why: 云台零力模式下电机不应动作
        DJIMotorStop(vl_motor);
        stall_detecting = 0; // 重置消抖标志

        if (vl_enable_flag) {
            // 收到使能信号, 进入运动状态
            // Why: 切换到MOVING前先启用电机并设定目标角度
            DJIMotorEnable(vl_motor);
            DJIMotorSetRef(vl_motor, VIDEO_LINK_TARGET_ANGLE);
            vl_state = VL_MOVING;
            stall_detecting = 0;
            LOGINFO("[video_link] state: IDLE -> MOVING, target=%.1f deg",
                    VIDEO_LINK_TARGET_ANGLE);
        }
        break;

    case VL_MOVING:
        // 运动状态: 位置环驱动中, 同时检测堵转
        if (!vl_enable_flag) {
            // 使能被撤销, 回到IDLE
            // Why: 云台切到零力模式时必须立即停止电机
            vl_state = VL_IDLE;
            DJIMotorStop(vl_motor);
            stall_detecting = 0;
            LOGINFO("[video_link] state: MOVING -> IDLE (disabled)");
            break;
        }

        // 持续设定目标角度 (防止被外部修改)
        DJIMotorSetRef(vl_motor, VIDEO_LINK_TARGET_ANGLE);

        // 堵转检测 + 消抖逻辑
        if (IsVideoLinkStalled()) {
            if (!stall_detecting) {
                // 首次检测到堵转条件, 开始消抖计时
                // Why: 需要持续一段时间才确认, 避免瞬态误触发
                stall_detecting = 1;
                stall_detect_start_time = current_time;
            } else {
                // 正在消抖中, 检查是否超过消抖时间
                if ((current_time - stall_detect_start_time) >= VL_STALL_DEBOUNCE_TIME) {
                    // 确认堵转, 停止电机, 进入锁定状态
                    // Why: 堵转意味着已经到达机械限位, 继续发力只会浪费功率和发热
                    DJIMotorStop(vl_motor);
                    vl_state = VL_LOCKED;
                    stall_detecting = 0;
                    LOGINFO("[video_link] state: MOVING -> LOCKED (stall detected)");
                }
            }
        } else {
            // 堵转条件不满足, 重置消抖
            // Why: 如果中途恢复转动说明之前是假堵转, 需要重新计时
            stall_detecting = 0;
        }
        break;

    case VL_LOCKED:
        // 锁定状态: 已到位, 电机保持停止
        // Why: 图传已固定到位, 无需继续发力
        DJIMotorStop(vl_motor);

        if (!vl_enable_flag) {
            // 使能被撤销, 回到IDLE(便于下次重新启动)
            vl_state = VL_IDLE;
            LOGINFO("[video_link] state: LOCKED -> IDLE (disabled)");
        }
        break;

    default:
        // 异常状态恢复
        vl_state = VL_IDLE;
        break;
    }
}

/**
 * @brief 使能图传电机
 *        在云台非零力模式下调用, 允许电机开始运动
 */
void VideoLinkMotorEnable(void)
{
    vl_enable_flag = 1; // 设置使能标志, 状态机在下一周期响应
}

/**
 * @brief 停止图传电机
 *        在云台零力模式下调用, 电机立即停止
 */
void VideoLinkMotorDisable(void)
{
    vl_enable_flag = 0; // 清除使能标志, 状态机将回到IDLE
}
