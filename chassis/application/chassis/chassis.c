/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "can.h"
#include "motor_def.h"
#include "robot_def.h"
#include "power_control.h"
#include "dmmotor.h"
#define USE_SUPER_CAP
#ifdef USE_SUPER_CAP
#include "super_cap.h" // [条件编译] 仅在启用超电时包含此头文件
#endif
#include "message_center.h"
#include "referee_task.h"
#include <arm_math.h> // for fabsf

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "user_lib.h"
#include "arm_math.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f) // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f) // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define DEFAULT_TEST_POWER 80.0f // 调试用的基础功率
#define LIFT_DIAL_MAX_SPEED_DPS 65.0f // What: 定义抬升拨轮映射后的最大调节角速度；Why: 提高抬升跟手性同时保持位置环仍在可控范围内
#define LIFT_RELATIVE_MIN_ANGLE -20.0f // What: 定义相对进入点的最小抬升角；Why: 给机构保留回落空间并避免误操作顶到底部极限
#define LIFT_RELATIVE_MAX_ANGLE 80.0f // What: 定义相对进入点的最大抬升角；Why: 用保守软件限位先保护机构，后续可按实车行程再放宽
#define CHASSIS_TASK_DT_FALLBACK 0.005f // What: 定义底盘任务积分后备周期；Why: DWT异常时仍按200Hz近似积分，避免抬升目标突变
#define FOLLOW_TRANSITION_HOLD_TICKS 20u // What: 定义小陀螺退跟随后接管保持拍数；Why: 约100ms的刹停窗口足够先卸掉残余自旋，再进入回正环能明显减少反向抽动。
#define FOLLOW_TRANSITION_EXIT_WZ_DPS 120.0f // What: 定义接管阶段允许提前退出的底盘角速度阈值；Why: 余旋已经很小时没必要继续硬刹，尽早恢复回正能减少体感拖滞。
#define FOLLOW_TRANSITION_BRAKE_KD 1.6f // What: 定义接管阶段只看车体角速度的阻尼增益；Why: 先用纯阻尼把小陀螺残余动量压下，避免偏角环和前馈同时抢控制权。
#define FOLLOW_TRANSITION_MAX_WZ 900.0f // What: 定义接管阶段底盘角速度输出上限；Why: 先刹停阶段只需要中等强度制动，限幅后更不容易把轮速环再次顶入饱和。
#define FOLLOW_ANGLE_FILTER_ALPHA 0.18f // What: 定义跟随偏角的一阶滤波系数；Why: 退出接管时先把机械回弹和单圈角小抖动滤掉，避免刚恢复P项就来回翻向。
#define FOLLOW_OUTPUT_SLEW_DPS_PER_S 30000.0f // What: 定义跟随输出角速度的变化率上限；Why: 保留足够快的接管制动，同时避免控制律切段时输出一步跳变。

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
// What: 在编译期校验底盘反馈结构体尺寸；Why: 双板通信仍需保证单帧CAN可以装下完整反馈数据。
_Static_assert(sizeof(Chassis_Upload_Data_s) <= CAN_COMM_MAX_BUFFSIZE,
               "Chassis_Upload_Data_s exceeds CAN_COMM_MAX_BUFFSIZE");
// What: 在编译期校验底盘控制结构体尺寸；Why: 新增上岛字段后必须继续保证命令帧不会溢出CANComm缓冲区。
_Static_assert(sizeof(Chassis_Ctrl_Cmd_s) <= CAN_COMM_MAX_BUFFSIZE,
               "Chassis_Ctrl_Cmd_s exceeds CAN_COMM_MAX_BUFFSIZE");
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub; // 用于发布底盘的数据
static Subscriber_t *chassis_sub; // 用于订阅底盘的控制命令
#endif // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv; // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t *referee_data = { NULL }; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

#ifdef USE_SUPER_CAP
static SuperCapInstance *cap = { NULL }; // [条件编译] 超级电容实例指针
#endif
#ifdef USE_ISLAND_ACTION
#include "island_action.h" // [条件编译] 仅在启用上岛机构时包含上岛头文件
#endif // USE_ISLAND_ACTION

static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

// 方案二，陀螺仪针对全麦纠偏PID
static PIDInstance yaw_lock_pid; // 航向锁定专用PID
static float lock_target_yaw = 0.0f; // 锁定的目标角度
static uint8_t is_manual_rotating = 0; // 标记是否正在手动旋转

static uint8_t last_cali_flag = 0; // 上一次的校准标志位

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy; // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

static float real_vx = 0.0f; // 真实前进速度 m/s
static float real_vy = 0.0f; // 真实横移速度 m/s
static float real_wz = 0.0f; // 真实旋转速度 deg/s
static chassis_mode_e last_chassis_mode = CHASSIS_ZERO_FORCE; // What: 记录上一拍底盘模式；Why: 底盘板本地也要能识别小陀螺退跟随边沿，避免上板请求偶发漏拍时接管逻辑失效。
static uint8_t last_follow_transition_request = 0u; // What: 记录接管请求上一拍电平；Why: 上板会保持几拍请求位，底盘侧只应在上升沿触发一次接管窗口。
static uint8_t follow_transition_ticks = 0u; // What: 记录当前跟随接管剩余拍数；Why: 用固定拍数窗口先刹停再回正，比直接混控更稳且实现确定性更强。
static float follow_angle_err_filtered = 0.0f; // What: 缓存滤波后的跟随偏角；Why: 刚退出接管时若直接吃原始偏角，容易被机械回弹和量测毛刺再次拉成反复摆动。
static float follow_wz_cmd_limited = 0.0f; // What: 缓存限斜率后的跟随输出；Why: 接管段切回正常跟随时沿用连续状态，避免指令一步跳变刺激轮速环。
static uint32_t follow_control_dwt_cnt = 0u; // What: 记录跟随控制的DWT时间基准；Why: 输出斜率限制要按真实周期换算，不能假设任务永远严格等于5ms。

// ==========================================
// 小陀螺模式配置
// ==========================================
#define VARIABLE_SPIN_ENABLED 1 // What: 保留小陀螺的轻微变速效果；Why: 贴边吃功率时仍保留一点扰动，降低被针对时的运动可预测性。
#define SPIN_BASE_INIT_SPEED 1400.0f // What: 定义小陀螺进入时的初始基础角速度；Why: 首拍就给到中高转速，避免每次进小陀螺都要从很低速度慢慢爬升。
#define SPIN_BASE_MIN_SPEED 900.0f // What: 定义功率闭环允许维持的最小基础角速度；Why: 即使功率余量吃紧也保留稳定自旋，不让姿态突然塌掉。
#define SPIN_TOP_MAX_SPEED 3200.0f // What: 提高小陀螺基础角速度上限；Why: 让功率控制器在合法范围内有足够目标可追，才能把可用功率真正吃满。
#define SPIN_POWER_TARGET_BASE_RATIO 0.96f // What: 定义小陀螺平均功率目标比例；Why: 让整段变速逻辑围绕贴边功率运行，而不是回到过去那种明显留手的保守状态。
#define SPIN_POWER_TARGET_WAVE_AMPLITUDE 0.025f // What: 定义变速波形对功率目标比例的调制幅度；Why: 高速段略多吃一点功率、低速段略收一点，变速节奏会更明显但仍留有安全余量。
#define SPIN_POWER_TRACK_GAIN 10.0f // What: 定义功率误差到角速度修正的比例系数；Why: 让小陀螺能跟着功率余量快速抬速，但不过分激进导致来回抽动。
#define SPIN_POWER_STEP_UP_MAX 20.0f // What: 限制单周期最大提速量；Why: 200Hz任务下给平滑爬升，避免目标角速度阶跃过大把轮速环瞬间顶饱和。
#define SPIN_POWER_STEP_DOWN_MAX 35.0f // What: 限制单周期最大降速量；Why: 超功率边缘时更快回收旋转目标，优先守住不超功率底线。
#define SPIN_POWER_DEADBAND_W 2.0f // What: 定义功率误差死区；Why: 吃满附近直接忽略微小波动，减少由裁判功率抖动带来的转速抖动。
#define SPIN_WAVE_SCALE_AMPLITUDE 0.15f // What: 定义小陀螺角速度包络波动幅度；Why: 把快慢节奏拉得更开一些，让变速逻辑在场上是明显可感知的。
#define SPIN_WAVE_REFRESH_MIN_MS 180u // What: 定义无节奏变速目标的最短刷新时间；Why: 保证变速方向不会切得过快，避免底盘体感变成抖动。
#define SPIN_WAVE_REFRESH_MAX_MS 520u // What: 定义无节奏变速目标的最长刷新时间；Why: 让目标保持时间也带随机性，避免形成“固定拍点”。
#define SPIN_WAVE_SMOOTH_ALPHA 0.06f // What: 定义当前波形向随机目标逼近的平滑系数；Why: 让无节奏变速保持连续过渡，不出现突兀阶跃。
#define TRANSLATION_PRIORITY_RATIO 0.6f // What: 定义平移优先系数；Why: 贴边吃功率时仍优先保证平移手感，避免横移一给就把整车拖死。
#define DEFAULT_TEST_BUFFER_ENERGY_J 60.0f // What: 定义无裁判系统时的默认缓冲能量；Why: 场下调车也要能走通超电功率策略，不能因为没裁判就永远进不到激进分支。
#define CHASSIS_SUPER_CAP_BONUS_LOW_W 35.0f // What: 定义超电低能量档的附加功率；Why: 电容电量不高时也先给一小档放电，让体感尽快从“没反应”变成“有帮助”。
#define CHASSIS_SUPER_CAP_BONUS_MID_W 60.0f // What: 定义超电中能量档的附加功率；Why: 电容进入可用区后直接给明显增益，体现比基础模式更激进的输出。
#define CHASSIS_SUPER_CAP_BONUS_HIGH_W 75.0f // What: 定义超电高能量档的附加功率；Why: 电容和裁判缓冲都充足时允许更猛地放电，把爆发优势真正打出来。
#define CHASSIS_SUPER_CAP_ROTATE_EXTRA_W 10.0f // What: 定义小陀螺工况额外附加的超电功率；Why: 自旋时功率起伏最大，需要再补一档预算才能让实际转速更敢放。
#define CHASSIS_SUPER_CAP_MIN_PERCENT 15.0f // What: 定义超电开始介入的最低电量百分比；Why: 把起放门槛从保守值下探，让超电更早参与而不是一直等到很满才出手。
#define CHASSIS_SUPER_CAP_MID_PERCENT 35.0f // What: 定义超电中档功率的电量阈值；Why: 分档控制比单阈值更容易同时兼顾激进体感和低电量保护。
#define CHASSIS_SUPER_CAP_HIGH_PERCENT 60.0f // What: 定义超电高档功率的电量阈值；Why: 只有电容余量明显充足时才拉到最高 bonus，避免长期贴顶后一下子掉空。
#define CHASSIS_SUPER_CAP_MIN_BUFFER_J 15.0f // What: 定义超电开始介入的最小裁判缓冲能量；Why: 即使电容电量一般，只要裁判缓冲还厚，也允许先上低档爆发。
#define CHASSIS_SUPER_CAP_MID_BUFFER_J 35.0f // What: 定义超电中档功率的裁判缓冲阈值；Why: 把裁判缓冲一起纳入决策，避免只看电容百分比导致机会窗口利用不足。
#define CHASSIS_SUPER_CAP_HIGH_BUFFER_J 60.0f // What: 定义超电高档功率的裁判缓冲阈值；Why: 缓冲和电容都高时直接进入最猛档，让整车更敢吃功率。
#define CHASSIS_TOTAL_POWER_LIMIT_MAX_W 165.0f // What: 定义超电介入后的底盘总功率硬上限；Why: 用户要求更激进，就把总预算再往上抬一档，但仍保留硬上限避免完全失控。

static float spin_power_base_wz = SPIN_BASE_INIT_SPEED; // What: 缓存小陀螺基础角速度闭环状态；Why: 通过跨周期累积调节把实际功率稳定贴到上限附近。
static float spin_wave_current = 0.0f; // What: 缓存当前无节奏变速波形值；Why: 通过连续状态平滑逼近随机目标，避免每拍直接跳变。
static float spin_wave_target = 0.0f; // What: 缓存当前随机变速目标；Why: 让一段时间内的快慢趋势保持一致，而不是完全白噪声式乱跳。
static uint32_t spin_wave_next_refresh_tick = 0u; // What: 记录下次刷新随机目标的时间戳；Why: 让每次目标切换间隔本身也不固定，进一步去掉节奏感。
static uint32_t spin_wave_rng_state = 0x13572468u; // What: 保存轻量级伪随机状态；Why: 裸机/RTOS环境下不用标准库随机数也能稳定生成无节奏变速序列。

#ifdef USE_SUPER_CAP
static float GetAggressiveSuperCapBonus(float buffer_energy_j, uint8_t chassis_output_allowed)
{
    float cap_percent;
    float bonus = 0.0f;

    // What: 统一封装超电激进功率加成决策；Why: 超电阈值、错误保护和模式判断分散写在任务里很容易互相打架，抽成单函数更不容易改坏。
    if (cap == NULL || cap->is_online == 0u || chassis_output_allowed == 0u) {
        return 0.0f;
    }

    if (SuperCapGetErrorCode(cap) != 0u || SuperCapIsOutputDisabled(cap) != 0u) {
        // What: 超电报真实错误或输出被禁用时直接不给 bonus；Why: 此时继续放大底盘功率预算只会制造“指令很猛但电源不给”的假象。
        return 0.0f;
    }

    if (chassis_cmd_recv.cap_mode != SUPER_CAP_ON &&
        fabsf(Chassis_IMU_data->Pitch) <= CHASSIS_SLOPE_THRESHOLD) {
        // What: 非常规爆发模式且不在坡道时不介入激进 bonus；Why: 让超电加成仍受上层意图约束，避免全工况都顶着最高功率跑。
        return 0.0f;
    }

    cap_percent = SuperCapGetEnergyPercent(cap);
    if (cap_percent >= CHASSIS_SUPER_CAP_HIGH_PERCENT ||
        buffer_energy_j >= CHASSIS_SUPER_CAP_HIGH_BUFFER_J) {
        bonus = CHASSIS_SUPER_CAP_BONUS_HIGH_W;
    } else if (cap_percent >= CHASSIS_SUPER_CAP_MID_PERCENT ||
               buffer_energy_j >= CHASSIS_SUPER_CAP_MID_BUFFER_J) {
        bonus = CHASSIS_SUPER_CAP_BONUS_MID_W;
    } else if (cap_percent >= CHASSIS_SUPER_CAP_MIN_PERCENT ||
               buffer_energy_j >= CHASSIS_SUPER_CAP_MIN_BUFFER_J) {
        bonus = CHASSIS_SUPER_CAP_BONUS_LOW_W;
    }

    if (bonus > 0.0f && chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE) {
        // What: 小陀螺工况额外叠一档 bonus；Why: 自旋时轮组功率波动最大，只靠通用 bonus 往往还不够把转速真正托起来。
        bonus += CHASSIS_SUPER_CAP_ROTATE_EXTRA_W;
    }

    return bonus;
}
#endif // USE_SUPER_CAP

/**
 * @brief 计算小陀螺的旋转目标速度 (核心优化逻辑：平移优先)
 *
 * @param base_target_wz 基础目标旋转速度 (如果是变速模式，这里输入的是随时间变化的值)
 * @param vx_cmd 当前的前进指令 (来自摇杆解析，范围对应最大如 6600)
 * @param vy_cmd 当前的横移指令 (来自摇杆解析，范围对应最大如 6600)
 * @return float 最终的旋转速度 target_wz
 */
static float OptimizedSpinSpeed(float base_target_wz, float vx_cmd, float vy_cmd)
{
    // 1. 计算当前的平移需求幅度
    float trans_speed = sqrtf(vx_cmd * vx_cmd + vy_cmd * vy_cmd);

    // 2. 归一化平移强度 (0.0 ~ 1.0)
    // 根据 robot_cmd.c 中的摇杆解算逻辑，推杆满位时 vx/vy 数值可达 6600。
    // 将阈值修正为 6600.0f 以匹配当前的指令单位尺度，恢复摇杆的线性与平移灵敏度。
    const float MAX_TRANS_SPEED_REF = 6600.0f;
    float trans_ratio = trans_speed / MAX_TRANS_SPEED_REF;
    if (trans_ratio > 1.0f)
        trans_ratio = 1.0f;

    // 3. 计算旋转缩放系数
    // 当 trans_ratio = 0 (静止) -> scale = 1.0 (全速旋转)
    // 当 trans_ratio = 1 (全速平移) -> scale = (1 - TRANSLATION_PRIORITY_RATIO) (例如 0.4)
    // 这样保证了全速移动时，旋转速度不会完全降为0（保留一点小陀螺效果），但主要功率留给平移
    float spin_scale = 1.0f - (trans_ratio * TRANSLATION_PRIORITY_RATIO);

    // 4. 应用缩放
    return base_target_wz * spin_scale;
}

/**
 * @brief 生成一个 -1.0 ~ 1.0 的伪随机值
 *
 * @return float 归一化的伪随机值
 */
static float GetSpinRandomSignedUnit(void)
{
    // What: 使用 xorshift 更新随机状态；Why: 算法开销极低，适合底盘高频任务里生成“无节奏但可控”的目标。
    spin_wave_rng_state ^= spin_wave_rng_state << 13;
    spin_wave_rng_state ^= spin_wave_rng_state >> 17;
    spin_wave_rng_state ^= spin_wave_rng_state << 5;

    // What: 将整数随机态映射到对称区间；Why: 后续同时调功率目标和角速度包络时需要一个中心在零点的有符号变量。
    return ((float)(spin_wave_rng_state & 0xFFFFu) / 32767.5f) - 1.0f;
}

/**
 * @brief 生成归一化的小陀螺无节奏变速波形
 *
 * @return float 当前时刻的波形值，范围约为 -1.0 ~ 1.0
 */
static float GetVariableSpinWave(void)
{
    // What: 使用系统节拍驱动随机目标刷新；Why: 只在到期时切换趋势，平时保持平滑逼近，才能兼顾无节奏与可控性。
    uint32_t current_time = HAL_GetTick();

    if (spin_wave_next_refresh_tick == 0u || (int32_t)(current_time - spin_wave_next_refresh_tick) >= 0) {
        uint32_t refresh_span = SPIN_WAVE_REFRESH_MAX_MS - SPIN_WAVE_REFRESH_MIN_MS;
        float random_wave = GetSpinRandomSignedUnit();

        // What: 刷新下一段随机变速目标；Why: 让快慢段的方向和幅度都不固定，避免被人听节奏或看轨迹读出来。
        spin_wave_target = random_wave;
        // What: 随机化下一次刷新间隔；Why: 即使目标幅度相近，切换时刻也不固定，进一步打散周期性。
        spin_wave_next_refresh_tick = current_time + SPIN_WAVE_REFRESH_MIN_MS + (spin_wave_rng_state % (refresh_span + 1u));
    }

    // What: 让当前波形缓慢追向随机目标；Why: 速度变化需要连续，不能因为目标随机就让输出也随机抽动。
    spin_wave_current += (spin_wave_target - spin_wave_current) * SPIN_WAVE_SMOOTH_ALPHA;
    LIMIT_MIN_MAX(spin_wave_current, -1.0f, 1.0f);
    return spin_wave_current;
}

/**
 * @brief 按实时功率余量自适应调整小陀螺基础角速度
 *
 * @param power_limit 当前底盘允许使用的总功率上限
 * @return float 已贴边调节后的基础角速度
 */
static float GetAdaptiveSpinBase(float power_limit)
{
    // What: 读取上一控制拍估算的底盘功率；Why: 功率限幅已经在电机层闭环完成，直接复用其估计值即可形成外层吃满控制。
    float measured_power = PowerControlGetChassisPower();
    float target_power_ratio = SPIN_POWER_TARGET_BASE_RATIO;
    // What: 预留少量余量作为贴边目标；Why: 实车裁判值和功率模型都存在抖动，完全打满更容易出现“忽超忽回收”。
#if VARIABLE_SPIN_ENABLED
    float spin_wave = GetVariableSpinWave();
    // What: 让变速波形直接调制功率目标比例；Why: 高速段会主动索取更多合法功率，确保变速逻辑不仅体现在目标值上，也体现在真实输出上。
    target_power_ratio += spin_wave * SPIN_POWER_TARGET_WAVE_AMPLITUDE;
#else
    float spin_wave = 0.0f;
#endif
    LIMIT_MIN_MAX(target_power_ratio, 0.90f, 0.99f);
    float target_power = power_limit * target_power_ratio;
    float power_error = target_power - measured_power;
    float wz_delta = 0.0f;

    // What: 仅在偏离目标较明显时修正基础角速度；Why: 避免已经吃满附近时还被功率噪声推着来回抽动。
    if (fabsf(power_error) > SPIN_POWER_DEADBAND_W) {
        wz_delta = power_error * SPIN_POWER_TRACK_GAIN;
        LIMIT_MIN_MAX(wz_delta, -SPIN_POWER_STEP_DOWN_MAX, SPIN_POWER_STEP_UP_MAX);
    }

    // What: 将功率误差积分到基础角速度状态上；Why: 让小陀螺能随着余量逐步抬速，直到后级功率控制开始稳定限幅。
    spin_power_base_wz += wz_delta;
    LIMIT_MIN_MAX(spin_power_base_wz, SPIN_BASE_MIN_SPEED, SPIN_TOP_MAX_SPEED);

#if VARIABLE_SPIN_ENABLED
    {
        float variable_spin_scale = 1.0f + spin_wave * SPIN_WAVE_SCALE_AMPLITUDE;
        float variable_spin_wz = spin_power_base_wz * variable_spin_scale;
        // What: 对附加了明显变速包络的目标再做一次限幅；Why: 即使波峰阶段主动索取更多功率，也不能让目标角速度越过软件安全边界。
        LIMIT_MIN_MAX(variable_spin_wz, SPIN_BASE_MIN_SPEED, SPIN_TOP_MAX_SPEED);
        return variable_spin_wz;
    }
#else
    return spin_power_base_wz;
#endif
}

static void ResetAdaptiveSpinBase(void)
{
    // What: 在退出小陀螺时恢复基础角速度状态；Why: 避免上一次贴边学到的高转速在下次切入时直接带来过猛的瞬时冲击。
    spin_power_base_wz = SPIN_BASE_INIT_SPEED;
    // What: 在退出小陀螺时清空无节奏变速状态；Why: 下次进入重新生成一段新的随机趋势，避免固定沿用上一段未走完的节奏。
    spin_wave_current = 0.0f;
    spin_wave_target = 0.0f;
    spin_wave_next_refresh_tick = 0u;
}

static float GetFollowControlDt(void)
{
    float dt_s = DWT_GetDeltaT(&follow_control_dwt_cnt);

    // What: 为跟随输出限斜率获取本拍真实周期；Why: RTOS调度和中断负载会带来轻微抖动，按真实dt换算比写死5ms更稳。
    if (dt_s <= 0.0f || dt_s > 0.05f) {
        dt_s = CHASSIS_TASK_DT_FALLBACK; // What: DWT异常时回退到任务标称周期；Why: 防止计时首拍或异常值把斜率限幅直接放大到不可控。
    }
    return dt_s;
}

static void ResetFollowControlState(void)
{
    // What: 退出跟随相关工况时统一清空接管、滤波和限斜率状态；Why: 下次再进跟随必须从当前姿态重新接管，不能沿用上一次的历史输出记忆。
    follow_transition_ticks = 0u;
    follow_angle_err_filtered = 0.0f;
    follow_wz_cmd_limited = 0.0f;
    last_follow_transition_request = 0u;
    DWT_GetDeltaT(&follow_control_dwt_cnt); // What: 顺手重置跟随控制时间基准；Why: 避免长时间不在跟随模式时下一次进入拿到异常大的dt。
}

static void StartFollowTransition(float current_angle_err)
{
    // What: 在小陀螺退跟随边沿启动底盘接管窗口；Why: 先把滤波状态贴到当前偏角，再给固定刹停窗口，能避免退出首拍就被旧自旋余量拉着来回抽。
    follow_transition_ticks = FOLLOW_TRANSITION_HOLD_TICKS;
    follow_angle_err_filtered = current_angle_err;
    follow_wz_cmd_limited = 0.0f;
    DWT_GetDeltaT(&follow_control_dwt_cnt); // What: 接管起点重置时间基准；Why: 让随后的斜率限制从稳定起点开始计算，而不是沿用模式外的旧周期。
}

static float ApplyFollowCommandSlew(float target_wz, float dt_s)
{
    float max_delta = FOLLOW_OUTPUT_SLEW_DPS_PER_S * dt_s;
    float delta = target_wz - follow_wz_cmd_limited;

    // What: 给跟随输出统一加变化率限制；Why: 控制律在“纯阻尼刹停”和“正常回正”之间切段时，若直接阶跃切换很容易再次激发底盘来回摆动。
    LIMIT_MIN_MAX(delta, -max_delta, max_delta);
    follow_wz_cmd_limited += delta;
    return follow_wz_cmd_limited;
}

void ChassisInit()
{
    Chassis_IMU_data = INS_Init();
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 3.7, // 4.5 3.7
                .Ki = 0.0, // 0.2
                .Kd = 0.0, // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.1,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 设置为开环，电机设定值由下面的功率控制设定，不走普通的pid
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    // 使用功率控制的电机需要使用PowerControlInit()函数初始化,因为电机的控制方式不同
    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL; //
    motor_lf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = PowerControlInit(&chassis_motor_config);

#ifdef USE_ISLAND_ACTION
    IslandActionInit(); // [条件编译] 仅在启用上岛机构时初始化前履带和后抬升电机
#endif // USE_ISLAND_ACTION

    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI
    PowerControl_EnableSlopeComp(1);

#ifdef USE_SUPER_CAP
    // [条件编译] 超级电容初始化配置
    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x061, // 超级电容默认接收id
            .rx_id = 0x051, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }
    };
    cap = SuperCapInit(&cap_conf); // 初始化超级电容模块./.....
#endif // USE_SUPER_CAP

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x011,
            .rx_id = 0x012,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
        .daemon_count = 200,
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化

    LOGINFO("[can_comm] Chassis_Upload_Data_s size: %d", sizeof(Chassis_Upload_Data_s));
    LOGINFO("[can_comm] Chassis_Ctrl_Cmd_s size: %d", sizeof(Chassis_Ctrl_Cmd_s));
    LOGINFO("[can_comm] CAN_COMM_MAX_BUFFSIZE: %d", CAN_COMM_MAX_BUFFSIZE);
    // 【新增调试日志】
    if (chasiss_can_comm != NULL) {
        LOGINFO("[CHASSIS_DEBUG] CAN Comm Init Success! TxID: %d, RxID: %d", chasiss_can_comm->can_ins->tx_id, chasiss_can_comm->can_ins->rx_id);
    } else {
        LOGERROR("[CHASSIS_DEBUG] CAN Comm Init Failed!");
    }
#endif // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    // 标准 X 型麦轮解算公式 (所有轮子 Forward 均为 +vx)
    // 假设右手系：x前, y左, z逆时针

    // LF: 前进(+vx), 左移需后转(+vy), 左转需后转(+wz)
    vt_lf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * LF_CENTER;

    // RF: 前进(+vx), 左移需前转(-vy), 左转需前转(-wz)
    vt_rf = chassis_vx - chassis_vy - chassis_cmd_recv.wz * RF_CENTER;

    // LB: 前进(+vx), 左移需前转(+vy), 左转需后转(-wz) (注意：左后为了向左平移，实际上是要向前转的，但在标准受力分析中，这里的符号取决于你的y定义。通常为 + -)
    // 修正：X型布局向左平移：左前向后(+)，左后向前(+)。
    // 等等，如果归一化了，我们再推导一次：
    // 向左平移(vy>0):
    //   LF(A轮): 需向后转 -> +vy
    //   RF(B轮): 需向前转 -> -vy
    //   LB(B轮): 需向前转 -> +vy (注意：X型左后轮向前转产生向左的分力)
    //   RB(A轮): 需向后转 -> -vy

    vt_lb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx - chassis_vy + chassis_cmd_recv.wz * RB_CENTER;
}

// 针对全麦和全向轮构型，方案一，前轮麦轮，后轮全向轮结算。方案二，利用陀螺仪强力纠正侧向漂移
static void HybridCalculate()
{
    // 前轮 (麦轮)：系数 = 轮距 + 轴距
    vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;

    // 计算后轮所需的旋转线速度分量
    float rear_rot_spd = chassis_cmd_recv.wz * HALF_TRACK_WIDTH * DEGREE_2_RAD;
    // 只有纵向速度 vy 参与，vy 对全向轮无效
    vt_lb = chassis_vy - rear_rot_spd;
    vt_rb = chassis_vy + rear_rot_spd; // 左右轮旋转项符号相反，形成力偶
}
/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
#ifdef USE_SUPER_CAP
    // [条件编译] 超级电容功率控制逻辑
    if (cap) {
        uint16_t referee_buffer_j = (uint16_t)DEFAULT_TEST_BUFFER_ENERGY_J;
        float referee_limit = DEFAULT_TEST_POWER;
        uint8_t chassis_output_allowed = 1u;

        if (referee_data != NULL) {
            referee_buffer_j = referee_data->PowerHeatData.buffer_energy;
            referee_limit = referee_data->GameRobotState.chassis_power_limit;
            if (referee_limit < 1.0f) {
                referee_limit = DEFAULT_TEST_POWER;
            }
            chassis_output_allowed = (uint8_t)(referee_data->GameRobotState.power_management_chassis_output != 0u);
        }

        // 1. 发送能量缓冲 (告诉超电当前裁判系统里还有多少缓冲能量)
        // What: 通过现有 helper 下发裁判缓冲能量；Why: 统一复用范围限幅逻辑，避免后续超电协议调整后底盘侧还在直接写裸字段。
        SuperCapSetEnergyBuffer(cap, referee_buffer_j);

        // 2. 发送功率限制
        // What: 始终把当前合法裁判功率限制同步给超电板；Why: 更激进的是底盘侧 bonus 策略，不是让超电板盲目突破裁判功率红线。
        SuperCapSetPowerLimit(cap, (uint16_t)referee_limit);

        // What: 只要裁判系统允许底盘输出且超电在线，就持续发送 DCDC 使能请求；Why: 让 C 板侧忽略 bit7=128，避免“输出禁用”状态被上层再次锁死。
        if (chassis_output_allowed != 0u && cap->is_online) {
            cap->tx_msg.enableDCDC = 1;
        } else {
            cap->tx_msg.enableDCDC = 0;
        }

        static uint32_t error_toggle_tick = 0;
        // What: 只对 bit0-bit6 的真实错误执行 2 秒关 / 2 秒开恢复；Why: bit7=128 只是输出禁用状态，不应再参与 C 板关断逻辑。
        if (SuperCapGetErrorCode(cap) != 0) {
            uint32_t now = HAL_GetTick();
            if (error_toggle_tick == 0)
                error_toggle_tick = now;
            uint32_t elapsed = (now - error_toggle_tick) % 4000; // 4秒周期
            if (elapsed < 2000) {
                cap->tx_msg.enableDCDC = 0; // 前2秒关
            } else {
                cap->tx_msg.enableDCDC = 1; // 后2秒开
            }
        } else {
            error_toggle_tick = 0; // 错误消除，重置
        }

        /* 发送 CAN 消息 */
        SuperCapSend(cap);
    }
#endif // USE_SUPER_CAP

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}
/**
 * @brief 根据电机反馈和底盘IMU数据，融合计算底盘实际运动速度
 *
 * 融合策略：
 * 1. 旋转速度(wz)：直接使用IMU陀螺仪数据，比电机逆解算更精确
 * 2. 线速度(vx/vy)：电机编码器逆解算为主，IMU加速度积分为辅，互补滤波融合
 *
 * @note 此函数应以固定频率(如1kHz)被调用，以保证积分精度
 */
static void EstimateSpeed()
{
    // ===== 1. 获取时间间隔 (秒) =====
    // 假设 ChassisTask 以固定 1ms 周期运行
    const float dt = 0.001f; // 1ms，若实际周期不同可使用 DWT 获取

    // ===== 2. 从 IMU 直接获取旋转角速度 (更精确) =====
    // Chassis_IMU_data->Gyro[2] 是 Z 轴角速度 (rad/s)
    // 转换为 deg/s 以与其他变量统一
    float imu_wz_dps = Chassis_IMU_data->Gyro[Z] * RAD_2_DEGREE; // Z 轴角速度 (deg/s)

    // ===== 3. 电机编码器逆运动学解算 =====
    // 获取电机转速 (度/秒)，并根据电机安装方向进行符号修正
    float v_lf = motor_lf->measure.speed_aps; // LF: NORMAL，保持原样
    float v_rf = -motor_rf->measure.speed_aps; // RF: REVERSE，取反
    float v_lb = motor_lb->measure.speed_aps; // LB: NORMAL，保持原样
    float v_rb = -motor_rb->measure.speed_aps; // RB: REVERSE，取反

    // 麦轮逆运动学公式 (由正解反推):
    // vx = (v_lf + v_rf + v_lb + v_rb) / 4
    // vy = (-v_lf + v_rf + v_lb - v_rb) / 4  (X型麦轮)
    // wz = (-v_lf + v_rf - v_lb + v_rb) / 4 / (R_wheel / geometry_sum)
    float avg_vx_raw = (v_lf + v_rf + v_lb + v_rb) / 4.0f; // 电机坐标系下的 vx (deg/s)
    float avg_vy_raw = (-v_lf + v_rf + v_lb - v_rb) / 4.0f; // 电机坐标系下的 vy (deg/s)
    float avg_wz_raw = (-v_lf + v_rf - v_lb + v_rb) / 4.0f; // 电机逆解算的 wz (deg/s)

    // 几何参数：用于从电机转速转换到底盘速度
    float geometry_sum = HALF_TRACK_WIDTH + HALF_WHEEL_BASE; // 半轮距 + 半轴距 (m)
    float wheel_radius_ratio = RADIUS_WHEEL * DEGREE_2_RAD; // 轮子半径 (m) * (rad/deg)

    // 电机编码器解算的线速度 (m/s)
    float encoder_vx = avg_vx_raw * wheel_radius_ratio; // 前后方向速度 (m/s)
    float encoder_vy = avg_vy_raw * wheel_radius_ratio; // 左右方向速度 (m/s)

    // 电机编码器解算的旋转速度 (deg/s)
    float encoder_wz = avg_wz_raw * (RADIUS_WHEEL / geometry_sum);

    // ===== 4. IMU 加速度积分获取线速度 (辅助) =====
    // Chassis_IMU_data->Accel[X/Y] 是机体系加速度 (m/s²)
    // 需要去除重力分量影响，这里假设底盘基本水平
    static float imu_vx = 0.0f; // IMU 积分得到的 vx
    static float imu_vy = 0.0f; // IMU 积分得到的 vy

    // 获取机体系加速度，并进行积分
    // 注意：需要根据 IMU 安装方向调整坐标轴对应关系
    float accel_x = Chassis_IMU_data->Accel[X]; // 前后方向加速度 (m/s²)
    float accel_y = Chassis_IMU_data->Accel[Y]; // 左右方向加速度 (m/s²)

    // 对加速度进行积分得到速度增量
    imu_vx += accel_x * dt;
    imu_vy += accel_y * dt;

    // ===== 5. 互补滤波融合 =====
    // 使用互补滤波融合编码器和 IMU 数据
    // 编码器：低频准确（无漂移），高频噪声大
    // IMU 积分：高频响应好，但有累积漂移
    // 策略：以编码器为主，IMU 积分作为高频补偿，并持续向编码器值收敛
    const float alpha_linear = 0.05f; // 线速度融合系数 (编码器权重较高)
    const float alpha_wz = 0.8f; // 角速度融合系数 (IMU 权重较高，因为陀螺仪精度高)

    // 让 IMU 积分值向编码器值收敛，防止累积漂移
    const float drift_correction = 0.02f; // 漂移修正系数
    imu_vx = imu_vx * (1.0f - drift_correction) + encoder_vx * drift_correction;
    imu_vy = imu_vy * (1.0f - drift_correction) + encoder_vy * drift_correction;

    // 融合线速度：编码器为主 + IMU 高频补偿
    static float last_vx = 0.0f, last_vy = 0.0f;
    float fused_vx = encoder_vx * (1.0f - alpha_linear) + imu_vx * alpha_linear;
    float fused_vy = encoder_vy * (1.0f - alpha_linear) + imu_vy * alpha_linear;

    // 融合角速度：IMU 陀螺仪为主，编码器为辅 (陀螺仪精度更高)
    static float last_wz = 0.0f;
    float fused_wz = imu_wz_dps * alpha_wz + encoder_wz * (1.0f - alpha_wz);

    // ===== 6. 低通滤波 (平滑输出) =====
    const float lpf_alpha = 0.3f; // 滤波系数，越小越平滑但滞后
    real_vx = (1.0f - lpf_alpha) * last_vx + lpf_alpha * fused_vx;
    real_vy = (1.0f - lpf_alpha) * last_vy + lpf_alpha * fused_vy;
    real_wz = (1.0f - lpf_alpha) * last_wz + lpf_alpha * fused_wz;

    // 保存上一次的值
    last_vx = real_vx;
    last_vy = real_vy;
    last_wz = real_wz;
}

static void RefereeUIUpdateData(void)
{
    // What: 汇总底盘板本地和双板下发的 UI 实时数据；Why: 把数据采集与 UI 绘制解耦后，裁判任务只关心显示调度，避免读多处模块造成状态不一致。
    ui_data.chassis_yaw_rate_dps = Chassis_IMU_data->Gyro[Z] * RAD_2_DEGREE;

#ifdef CHASSIS_BOARD
    if (chasiss_can_comm != NULL && CANCommIsOnline(chasiss_can_comm) != 0u) {
        // What: 双板在线时直接采用云台板下发的真实姿态、相对偏角与摩擦轮状态；Why: 这些量在双板协同控制链里已经对齐，底盘板本地无需再自行推导。
        ui_data.gimbal_pitch_deg = chassis_cmd_recv.gimbal_pitch_deg;
        ui_data.gimbal_yaw_rate_dps = chassis_cmd_recv.gimbal_gyro_z;
        ui_data.chassis_gimbal_offset_deg = theta_format(chassis_cmd_recv.offset_angle);
        ui_data.friction_on = chassis_cmd_recv.friction_on;
    } else {
        // What: 双板离线时冻结 pitch 和相对偏角并清零其余云台相关量；Why: 相对姿态保留最后一次有效值更利于排查问题，而角速度和摩擦轮状态必须立即回落避免误导操作手。
        ui_data.gimbal_yaw_rate_dps = 0.0f;
        ui_data.friction_on = 0u;
    }
#else
    // What: 单板构型下先给云台相关 UI 量安全默认值；Why: 当前相对姿态方案主要面向双板，未补全单板数据通路前不能让显示读到未定义数据。
    ui_data.gimbal_yaw_rate_dps = 0.0f;
    ui_data.chassis_gimbal_offset_deg = 0.0f;
    ui_data.friction_on = 0u;
#endif

#ifdef USE_SUPER_CAP
    if (SuperCapIsOnline(cap) != 0u) {
        // What: 超电在线时优先显示其回传的真实底盘功率与输出状态；Why: 该值最接近实际能量链路表现，能直接反映 buffer 与 DCDC 是否正在工作。
        ui_data.chassis_power_w = SuperCapGetChassisPower(cap);
        ui_data.cap_on = (SuperCapIsOutputDisabled(cap) == 0u) ? 1u : 0u;
        return;
    }
#endif

    // What: 超电离线时回退到底盘功率控制模块的本地估算值；Why: 即使辅助供电链路失效，选手端仍需要持续看到一个稳定更新的功率读数。
    ui_data.chassis_power_w = PowerControlGetChassisPower();
    ui_data.cap_on = 0u;
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    float gimbal_wz = 0.0f;
    uint8_t follow_transition_request_rise = 0u;
    uint8_t rotate_to_follow_edge = 0u;
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD
    gimbal_wz = chassis_cmd_recv.gimbal_gyro_z; // What: 使用最新一帧云台角速度前馈；Why: 避免先读取旧值再更新命令导致跟随支路固定滞后一个控制周期。
    follow_transition_request_rise = (uint8_t)(chassis_cmd_recv.follow_transition_request != 0u && last_follow_transition_request == 0u); // What: 检测接管请求上升沿；Why: 上板会保持数拍请求位，底盘侧只应在真正的边沿触发一次接管窗口。
    rotate_to_follow_edge = (uint8_t)(last_chassis_mode == CHASSIS_ROTATE &&
                                      chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW); // What: 本地补一份小陀螺退跟随边沿检测；Why: 即使上板请求偶发漏拍，底盘也能靠本地模式边沿兜底进入接管。
    last_follow_transition_request = chassis_cmd_recv.follow_transition_request;

    if (chassis_cmd_recv.ui_refresh_request != 0u) {
        // What: 收到上板的一次性 UI 刷新请求后转交给裁判 UI 任务；Why: 真正的绘图发包必须在 UI 线程内串行执行，底盘控制线程不应直接插手。
        UIRequestRefresh();
    }

    /* 功率控制策略 */
    // 1. 获取裁判系统功率限制 (原始最大值)
    float referee_power_limit;

    // 【修复】先判断指针是否为空
    // 如果没有初始化裁判系统（referee_data == NULL），直接使用测试功率
    if (referee_data == NULL) {
        referee_power_limit = DEFAULT_TEST_POWER; // 强制使用测试功率
    } else {
        // 只有指针有效时才去读取
        referee_power_limit = referee_data->GameRobotState.chassis_power_limit;
        // 即使有指针，如果读出来是0（裁判系统刚启动），也用测试功率兜底
        if (referee_power_limit < 1.0f) {
            referee_power_limit = DEFAULT_TEST_POWER;
        }
    }

    float referee_buffer_energy = DEFAULT_TEST_BUFFER_ENERGY_J;
    uint8_t chassis_output_allowed = 1u;
    if (referee_data != NULL) {
        // What: 同步取一份裁判缓冲能量与输出许可；Why: 超电 bonus 需要和同一拍裁判状态对齐，不能只靠上一次的超电板回包做决策。
        referee_buffer_energy = (float)referee_data->PowerHeatData.buffer_energy;
        chassis_output_allowed = (uint8_t)(referee_data->GameRobotState.power_management_chassis_output != 0u);
    }

    // 2. [新增] 平地/坡道功率缩放策略
    // 平地：当前同样使用裁判系统允许的 100%，优先把更激进的功率策略完整打出来
    // 坡道：同样保持 100%，避免冲坡时再被额外的软件缩放拖慢
    float final_power_limit;
    if (fabsf(Chassis_IMU_data->Pitch) > CHASSIS_SLOPE_THRESHOLD) {
        // 坡道模式：使用 100% 功率
        final_power_limit = referee_power_limit * 1.00f;
    } else {
        // 平地模式：同样使用 100% 功率
        final_power_limit = referee_power_limit * 1.0f;
    }

    // 3. [条件编译] 超电爆发功率策略
    // 判断是否可以爆发 (上层允许超电或坡道触发，且超电在线、无故障、输出未禁用)
#ifdef USE_SUPER_CAP
    {
        float super_cap_bonus = GetAggressiveSuperCapBonus(referee_buffer_energy, chassis_output_allowed);
        if (super_cap_bonus > 0.0f) {
            // What: 按电容电量和裁判缓冲动态叠加更激进的超电 bonus；Why: 固定 bonus 只能在一种工况下合适，分档后才能既更猛又不至于一下子放空。
            final_power_limit += super_cap_bonus;
        }
    }
#endif // USE_SUPER_CAP

    // 4. 最终限幅保护
    if (final_power_limit > CHASSIS_TOTAL_POWER_LIMIT_MAX_W)
        final_power_limit = CHASSIS_TOTAL_POWER_LIMIT_MAX_W; // What: 将底盘侧总功率硬上限放宽到 165W；Why: 让更大的超电加成真正落到底盘电机功率控制，而不是被旧上限提前卡死。
    // 5. 设置给底盘功率控制算法 (这个函数控制电机的电流)
    SetPowerLimit(final_power_limit);

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
#ifdef USE_ISLAND_ACTION
        IslandActionStop(); // [条件编译] 仅在启用上岛机构时停止辅助机构
#endif // USE_ISLAND_ACTION
    } else { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode) {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        ResetAdaptiveSpinBase(); // What: 退出小陀螺时清空贴边状态；Why: 下次重新进入时从统一初始条件起步，避免继承旧工况的高转速记忆。
        ResetFollowControlState(); // What: 离开底盘跟随时复位接管状态；Why: 跟随专用的滤波和限斜率记忆不应泄漏到自由平移模式。
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: {
        ResetAdaptiveSpinBase(); // What: 跟随模式下复位小陀螺状态；Why: 跟随控制依赖独立角度环，不应继续带着自旋功率闭环状态运行。
        const float follow_yaw_kp = 21.0f; // What: 跟随模式位置环比例增益；Why: 直接按角度误差生成回正速度，比二次项更线性且更容易调到“快但不炸”
        const float follow_yaw_kd = 0.5f; // What: 跟随模式底盘角速度阻尼增益；Why: 使用底盘真实角速度做D项，专门抑制回中穿越和反向摆动
        const float follow_yaw_kff = 1.3f; // What: 跟随模式云台角速度前馈增益；Why: 云台先动时提前拉动底盘，减少纯靠角度误差追赶带来的滞后
        const float follow_yaw_deadband = 0.5f; // What: 跟随模式角度死区；Why: 回中附近直接清零小误差，避免机械间隙和噪声触发来回抖动
        const float follow_yaw_max_wz = 7500.0f; // What: 跟随模式角速度输出上限；Why: 防止大角度时给电机速度环过猛目标，降低饱和后再过冲的概率
        float chassis_wz = Chassis_IMU_data->Gyro[Z] * RAD_2_DEGREE; // What: 读取底盘当前真实角速度；Why: D项必须基于被控对象自身速度才能形成真实阻尼
        float angle_err = 0.0f;
        float raw_angle_err = chassis_cmd_recv.offset_angle; // What: 缓存当前底盘相对云台的原始角度误差；Why: 接管启动时需要把滤波状态贴齐当前值，避免退出窗口首拍再跳一次。
        float relative_gimbal_wz = gimbal_wz - chassis_wz; // What: 计算云台相对底盘的角速度；Why: gimbal_gyro_z含底盘旋转分量，直接前馈会形成正反馈振荡
        float raw_follow_wz = 0.0f;
        float follow_dt_s = GetFollowControlDt();

        if (follow_transition_request_rise != 0u || rotate_to_follow_edge != 0u) {
            StartFollowTransition(raw_angle_err); // What: 在小陀螺退跟随边沿启动接管窗口；Why: 先刹停再回正，避免残余自旋和偏角环在同一拍里互相打架。
        } else if (last_chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW) {
            // What: 从其它模式首次进入普通跟随时把滤波状态贴齐当前偏角；Why: 避免滤波器从0起步把第一拍回正量平白压小，导致跟随接管变慢。
            follow_angle_err_filtered = raw_angle_err;
        }
        follow_angle_err_filtered += (raw_angle_err - follow_angle_err_filtered) * FOLLOW_ANGLE_FILTER_ALPHA; // What: 对偏角做一阶滤波；Why: 先滤掉机械回弹和量测毛刺，退出接管后P项不容易马上反向抽动。
        angle_err = follow_angle_err_filtered;
        if (fabsf(angle_err) < follow_yaw_deadband) {
            angle_err = 0.0f; // What: 清零死区内误差；Why: 小角度时让前馈和阻尼接管，避免位置项在零点附近反复翻转
        }

        if (follow_transition_ticks != 0u) {
            raw_follow_wz = -FOLLOW_TRANSITION_BRAKE_KD * chassis_wz; // What: 接管窗口内只按车体角速度做阻尼刹停；Why: 先卸掉小陀螺余旋，比同时引入偏角环和前馈更不容易振荡。
            LIMIT_MIN_MAX(raw_follow_wz, -FOLLOW_TRANSITION_MAX_WZ, FOLLOW_TRANSITION_MAX_WZ); // What: 限制接管阶段制动输出；Why: 刹停只需中等强度，过猛反而容易把轮速环再度推饱和。
            if (fabsf(chassis_wz) < FOLLOW_TRANSITION_EXIT_WZ_DPS) {
                follow_transition_ticks = 0u; // What: 当余旋已经足够小时提前结束接管；Why: 这样可以更早恢复正常回正，减少“刹得过久”的拖滞手感。
            } else {
                follow_transition_ticks--;
            }
        } else {
            raw_follow_wz = -follow_yaw_kp * angle_err - follow_yaw_kd * chassis_wz - follow_yaw_kff * relative_gimbal_wz; // What: 接管结束后恢复正常跟随控制律；Why: 仍保留P回正、D阻尼和相对角速度前馈来兼顾速度与稳定性。
            LIMIT_MIN_MAX(raw_follow_wz, -follow_yaw_max_wz, follow_yaw_max_wz); // What: 限制正常跟随模式角速度输出；Why: 避免外环瞬时给出过大目标把电机内环推入饱和。
        }
        chassis_cmd_recv.wz = ApplyFollowCommandSlew(raw_follow_wz, follow_dt_s); // What: 对最终跟随输出做限斜率；Why: 从接管刹停切回正常回正时保持连续，避免指令跳变再次激起摆振。
        break;
    }
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动
    {
        ResetFollowControlState(); // What: 进入小陀螺时复位跟随接管状态；Why: 小陀螺的控制目标完全不同，不应继续保留跟随环的滤波和输出记忆。
        // What: 按实时功率余量闭环抬高小陀螺基础角速度；Why: 让后级功率限制器长期工作在贴边状态，从而把合法功率尽量吃满。
        float base_wz = GetAdaptiveSpinBase(final_power_limit);
        // What: 在功率贴边基础上继续应用平移优先；Why: 小陀螺再猛也不能把驾驶员横移和前后机动直接抢没。
        chassis_cmd_recv.wz = OptimizedSpinSpeed(base_wz, chassis_cmd_recv.vx, chassis_cmd_recv.vy);
    } break;
    default:
        ResetAdaptiveSpinBase(); // What: 其它模式统一复位小陀螺状态；Why: 避免未覆盖模式残留旧的小陀螺闭环输出。
        ResetFollowControlState(); // What: 其它模式统一复位跟随接管状态；Why: 任何非跟随工况都不该继续保存跟随专用的边沿和输出历史。
        break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    // 修正旋转矩阵：底盘为右手系（X向前方，Y向右方），且 offset_angle 以逆时针方向为正。
    // 因此在分解目标云台向上的平移速度(vx, vy)时，sin 项的符号需要与标准矩阵相反，避免出现推杆前进而横移的现象。
    chassis_vx = chassis_cmd_recv.vx * cos_theta + chassis_cmd_recv.vy * sin_theta;
    chassis_vy = -chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    if (chassis_cmd_recv.chassis_mode == CHASSIS_NO_FOLLOW) {
        // ChassisHeadLock();
    }

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();
    // HybridCalculate();
    PowerControl_UpdateIMU(Chassis_IMU_data->Pitch * DEGREE_2_RAD,
                           Chassis_IMU_data->Roll * DEGREE_2_RAD);

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // What: 在底盘输出与功率策略完成后刷新一份 UI 实时数据快照；Why: 这样 UI 读到的功率、超电和角速度都对应本拍最新控制结果。
    RefereeUIUpdateData();

#ifdef USE_ISLAND_ACTION
    if (chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE) {
        // What: 在底盘主运动解算后独立控制履带与抬升，同时传入IMU pitch角度；Why: 上岛辅助机构不参与麦轮功率分配，自动调平需要实时pitch反馈但不应直接访问底盘IMU全局变量。
        IslandActionControl(&chassis_cmd_recv, Chassis_IMU_data->Pitch);
    }
#endif // USE_ISLAND_ACTION

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    // EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
    last_chassis_mode = chassis_cmd_recv.chassis_mode; // What: 在任务末尾刷新上一拍模式缓存；Why: 下一拍需要用它识别本地的小陀螺退跟随边沿并兜底触发接管。
}
