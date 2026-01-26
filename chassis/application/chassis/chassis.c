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
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "chassis_follow.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f) // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f) // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define DEFAULT_TEST_POWER 85.0f // 调试用的基础功率

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
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

static SuperCapInstance *cap = { NULL }; // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

static ChassisFollowInstance *chassis_follow_ptr = NULL;

// 方案二，陀螺仪针对全麦纠偏PID
static PIDInstance yaw_lock_pid; // 航向锁定专用PID
static float lock_target_yaw = 0.0f; // 锁定的目标角度
static uint8_t is_manual_rotating = 0; // 标记是否正在手动旋转

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy; // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 4.5, // 4.5
                .Ki = 0, // 0
                .Kd = 0, // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.3,
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
    chassis_motor_config.can_init_config.can_handle = &hcan1;
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle = &hcan1;
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = PowerControlInit(&chassis_motor_config);

    // referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x061, // 超级电容默认接收id
            .rx_id = 0x051, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }
    };

    // cap = SuperCapInit(&cap_conf); // 超级电容初始化

    PID_Init_Config_s yaw_lock_conf = {
        .Kp = -29.0f, // 强力纠正
        .Ki = 12.0f, // 消除静差
        .Kd = 5.0f, // 抑制震荡
        .IntegralLimit = 500.0f, // 积分限幅
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 7000.0f, // 输出限幅 (对应 chassis_cmd_recv.wz 的量级)
        .Output_LPF_RC = 0.0f,
        .DeadBand = 0.5f,
    };
    PIDInit(&yaw_lock_pid, &yaw_lock_conf);
    // 底盘跟随云台
    ChassisFollow_Config_s follow_config = {
        .deadzone_angle = 3.0f, // 1.5度死区

        // 位置环参数 (外环)
        .angle_pid = {
            .kp = 4.2f, // 需调试: 响应速度
            .ki = 0.0f,
            .kd = 0.55f,
            .IntegralLimit = 100.0f,
            .max_out = 300.0f, // 最大跟随速度 (度/秒)
        },

        // 速度环参数 (内环)
        .speed_pid = {
            .kp = 4.2f, // 需调试: 刚性
            .ki = 0.4f,
            .kd = 0.02f,
            .IntegralLimit = 700.0f,
            .max_out = 7000.0f // 电机最大输出
        }
    };
    chassis_follow_ptr = ChassisFollowInit(&follow_config);
    if (chassis_follow_ptr == NULL) {
        // 错误处理，例如亮红灯或记录日志
        LOGERROR("Chassis Follow Init Failed!");
    }
    Chassis_IMU_data = INS_Init();

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
    if (chasiss_can_comm != NULL) {
        LOGINFO("[DEBUG] Chassis CAN Comm Init SUCCESS! Handle: %p, recv:%d, send:%d",
                chasiss_can_comm, sizeof(Chassis_Ctrl_Cmd_s), sizeof(Chassis_Upload_Data_s));
    } else {
        LOGERROR("[DEBUG] Chassis CAN Comm Init FAILED! Returned NULL.");
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
    vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * RF_CENTER;
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
    // 超级电容功率控制
    if (cap) {
        // 1. 发送能量缓冲 (告诉超电当前裁判系统里还有多少缓冲能量)
        // 保持原样，发送实时buffer是正确的，超电板会根据这个决定是否全力充电
        cap->tx_msg.refereeEnergyBuffer = referee_data->PowerHeatData.buffer_energy;

        // 2. 发送功率限制
        float referee_limit = referee_data->GameRobotState.chassis_power_limit;

        float safe_limit = referee_limit;
        if (safe_limit < 30.0f)
            safe_limit = 30.0f; // 兜底防止过低

        // 无论是否开启爆发模式，给超电的永远是"合法的电池功率上限"
        cap->tx_msg.refereePowerLimit = (uint16_t)safe_limit;

        // 3. DCDC 开关逻辑 (保持你原有的逻辑，稍作优化)
        // if (chassis_cmd_recv.cap_mode == SUPER_CAP_ON) {
        // 裁判系统允许底盘输出 && 超电在线 && 无关键错误
        if (referee_data->GameRobotState.power_management_chassis_output != 0 &&
            cap->is_online &&
            !SuperCapIsOutputDisabled(cap)) // 使用 super_cap.c 里的辅助函数判断错误
        {
            // 电量充足时开启 DCDC
            if (cap->rx_msg.capEnergyPercent > 30) {
                cap->tx_msg.enableDCDC = 1;
            } else {
                // 低电量保护，可以不关DCDC但超电板内部要有限制，
                // 这里为了保险可以选择关闭，或者相信超电板的低压保护
                cap->tx_msg.enableDCDC = 0;
            }
        } else {
            cap->tx_msg.enableDCDC = 0;
        }
        static uint32_t error_toggle_tick = 0;
        if (cap->rx_msg.errorCode != 0) {
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

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}
/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 * 对于双板的情况,考虑增加来自底盘板IMU的数据
 */
static void EstimateSpeed()
{
}

static void ChassisHeadLock()
{
    if (Chassis_IMU_data == NULL)
        return;

    // --- 1. 初始化逻辑 (防止上电瞬间乱转) ---
    static uint8_t is_initialized = 0;
    if (!is_initialized) {
        lock_target_yaw = Chassis_IMU_data->Yaw; // 上电第一刻，锁定当前角度
        is_initialized = 1;
    }

    // --- 2. 获取输入并积分 (核心步骤) ---
    // 假设摇杆 wz 范围是 -660 ~ +660
    float input_wz = chassis_cmd_recv.gimbal_cmd_wz;

    // 死区处理：防止摇杆回中时的微小漂移导致目标角度缓慢移动
    if (fabsf(input_wz) < 100.0f) {
        input_wz = 0.0f;
    }

    // 灵敏度系数：决定了你推满摇杆时，底盘旋转得有多快
    // 计算公式：最大转速(度/秒) = 660 * 系数 * 控制频率(Hz)
    // 例如：0.002 * 660 * 500Hz(假设) = 660度/秒 (约2圈/秒)
    // 建议从 0.001f 开始调，觉得慢了就加大
    const float SENSITIVITY = 0.00015f;

    // 积分：输入改变的是“目标”，而不是直接改变“速度”
    lock_target_yaw += input_wz * SENSITIVITY;

    // --- 3. 目标角度过零处理 (归一化到 -180 ~ 180) ---
    // 这一步至关重要！否则转几圈后 target 变成 720度，PID 就失效了
    if (lock_target_yaw > 180.0f) {
        lock_target_yaw -= 360.0f;
    } else if (lock_target_yaw < -180.0f) {
        lock_target_yaw += 360.0f;
    }

    // --- 4. 计算最短路径误差 ---
    float current_yaw = Chassis_IMU_data->Yaw;
    float err_angle = lock_target_yaw - current_yaw;

    // 处理跨越 ±180 度的情况 (例如 目标179，当前-179，实际只差2度)
    if (err_angle > 180.0f) {
        err_angle -= 360.0f;
    } else if (err_angle < -180.0f) {
        err_angle += 360.0f;
    }

    // --- 5. PID 计算与输出 ---
    // 此时 PID 全时在线，负责把底盘拉向 target
    // 你的 PID 配置中启用了 PID_Derivative_On_Measurement，这非常棒！
    // 它可以防止当你快速推摇杆改变 target 时，D项产生冲击。
    float pid_out = PIDCalculate(&yaw_lock_pid, 0.0f, err_angle);

    // 最终将 PID 计算出的力矩/速度赋值给 wz
    chassis_cmd_recv.wz = pid_out;
}
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    float gimbal_wz = 0.0f;
    gimbal_wz = chassis_cmd_recv.gimbal_gyro_z;

    float chassis_wz = 0.0f;
    if (Chassis_IMU_data != NULL) {
        chassis_wz = Chassis_IMU_data->Gyro[2] * RAD_2_DEGREE; // 底盘IMU的z轴角速度
    }

    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    /* 超级电容爆发功率策略 */
    /* 超级电容爆发功率策略 */
    // 1. 获取基础限制
    float final_power_limit = referee_data->GameRobotState.chassis_power_limit;
    if (final_power_limit < 1.0f) // 简单判断裁判系统是否在线/有效
    {
        final_power_limit = DEFAULT_TEST_POWER;
    }

    // 2. 判断是否可以爆发 (电容模式开启 + 电容在线 + 电量充足 + DCDC已使能)
    // 注意：一定要判断DCDC是否真的开了，不然电机要110W，电池只能给80W，电压会瞬间拉低导致重启
    if (cap && cap->is_online &&
        chassis_cmd_recv.cap_mode == SUPER_CAP_ON &&
        cap->rx_msg.capEnergyPercent > 30 &&
        cap->tx_msg.enableDCDC == 1) // 确保我们已经请求开启DCDC
    {
        // 允许爆发，电机功率上限 = 裁判限制 + 电容贡献(30W-40W)
        // 具体加多少取决于你的电容板最大输出能力
        final_power_limit += 35.0f;
    }

    // 3. 最终限幅保护
    if (final_power_limit > 150.0f)
        final_power_limit = 150.0f; // 物理极限
    // 4. 设置给底盘功率控制算法 (这个函数控制电机的电流)
    SetPowerLimit(final_power_limit);

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    } else { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode) {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态

        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        // chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        if (chassis_follow_ptr != NULL) {
            float err = chassis_cmd_recv.offset_angle;
            if (fabs(err) < 10) {
                err = 0;
            }

            chassis_cmd_recv.wz = ChassisFollowCalc(
                chassis_follow_ptr, // 实例指针
                -err, // 角度误差
                gimbal_wz, // 前馈速度
                chassis_wz // 反馈速度
            );
            break;
        case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
            chassis_cmd_recv.wz = 4000;
            break;
        default:
            ChassisFollowReset(chassis_follow_ptr);

            break;
        }
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    if (chassis_cmd_recv.chassis_mode == CHASSIS_NO_FOLLOW) {
        // ChassisHeadLock();
    }
    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();
    // HybridCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

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
}
