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
#include "robot_def.h"
#include "power_control.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f) // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f) // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define DEFAULT_TEST_POWER 55.0f // 调试用的基础功率

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

static PIDInstance buffer_PID; // 用于底盘的缓冲能量PID
static referee_info_t *referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap; // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy; // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan2,
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
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    // motor_lf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    // motor_rf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    // motor_lb = PowerControlInit(&chassis_motor_config);
    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    // motor_rb = PowerControlInit(&chassis_motor_config);

    // referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x061, // 超级电容默认接收id
            .rx_id = 0x051, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }
    };

    // cap = SuperCapInit(&cap_conf); // 超级电容初始化

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
    // 1. 获取输入 (已经是 m/s 了，因为遥控器那边乘过了)
    float vx = chassis_vx;
    float vy = chassis_vy;

    // 2. 旋转速度 (deg/s)
    // 注意：robot_cmd 里发过来的 wz 建议是物理值 (deg/s)，比如直接发 200.0f
    // 如果发过来的是比例 (1.0)，这里要乘 MAX_CHASSIS_WZ_SPEED
    float wz = chassis_cmd_recv.wz;

    // 3. 计算旋转产生的线速度 (m/s)
    // LF_CENTER 宏里已经包含了转换系数
    float v_rot_lf = wz * LF_CENTER;
    float v_rot_rf = wz * RF_CENTER;
    float v_rot_lb = wz * LB_CENTER;
    float v_rot_rb = wz * RB_CENTER;

    //
    // 假设电机安装方向逻辑是：前轮负为前，后轮正为前（根据您原代码推断）
    // 必须有加有减才能旋转！
    float v_lf_m_s = -vx - vy + v_rot_lf; // 左前: 旋转给正 (后退)
    float v_rf_m_s = -vx + vy - v_rot_rf; // 右前: 旋转给负 (前进) -> 形成逆时针转
    float v_lb_m_s = vx - vy - v_rot_lb; // 左后: 旋转给负 (后退)
    float v_rb_m_s = vx + vy + v_rot_rb; // 右后: 旋转给正 (前进)

    // 5. ✅ 单位转换 (关键！把 3.0 m/s 变成 ~2000 deg/s)
    vt_lf = v_lf_m_s * CHASSIS_M_TO_DEG;
    vt_rf = v_rf_m_s * CHASSIS_M_TO_DEG;
    vt_lb = v_lb_m_s * CHASSIS_M_TO_DEG;
    vt_rb = v_rb_m_s * CHASSIS_M_TO_DEG;
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

        // 2. 发送功率限制 (关键修改！！！)
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
    // 1. 获取电机转速 (deg/s) 并转换为轮子线速度 (m/s)
    // 公式: v = speed_aps * (PI/180) * R
    // 注意：dji_motor的speed_aps是度/秒
    float v_lf = motor_lf->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL;
    float v_rf = motor_rf->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL;
    float v_lb = motor_lb->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL;
    float v_rb = motor_rb->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL;

    // 2. 逆运动学解算 (Inverse Kinematics)
    // 根据 MecanumCalculate 中的正向公式反推:
    // vx = (v_rb - v_rf + v_lb - v_lf) / 4
    // vy = (v_rb - v_lb + v_rf - v_lf) / 4
    // wz = -(v_lf + v_rf + v_lb + v_rb) / (4 * (a+b))

    // 计算底盘实际的前进速度 (m/s)
    chassis_feedback_data.real_vx = (v_rb - v_rf + v_lb - v_lf) / 4.0f;

    // 计算底盘实际的平移速度 (m/s)
    chassis_feedback_data.real_vy = (v_rb - v_lb + v_rf - v_lf) / 4.0f;

    // 3. 计算角速度 (deg/s)
    // 优先使用 IMU 陀螺仪数据，因为轮子打滑会导致里程计计算的角速度很不准
#ifdef CHASSIS_BOARD
    if (Chassis_IMU_data != NULL) {
        // 使用板载IMU的Z轴角速度 (注意单位，假设Gyro数据为 rad/s，需转为 deg/s，如果本身是 deg/s 则直接用)
        // 通常 BMI088 驱动解算出的 Gyro 单位是 rad/s
        chassis_feedback_data.real_wz = Chassis_IMU_data->Gyro[2] * RAD_2_DEGREE;
    } else {
        // IMU 离线时的兜底方案：使用轮子解算
        // LF_CENTER 包含了 R * (PI/180)，所以这里除回去直接得到 deg/s
        chassis_feedback_data.real_wz = -(v_lf + v_rf + v_lb + v_rb) / (4.0f * LF_CENTER);
    }
#else
    // 单板模式或无IMU数据时，使用轮子解算
    // 这里的 LF_CENTER 必须与 MecanumCalculate 中使用的宏一致
    chassis_feedback_data.real_wz = -(v_lf + v_rf + v_lb + v_rb) / (4.0f * LF_CENTER);
#endif
}
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
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

    SetPowerLimit(referee_data->GameRobotState.chassis_power_limit); // 设置功率限制
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
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 4000;
        break;
    default:
        break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

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
