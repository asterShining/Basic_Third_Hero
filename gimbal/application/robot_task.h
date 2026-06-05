/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "dmmotor.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"
#include "custom_image_bridge.h"

#include "bsp_log.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;
osThreadId customImageBridgeTaskHandle;

// 这里把图像桥接任务周期单独抽成常量，作用是显式绑定 0x0310 的 50Hz 发送上限；
// 原因是协议限制决定了下位机最多每 20ms 只应推进一包 300B payload，若以后有人顺手把 osDelay 改快，
// 不仅不会提高有效带宽，还会把 USART6 busy_skip 和无效轮询噪声一起拉高。
#define CUSTOM_IMAGE_BRIDGE_TASK_PERIOD_MS 20u

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartCUSTOMIMAGEBRIDGETASK(void const *argument);
// void StartUITASK(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    // 这里把图像桥接任务放到控制主任务之前创建，作用是让 USB CDC -> USART6 TX 的自定义图像链路尽早开始稳定工作；
    // 原因是该任务优先级被刻意压低，不会抢控制链，但提前启动可以减少联调时的“上位机先发，云台板还没接”的空窗。
    osThreadDef(customimagebridgetask, StartCUSTOMIMAGEBRIDGETASK, osPriorityBelowNormal, 0, 256);
    customImageBridgeTaskHandle = osThreadCreate(osThread(customimagebridgetask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    // osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    // uiTaskHandle = osThreadCreate(osThread(uitask), NULL);
    DMMotorControlInit();
    // HTMotorControlInit(); // 没有注册HT电机则不会执行
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init(); // 确保BMI088被正确初始化.
    LOGINFO("[freeRTOS] INS Task Start");
    for (;;) {
        // 1kHz
        ins_start = DWT_GetTimeline_ms();
        INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;

        // 修改点：移除 &，将 float 转为 int (微秒)，使用 %d 打印
        if (ins_dt > 1)
            LOGERROR("[freeRTOS] INS Task DELAY! dt = %d us", (int)(ins_dt * 1000));

        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;) {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        // 控制频率改为500hz
        //  修改点：移除 &，将 float 转为 int (微秒)，使用 %d 打印
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task DELAY! dt = %d us", (int)(motor_dt * 1000));

        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;) {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        BuzzerTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;

        // 修改点：移除 &，将 float 转为 int (微秒)，使用 %d 打印
        if (daemon_dt > 10)
            LOGERROR("[freeRTOS] Daemon Task DELAY! dt = %d us", (int)(daemon_dt * 1000));

        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    static float robot_dt;
    static float robot_start;
    LOGINFO("[freeRTOS] ROBOT core Task Start");
    // 200Hz-500Hz
    for (;;) {
        robot_start = DWT_GetTimeline_ms();
        RobotTask();
        robot_dt = DWT_GetTimeline_ms() - robot_start;

        // 修改点：移除 &，将 float 转为 int (微秒)，使用 %d 打印
        if (robot_dt > 5)
            LOGERROR("[freeRTOS] ROBOT core Task DELAY! dt = %d us", (int)(robot_dt * 1000));

        osDelay(5);
    }
}

__attribute__((noreturn)) void StartCUSTOMIMAGEBRIDGETASK(void const *argument)
{
    static float custom_image_bridge_dt;
    static float custom_image_bridge_start;
    (void)argument;
    LOGINFO("[freeRTOS] CUSTOM IMAGE BRIDGE Task Start");
    for (;;) {
        custom_image_bridge_start = DWT_GetTimeline_ms();
        CustomImageBridgeTask();
        custom_image_bridge_dt = DWT_GetTimeline_ms() - custom_image_bridge_start;

        // 这里单独监控桥接任务执行耗时，作用是确保新增图像链路没有偷偷长时间占住 CPU；
        // 原因是用户要求“不影响原本控制逻辑”，因此新增任务必须被显式约束在很小的时间预算内。
        if (custom_image_bridge_dt > 1)
            LOGERROR("[freeRTOS] CUSTOM IMAGE BRIDGE Task DELAY! dt = %d us", (int)(custom_image_bridge_dt * 1000));

        osDelay(CUSTOM_IMAGE_BRIDGE_TASK_PERIOD_MS);
    }
}

__attribute__((noreturn)) void StartUITASK(void const *argument)
{
    LOGINFO("[freeRTOS] UI Task Start");
    MyUIInit();
    LOGINFO("[freeRTOS] UI Init Done");
    for (;;) {
        UITask();
        osDelay(1);
    }
}
