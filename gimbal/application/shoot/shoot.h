#ifndef SHOOT_H
#define SHOOT_H

#include <stdint.h>

typedef struct
{
    float inner_left; // 内圈左 (m/s)
    float inner_right; // 内圈右 (m/s)
    float inner_down; // 内圈下 (m/s)
    float outer_left; // 外圈左 (m/s)
    float outer_right; // 外圈右 (m/s)
    float outer_down; // 外圈下 (m/s)
} ShootDebugSpeed_s;
/**
 * @brief 发射初始化,会被RobotInit()调用
 *
 */
void ShootInit();

/**
 * @brief 发射任务
 *
 */
void ShootTask();

/**
 * @brief 设置摩擦轮速度
 *
 * @param speed_mps 速度 m/s
 */
void ShootSetSpeed(float speed_mps);

/**
 * @brief 获取发射确认标志
 * @return 1 表示最近一次发射已确认, 0 表示未确认
 */
uint8_t ShootGetFiredFlag(void);

/**
 * @brief 获取缺弹标志
 * @return 1 表示检测到缺弹, 0 表示正常
 */
uint8_t ShootGetEmptyFlag(void);

/**
 * @brief 清除缺弹标志 (需要手动调用复位)
 */
void ShootClearEmptyFlag(void);

/**
 * @brief 获取已确认发射计数
 * @return 累计确认发射的弹丸数量
 */
uint16_t ShootGetFireCount(void);

/**
 * @brief 检查拨盘是否被锁定 (防多发机制)
 * @return 1 表示锁定中, 应拒绝新的发射指令
 */
uint8_t ShootIsLoaderLocked(void);

#endif // SHOOT_H