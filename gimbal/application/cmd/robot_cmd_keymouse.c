#include "robot_cmd_private.h"

/**
 * @brief 将单圈目标角映射为距离当前累计角最近的累计角目标
 *
 * @param current_angle_deg 当前累计角度
 * @param target_angle_norm_deg 目标单圈角度
 * @return float 最近的累计角目标
 */
static float FindClosestCumulativeAngle(float current_angle_deg, float target_angle_norm_deg)
{
    float target_norm = target_angle_norm_deg;
    float current_round;
    float target_angle_deg;
    float diff_deg;

    // 先把输入目标约束回单圈范围；上层只关心“反向 180 度”这类单圈语义，统一归一化后再映射到累计角可以避免跨圈歧义。
    while (target_norm > 180.0f) {
        target_norm -= 360.0f;
    }
    while (target_norm <= -180.0f) {
        target_norm += 360.0f;
    }

    // 先按当前累计角所在的最近圈数生成一个初始目标；云台 yaw 现在按 IMU 累计角闭环，必须把单圈语义落到最近圈才能避免突然多转整圈。
    current_round = roundf(current_angle_deg / 360.0f);
    target_angle_deg = current_round * 360.0f + target_norm;
    diff_deg = target_angle_deg - current_angle_deg;

    // 若初始目标离当前姿态超过半圈，则补偿一整圈到更近的等价目标；这样最终执行的一定是机械上最近的那一个等价角，而不是远端同姿态目标。
    if (diff_deg > 180.0f) {
        target_angle_deg -= 360.0f;
    } else if (diff_deg < -180.0f) {
        target_angle_deg += 360.0f;
    }

    return target_angle_deg;
}

/**
 * @brief 按斜坡限速把当前值推进到目标值
 *
 * @param current 当前输出
 * @param target 目标输出
 * @param dt_s 本拍时间
 * @return float 平滑后的输出
 */
static float RampKeyboardAxis(float current, float target, float dt_s)
{
    float max_step;

    // 这里根据“加速/减速/反向”场景选择不同步长，作用是让起步柔和但松键更干脆；
    // 原因是底盘平移最怕的就是起步突兀和停车拖尾，两种手感不能共用一套斜率。
    if (target == 0.0f || (current > 0.0f && target < current) || (current < 0.0f && target > current)) {
        max_step = KEYBOARD_CHASSIS_RAMP_DOWN_PER_SEC * dt_s;
    } else {
        max_step = KEYBOARD_CHASSIS_RAMP_UP_PER_SEC * dt_s;
    }

    if (target > current + max_step) {
        current += max_step;
    } else if (target < current - max_step) {
        current -= max_step;
    } else {
        current = target;
    }

    if (current < KEYBOARD_CHASSIS_CMD_EPSILON && current > -KEYBOARD_CHASSIS_CMD_EPSILON && target == 0.0f) {
        current = 0.0f;
    }
    return current;
}

/**
 * @brief 选择当前生效的键鼠输入源
 *
 * @return const RC_ctrl_t* 当前键鼠输入源
 */
static const RC_ctrl_t *GetActiveMouseKeySource(void)
{
    // 键鼠输入跟随当前主控源取数；VT03 在线时需要完整接管图传键鼠，断链后再无缝回退到 DBUS。
    if (current_control_source == CONTROL_SOURCE_VT03 && video_link_data != NULL) {
        return &video_link_data[TEMP];
    }
    return &rc_data[TEMP];
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
void MouseKeySet(void)
{
    const RC_ctrl_t *mouse_key_source = GetActiveMouseKeySource();
    uint8_t video_link_online = IsVideoLinkControlReady();
    uint16_t key_bits = mouse_key_source->key[KEY_PRESS].keys;
    uint8_t friction_toggle_pressed = (uint8_t)((key_bits >> Key_F) & 0x1u);
    uint8_t bullet_speed_toggle_pressed = (uint8_t)((key_bits >> Key_R) & 0x1u);
    uint8_t free_toggle_pressed = (uint8_t)((key_bits >> Key_B) & 0x1u);
    uint8_t turnback_toggle_pressed = 0u;
    uint8_t turnback_toggle_raw_pressed = (uint8_t)((key_bits >> Key_V) & 0x1u);
    uint8_t turnback_sample_updated = 0u;
    uint8_t ui_refresh_pressed = (uint8_t)((key_bits >> Key_G) & 0x1u);
    uint8_t spin_toggle_pressed = (uint8_t)((key_bits >> Key_X) & 0x1u);
    int8_t keyboard_vx = (int8_t)((key_bits >> Key_W) & 0x1u) - (int8_t)((key_bits >> Key_S) & 0x1u);
    int8_t keyboard_vy = (int8_t)((key_bits >> Key_D) & 0x1u) - (int8_t)((key_bits >> Key_A) & 0x1u);
    uint8_t mouse_left_pressed = mouse_key_source->mouse.press_l;
    uint32_t now_ms = DWT_GetTimeline_ms();
    uint32_t turnback_frame_serial = GetControlSourceKeyFrameSerial(current_control_source);
    float dt_s = 0.005f;
    float keyboard_target_vx = (float)keyboard_vx * KEYBOARD_CHASSIS_CMD_SCALE;
    float keyboard_target_vy = (float)keyboard_vy * KEYBOARD_CHASSIS_CMD_SCALE;

    // 这里仅刷新图传在线记忆，作用是让后续周期继续识别新的离线边沿；
    // 原因是主控切换时需要清掉哪些旧状态，已经在 `HandleControlSourceSwitch` 里按新源统一处理过，这里再重置会破坏刚同步好的按键边沿。
    last_video_link_online = video_link_online;

    if (keyboard_ramp_last_ms != 0u) {
        uint32_t dt_ms = now_ms - keyboard_ramp_last_ms;
        if (dt_ms > 0u && dt_ms < 20u) {
            dt_s = (float)dt_ms * 0.001f;
        }
    }
    keyboard_ramp_last_ms = now_ms;

    // 遇到急停或零力模式时直接清空键鼠锁存，作用是确保任何鼠标残留命令都不会越过急停；
    // 原因是键鼠现在和遥控器并行输入，停机优先级必须最高。
    if (robot_state == ROBOT_STOP ||
        gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE ||
        chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE) {
        ResetMouseControlLatchState();
        ResetKeyboardMotionState();
        return;
    }

    // 这里把键盘目标值通过斜坡推进到输出，作用是让起步和停车都不是阶跃命令；
    // 原因是底盘动力链对阶跃输入很敏感，直接满量纲切换会表现成前后左右猛冲。
    keyboard_vx_smoothed = RampKeyboardAxis(keyboard_vx_smoothed, keyboard_target_vx, dt_s);
    keyboard_vy_smoothed = RampKeyboardAxis(keyboard_vy_smoothed, keyboard_target_vy, dt_s);

    if (keyboard_vx != 0 || keyboard_vx_smoothed != 0.0f) {
        chassis_cmd_send.vx = keyboard_vx_smoothed;
    }
    if (keyboard_vy != 0 || keyboard_vy_smoothed != 0.0f) {
        chassis_cmd_send.vy = keyboard_vy_smoothed;
    }

    // 鼠标在遥控器目标上继续叠加云台增量，作用是保留遥控微调同时给电脑端快速修正；
    // 原因是旧工程的键鼠本来就是通过 DBUS 叠加进来，本轮需要恢复这种混控手感。
    gimbal_cmd_send.yaw -= (float)mouse_key_source->mouse.x * MOUSE_YAW_SENSITIVITY_DEG;
    gimbal_cmd_send.pitch += (float)mouse_key_source->mouse.y * MOUSE_PITCH_SENSITIVITY_DEG;
    LimitGimbalPitchTarget();

    // F 键现在只负责摩擦轮的显式开关，目的是把“预热”和“拨弹”彻底拆开，确保鼠标左键不会再隐式带起摩擦轮。
    if (friction_toggle_pressed && !keyboard_friction_toggle_last) {
        if (mouse_fire_friction_latched) {
            ResetMouseFireState();
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.load_mode = LOAD_STOP;
            shoot_cmd_send.bullet_speed = BULLET_SPEED_NONE;
            shoot_cmd_send.shoot_rate = 0.0f;
        } else {
            // 只锁存摩擦轮开启意图而不立即拨弹，目的是用户按下 F 后必须再明确点一次左键，才能触发真正的发射动作。
            mouse_fire_friction_latched = 1u;
        }
    }
    keyboard_friction_toggle_last = friction_toggle_pressed;

    // R 键独立切换 12/16m/s 预选档位，目的是让操作者在不开摩擦轮时也能先选好档位，并立即把结果同步到 UI。
    if (bullet_speed_toggle_pressed && !keyboard_bullet_speed_toggle_last) {
        if (keyboard_bullet_speed_selected == BIG_AMU_16) {
            keyboard_bullet_speed_selected = BIG_AMU_12;
        } else {
            keyboard_bullet_speed_selected = BIG_AMU_16;
        }
    }
    keyboard_bullet_speed_toggle_last = bullet_speed_toggle_pressed;

    if (free_toggle_pressed && !keyboard_free_toggle_last) {
        // B 键显式切模式前先取消一键掉头状态 用户已经给出新的自由模式意图，旧掉头任务和强制跟随锁存都不应继续霸占控制权。
        ClearKeyboardTurnbackState();
        // 用 B 键上升沿翻转“云台自由 + 底盘自由”锁存 用户希望按一下直接进入该模式，而不是依赖遥控器档位或持续按住按键。
        keyboard_free_mode_latched = (uint8_t)!keyboard_free_mode_latched;
        // 进入自由模式时主动清掉小陀螺锁存；自由模式与小陀螺控制语义互斥，保留旧锁存会导致模式争抢。
        if (keyboard_free_mode_latched != 0u) {
            keyboard_spin_mode_latched = 0u;
        }
        // 自由模式切换边沿统一贴齐当前姿态，目的是避免模式切换后继续追旧目标导致云台突跳。
        SyncGimbalTargetToCurrentAttitude();
    }
    keyboard_free_toggle_last = free_toggle_pressed;

    if (ui_refresh_pressed && !keyboard_ui_refresh_last) {
        // 用 G 键上升沿触发一次底盘 UI 全量刷新；复用现有底盘 UI 重建入口，比临时拼局部刷新包更稳。
        chassis_cmd_send.ui_refresh_request = 1u;
    }
    keyboard_ui_refresh_last = ui_refresh_pressed;

    if (spin_toggle_pressed && !keyboard_spin_toggle_last) {
        // X 键显式切模式前先取消一键掉头状态；用户已经要求转入小陀螺，旧掉头任务和强制跟随锁存必须立刻失效，避免两个模式同时抢控制权。
        ClearKeyboardTurnbackState();
        // 用 X 键上升沿翻转键鼠小陀螺锁存；用户要求按一次切一次，而不是按住期间临时进入小陀螺。
        keyboard_spin_mode_latched = (uint8_t)!keyboard_spin_mode_latched;
        // 进入小陀螺时主动退出自由模式锁存；两种模式都要最终覆盖底盘/云台状态，互斥处理能避免分支互相打架。
        if (keyboard_spin_mode_latched != 0u) {
            keyboard_free_mode_latched = 0u;
        }
        // 小陀螺开关切换时同步云台目标到当前姿态，目的是避免在自由/跟随/小陀螺之间切换时继续追旧目标产生瞬时跳变。
        SyncGimbalTargetToCurrentAttitude();
    }
    keyboard_spin_toggle_last = spin_toggle_pressed;

    // 只有收到新的输入帧时才推进 V 键确认状态；控制任务会重复读取同一帧，若按周期累加会把一帧毛刺误当成多帧稳定按下。
    turnback_sample_updated = (uint8_t)(turnback_frame_serial != keyboard_turnback_last_frame_serial);
    if (turnback_sample_updated != 0u) {
        // 记录这次已经消费过的输入帧序号；下一拍若仍然是同一帧，就不应该再次推进 V 键去抖计数。
        keyboard_turnback_last_frame_serial = turnback_frame_serial;
        if (turnback_toggle_raw_pressed != 0u) {
            // 仅在新帧仍然报告 V 按下时累加确认计数；真按键会跨多帧保持，而单帧脏数据和旧帧重放都过不了这道门槛。
            if (keyboard_turnback_press_frame_count < TURNBACK_TRIGGER_STABLE_FRAMES) {
                keyboard_turnback_press_frame_count++;
            }
        } else {
            // 只要新帧已经显示 V 松开，就立刻把确认计数清零；掉头键必须重新经历完整按下过程后才允许再次触发。
            keyboard_turnback_press_frame_count = 0u;
        }
    }
    // 只有连续多帧确认按下后，才把当前 V 视为真正有效；用户现场已经出现“没按却触发”，这里必须把瞬时脏帧挡在业务逻辑之外。
    turnback_toggle_pressed = (uint8_t)(keyboard_turnback_press_frame_count >= TURNBACK_TRIGGER_STABLE_FRAMES);

    if (turnback_toggle_pressed && !keyboard_turnback_toggle_last) {
        if (gimbal_fetch_data.yaw_motor_online != 0u &&
            keyboard_spin_mode_latched == 0u &&
            keyboard_free_mode_latched == 0u &&
            chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
            float current_yaw_total = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
            float reverse_gimbal_yaw_norm;

            // V 键触发的是一次性掉头动作，因此先清掉旧掉头会话残留；连续多次按 V 时每次都应该从当前姿态重新计算新的 180 度目标，而不是继承旧会话的计时和稳定计数。
            ClearKeyboardTurnbackState();
            // 启动掉头前先把目标贴齐当前姿态，目的是让新的 180 度动作从当前真实反馈平滑起步，而不是叠着旧目标直接猛转。
            SyncGimbalTargetToCurrentAttitude();

            // 直接以当前云台世界系朝向作为掉头参考；用户现在要的是“当前看到哪就以该方向反向 180 度作为新前方”，而不是先回到底盘中线再反向。
            reverse_gimbal_yaw_norm = theta_format(current_yaw_total + 180.0f);
            // 把“当前云台朝向反向 180 度”的单圈目标映射成距离当前累计角最近的累计角；云台 yaw 闭环吃的是累计角，仍要选最近等价目标来避免无意义的跨圈。
            keyboard_turnback_target_yaw = FindClosestCumulativeAngle(current_yaw_total, reverse_gimbal_yaw_norm);
            // 记录这次掉头完成后应提交的新前方向参考；用户要求底盘在掉头期间不动，但完成后又要把反转后的云台朝向认定为新的前方。
            keyboard_turnback_follow_front_target_deg = theta_format(chassis_cmd_send.offset_angle + 180.0f);
            // 掉头触发拍立即进入执行态；用户要的是按一次立刻开始云台 180 度掉头，而不是等待下一拍再起动作。
            keyboard_turnback_active = 1u;
            keyboard_turnback_start_ms = now_ms;
            keyboard_turnback_stable_ticks = 0u;
            // 触发拍直接把 yaw 目标切到本次掉头目标；这样同一拍下发出去的就是新的 180 度参考，不需要再等下一控制周期才起动作。
            gimbal_cmd_send.yaw = keyboard_turnback_target_yaw;
        }
    }
    keyboard_turnback_toggle_last = turnback_toggle_pressed;

    if (keyboard_turnback_active != 0u) {
        float yaw_error_deg = fabsf(theta_format(keyboard_turnback_target_yaw - gimbal_fetch_data.gimbal_imu_data.YawTotalAngle));

        // 一键掉头执行期间持续请求底盘进入“只阻尼刹停”子状态；当前用户反馈的前段跟转来自跟随环历史量和云台角速度前馈，这里必须让底盘暂时完全退出追随云台。
        chassis_cmd_send.follow_brake_request = 1u;
        // 一键掉头执行期间强制把跟随误差压成 0；用户明确要求整个动作只有云台转动，底盘不能因为看到真实夹角变大就跟着补转。
        chassis_cmd_send.follow_offset_angle = 0.0f;
        // 同步把最近可信跟随误差也更新为 0；若 yaw 在掉头中途离线，底盘侧冻结时也必须继续保持“不跟着转”的结果。
        last_valid_follow_offset_angle = 0.0f;

        if (gimbal_fetch_data.yaw_motor_online == 0u) {
            // yaw 电机若在执行途中离线就立刻取消本次掉头；失去可靠反馈后已经无法确认云台真实到位情况，继续执行或提交新前方都会把整车语义带偏。
            ClearKeyboardTurnbackState();
        } else if ((now_ms - keyboard_turnback_start_ms) >= TURNBACK_TIMEOUT_MS) {
            // 一键掉头执行态次优先检查超时；机构阻塞或姿态估计异常时不能一直霸占目标强制覆盖，超时后必须退出并保留旧前方语义。
            ClearKeyboardTurnbackState();
        } else if (yaw_error_deg < TURNBACK_COMPLETE_YAW_ERROR_DEG) {
            // 只有云台目标误差足够小，才累计“完成稳定拍数”；这次需求明确只让云台自己转到位，底盘并不参与完成判据。
            if (keyboard_turnback_stable_ticks < TURNBACK_COMPLETE_STABLE_TICKS) {
                keyboard_turnback_stable_ticks++;
            }
            // 连续多拍稳定满足完成条件后提交新的前方向参考并退出执行态；这样可以滤掉短暂穿越阈值的假到位，只让真正完成的掉头改写后续前方语义。
            if (keyboard_turnback_stable_ticks >= TURNBACK_COMPLETE_STABLE_TICKS) {
                follow_front_offset_deg = keyboard_turnback_follow_front_target_deg;
                ClearKeyboardTurnbackState();
            }
        } else {
            // 只要任一完成条件被破坏，就重新开始稳定计数，目的是真正完成必须连续稳定，而不是零星几拍偶然接近目标。
            keyboard_turnback_stable_ticks = 0u;
        }
    }

    // 只有 F 已经把摩擦轮明确打开后，左键上升沿才允许拨一发，目的是把“误触鼠标直接起火”的风险压到最低。
    if (mouse_fire_friction_latched != 0u) {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.bullet_speed = keyboard_bullet_speed_selected;
        if (mouse_left_pressed && !mouse_left_last) {
            shoot_cmd_send.load_mode = LOAD_1_BULLET;
        }
    }

    if (keyboard_turnback_active != 0u) {
        // 一键掉头执行期间只强制保持云台工作在陀螺仪模式；掉头动作本身只属于云台 yaw 目标覆盖，不应该顺带改写底盘模式导致底盘跟着转。
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        // 仅在掉头执行态内持续覆盖 yaw 目标；执行完成后应恢复普通跟随下的手动 yaw 控制，而不是继续把云台锁死在旧目标上。
        gimbal_cmd_send.yaw = keyboard_turnback_target_yaw;
    } else if (keyboard_spin_mode_latched != 0u) {
        // X 锁存生效后在键鼠层最终覆盖成小陀螺 + 陀螺仪模式；用户要求 X 优先于当前遥控器挡位，直到再次按 X 或被安全链清锁。
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    } else if (keyboard_free_mode_latched != 0u) {
        // B 锁存生效后最终覆盖成底盘自由平移 + 云台自由控制；用户要求键盘提供一个不依赖遥控器档位的自由模式快捷入口。
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }

    mouse_left_last = mouse_left_pressed;
}
