#include "robot_cmd_private.h"

/**
 * @brief 对摇杆量应用统一死区
 *
 * @param value 原始摇杆量
 * @return float 死区处理后的摇杆量
 */
static float ApplyRCDeadzone(float value)
{
    // 统一复用同一套摇杆死区，目的是VT03 和 DT7 都应该保持相同的中位抖动抑制手感。
    if (value > -RC_DEADZONE && value < RC_DEADZONE) {
        return 0.0f;
    }
    return value;
}

#ifdef USE_ISLAND_ACTION
/**
 * @brief 将拨轮原始值归一化为抬升输入
 *
 * @param raw_dial 遥控器原始拨轮值
 * @return float 归一化后的抬升输入
 */
static float NormalizeLiftDialInput(int16_t raw_dial)
{
    float normalized = -((float)raw_dial) / AUX_DIAL_INPUT_MAX;
    float deadzone = AUX_DIAL_INPUT_DEADZONE / AUX_DIAL_INPUT_MAX;

    // 统一把拨轮上拨解释成正向抬升并做死区与限幅，目的是底盘侧只消费稳定标准化命令，避免双板两边各自解释方向和抖动。
    if (normalized > -deadzone && normalized < deadzone)
        return 0.0f;

    LIMIT_MIN_MAX(normalized, -1.0f, 1.0f);
    return normalized;
}
#endif

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void ApplyRemoteGimbalStickControl(float rocker_lx, float rocker_ly, int16_t dial_input)
{
    float yaw_sensitivity = 0.001f;
    float pitch_sensitivity = REMOTE_PITCH_SENSITIVITY;
    float yaw_gyro_dps = gimbal_fetch_data.gimbal_imu_data.Gyro[2] * RAD_2_DEGREE;

    if (dial_input > 100) {
        yaw_sensitivity *= 0.3f;
        pitch_sensitivity *= 0.3f;
    }

    // 统一复用当前积分式云台手感，目的是本轮只恢复 VT03 主控，不应该顺带重调云台控制参数。
    gimbal_cmd_send.yaw -= yaw_sensitivity * rocker_lx;
    gimbal_cmd_send.pitch += pitch_sensitivity * rocker_ly;

    // 仅在低角速度且非小陀螺工况下，才把 yaw 目标缓慢拉向当前反馈，目的是原逻辑在小陀螺时会把真实扰动当成“新目标”写回去，表现成云台越转越歪。
    if (rocker_lx == 0.0f &&
        chassis_cmd_send.chassis_mode != CHASSIS_ROTATE &&
        chassis_cmd_send.chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW &&
        yaw_gyro_dps < YAW_DRIFT_LOCK_GYRO_TH_DPS &&
        yaw_gyro_dps > -YAW_DRIFT_LOCK_GYRO_TH_DPS) {
        gimbal_cmd_send.yaw = YAW_DRIFT_LOCK_COEF * gimbal_cmd_send.yaw +
                              (1.0f - YAW_DRIFT_LOCK_COEF) * gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    }

    LimitGimbalPitchTarget();
}

/**
 * @brief 按当前遥控器手感更新底盘平移指令
 *
 * @param rocker_rx 右摇杆横向
 * @param rocker_ry 右摇杆纵向
 */
static void ApplyRemoteChassisStickControl(float rocker_rx, float rocker_ry)
{
    // 继续沿用当前底盘历史量纲，目的是底盘侧仍按“摇杆值乘 10”解释，不能在恢复 VT03 时再顺手改单位。
    chassis_cmd_send.vx = 10.0f * rocker_ry;
    chassis_cmd_send.vy = 10.0f * rocker_rx;
}

/**
 * @brief 控制输入为 DT7 遥控器时的模式和控制量设置
 *
 */
static void RemoteControlSetDT7(void)
{
    uint16_t current_switch_right = rc_data[TEMP].rc.switch_right;
    uint16_t current_switch_left = rc_data[TEMP].rc.switch_left;
    uint8_t dt7_cali_combo_active = 0u;
    uint8_t left_mid_to_up = (uint8_t)(switch_is_up(current_switch_left) && switch_is_mid(last_switch_left));
    uint8_t left_mid_to_down = (uint8_t)(switch_is_down(current_switch_left) && switch_is_mid(last_switch_left));
    float rocker_lx;
    float rocker_ly;
    float rocker_rx;
    float rocker_ry;

#ifdef USE_ISLAND_ACTION
    // 每拍先清空辅助机构瞬态命令，目的是上岛开关是锁存状态，但履带速度和抬升拨轮输入都不应该沿用旧值。
    chassis_cmd_send.front_track_mode = front_track_switch_state ? FRONT_TRACK_ON : FRONT_TRACK_OFF;
    chassis_cmd_send.lift_mode = LIFT_OFF;
    chassis_cmd_send.front_track_speed_ref = 0.0f;
    chassis_cmd_send.lift_dial_input = 0.0f;
#endif

    if (switch_is_down(current_switch_right)) {
        uint8_t is_inner_eight;

        EmergencyHandler();
        is_inner_eight = (uint8_t)(switch_is_down(current_switch_left) &&
                                   (rc_data[TEMP].rc.rocker_l_ > RC_TRIGGER_TH) &&
                                   (rc_data[TEMP].rc.rocker_l1 < -RC_TRIGGER_TH) &&
                                   (rc_data[TEMP].rc.rocker_r_ < -RC_TRIGGER_TH) &&
                                   (rc_data[TEMP].rc.rocker_r1 < -RC_TRIGGER_TH));
        if (is_inner_eight != 0u) {
            dt7_cali_combo_active = 1u;
            if (cali_triggered == 0u) {
                // 在 DT7 急停态内八组合下触发一次 yaw DM 校零，目的是恢复历史零点设置入口，同时只允许每次组合发送一次指令避免重复改零点。
                GimbalCalibrate();
                yaw_align_offset_deg = 0.0f;
                // DT7 触发 yaw 校零时同步复位虚拟前方参考，目的是物理零位坐标系一旦重建，旧前方向参考会立刻失效，继续沿用只会把底盘跟随带偏。
                follow_front_offset_deg = 0.0f;
                SyncGimbalTargetAndRequestYawReset();
                if (hint_buzzer != NULL) {
                    // DT7 校零触发时开启蜂鸣器提示，目的是操作者需要立即知道本次内八已经成功进入校零链路。
                    AlarmSetStatus(hint_buzzer, ALARM_ON);
                }
                cali_triggered = 1u;
            }
        }
    } else if (switch_is_up(current_switch_right)) {
        if (!switch_is_up(last_switch_right)) {
            // DT7 右拨杆模式发生变化时立即取消一键掉头状态，目的是用户已经通过物理挡位给出新的模式意图，旧掉头任务不应继续抢控制权。
            ClearKeyboardTurnbackState();
            SyncGimbalTargetAndRequestYawReset();
        }

        robot_state = ROBOT_READY;
        // 上档恢复为小陀螺 + 云台陀螺仪模式，目的是保持现有 DT7 主驾驶语义，也避免顶部逻辑和 VT03 分支分叉两套行为。
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    } else if (switch_is_mid(current_switch_right)) {
        if (!switch_is_mid(last_switch_right)) {
            // DT7 右拨杆模式发生变化时立即取消一键掉头状态，目的是从其它模式切进跟随属于新的操控会话，旧掉头任务必须立刻失效。
            ClearKeyboardTurnbackState();
            SyncGimbalTargetAndRequestYawReset();
        }

        robot_state = ROBOT_READY;
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }

    if (dt7_cali_combo_active == 0u && cali_triggered != 0u) {
        // 内八组合失效后立即清掉 DT7 校零锁存与提示音，目的是只有完整重新摆出组合才允许再次触发，避免切模式后蜂鸣器残留。
        if (hint_buzzer != NULL) {
            AlarmSetStatus(hint_buzzer, ALARM_OFF);
        }
        cali_triggered = 0u;
    }

    rocker_lx = ApplyRCDeadzone((float)rc_data[TEMP].rc.rocker_l_);
    rocker_ly = ApplyRCDeadzone((float)rc_data[TEMP].rc.rocker_l1);
    rocker_rx = ApplyRCDeadzone((float)rc_data[TEMP].rc.rocker_r_);
    rocker_ry = ApplyRCDeadzone((float)rc_data[TEMP].rc.rocker_r1);

    if (!switch_is_down(current_switch_right)) {
        // 非急停状态下才允许把摇杆增量叠加到云台目标，目的是急停拍里保留任何姿态改写都可能破坏零力安全语义。
        ApplyRemoteGimbalStickControl(rocker_lx, rocker_ly, rc_data[TEMP].rc.dial);
    }

    if (switch_is_up(current_switch_right)) {
        if (gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE) {
            float current_yaw_total = gimbal_fetch_data.gimbal_imu_data.VISION_YAW_AXIS * VISION_YAW_SIGN;
            float current_pitch = gimbal_fetch_data.gimbal_imu_data.VISION_PITCH_AXIS * VISION_PITCH_SIGN;

            // 继续复用现有自瞄入口，目的是本轮拆文件只做职责划分，不改变自瞄接管条件和输出写回方式。
            auto_aim_state = AutoGimbalRun(
                vision_recv_data,
                current_yaw_total,
                current_pitch,
                &gimbal_cmd_send.yaw,
                &gimbal_cmd_send.pitch);

            if (auto_aim_state == AUTO_AIM_TRACKING) {
                // 当前保持原策略不在此处强改底盘模式，目的是用户这次只要求拆文件，不额外引入新的战术决策变化。
            }
        } else {
            auto_aim_state = AUTO_AIM_IDLE;
        }
    }

    ApplyRemoteChassisStickControl(rocker_rx, rocker_ry);

    if (!switch_is_down(current_switch_right)) {
        if (left_mid_to_up) {
            // 左拨杆上翻优先处理当前功能域，目的是同一个拨杆要在摩擦轮与上岛辅助机构之间复用，必须按当前锁存状态解释边沿。
            if (friction_switch_state != 0u) {
                friction_switch_state = 0u;
            } else {
#ifdef USE_ISLAND_ACTION
                if (front_track_switch_state == 0u)
#endif
                {
                    friction_switch_state = 1u;
                }
            }
        }

#ifdef USE_ISLAND_ACTION
        if (friction_switch_state == 0u && left_mid_to_down) {
            // 仅在摩擦轮关闭时响应履带切换，目的是保留原有左拨杆下拨开火语义，不让上岛逻辑抢占发射入口。
            front_track_switch_state = !front_track_switch_state;
        }
#endif

        if (friction_switch_state == 1u) {
            shoot_cmd_send.friction_mode = FRICTION_ON;
            shoot_cmd_send.bullet_speed = BIG_AMU_16;

            switch (fire_mode_state) {
            case 0:
                if (switch_is_down(current_switch_left)) {
                    shoot_cmd_send.load_mode = LOAD_1_BULLET;
                } else {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                    shoot_cmd_send.shoot_rate = 0.0f;
                }
                break;

            case 1:
                if (switch_is_down(current_switch_left)) {
                    shoot_cmd_send.load_mode = LOAD_2_BULLET;
                } else {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
                break;

            case 2:
                if (switch_is_down(current_switch_left)) {
                    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                    shoot_cmd_send.shoot_rate = BURST_FIRE_RATE;
                } else {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
                break;

            default:
                shoot_cmd_send.load_mode = LOAD_STOP;
                break;
            }
        } else {
            // 摩擦轮锁存关闭时同步清掉装填与射频输出，目的是发射应用不能在“预热关闭”状态下继续保留上一拍的 load 指令。
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.load_mode = LOAD_STOP;
            shoot_cmd_send.shoot_rate = 0.0f;
        }

#ifdef USE_ISLAND_ACTION
        chassis_cmd_send.front_track_mode = front_track_switch_state ? FRONT_TRACK_ON : FRONT_TRACK_OFF;
        if (front_track_switch_state != 0u) {
            float lift_dial_input = NormalizeLiftDialInput(rc_data[TEMP].rc.dial);

            // 履带开启时下发更保守的固定速度与抬升输入，目的是先把前履带速度降下来，同时让左拨杆上档就能直接触发后轮抬升。
            chassis_cmd_send.front_track_speed_ref = FRONT_TRACK_SPEED_REF_DEFAULT;
            if (switch_is_up(current_switch_left)) {
                if (lift_dial_input == 0.0f)
                    lift_dial_input = LIFT_SWITCH_UP_INPUT_DEFAULT;

                // 左拨杆上档时直接进入抬升调高，目的是用户操作上希望上档立刻有反应，不需要再依赖隐藏锁存或额外拨轮动作。
                chassis_cmd_send.lift_dial_input = lift_dial_input;
                chassis_cmd_send.lift_mode = LIFT_ADJUST;
            } else {
                // 左拨杆离开上档后保持当前位置，目的是后轮抬起后应稳定保持高度，避免松手就回落。
                chassis_cmd_send.lift_dial_input = 0.0f;
                chassis_cmd_send.lift_mode = LIFT_HOLD;
            }
        }
#endif

    } else {
        // 急停时强制清空所有遥控器锁存状态，目的是解除急停后必须从安全基线重新进入，而不是继续沿用旧会话。
        friction_switch_state = 0u;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.shoot_rate = 0.0f;
        ResetChassisAuxState();
    }

    last_switch_left = current_switch_left;
    last_switch_right = current_switch_right;
}

/**
 * @brief 处理 VT03 遥控器发射逻辑
 *
 * @param video_link_remote_state VT03 遥控器状态
 */
static void ApplyVT03ShootLogic(const VideoLinkKM_RemoteState_s *video_link_remote_state)
{
    uint8_t fn_right_pressed;
    uint8_t trigger_pressed;

    if (video_link_remote_state == NULL) {
        return;
    }

    fn_right_pressed = video_link_remote_state->fn_right_button_down;
    trigger_pressed = video_link_remote_state->trigger_button_down;

    if (fn_right_pressed && !vt03_fn_right_last) {
        // 使用 VT03 右自定义键切换摩擦轮，目的是VT03 遥控器没有 DT7 的拨杆边沿接口，需要在 cmd 层用自定义键承接启停。
        friction_switch_state = (uint8_t)!friction_switch_state;
    }

    if (friction_switch_state != 0u) {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.bullet_speed = BIG_AMU_16;

        // 扳机只响应单发上升沿，目的是用户要求 VT03 遥控器保持“扳机点一下打一发”的安全语义。
        if (trigger_pressed && !vt03_trigger_last) {
            shoot_cmd_send.load_mode = LOAD_1_BULLET;
        } else {
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
    } else {
        // VT03 摩擦轮关闭时同步清空装填和射频命令，目的是防止扳机边沿在关闭状态下继续残留到发射应用。
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.shoot_rate = 0.0f;
    }

    vt03_fn_right_last = fn_right_pressed;
    vt03_trigger_last = trigger_pressed;
}

/**
 * @brief 控制输入为 VT03 遥控器时的模式和控制量设置
 *
 */
static void RemoteControlSetVT03(void)
{
    const VideoLinkKM_RemoteState_s *video_link_remote_state = VideoLinkKMGetRemoteState();
    uint8_t pause_pressed;
    uint32_t now_ms;
    float rocker_lx;
    float rocker_ly;
    float rocker_rx;
    float rocker_ry;

    if (video_link_remote_state == NULL || video_link_data == NULL) {
        // VT03 状态视图失效时只复位 Pause 本次按压会话，目的是图传临时掉帧不应把已经锁住的零力状态顺手解除，否则链路恢复时会出现“自己突然使能”。
        ResetVT03PausePressState();
        vt03_pause_last = 0u;
        EmergencyHandler();
        return;
    }

    pause_pressed = video_link_remote_state->pause_button_down;
    now_ms = (uint32_t)DWT_GetTimeline_ms();
    if (pause_pressed && !vt03_pause_last) {
        // Pause 按下沿启动一次新的长按会话，目的是需要区分“按下进入零力”“零力中继续长按校零”和“松手退出零力”三种不同语义。
        vt03_pause_press_start_ms = now_ms;
        vt03_pause_longpress_handled = 0u;
        if (vt03_pause_zero_force_latched != 0u) {
            // 记录本次 Pause 是在零力态中按下，目的是只有已经零力时的长按才允许进入 yaw 校零，防止有力态误触。
            vt03_pause_press_started_in_zero_force = 1u;
        } else {
            // 非零力态按下 Pause 时立即进入零力，目的是保留 VT03 的短按失能入口，同时禁止第一次按住就直接跨过零力触发校零。
            vt03_pause_press_started_in_zero_force = 0u;
            vt03_pause_zero_force_latched = 1u;
            vt03_fn_right_last = video_link_remote_state->fn_right_button_down;
            vt03_trigger_last = video_link_remote_state->trigger_button_down;
            vt03_mode_sw_last = video_link_remote_state->mode_sw;
        }
    } else if (pause_pressed &&
               vt03_pause_press_started_in_zero_force != 0u &&
               vt03_pause_longpress_handled == 0u &&
               (now_ms - vt03_pause_press_start_ms) >= VT03_PAUSE_CALIB_HOLD_MS) {
        // 零力中长按达到阈值后只触发一次 yaw 校零，目的是DM 零点指令是重动作，同一次按住里重复发送没有收益且会增加风险。
        TriggerVT03YawCalibration();
        vt03_pause_longpress_handled = 1u;
    } else if (!pause_pressed && vt03_pause_last) {
        if (vt03_pause_press_started_in_zero_force != 0u && vt03_pause_longpress_handled == 0u) {
            // 零力中短按并松开时退出零力，目的是用户需要在不校零的情况下，通过一次短按恢复正常有力控制。
            vt03_pause_zero_force_latched = 0u;
            vt03_fn_right_last = video_link_remote_state->fn_right_button_down;
            vt03_trigger_last = video_link_remote_state->trigger_button_down;
            vt03_mode_sw_last = video_link_remote_state->mode_sw;
            SyncGimbalTargetAndRequestYawReset();
        }
        // Pause 松开沿统一结束当前按压会话，目的是下一次按键必须重新从 0 开始计时，蜂鸣器也应在此时关闭。
        ResetVT03PausePressState();
    }
    vt03_pause_last = pause_pressed;

    if (vt03_pause_zero_force_latched != 0u) {
        // Pause 锁存零力期间持续更新 VT03 边沿历史，目的是用户在零力时改挡位或按住扳机，恢复时不应该被当成新的边沿事件。
        vt03_fn_right_last = video_link_remote_state->fn_right_button_down;
        vt03_trigger_last = video_link_remote_state->trigger_button_down;
        vt03_mode_sw_last = video_link_remote_state->mode_sw;
        EmergencyHandler();
        return;
    }

    if (vt03_mode_sw_last != video_link_remote_state->mode_sw) {
        // VT03 挡位变化时立即取消一键掉头状态，目的是操作者已经显式切了主控模式，旧掉头任务不应继续绑死跟随和 yaw 目标。
        ClearKeyboardTurnbackState();
        SyncGimbalTargetAndRequestYawReset();
    }

    robot_state = ROBOT_READY;
    if (video_link_remote_state->mode_sw == VT03_MODE_SW_C) {
        // VT03 的 C 挡进入小陀螺 + 云台陀螺仪模式，目的是用户明确要求切到 C 挡时直接进入小陀螺。
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    } else if (video_link_remote_state->mode_sw == VT03_MODE_SW_N) {
        // VT03 的 N 挡进入底盘跟随云台，目的是保留常规驾驶模式，让 N 挡承担正常跟随操控。
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    } else if (video_link_remote_state->mode_sw == VT03_MODE_SW_S) {
        // VT03 的 S 挡进入底盘自由 + 云台自由，目的是保持遥控器 S 挡作为“完全手动自由控制”的直觉语义。
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    } else {
        // 异常挡位兜底回到底盘跟随云台，目的是协议层虽然已限幅，但上层仍保留保守回退，避免异常值直接打到自由或小陀螺。
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    auto_aim_state = AUTO_AIM_IDLE;

    rocker_lx = ApplyRCDeadzone((float)video_link_data[TEMP].rc.rocker_l_);
    rocker_ly = ApplyRCDeadzone((float)video_link_data[TEMP].rc.rocker_l1);
    rocker_rx = ApplyRCDeadzone((float)video_link_data[TEMP].rc.rocker_r_);
    rocker_ry = ApplyRCDeadzone((float)video_link_data[TEMP].rc.rocker_r1);

    ApplyRemoteGimbalStickControl(rocker_lx, rocker_ly, video_link_data[TEMP].rc.dial);
    ApplyRemoteChassisStickControl(rocker_rx, rocker_ry);

    // VT03 遥控分支显式关闭当前未接到图传遥控器上的辅助机构，目的是本轮只恢复主驾驶和发射，不让上岛逻辑误吃到图传状态。
    ResetChassisAuxState();
    ApplyVT03ShootLogic(video_link_remote_state);
    vt03_mode_sw_last = video_link_remote_state->mode_sw;
}

/**
 * @brief 根据当前主控输入源设置模式和控制量
 *
 * @param control_source 当前主控输入源
 */
void RemoteControlSet(ControlSource_e control_source)
{
    if (control_source == CONTROL_SOURCE_VT03) {
        RemoteControlSetVT03();
    } else if (control_source == CONTROL_SOURCE_DT7) {
        RemoteControlSetDT7();
    } else {
        EmergencyHandler();
    }
}
