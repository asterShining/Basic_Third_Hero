#include "robot_cmd_private.h"

/**
 * @brief 清空键鼠射击锁存状态
 *
 */
void ResetMouseFireState(void)
{
    // 这里只清空键鼠射击链自己的跨拍状态，目的是 F 关闭、急停或切源后不残留任何拨弹与档位历史，同时又不影响小陀螺和自由模式等其它键鼠锁存。
    mouse_fire_friction_latched = 0;
    mouse_left_last = 0;
    keyboard_friction_toggle_last = 0;
    keyboard_bullet_speed_toggle_last = 0;
    keyboard_bullet_speed_selected = BIG_AMU_12;
}

/**
 * @brief 清空键盘一键掉头状态
 *
 */
void ClearKeyboardTurnbackState(void)
{
    // 一次性清空掉头执行态、目标角、新前方向候选值和完成计时，目的是被模式切换、切源或急停打断后绝不能继续沿用旧掉头任务驱动云台，或在异常结束后误提交新的前方向语义。
    keyboard_turnback_active = 0u;
    keyboard_turnback_target_yaw = 0.0f;
    keyboard_turnback_follow_front_target_deg = 0.0f;
    keyboard_turnback_start_ms = 0u;
    keyboard_turnback_stable_ticks = 0u;
}

/**
 * @brief 获取当前主控源最近一次已解析输入帧的序号
 *
 * @param source 当前主控源
 * @return uint32_t 累计输入帧序号
 */
uint32_t GetControlSourceKeyFrameSerial(ControlSource_e source)
{
    const VideoLinkKM_Diag_s *video_link_diag = NULL;

    if (source == CONTROL_SOURCE_VT03) {
        // VT03 分支直接读取图传模块的有效帧计数，目的是只有真正通过校验并完成解包的新图传帧，才应该被当作新的键盘按键样本。
        video_link_diag = VideoLinkKMGetDiag();
        if (video_link_diag != NULL) {
            return video_link_diag->valid_frame_count;
        }
        return 0u;
    }
    if (source == CONTROL_SOURCE_DT7) {
        // DT7 分支读取 DBUS 成功解析帧计数，目的是让 V 键去抖同样基于“新帧到达”而不是控制任务重复读同一帧。
        return RemoteControlGetFrameCount();
    }
    return 0u;
}

/**
 * @brief 清空键鼠控制锁存状态
 *
 */
void ResetMouseControlLatchState(void)
{
    // 这里保留“安全出口硬清理”语义，作用是急停、零力或真正双离线时把全部键鼠模式锁存一起打回安全基线；
    // 原因是这些场景下用户的旧模式意图已经失效，恢复后必须重新显式给出命令，不能让小陀螺或自由模式自己回来。
    ResetMouseFireState();
    keyboard_spin_toggle_last = 0u;
    keyboard_free_toggle_last = 0u;
    keyboard_turnback_toggle_last = 0u;
    keyboard_turnback_last_frame_serial = 0u;
    keyboard_turnback_press_frame_count = 0u;
    keyboard_ui_refresh_last = 0u;
    keyboard_spin_mode_latched = 0u;
    keyboard_free_mode_latched = 0u;
    ClearKeyboardTurnbackState();
}

/**
 * @brief 清空键盘平移斜坡状态
 *
 */
void ResetKeyboardMotionState(void)
{
    // 这里把键盘平移斜坡状态清零，作用是急停/断链后不保留上一拍的加减速尾巴；
    // 原因是平滑状态本身是跨周期记忆量，不主动复位就会在恢复控制时带出历史速度。
    keyboard_vx_smoothed = 0.0f;
    keyboard_vy_smoothed = 0.0f;
    keyboard_ramp_last_ms = 0u;
}

/**
 * @brief 将云台目标同步到当前姿态
 *
 */
void SyncGimbalTargetToCurrentAttitude(void)
{
    // 在控制源切换或模式切换时把云台目标同步到当前反馈，目的是避免切源后继续追旧目标导致云台突然跳转。
    gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
    LimitGimbalPitchTarget();
}

/**
 * @brief 请求 pitch 恢复时贴齐当前姿态
 *
 */
void RequestPitchRecoverToCurrentAttitude(void)
{
    // 在触发恢复的这一拍先把目标角记成当前 IMU pitch，目的是用户要的是“恢复时保持当前枪口姿态”，因此不能沿用旧锁存目标，也不能改成固定 0 度。
    pitch_recover_target_deg = gimbal_fetch_data.gimbal_imu_data.Pitch;
    // 再锁存几拍恢复同步请求，目的是最终发包阶段还会经过遥控器、键鼠和模式覆盖，只有短暂保持才能确保这次“贴当前姿态”真正落到底层。
    pitch_target_sync_hold_ticks = PITCH_TARGET_SYNC_HOLD_TICKS;
}

/**
 * @brief 请求底盘侧进入“小陀螺退跟随”接管窗口
 *
 */
void RequestFollowTransition(void)
{
    // 本拍立即拉高底盘接管请求并启动多拍保持，目的是退出小陀螺到跟随是边沿事件，保持几拍才能避免双板调度相位错开时底盘漏收。
    chassis_cmd_send.follow_transition_request = 1u;
    follow_transition_request_hold_ticks = FOLLOW_TRANSITION_REQUEST_HOLD_TICKS;
}

/**
 * @brief 清空 VT03 Pause 本次按压会话状态
 *
 */
void ResetVT03PausePressState(void)
{
    // 清空 VT03 Pause 的长按计时和已处理标记，目的是松手、切源或断链后本次按压语义已经失效，继续沿用会把下一次短按误判成长按。
    vt03_pause_press_start_ms = 0u;
    vt03_pause_press_started_in_zero_force = 0u;
    vt03_pause_longpress_handled = 0u;
    if (hint_buzzer != NULL) {
        // 清会话时同步关闭校零提示蜂鸣器，目的是蜂鸣器只应反映当前一次校零动作，不能跨按压会话残留鸣叫。
        AlarmSetStatus(hint_buzzer, ALARM_OFF);
    }
}

/**
 * @brief 执行一次 VT03 的 yaw 轴零点校准
 *
 */
void TriggerVT03YawCalibration(void)
{
    // 统一封装 VT03 长按 Pause 的 yaw 校零动作，目的是校零后还要同步软件零位和云台目标，集中处理能避免恢复后漏掉目标同步链路。
    GimbalCalibrate();
    yaw_align_offset_deg = 0.0f;
    // yaw 硬件零点被重标后同步把虚拟前方参考复位到 0，目的是旧的前方向参考依赖上一次零位坐标系，继续保留会让跟随误差整体偏移。
    follow_front_offset_deg = 0.0f;
    SyncGimbalTargetToCurrentAttitude();
    if (hint_buzzer != NULL) {
        // 校零真正触发后开启蜂鸣器提示，目的是现场操作需要一个明确反馈来确认 DM 零点指令已经发出。
        AlarmSetStatus(hint_buzzer, ALARM_ON);
    }
}

/**
 * @brief 判断 VT03 是否已经具备主控资格
 *
 * @return uint8_t 1:可用 0:不可用
 */
uint8_t IsVideoLinkControlReady(void)
{
    // 统一封装 VT03 主控可用条件，目的是主控仲裁和键鼠取源都必须依赖同一套在线且有有效帧的判断。
    return (uint8_t)((video_link_data != NULL) && VideoLinkKMIsOnline() && VideoLinkKMHasValidFrame());
}

/**
 * @brief 获取当前主控输入源
 *
 * @return ControlSource_e 当前主控输入源
 */
ControlSource_e GetActiveControlSource(void)
{
    // 固定采用 VT03 优先、DT7 回退、双离线零力的顺序，目的是用户要求 VT03 遥控器恢复主控，DT7 只在断链时兜底。
    if (IsVideoLinkControlReady()) {
        return CONTROL_SOURCE_VT03;
    }
    if (RemoteControlIsOnline()) {
        return CONTROL_SOURCE_DT7;
    }
    return CONTROL_SOURCE_NONE;
}

/**
 * @brief 将键鼠边沿状态对齐到当前输入源的当前电平
 *
 * @param source 当前主控源
 * @param mouse_key_source 当前输入源的键鼠数据视图
 */
static void SyncMouseKeyEdgeState(ControlSource_e source, const RC_ctrl_t *mouse_key_source)
{
    uint16_t key_bits = 0u;
    uint8_t turnback_pressed = 0u;

    if (mouse_key_source != NULL) {
        key_bits = mouse_key_source->key[KEY_PRESS].keys;
        mouse_left_last = mouse_key_source->mouse.press_l;
    } else {
        mouse_left_last = 0u;
    }

    // 切换输入源时对齐键盘 F 键和鼠标左键边沿历史，目的是切源首拍不能把“本来就按住”的按键误判成新的安全动作或拨弹边沿。
    keyboard_friction_toggle_last = (uint8_t)((key_bits >> Key_F) & 0x1u);
    // 同步对齐 R 键边沿历史，目的是用户若切源时正按着 R，不应该在新输入源首拍又被当成一次新的档位切换。
    keyboard_bullet_speed_toggle_last = (uint8_t)((key_bits >> Key_R) & 0x1u);
    // 同步对齐 B 键边沿历史，目的是主控切换时若用户正按着 B，不应在切源首拍被误判成新的自由模式切换。
    keyboard_free_toggle_last = (uint8_t)((key_bits >> Key_B) & 0x1u);
    turnback_pressed = (uint8_t)((key_bits >> Key_V) & 0x1u);
    // 切源时把 V 键确认态直接贴到当前电平，目的是用户若本来就按着 V，切源后的第一拍不应该因为重新计数而被当成一次新的掉头触发。
    keyboard_turnback_toggle_last = turnback_pressed;
    // 同步记录当前主控源最近一帧序号，目的是切源后要从新的输入流基线继续去抖，不能拿旧源的帧号和新源混算。
    keyboard_turnback_last_frame_serial = GetControlSourceKeyFrameSerial(source);
    // 若切源瞬间 V 已经按下，就直接把确认帧数补齐，目的是这样后续保持按住只会被视为“持续按住”，不会在若干帧后又凭空补触发一次。
    keyboard_turnback_press_frame_count = (turnback_pressed != 0u) ? TURNBACK_TRIGGER_STABLE_FRAMES : 0u;
    // 同步对齐 G 键边沿历史，目的是主控切换时用户若正按着 G，不应该在切源首拍误触发一次 UI 全量刷新。
    keyboard_ui_refresh_last = (uint8_t)((key_bits >> Key_G) & 0x1u);
    // 同步对齐 X 键边沿历史，目的是主控切换时若用户正按着 X，不应在切源首拍被误判成新的小陀螺切换。
    keyboard_spin_toggle_last = (uint8_t)((key_bits >> Key_X) & 0x1u);
}

/**
 * @brief 处理主控输入源切换时的锁存复位和边沿同步
 *
 * @param new_source 新的主控输入源
 */
void HandleControlSourceSwitch(ControlSource_e new_source)
{
    const VideoLinkKM_RemoteState_s *video_link_remote_state = VideoLinkKMGetRemoteState();
    const RC_ctrl_t *mouse_key_source = NULL;
    uint8_t need_sync_on_source_switch = (uint8_t)(new_source != CONTROL_SOURCE_NONE);

    if (new_source == current_control_source) {
        return;
    }

    // 切换主控源时统一清空跨周期键鼠锁存，目的是恢复旧逻辑，让不同链路的模式状态不要互相继承。
    // 这样做的代价是 X 小陀螺和 B 自由模式都会在切源时失效，但行为更贴近之前依赖当前主控源重新给模式意图的方案。
    ResetMouseControlLatchState();
    ResetKeyboardMotionState();
    ResetChassisAuxState();
    friction_switch_state = 0u;
    shoot_cmd_send.shoot_rate = 0.0f;
    // 切换主控源时同步清掉 DT7 校零锁存，目的是内八组合已经失效，继续沿用旧触发状态会让蜂鸣器残留或下次组合无法重新触发。
    cali_triggered = 0u;
    // 切换主控源时只清理 Pause 本次按压会话，不清零力锁存，目的是VT03 短暂掉帧或切回 DT7 时若把失能锁存一起清掉，链路恢复后机器人会自己突然重新使能。
    ResetVT03PausePressState();
    vt03_pause_last = 0u;
    vt03_mode_sw_last = VT03_MODE_SW_INVALID;

    if (new_source == CONTROL_SOURCE_DT7) {
        last_switch_left = rc_data[TEMP].rc.switch_left;
        last_switch_right = rc_data[TEMP].rc.switch_right;
        vt03_fn_left_last = 0u;
        vt03_fn_right_last = 0u;
        vt03_trigger_last = 0u;
        mouse_key_source = &rc_data[TEMP];
    } else if (new_source == CONTROL_SOURCE_VT03) {
        last_switch_left = RC_SW_DOWN;
        last_switch_right = RC_SW_DOWN;
        if (video_link_remote_state != NULL) {
            vt03_fn_left_last = video_link_remote_state->fn_left_button_down;
            vt03_fn_right_last = video_link_remote_state->fn_right_button_down;
            vt03_trigger_last = video_link_remote_state->trigger_button_down;
            vt03_pause_last = video_link_remote_state->pause_button_down;
            vt03_mode_sw_last = video_link_remote_state->mode_sw;
        } else {
            vt03_fn_left_last = 0u;
            vt03_fn_right_last = 0u;
            vt03_trigger_last = 0u;
            vt03_pause_last = 0u;
        }
        if (video_link_data != NULL) {
            mouse_key_source = &video_link_data[TEMP];
        }
        if (vt03_boot_zero_force_pending != 0u) {
            // VT03 第一次真正接管时先锁进零力，目的是用户要求 VT03 上电后默认失能，必须等操作者明确按一次 Pause 才解除。
            vt03_pause_zero_force_latched = 1u;
            vt03_boot_zero_force_pending = 0u;
            // 首次接管默认零力时同步清空 Pause 按压会话，目的是若沿用接管瞬间的电平或旧计时，可能把第一次短按误判成释放或长按。
            ResetVT03PausePressState();
            // 首次接管默认零力时不做目标同步，目的是当前拍本来就要保持失能，额外下发恢复链只会制造无意义的边沿扰动。
            need_sync_on_source_switch = 0u;
        }
    } else {
        last_switch_left = RC_SW_DOWN;
        last_switch_right = RC_SW_DOWN;
        vt03_fn_left_last = 0u;
        vt03_fn_right_last = 0u;
        vt03_trigger_last = 0u;
    }

    SyncMouseKeyEdgeState(new_source, mouse_key_source);
    last_video_link_online = IsVideoLinkControlReady();

    if (need_sync_on_source_switch != 0u) {
        SyncGimbalTargetToCurrentAttitude();
    }

    current_control_source = new_source;
}

/**
 * @brief 清空上岛辅助机构状态
 *
 */
void ResetChassisAuxState(void)
{
    // 一次性复位前履带与抬升状态及下发值，目的是急停或退出上岛时不能沿用上一拍辅助机构命令继续动作。
#ifdef USE_ISLAND_ACTION
    front_track_switch_state = 0u;
    chassis_cmd_send.front_track_mode = FRONT_TRACK_OFF;
    chassis_cmd_send.lift_mode = LIFT_OFF;
    chassis_cmd_send.front_track_speed_ref = 0.0f;
    chassis_cmd_send.lift_dial_input = 0.0f;
#endif
}
