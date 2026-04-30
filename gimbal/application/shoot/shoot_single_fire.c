#include "shoot_private.h"

/**
 * @brief 记录单发控制使用的内圈掉速基线
 */
static void RecordControlDipBaseline(void)
{
    // 控制链单独维护一份掉速基线，目的是出弹计数不能依赖调试模块内部状态，而要有自己可控的一份峰值基线。
    dip_control.inner_left_baseline = fabsf(GetMotorSpeedAps(friction_inner_left));
    dip_control.inner_right_baseline = fabsf(GetMotorSpeedAps(friction_inner_right));
    dip_control.inner_down_baseline = fabsf(GetMotorSpeedAps(friction_inner_down));
    dip_control.outer_left_baseline = fabsf(GetMotorSpeedAps(friction_outer_left));
    dip_control.outer_right_baseline = fabsf(GetMotorSpeedAps(friction_outer_right));
    dip_control.outer_down_baseline = fabsf(GetMotorSpeedAps(friction_outer_down));
}

/**
 * @brief 以峰值保持方式更新单发控制基线
 */
static void UpdateControlDipPeakBaseline(void)
{
    float current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_inner_left));
    if (current_abs > dip_control.inner_left_baseline)
        dip_control.inner_left_baseline = current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_inner_right));
    if (current_abs > dip_control.inner_right_baseline)
        dip_control.inner_right_baseline = current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_inner_down));
    if (current_abs > dip_control.inner_down_baseline)
        dip_control.inner_down_baseline = current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_outer_left));
    if (current_abs > dip_control.outer_left_baseline)
        dip_control.outer_left_baseline = current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_outer_right));
    if (current_abs > dip_control.outer_right_baseline)
        dip_control.outer_right_baseline = current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_outer_down));
    if (current_abs > dip_control.outer_down_baseline)
        dip_control.outer_down_baseline = current_abs;
}

/**
 * @brief 计算当前喂弹步的推进量
 * @return 当前推进的机械行程，单位为 bullet
 */
static float GetCurrentFeedProgressBullet(void)
{
    float progress_angle = loader->measure.total_angle - single_fire.rush_start_angle;

    if (progress_angle < 0.0f)
        progress_angle = 0.0f;

    return progress_angle / LOADER_MOTOR_ANGLE_PER_BULLET;
}

/**
 * @brief 以内圈基线计算当前掉速电机数量和平均掉速
 * @param out_avg_dip 输出内圈平均掉速，单位为 deg/s
 * @return 当前超过掉速阈值的内圈电机数量
 */
static uint8_t GetInnerDipMetrics(float *out_avg_dip)
{
    uint8_t dip_count = 0;
    float dip_left = dip_control.inner_left_baseline - fabsf(GetMotorSpeedAps(friction_inner_left));
    float dip_right = dip_control.inner_right_baseline - fabsf(GetMotorSpeedAps(friction_inner_right));
    float dip_down = dip_control.inner_down_baseline - fabsf(GetMotorSpeedAps(friction_inner_down));

    if (out_avg_dip != NULL)
        *out_avg_dip = (dip_left + dip_right + dip_down) * 0.3333333f;

    if (dip_left > FRICTION_SPEED_DIP_THRESHOLD)
        dip_count++;
    if (dip_right > FRICTION_SPEED_DIP_THRESHOLD)
        dip_count++;
    if (dip_down > FRICTION_SPEED_DIP_THRESHOLD)
        dip_count++;

    return dip_count;
}

/**
 * @brief 以外圈基线计算当前掉速电机数量和平均掉速
 * @param out_avg_dip 输出外圈平均掉速，单位为 deg/s
 * @return 当前超过阈值的外圈电机数量
 */
static uint8_t GetOuterDipMetrics(float *out_avg_dip)
{
    uint8_t dip_count = 0;
    float dip_left;
    float dip_right;
    float dip_down;

    if (!HasOuterFrictionWheel()) {
        if (out_avg_dip != NULL)
            *out_avg_dip = 0.0f;
        return 0;
    }

    dip_left = dip_control.outer_left_baseline - fabsf(GetMotorSpeedAps(friction_outer_left));
    dip_right = dip_control.outer_right_baseline - fabsf(GetMotorSpeedAps(friction_outer_right));
    dip_down = dip_control.outer_down_baseline - fabsf(GetMotorSpeedAps(friction_outer_down));

    if (out_avg_dip != NULL)
        *out_avg_dip = (dip_left + dip_right + dip_down) * 0.3333333f;

    if (dip_left > OUTER_DIP_CONFIRM_THRESHOLD)
        dip_count++;
    if (dip_right > OUTER_DIP_CONFIRM_THRESHOLD)
        dip_count++;
    if (dip_down > OUTER_DIP_CONFIRM_THRESHOLD)
        dip_count++;

    return dip_count;
}

/**
 * @brief 判断当前掉速是否足以计为一发弹丸
 * @return 1 表示可以累计一次发射计数，0 表示当前掉速仍应视为扰动或尚未咬弹
 */
static uint8_t ShouldCountByFrictionDip(void)
{
    float inner_avg_dip = 0.0f;
    float outer_avg_dip = 0.0f;
    uint8_t dip_count;
    uint8_t outer_dip_count;

    if (GetCurrentFeedProgressBullet() < SF_MIN_VALID_DIP_PROGRESS_BULLET) {
        single_fire.inner_dip_stable_count = 0;
        return 0;
    }

    dip_count = GetInnerDipMetrics(&inner_avg_dip);
    if (dip_count >= DIP_MIN_MOTOR_COUNT) {
        single_fire.inner_dip_stable_count = 0;
        return 1;
    }

    outer_dip_count = GetOuterDipMetrics(&outer_avg_dip);

    if (inner_avg_dip > FRICTION_SPEED_DIP_THRESHOLD) {
        if (single_fire.inner_dip_stable_count < 0xFFu)
            single_fire.inner_dip_stable_count++;
    } else {
        single_fire.inner_dip_stable_count = 0;
    }

    if (single_fire.inner_dip_stable_count < SF_DIP_AVG_STABLE_CYCLES)
        return 0;

    if (!HasOuterFrictionWheel()) {
        // 外圈不存在时保留平均掉速确认链路，目的是当前工程允许裁剪外圈配置，辅助确认不能让无外圈平台完全失去出弹计数能力。
        return 1;
    }

    // 对“弱但连续”的内圈平均掉速增加外圈辅助确认，目的是减少半咬弹或随机扰动被误记为真实出弹。
    return (outer_dip_count >= 1u) || (outer_avg_dip > OUTER_DIP_CONFIRM_THRESHOLD);
}

/**
 * @brief 在检测到有效掉速时只累计发射数，不改变拨盘目标角度
 */
static void CountFiredBulletByDipIfNeeded(void)
{
    if (single_fire.shot_counted != 0u)
        return;

    if (!ShouldCountByFrictionDip())
        return;

    // 掉速抓拍仍然绑定到首次有效掉速，目的是调试数据和 fire_count 对应同一颗弹丸，但这里不再调用收口逻辑，避免摩擦轮掉速提前停止拨弹盘。
    TakeDipSnapshot();
    ValidateAndSaveDipSnapshot();
    single_fire.fire_count++;
    single_fire.shot_counted = 1u;
}

/**
 * @brief 接受一个待处理单发请求并进入待速态
 * @param current_time 当前系统时间，单位为 ms
 */
static void AcceptPendingSingleFireRequest(float current_time)
{
    fire_trigger.pending_fire = 0;
    single_fire.state = SF_WAIT_SPEED;
    single_fire.retry_count = 0;
    single_fire.shot_start_time = 0.0f;
    single_fire.feed_start_time = 0.0f;
    single_fire.retry_start_time = 0.0f;
    single_fire.inner_dip_stable_count = 0;
    single_fire.recover_stable_count = 0;
    single_fire.shot_counted = 0u;
    single_fire.lock_target_angle = loader->measure.total_angle;
    single_fire.brake_start_time = current_time;
    LoaderSetAngleRef(single_fire.lock_target_angle);
}

/**
 * @brief 中止单发状态机
 */
void AbortSingleFire(void)
{
    single_fire.state = SF_IDLE;
    single_fire.retry_count = 0;
    single_fire.shot_start_time = 0.0f;
    single_fire.feed_start_time = 0.0f;
    single_fire.retry_start_time = 0.0f;
    single_fire.inner_dip_stable_count = 0;
    single_fire.recover_stable_count = 0;
    single_fire.shot_counted = 0u;
    fire_trigger.pending_fire = 0;
    SetFrictionFeedforward(0.0f, 0.0f);
    ff_loader = 0.0f;
}

/**
 * @brief 判断单发事务是否处于历史补发兼容状态
 * @return 1 表示当前停留在旧补发状态或旧补发推进中，0 表示仍按固定 80 度事务自保持处理
 */
uint8_t SingleFireIsRetryActive(void)
{
    if (single_fire.state == SF_RETRYING)
        return 1;

    if (single_fire.state == SF_FEEDING && single_fire.retry_count > 0)
        return 1;

    return 0;
}

/**
 * @brief 开始一次单发喂弹尝试
 * @param current_time 当前系统时间 (ms)
 * @param feed_bullet_count 本次喂弹步距，单位为 bullet
 * @param reset_transaction_timer 1 表示重置整次事务计时，0 表示沿用已有计时
 */
static void BeginSingleFireFeedAttempt(float current_time, float feed_bullet_count, uint8_t reset_transaction_timer)
{
    single_fire.state = SF_FEEDING;
    single_fire.feed_start_time = current_time;
    if (reset_transaction_timer) {
        single_fire.shot_start_time = current_time;
    }
    single_fire.rush_start_angle = loader->measure.total_angle;
    single_fire.rush_target_angle = single_fire.rush_start_angle +
                                    LoaderBulletCountToMotorAngle(feed_bullet_count);
    single_fire.lock_target_angle = single_fire.rush_start_angle;
    single_fire.baseline_speed = GetInnerFrictionAvgSpeed();
    single_fire.outer_baseline_speed = GetOuterFrictionAvgSpeed();
    single_fire.inner_dip_stable_count = 0;
    if (reset_transaction_timer) {
        // 只有一轮全新的单发事务才清掉计数锁存，目的是同一发在固定 80 度行程内可能持续掉速多拍，但只能贡献一次 fire_count。
        single_fire.shot_counted = 0u;
    }

    RecordDipBaseline();
    RecordControlDipBaseline();
    SetFrictionFeedforward(0.0f, 0.0f);
    LoaderSetAngleRef(single_fire.rush_target_angle);
}

/**
 * @brief 更新回速稳定计数并返回是否已经恢复
 * @return 1 表示摩擦轮已连续稳定恢复，0 表示仍需等待
 */
static uint8_t ShootIsSpeedRecovered(void)
{
    if (IsAllFrictionStableAgainstTarget(FRICTION_SPEED_RECOVER_THRESHOLD)) {
        if (single_fire.recover_stable_count < 0xFFu)
            single_fire.recover_stable_count++;
    } else {
        single_fire.recover_stable_count = 0;
    }

    return single_fire.recover_stable_count >= FRICTION_RECOVER_STABLE_CYCLES;
}

/**
 * @brief 结束本次单发事务并锁住当前位置
 * @param current_time 当前系统时间 (ms)
 */
static void FinishSingleFire(float current_time)
{
    single_fire.brake_start_time = current_time;
    if (fabsf(single_fire.rush_target_angle - loader->measure.total_angle) < SF_RUSH_REACHED_TOLERANCE) {
        // 正常到位收口时继续锁住固定 80 度目标，目的是状态切到等待回速或锁角后仍让位置环补完整个机械行程，而不是把当前测量值当成新的提前停止点。
        single_fire.lock_target_angle = single_fire.rush_target_angle;
    } else {
        // 超时或历史兼容状态收口时锁住当前位置，目的是拨盘异常不到位时优先停止继续追目标，避免卡滞状态下长时间输出。
        single_fire.lock_target_angle = loader->measure.total_angle;
    }
    single_fire.inner_dip_stable_count = 0;
    single_fire.recover_stable_count = 0;
    // 单发事务结束时主动清空挂起请求，目的是当前策略明确禁止“上一发执行过程中顺延排队下一发”，否则一次点击仍可能被拆成连续两发。
    fire_trigger.pending_fire = 0;
    SetFrictionFeedforward(0.0f, 0.0f);
    ff_loader = 0.0f;

    if (single_fire.shot_counted != 0u) {
        // 已经由掉速确认过真实出弹时，事务结束后先进入回速等待；这里不再增加 fire_count，目的是计数只发生在掉速首次确认那一拍。
        single_fire.state = SF_WAIT_RECOVER;
    } else {
        // 固定 80 度行程结束仍未出现有效掉速时，按未确认出弹处理，目的是真正做到掉速只负责计数，而不是为了寻找掉速继续补发推进。
        single_fire.state = SF_LOCKING;
        single_fire.feed_timeout_count++;
    }

    LoaderSetAngleRef(single_fire.lock_target_angle);
}

/**
 * @brief 检测摩擦轮转速是否就绪 (均在误差范围内)
 * @return 1=就绪, 0=未就绪
 */
static uint8_t ShootIsSpeedReady(void)
{
    // 目标速度为 0 时直接视为就绪，目的是关闭摩擦轮或异常回退分支不应该被 ready 判定反向卡住。
    if (target_inner_left_deg_abs == 0.0f &&
        target_inner_right_deg_abs == 0.0f &&
        target_inner_down_deg_abs == 0.0f &&
        target_outer_left_deg_abs == 0.0f &&
        target_outer_right_deg_abs == 0.0f &&
        target_outer_down_deg_abs == 0.0f)
        return 1;

    return IsAllFrictionStableAgainstTarget(SHOOT_SPEED_READY_THRESHOLD);
}

/**
 * @brief 检测摩擦轮是否发生掉速 (双重检测: 内圈 OR 外圈)
 * @return 1 表示检测到掉速, 0 表示正常
 */
static uint8_t IsFrictionDipping(void)
{
    float current_inner = GetInnerFrictionAvgSpeed();
    float current_outer = GetOuterFrictionAvgSpeed();
    uint8_t inner_dip = (single_fire.baseline_speed - current_inner) > FRICTION_SPEED_DIP_THRESHOLD;
    uint8_t outer_dip = (single_fire.outer_baseline_speed - current_outer) > FRICTION_SPEED_DIP_THRESHOLD;

    return inner_dip || outer_dip;
}

/**
 * @brief 单发处理逻辑 (固定 80 度位置环送弹 + 掉速计数)
 * @param trigger_active 是否触发 (边沿信号)
 */
void HandleSingleFire(uint8_t trigger_active)
{
    float current_time = DWT_GetTimeline_ms();
    SingleFireDebug_s *p_sf = ShootDebug_GetSingleFirePtr();
    float inner_speed = GetInnerFrictionAvgSpeed();
    float outer_speed = GetOuterFrictionAvgSpeed();
    uint8_t has_pending_request = fire_trigger.pending_fire != 0u;

    switch (single_fire.state) {
    case SF_IDLE:
        if (has_pending_request) {
            // 空闲态有待处理请求时进入待速，目的是鼠标和 VT03 都是边沿输入，请求必须先缓存住，再等摩擦轮真正 ready 后执行。
            AcceptPendingSingleFireRequest(current_time);
        } else {
            LoaderSetSpeedRef(0.0f);
        }
        break;

    case SF_WAIT_SPEED:
        if (ShootIsSpeedReady()) {
            // 只有最终目标速度 ready 后才允许固定 80 度送弹，目的是保持发射能量稳定，同时确保拨盘目标只由机械行程决定。
            BeginSingleFireFeedAttempt(current_time, SF_RUSH_BULLET_COUNT, 1u);
        } else {
            // 待速期间只锁当前位置不再前推，目的是用户反馈“点一下先动一下”就是旧前推逻辑带来的预拨感。
            LoaderSetAngleRef(single_fire.lock_target_angle);
        }
        break;

    case SF_FEEDING:
        if ((current_time - single_fire.feed_start_time) < FRICTION_FEEDFORWARD_TIME) {
            // 冲刺初段给摩擦轮额外前馈，目的是拨盘位置环起步扭矩更猛时，摩擦轮需要提前补能量来压住掉速塌陷。
            SetFrictionFeedforward(FRICTION_FEEDFORWARD_CURRENT, 400.0f);
        } else {
            SetFrictionFeedforward(0.0f, 0.0f);
        }

        // 拨弹盘线性电流前馈：根据当前位置误差动态计算，
        // 误差大时提供更强推力以减轻 PID 负担，到位时自然衰减为零
        UpdateLoaderFeedforward();

        // 基准线随峰值更新，目的是掉速检测依赖“基准 - 当前”，若基线不跟峰值走就会把正常升速误判成掉速不足。
        if (inner_speed > single_fire.baseline_speed) {
            single_fire.baseline_speed = inner_speed;
        }
        if (outer_speed > single_fire.outer_baseline_speed) {
            single_fire.outer_baseline_speed = outer_speed;
        }
        UpdateControlDipPeakBaseline();

        ShootDebug_UpdatePeakBaseline(
            GetMotorSpeedAps(friction_inner_left),
            GetMotorSpeedAps(friction_inner_right),
            GetMotorSpeedAps(friction_inner_down),
            GetMotorSpeedAps(friction_outer_left),
            GetMotorSpeedAps(friction_outer_right),
            GetMotorSpeedAps(friction_outer_down));

        CountFiredBulletByDipIfNeeded();

        if ((single_fire.shot_start_time > 0.0f) &&
            ((current_time - single_fire.shot_start_time) > SF_TRANSACTION_TIMEOUT)) {
            // 固定 80 度事务超过总时限后直接收口，目的是拨盘异常不到位时不能为了等待掉速继续输出。
            FinishSingleFire(current_time);
        } else if (fabsf(single_fire.rush_target_angle - loader->measure.total_angle) < SF_RUSH_REACHED_TOLERANCE) {
            // 只要固定 80 度机械行程已经到位就结束本次拨弹，目的是摩擦轮掉速不再拥有停止或补发拨弹盘的控制权。
            FinishSingleFire(current_time);
        } else {
            LoaderSetAngleRef(single_fire.rush_target_angle);
        }
        break;

    case SF_RETRYING:
        // 新策略不再补发推进；保留这个兜底分支是为了旧状态残留时也能立即收口，而不是继续执行历史补发路径。
        FinishSingleFire(current_time);
        break;

    case SF_WAIT_RECOVER:
        if (ShootIsSpeedRecovered()) {
            // 回速完成后统一退回锁角保持，目的是当前需求是防双发优先，因此即便恢复期间出现新点击，也不能在这一拍自动续上一发。
            single_fire.state = SF_LOCKING;
        }
        LoaderSetAngleRef(single_fire.lock_target_angle);
        break;

    case SF_LOCKING:
        if (has_pending_request) {
            // 锁角态存在缓存请求时重新进入待速，目的是这样单发请求的消费点只在状态机内，行为更可预测。
            AcceptPendingSingleFireRequest(current_time);
        }
        LoaderSetAngleRef(single_fire.lock_target_angle);
        break;
    }

    // 调试信息在状态机执行后再写回，目的是单发状态可能在一次调用内切换，先写调试值会导致观察结果永远滞后一拍。
    p_sf->state = single_fire.state;
    p_sf->baseline_speed = single_fire.baseline_speed;
    p_sf->outer_baseline_speed = single_fire.outer_baseline_speed;
    p_sf->current_speed = GetInnerFrictionAvgSpeed();
    p_sf->current_outer_speed = GetOuterFrictionAvgSpeed();
    p_sf->loader_speed = loader->measure.speed_aps;
    p_sf->is_dipping = IsFrictionDipping();
    p_sf->speed_diff = single_fire.baseline_speed - p_sf->current_speed;
    p_sf->trigger_edge = trigger_active;
    p_sf->retry_count = single_fire.retry_count;
    p_sf->fire_count = single_fire.fire_count;
    p_sf->feed_timeout_count = single_fire.feed_timeout_count;
    p_sf->brake_start_time = single_fire.brake_start_time;
    p_sf->feed_start_time = single_fire.feed_start_time;
    p_sf->retry_start_time = single_fire.retry_start_time;
}
