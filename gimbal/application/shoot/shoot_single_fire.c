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
 * @brief 获取外圈摩擦轮掉速指标
 * @param out_avg_dip 输出外圈三电机平均掉速量 (deg/s)
 * @return 外圈掉速超过阈值的电机数量
 */
static uint8_t GetOuterDipMetrics(float *out_avg_dip)
{
    float dip_left = dip_control.outer_left_baseline - fabsf(GetMotorSpeedAps(friction_outer_left));
    float dip_right = dip_control.outer_right_baseline - fabsf(GetMotorSpeedAps(friction_outer_right));
    float dip_down = dip_control.outer_down_baseline - fabsf(GetMotorSpeedAps(friction_outer_down));
    uint8_t count = 0;

    *out_avg_dip = (dip_left + dip_right + dip_down) * 0.3333333f;

    if (dip_left > OUTER_DIP_CONFIRM_THRESHOLD)
        count++;
    if (dip_right > OUTER_DIP_CONFIRM_THRESHOLD)
        count++;
    if (dip_down > OUTER_DIP_CONFIRM_THRESHOLD)
        count++;

    return count;
}

/**
 * @brief 外圈掉速时累计发射数与抓拍，弹丸完整通过两级摩擦轮才计为一发
 */
static uint8_t CountFiredBulletByDipIfNeeded(float feed_progress)
{
    float outer_avg_dip = 0.0f;
    uint8_t outer_dip_count;

    if (single_fire.shot_counted != 0u)
        return 0u;

    if (feed_progress < LoaderBulletCountToMotorAngle(SF_MIN_VALID_DIP_PROGRESS_BULLET)) {
        // 掉速确认必须等拨盘至少推进到可能咬弹的位置，目的是把起步电流浪涌或摩擦轮自身抖动挡在“真实出弹”判定之外。
        single_fire.inner_dip_stable_count = 0u;
        return 0u;
    }

    outer_dip_count = GetOuterDipMetrics(&outer_avg_dip);

    // 至少1个外圈电机掉速超过阈值，或者外圈平均掉速超过阈值才确认出弹
    if (outer_dip_count < 1u && outer_avg_dip <= OUTER_DIP_CONFIRM_THRESHOLD)
        return 0u;

    TakeDipSnapshot();
    ValidateAndSaveDipSnapshot();
    single_fire.fire_count++;
    single_fire.shot_counted = 1u;
    return 1u;
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
    single_fire.preload_start_angle = loader->measure.total_angle;
    single_fire.preload_start_time = 0.0f;
    single_fire.preload_current_stable_count = 0u;
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
    single_fire.preload_start_angle = 0.0f;
    single_fire.preload_start_time = 0.0f;
    single_fire.preload_current_stable_count = 0u;
    single_fire.shot_counted = 0u;
    fire_trigger.pending_fire = 0;
    SetFrictionFeedforward(0.0f, 0.0f);
    ff_loader = 0.0f;
}

/**
 * @brief 判断单发事务是否处于历史补发兼容状态
 * @return 1 表示当前停留在旧补发状态或旧补发推进中，0 表示仍按固定一发事务自保持处理
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
    // 安全上限设为最大总行程，目的是一次事务最多走到上限就停止，防止空拨无限步进
    single_fire.rush_target_angle = single_fire.rush_start_angle +
                                    LoaderBulletCountToMotorAngle(SF_MAX_TOTAL_FEED_BULLET);
    // 首个增量目标只推进一小步，每步完成后检查掉速
    single_fire.increment_target_angle = single_fire.rush_start_angle +
                                         SF_INCREMENT_MOTOR_ANGLE;
    single_fire.lock_target_angle = single_fire.rush_start_angle;
    single_fire.baseline_speed = GetInnerFrictionAvgSpeed();
    single_fire.outer_baseline_speed = GetOuterFrictionAvgSpeed();
    single_fire.inner_dip_stable_count = 0;
    if (reset_transaction_timer) {
        // 只有一轮全新的单发事务才清掉计数锁存，目的是同一发在固定一发行程内可能持续掉速多拍，但只能贡献一次 fire_count。
        single_fire.shot_counted = 0u;
    }
    single_fire.preload_start_angle = single_fire.rush_start_angle;
    single_fire.preload_start_time = 0.0f;
    single_fire.preload_current_stable_count = 0u;

    RecordDipBaseline();
    RecordControlDipBaseline();
    SetFrictionFeedforward(0.0f, 0.0f);
    LoaderSetAngleRef(single_fire.increment_target_angle);
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
        // 正常到位收口时继续锁住固定一发目标，目的是状态切到等待回速或锁角后仍让位置环补完整个机械行程，而不是把当前测量值当成新的提前停止点。
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
        // 固定一发行程结束仍未出现有效掉速时，按未确认出弹处理，目的是真正做到掉速只负责计数，而不是为了寻找掉速继续补发推进。
        single_fire.state = SF_LOCKING;
        single_fire.feed_timeout_count++;
    }

    LoaderSetAngleRef(single_fire.lock_target_angle);
}

/**
 * @brief 获取拨盘电流绝对值
 * @return 拨盘反馈电流的绝对值，拨盘未初始化时返回 0
 */
static int16_t GetLoaderCurrentAbs(void)
{
    int16_t current;

    if (loader == NULL)
        return 0;

    current = loader->measure.real_current;
    return current >= 0 ? current : (int16_t)-current;
}

/**
 * @brief 获取拨盘速度绝对值
 * @return 拨盘速度绝对值，拨盘未初始化时返回 0
 */
static float GetLoaderSpeedAbs(void)
{
    if (loader == NULL)
        return 0.0f;

    return fabsf(loader->measure.speed_aps);
}

/**
 * @brief 出弹确认后进入下一发慢速预压
 * @param current_time 当前系统时间，单位为 ms
 */
static void BeginNextBulletPreload(float current_time)
{
    single_fire.state = SF_PRELOAD_NEXT;
    single_fire.preload_start_angle = loader->measure.total_angle;
    single_fire.preload_start_time = current_time;
    single_fire.preload_current_stable_count = 0u;
    single_fire.recover_stable_count = 0u;
    // 出弹后预压下一发时必须退出位置环大步进和速度前馈，目的是让拨盘只用低速轻顶去找机械限位，不再把下一发继续强行打进摩擦轮。
    single_fire.lock_target_angle = loader->measure.total_angle;
    SetFrictionFeedforward(0.0f, 0.0f);
    ff_loader = 0.0f;
    LoaderSetSpeedRef(SF_PRELOAD_SPEED_DPS);
}

/**
 * @brief 判断下一发预压是否已经抵到单发限制位置
 * @return 1 表示应停止预压并锁角，0 表示继续低速轻顶
 */
static uint8_t IsNextBulletPreloadLimited(void)
{
    int16_t loader_current_abs = GetLoaderCurrentAbs();
    float loader_speed_abs = GetLoaderSpeedAbs();

    if (loader_current_abs >= STALL_CURRENT_THRESHOLD)
        return 1u;

    if (loader_current_abs >= SF_PRELOAD_CURRENT_THRESHOLD &&
        loader_speed_abs <= SF_PRELOAD_SPEED_THRESHOLD) {
        if (single_fire.preload_current_stable_count < 0xFFu)
            single_fire.preload_current_stable_count++;
    } else {
        single_fire.preload_current_stable_count = 0u;
    }

    return single_fire.preload_current_stable_count >= SF_PRELOAD_CURRENT_STABLE_CYCLES;
}

/**
 * @brief 结束下一发预压并锁住当前位置
 * @param current_time 当前系统时间，单位为 ms
 */
static void FinishNextBulletPreload(float current_time)
{
    single_fire.brake_start_time = current_time;
    single_fire.lock_target_angle = loader->measure.total_angle;
    single_fire.preload_current_stable_count = 0u;
    single_fire.recover_stable_count = 0u;
    // 预压结束后切回位置环锁当前触限位置，目的是下一次单发从已经抵住限制位的机械状态起步，而不是继续速度环低速推压。
    single_fire.state = SF_WAIT_RECOVER;
    SetFrictionFeedforward(0.0f, 0.0f);
    ff_loader = 0.0f;
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
 * @brief 单发处理逻辑 (固定一发位置环送弹 + 掉速计数)
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
            HoldLoaderIdlePosition();
        }
        break;

    case SF_WAIT_SPEED:
        if (ShootIsSpeedReady()) {
            // 只有最终目标速度 ready 后才允许固定一发送弹，目的是保持发射能量稳定，同时确保拨盘目标只由机械行程决定。
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

        // 拨弹盘线性速度前馈：根据当前增量目标动态计算，
        // 误差大时提供更强推力以减轻 PID 负担，到位时自然衰减为零
        UpdateLoaderFeedforward();

        // 增量期间保证前馈速度不低于地板值，目的是位置环PID在小误差时输出不足，
        // 通过前馈地板维持高速，到位时容差内自动切换下一增量
        if (ff_loader < LOADER_FF_INCREMENT_FLOOR)
            ff_loader = LOADER_FF_INCREMENT_FLOOR;

        // 基准线随峰值更新，目的是掉速检测依赖”基准 - 当前”，若基线不跟峰值走就会把正常升速误判成掉速不足。
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

        if ((single_fire.shot_start_time > 0.0f) &&
            ((current_time - single_fire.shot_start_time) > SF_TRANSACTION_TIMEOUT)) {
            // 固定一发事务超过总时限后直接收口，目的是堵转强推策略只允许在短窗口内持续顶推，避免限制位紧张时长期追目标导致机构过载。
            FinishSingleFire(current_time);
        } else {
            float feed_progress = loader->measure.total_angle - single_fire.rush_start_angle;

            // 外圈掉速确认真实出弹后不再立即锁住当前位置，而是切到低速预压下一发，满足“下一颗弹丸慢慢抵到单发限制位置”的机械需求。
            if (CountFiredBulletByDipIfNeeded(feed_progress) != 0u) {
                BeginNextBulletPreload(current_time);
                break;
            }

            // 拨盘推进到最小有效咬弹行程后才允许外圈掉速触发停止，目的是忽略起步阶段电机启动电流浪涌导致的摩擦轮瞬时扰动。
            if (feed_progress >= LoaderBulletCountToMotorAngle(SF_MIN_VALID_DIP_PROGRESS_BULLET)) {
                float outer_avg_dip = 0.0f;
                uint8_t outer_dip_count = GetOuterDipMetrics(&outer_avg_dip);

                // 这里保留掉速停止兜底，目的是即使计数函数因为本拍已计数等原因没有切状态，也不能在确认出弹后继续推进送弹增量。
                if (outer_dip_count >= 1u || outer_avg_dip > OUTER_DIP_CONFIRM_THRESHOLD) {
                    BeginNextBulletPreload(current_time);
                    break;
                }
            }

            // 当前增量到位且无掉速时推进到下一增量目标
            if (fabsf(single_fire.increment_target_angle - loader->measure.total_angle) < SF_INCREMENT_REACHED_TOLERANCE) {
                float next_increment = single_fire.increment_target_angle + SF_INCREMENT_MOTOR_ANGLE;
                if (next_increment <= single_fire.rush_target_angle) {
                    single_fire.increment_target_angle = next_increment;
                }
                // 超出总行程安全上限时不再推进，等待超时兜底收口
            }

            LoaderSetAngleRef(single_fire.increment_target_angle);
        }
        break;

    case SF_RETRYING:
        // 新策略不再补发推进；保留这个兜底分支是为了旧状态残留时也能立即收口，而不是继续执行历史补发路径。
        FinishSingleFire(current_time);
        break;

    case SF_PRELOAD_NEXT:
        SetFrictionFeedforward(0.0f, 0.0f);
        ff_loader = 0.0f;
        if ((current_time - single_fire.preload_start_time) >= SF_PRELOAD_TIMEOUT_MS) {
            // 低速预压超过时间上限后直接锁住当前位置，目的是缺下一发或触限电流不明显时也不能让拨盘持续慢卷。
            FinishNextBulletPreload(current_time);
        } else if ((loader->measure.total_angle - single_fire.preload_start_angle) >=
                   LoaderBulletCountToMotorAngle(SF_PRELOAD_MAX_BULLET)) {
            // 电流触限漏检时最多补偿一个弹位，目的是保留“抵到限位”的机会，同时避免继续推过下一颗弹丸形成连发。
            FinishNextBulletPreload(current_time);
        } else if (IsNextBulletPreloadLimited()) {
            // 电流连续确认且速度已经降下来时认为下一发抵到单发限制位置，立即锁角等待摩擦轮回速。
            FinishNextBulletPreload(current_time);
        } else {
            // 未触限时保持低速速度环轻顶，目的是让下一颗弹丸靠机械阻力自然抵到限位，而不是用位置环大目标硬推。
            LoaderSetSpeedRef(SF_PRELOAD_SPEED_DPS);
        }
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
