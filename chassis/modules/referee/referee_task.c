/**
 * @file referee_task.c
 * @author OpenAI
 * @brief 底盘裁判 UI 正式任务，按生成器布局在官方选手端绘制并增量刷新主画面
 *
 * @version 1.0
 * @date 2026-03-27
 */
#include "referee_task.h"
#include "referee_UI.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "string.h"
#include "math.h"

#define UI_LAYER_MAIN 0u

#define UI_INIT_PACKET_GAP_MS 60u
#define UI_RUNTIME_PACKET_GAP_MS 25u
#define UI_MOVE_PERIOD_MS 50u
#define UI_DATA_PERIOD_MS 100u
#define UI_SELF_HEAL_DELAY_MS 1000u
#define UI_SHOOT_SPEED_STALE_MS 500u

#define UI_ENABLE_SELF_HEAL_REBUILD 0u // What: 默认关闭第二轮 delete-all 自愈重建；Why: 当前现场优先保证首轮 UI 稳定落图，避免排障阶段被额外全删重建干扰。
#define UI_RECOVERY_ENABLE_LABEL_STRINGS 1u // What: 默认保留标签字符串初始化；Why: 先把生成器静态语义层稳定显示出来，便于现场确认布局已生效。
#define UI_RECOVERY_ENABLE_STATE_STRINGS 1u // What: 默认保留 fric/RFID/cap 的 on/off 字符串；Why: 这部分是低风险单字符串包，能先验证状态显示链路。
#define UI_RECOVERY_ENABLE_MOVE_PACKET 1u // What: 默认启用正式运动组；Why: 当前需求已经明确需要 body 和 qiang 的动态图形在选手端实时生效。
#define UI_RECOVERY_ENABLE_DATA_PACKET 1u // What: 启用正式 Draw5 数据组；Why: 当前需求已经切换到数字动态 UI，需要把数值图元真正发到官方客户端。
#define UI_RECOVERY_ENABLE_BODY_ROTATION 1u // What: 默认启用 body 旋转；Why: 用户已要求恢复底盘陀螺仪驱动的车体动态可视化。
#define UI_RECOVERY_PERIODIC_REBUILD_MS 0u // What: 默认关闭周期性整页重建；Why: 静态 UI 一旦已经稳定显示，就不应再周期 delete-all 重建，否则选手端会出现肉眼可见的闪烁。

#define UI_BUFFER_FULL_SCALE_J 60.0f
#define UI_DEG_TO_RAD 0.01745329251994329577f
#define UI_COORD_MAX 2047u

#define UI_CROSSHAIR_START_X 585u
#define UI_CROSSHAIR_START_Y 653u
#define UI_CROSSHAIR_END_X 1330u
#define UI_CROSSHAIR_END_Y 658u
#define UI_CROSSHAIR_WIDTH 2u // What: 将中心上方黄色准星线改细；Why: 用户明确要求弱化该线条的视觉占比。

#define UI_BUFFER_START_X 563u
#define UI_BUFFER_START_Y 117u
#define UI_BUFFER_END_X_MAX 1395u
#define UI_BUFFER_END_Y 116u
#define UI_BUFFER_WIDTH 27u

#define UI_BODY_CENTER_X 1597.0f
#define UI_BODY_CENTER_Y 737.0f
#define UI_BODY_HALF_SIZE 30.0f // What: 将 body 纵向半长从 20 提到 40；Why: 用户要求车身长度再翻倍，让整体更接近长车身而不是短方块。
#define UI_BODY_FILL_WIDTH 17u // What: 将 body 填充线再次明显加宽；Why: 用户要求更激进的车身放大效果，让图标一眼看上去就是车体本身。

#define UI_QIANG_WIDTH 15u // What: 将 qiang 也略微加粗；Why: 用户要求 body 和 qiang 都更激进，枪管不能在更宽车身下显得太细。
#define UI_QIANG_BASE_START_X 0.0f // What: 让 qiang 起点落在 body 正中线上；Why: 用户明确要求更规整、更方正的几何关系。
#define UI_QIANG_BASE_START_Y 24.0f // What: 将 qiang 起点抬到更靠近车身上沿的位置；Why: 车身变长后若仍从过深位置起笔，会让枪管埋得太多、外露过少。
#define UI_QIANG_BASE_END_X 0.0f // What: 让 qiang 终点也保持在正中线上；Why: 避免旋转前就存在横向偏置，导致静态观感发歪。
#define UI_QIANG_BASE_END_Y 64.0f // What: 将 qiang 在当前基础上继续缩短；Why: 用户明确要求枪管再短一点，避免长车身下枪管比例失衡。

#define UI_PITCH_VALUE_X 344u
#define UI_PITCH_VALUE_Y 552u // What: 将 pitch 数值在现有基础上再上移两段共 24px；Why: 现场反馈仍然偏低，需要继续整体抬升以完全避开吞字区域。
#define UI_PITCH_VALUE_FONT 20u
#define UI_PITCH_VALUE_WIDTH 2u

#define UI_BULLET_SPEED_X 1462u
#define UI_BULLET_SPEED_Y 573u
#define UI_BULLET_SPEED_FONT 20u
#define UI_BULLET_SPEED_WIDTH 2u

#define UI_BULLET_NUM_X 1469u
#define UI_BULLET_NUM_Y 520u
#define UI_BULLET_NUM_FONT 20u
#define UI_BULLET_NUM_WIDTH 2u

#define UI_POWER_X 179u
#define UI_POWER_Y 237u
#define UI_POWER_FONT 20u
#define UI_POWER_WIDTH 2u

#define UI_RFID_LABEL_X 79u
#define UI_RFID_LABEL_Y 798u
#define UI_RFID_ON_X 208u
#define UI_RFID_ON_Y 800u
#define UI_RFID_OFF_X 291u
#define UI_RFID_OFF_Y 800u

#define UI_FRIC_LABEL_X 75u
#define UI_FRIC_LABEL_Y 736u
#define UI_FRIC_ON_X 204u
#define UI_FRIC_ON_Y 737u
#define UI_FRIC_OFF_X 290u
#define UI_FRIC_OFF_Y 736u

#define UI_CAP_LABEL_X 95u
#define UI_CAP_LABEL_Y 671u
#define UI_CAP_ON_X 206u
#define UI_CAP_ON_Y 678u
#define UI_CAP_OFF_X 289u
#define UI_CAP_OFF_Y 679u

#define UI_PITCH_LABEL_X 351u
#define UI_PITCH_LABEL_Y 599u // What: 将 pitch 标签在现有基础上再上移两段共 24px；Why: 需要与数值保持一致的整体抬升，避免标签和数值错位。

#define UI_STATUS_LABEL_FONT 27u
#define UI_STATUS_LABEL_WIDTH 3u
#define UI_PITCH_LABEL_FONT 20u
#define UI_PITCH_LABEL_WIDTH 2u
#define UI_DYNAMIC_FLOAT_DIGIT 1u // What: 统一让数字动态 UI 的浮点项显示 1 位小数；Why: pitch 必须带小数点，同时弹速和功率保持一致的显示风格。

typedef enum {
    UI_STATIC_FIGURE_CROSSHAIR = 0,
    UI_STATIC_FIGURE_COUNT,
} UIStaticFigureIndex_e;

typedef enum {
    UI_MOVE_FIGURE_BODY_0 = 0,
    UI_MOVE_FIGURE_BODY_1,
    UI_MOVE_FIGURE_BODY_2,
    UI_MOVE_FIGURE_BODY_3,
    UI_MOVE_FIGURE_QIANG,
    UI_MOVE_FIGURE_COUNT,
} UIMoveFigureIndex_e;

typedef enum {
    UI_DATA_FIGURE_PITCH_VALUE = 0,
    UI_DATA_FIGURE_BULLET_SPEED,
    UI_DATA_FIGURE_BULLET_NUM,
    UI_DATA_FIGURE_POWER,
    UI_DATA_FIGURE_BUFFER,
    UI_DATA_FIGURE_COUNT,
} UIDataFigureIndex_e;

typedef enum {
    UI_STRING_RFID_LABEL = 0,
    UI_STRING_FRIC_LABEL,
    UI_STRING_CAP_LABEL,
    UI_STRING_PITCH_LABEL,
    UI_STRING_RFID_ON,
    UI_STRING_RFID_OFF,
    UI_STRING_FRIC_ON,
    UI_STRING_FRIC_OFF,
    UI_STRING_CAP_ON,
    UI_STRING_CAP_OFF,
    UI_STRING_COUNT,
} UIStringIndex_e;

typedef enum {
    UI_INIT_STAGE_IDLE = 0,
    UI_INIT_STAGE_DELETE_ALL,
    UI_INIT_STAGE_DRAW_STATIC_FIGURES,
    UI_INIT_STAGE_DRAW_MOVE_FIGURES,
    UI_INIT_STAGE_DRAW_DATA_FIGURES,
    UI_INIT_STAGE_DRAW_STRING_0,
} UIInitStage_e;

typedef enum {
    UI_RUNTIME_PACKET_NONE = 0,
    UI_RUNTIME_PACKET_MOVE,
    UI_RUNTIME_PACKET_DATA,
} UIRuntimePacket_e;

typedef struct
{
    uint8_t rfid_on;
    uint8_t fric_on;
    uint8_t cap_on;
} UIIndicatorState_t;

typedef struct
{
    float pitch_deg;
    float body_relative_angle_deg;
    float qiang_relative_angle_deg;
    int32_t pitch_value_milli_deg;
    int32_t bullet_speed_milli_mps;
    int32_t power_milli_w;
    int32_t bullet_num;
    uint32_t buffer_end_x;
    UIIndicatorState_t indicator_state;
} UIDisplaySnapshot_t;

typedef struct
{
    UIInitStage_e init_stage;
    uint32_t last_packet_tick_ms;
    uint32_t next_move_due_tick_ms;
    uint32_t next_data_due_tick_ms;
    uint32_t next_rebuild_due_tick_ms;
    uint32_t self_heal_due_tick_ms;
    uint16_t dirty_string_mask;
    uint8_t init_cycle_count;
    uint8_t self_heal_wait_first_complete;
    uint8_t self_heal_rebuild_armed;
    Graph_Data_t last_move_figures[UI_MOVE_FIGURE_COUNT];
    Graph_Data_t last_data_figures[UI_DATA_FIGURE_COUNT];
    UIIndicatorState_t last_indicator_state;
} UIRuntime_t;

static Referee_Interactive_info_t *interactive_data = NULL; // What: 缓存 UI 实时数据入口；Why: 裁判任务只从这一处读取底盘和双板同步后的显示量，避免分散取数失配。
static referee_info_t *referee_recv_info = NULL; // What: 缓存裁判接收数据入口；Why: 客户端 ID、buffer、弹量和 RFID 状态都依赖裁判原始结构。
uint8_t UI_Seq = 0u; // What: 维持 0x0301 帧序号；Why: 现有 referee_UI.c 会直接消费该序号并写入协议头。

static UIRuntime_t ui_runtime;
static Graph_Data_t ui_static_figures[UI_STATIC_FIGURE_COUNT];
static Graph_Data_t ui_move_figures[UI_MOVE_FIGURE_COUNT];
static Graph_Data_t ui_data_figures[UI_DATA_FIGURE_COUNT];
static String_Data_t ui_strings[UI_STRING_COUNT];

static void DetermineRobotID(void);
static void UIRuntimeReset(void);
static void UIStartInitCycle(uint32_t now_tick_ms);
static void UIFinishInitCycle(uint32_t now_tick_ms);
static void UIAdvanceInitStage(uint32_t now_tick_ms);
static void UIBuildDisplaySnapshot(UIDisplaySnapshot_t *snapshot);
static void UIBuildStaticFigures(void);
static void UIBuildMoveFigures(uint32_t operate_type, const UIDisplaySnapshot_t *snapshot);
static void UIBuildDataFigures(uint32_t operate_type, const UIDisplaySnapshot_t *snapshot);
static void UIBuildAllStrings(uint32_t operate_type, const UIIndicatorState_t *indicator_state);
static void UIRefreshIndicatorChanges(const UIIndicatorState_t *indicator_state);
static void UIProcessRuntimeUpdate(uint32_t now_tick_ms);
static uint8_t UIStringInitStageEnabled(uint8_t string_index);
static uint8_t UISendNextDirtyString(uint32_t now_tick_ms);
static void UISendMovePacket(uint32_t now_tick_ms);
static void UISendDataPacket(uint32_t now_tick_ms);
static uint8_t UIMovePacketIsDirty(const UIDisplaySnapshot_t *snapshot);
static uint8_t UIDataPacketIsDirty(const UIDisplaySnapshot_t *snapshot);
static UIRuntimePacket_e UISelectRuntimePacket(uint32_t now_tick_ms, uint8_t move_dirty, uint8_t data_dirty);
static uint8_t UIIsRFIDActive(void);
static float UIClampFloat(float value, float min_value, float max_value);
static int32_t UIRoundFloatToInt(float value);
static uint32_t UIClampCoord(int32_t coord_value);
static void UIRotateRelativePoint(float base_x, float base_y, float angle_deg, int32_t *out_x, int32_t *out_y);

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    // What: 初始化裁判接收模块并缓存 UI 数据指针；Why: 正式 UI 仍必须沿用原有任务入口，避免改坏 chassis 现有启动链路。
    referee_recv_info = RefereeInit(referee_usart_handle);
    interactive_data = UI_data;
    referee_recv_info->init_flag = 1u;
    return referee_recv_info;
}

void MyUIInit(void)
{
    if (referee_recv_info == NULL || interactive_data == NULL || referee_recv_info->init_flag == 0u) {
        // What: 初始化失败时主动删除 UI 任务；Why: 若基础数据入口未建立，继续发包只会制造误导性链路现象。
        vTaskDelete(NULL);
    }

    while (referee_recv_info->GameRobotState.robot_id == 0u) {
        // What: 等待裁判系统给出本机 robot_id；Why: 客户端 ID 依赖 robot_id，未拿到前绘图一定发不到正确选手端。
        osDelay(100);
    }

    DetermineRobotID();
    UIRuntimeReset();
    UIStartInitCycle(HAL_GetTick());

    LOGINFO("[ui] robot_id:%u client_id:0x%04X",
            (unsigned int)referee_recv_info->referee_id.Robot_ID,
            (unsigned int)referee_recv_info->referee_id.Cilent_ID);
}

void UITask(void)
{
    const uint32_t now_tick_ms = HAL_GetTick();

    if (referee_recv_info == NULL || interactive_data == NULL) {
        return;
    }

    if (ui_runtime.init_stage != UI_INIT_STAGE_IDLE) {
        if ((now_tick_ms - ui_runtime.last_packet_tick_ms) >= UI_INIT_PACKET_GAP_MS) {
            // What: 初始化阶段按较宽松的 60ms 间隔逐包建图；Why: 首次 ADD 链路更敏感，放慢节奏能减少客户端丢图层概率。
            UIAdvanceInitStage(now_tick_ms);
        }
        return;
    }

    if (UI_ENABLE_SELF_HEAL_REBUILD != 0u &&
        ui_runtime.self_heal_rebuild_armed != 0u && now_tick_ms >= ui_runtime.self_heal_due_tick_ms) {
        // What: 首轮建图完成后再做一次整页重建；Why: 若某个 ADD 包首轮丢失，补建一次能快速自愈而不引入长期全量刷新。
        ui_runtime.self_heal_rebuild_armed = 0u;
        UIStartInitCycle(now_tick_ms);
        return;
    }

    if (UI_RECOVERY_PERIODIC_REBUILD_MS != 0u &&
        now_tick_ms >= ui_runtime.next_rebuild_due_tick_ms) {
        // What: 在恢复模式下定时触发一次低频整页重建；Why: 这样即便客户端晚于上电进入选手端画面，也能再次收到 ADD 包而不是永远空白。
        LOGINFO("[ui] periodic_rebuild");
        ui_runtime.next_rebuild_due_tick_ms = now_tick_ms + UI_RECOVERY_PERIODIC_REBUILD_MS;
        UIStartInitCycle(now_tick_ms);
        return;
    }

    UIProcessRuntimeUpdate(now_tick_ms);
}

static void DetermineRobotID(void)
{
    const uint16_t robot_id = referee_recv_info->GameRobotState.robot_id;

    // What: 按 2026 协议附录直接计算官方选手端 ID；Why: 当前车种属于英雄/工程/步兵/空中，客户端 ID 与 robot_id 一一对应。
    referee_recv_info->referee_id.Robot_Color = (robot_id >= 100u) ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = robot_id;
    referee_recv_info->referee_id.Cilent_ID = (uint16_t)(0x0100u + robot_id);
    referee_recv_info->referee_id.Receiver_Robot_ID = 0u;
}

static void UIRuntimeReset(void)
{
    UIDisplaySnapshot_t snapshot;

    memset(&ui_runtime, 0, sizeof(ui_runtime));
    memset(ui_static_figures, 0, sizeof(ui_static_figures));
    memset(ui_move_figures, 0, sizeof(ui_move_figures));
    memset(ui_data_figures, 0, sizeof(ui_data_figures));
    memset(ui_strings, 0, sizeof(ui_strings));

    UI_Seq = 0u;
    ui_runtime.last_packet_tick_ms = HAL_GetTick();
    ui_runtime.next_rebuild_due_tick_ms = HAL_GetTick() + UI_RECOVERY_PERIODIC_REBUILD_MS;
    ui_runtime.self_heal_wait_first_complete = (uint8_t)UI_ENABLE_SELF_HEAL_REBUILD; // What: 按恢复开关决定是否允许首轮建图后的自愈；Why: 排障阶段需要可预测的单轮建图行为。

    // What: 复位后先同步一次当前状态快照；Why: 这样初始化建图时就能直接用实时状态，不会先闪一下占位数据。
    UIBuildDisplaySnapshot(&snapshot);
    ui_runtime.last_indicator_state = snapshot.indicator_state;
}

static void UIStartInitCycle(uint32_t now_tick_ms)
{
    // What: 每次整页建图都从 delete all 开始；Why: 这样能把客户端状态拉回已知初值，避免旧对象残留影响本轮结果。
    ui_runtime.init_stage = UI_INIT_STAGE_DELETE_ALL;
    ui_runtime.last_packet_tick_ms = now_tick_ms - UI_INIT_PACKET_GAP_MS;
    ui_runtime.dirty_string_mask = 0u;
    ui_runtime.init_cycle_count++;

    LOGINFO("[ui] init_cycle:%u start client:0x%04X label:%u state:%u move:%u data:%u self_heal:%u",
            (unsigned int)ui_runtime.init_cycle_count,
            (unsigned int)referee_recv_info->referee_id.Cilent_ID,
            (unsigned int)UI_RECOVERY_ENABLE_LABEL_STRINGS,
            (unsigned int)UI_RECOVERY_ENABLE_STATE_STRINGS,
            (unsigned int)UI_RECOVERY_ENABLE_MOVE_PACKET,
            (unsigned int)UI_RECOVERY_ENABLE_DATA_PACKET,
            (unsigned int)UI_ENABLE_SELF_HEAL_REBUILD);
}

static void UIFinishInitCycle(uint32_t now_tick_ms)
{
    UIDisplaySnapshot_t snapshot;

    // What: 建图完成后立即准备运行期基线数据；Why: 这样进入增量刷新时不会把刚 ADD 的对象再误判成“首拍全脏”。
    ui_runtime.init_stage = UI_INIT_STAGE_IDLE;
    ui_runtime.last_packet_tick_ms = now_tick_ms;
    ui_runtime.next_move_due_tick_ms = now_tick_ms + UI_MOVE_PERIOD_MS;
    ui_runtime.next_data_due_tick_ms = now_tick_ms + UI_DATA_PERIOD_MS;
    ui_runtime.next_rebuild_due_tick_ms = now_tick_ms + UI_RECOVERY_PERIODIC_REBUILD_MS;

    UIBuildDisplaySnapshot(&snapshot);
    if (UI_RECOVERY_ENABLE_MOVE_PACKET != 0u) {
        // What: 仅在启用运动组时建立运行期基线；Why: 被禁用的高风险包不应参与后续脏检查，避免误发 change。
        UIBuildMoveFigures(UI_Graph_Change, &snapshot);
        memcpy(ui_runtime.last_move_figures, ui_move_figures, sizeof(ui_runtime.last_move_figures));
    } else {
        memset(ui_runtime.last_move_figures, 0, sizeof(ui_runtime.last_move_figures));
    }

    if (UI_RECOVERY_ENABLE_DATA_PACKET != 0u) {
        // What: 仅在启用数据组时建立运行期基线；Why: 这样排障阶段可以把数字图元完全从刷新链里摘掉。
        UIBuildDataFigures(UI_Graph_Change, &snapshot);
        memcpy(ui_runtime.last_data_figures, ui_data_figures, sizeof(ui_runtime.last_data_figures));
    } else {
        memset(ui_runtime.last_data_figures, 0, sizeof(ui_runtime.last_data_figures));
    }
    ui_runtime.last_indicator_state = snapshot.indicator_state;
    ui_runtime.dirty_string_mask = 0u;

    if (UI_ENABLE_SELF_HEAL_REBUILD != 0u && ui_runtime.self_heal_wait_first_complete != 0u) {
        // What: 首轮建图完成后挂起一次延迟自愈重建；Why: 比起长期周期性全量刷新，这种一次性补建更省预算且足够兜底。
        ui_runtime.self_heal_wait_first_complete = 0u;
        ui_runtime.self_heal_rebuild_armed = 1u;
        ui_runtime.self_heal_due_tick_ms = now_tick_ms + UI_SELF_HEAL_DELAY_MS;
    } else {
        // What: 在恢复模式下显式清掉自愈状态；Why: 防止旧状态残留导致后续任务再次触发全删重建。
        ui_runtime.self_heal_wait_first_complete = 0u;
        ui_runtime.self_heal_rebuild_armed = 0u;
    }

    LOGINFO("[ui] init_cycle:%u done",
            (unsigned int)ui_runtime.init_cycle_count);
}

static void UIAdvanceInitStage(uint32_t now_tick_ms)
{
    UIDisplaySnapshot_t snapshot;
    uint8_t string_index = 0u;

    UIBuildDisplaySnapshot(&snapshot);

    switch (ui_runtime.init_stage) {
    case UI_INIT_STAGE_DELETE_ALL:
        LOGINFO("[ui] init_stage:delete_all");
        UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0u);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_STATIC_FIGURES;
        break;

    case UI_INIT_STAGE_DRAW_STATIC_FIGURES:
        LOGINFO("[ui] init_stage:draw_static Draw1");
        UIBuildStaticFigures();
        UIGraphRefresh(&referee_recv_info->referee_id, 1,
                       ui_static_figures[UI_STATIC_FIGURE_CROSSHAIR]);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_MOVE_FIGURES;
        break;

    case UI_INIT_STAGE_DRAW_MOVE_FIGURES:
        if (UI_RECOVERY_ENABLE_MOVE_PACKET == 0u) {
            // What: 恢复模式下直接跳过正式 Draw5 运动组；Why: 先保住静态和字符串显示，再单独验证动态图元包。
            LOGINFO("[ui] init_stage:skip_move Draw5");
            ui_runtime.init_stage = UI_INIT_STAGE_DRAW_DATA_FIGURES;
            break;
        }
        LOGINFO("[ui] init_stage:draw_move Draw5");
        UIBuildMoveFigures(UI_Graph_ADD, &snapshot);
        UIGraphRefresh(&referee_recv_info->referee_id, 5,
                       ui_move_figures[UI_MOVE_FIGURE_BODY_0],
                       ui_move_figures[UI_MOVE_FIGURE_BODY_1],
                       ui_move_figures[UI_MOVE_FIGURE_BODY_2],
                       ui_move_figures[UI_MOVE_FIGURE_BODY_3],
                       ui_move_figures[UI_MOVE_FIGURE_QIANG]);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_DATA_FIGURES;
        break;

    case UI_INIT_STAGE_DRAW_DATA_FIGURES:
        if (UI_RECOVERY_ENABLE_DATA_PACKET == 0u) {
            // What: 恢复模式下直接跳过正式 Draw5 数据组；Why: 先把数值图元从问题空间里剥离出来，确认字符串链路正常。
            LOGINFO("[ui] init_stage:skip_data Draw5");
            ui_runtime.init_stage = UI_INIT_STAGE_DRAW_STRING_0;
            break;
        }
        LOGINFO("[ui] init_stage:draw_data Draw5");
        UIBuildDataFigures(UI_Graph_ADD, &snapshot);
        UIGraphRefresh(&referee_recv_info->referee_id, 5,
                       ui_data_figures[UI_DATA_FIGURE_PITCH_VALUE],
                       ui_data_figures[UI_DATA_FIGURE_BULLET_SPEED],
                       ui_data_figures[UI_DATA_FIGURE_BULLET_NUM],
                       ui_data_figures[UI_DATA_FIGURE_POWER],
                       ui_data_figures[UI_DATA_FIGURE_BUFFER]);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_STRING_0;
        break;

    default:
        if (ui_runtime.init_stage >= UI_INIT_STAGE_DRAW_STRING_0 &&
            ui_runtime.init_stage < (UI_INIT_STAGE_DRAW_STRING_0 + UI_STRING_COUNT)) {
            string_index = (uint8_t)(ui_runtime.init_stage - UI_INIT_STAGE_DRAW_STRING_0);

            if (UIStringInitStageEnabled(string_index) == 0u) {
                // What: 根据恢复分层开关跳过不需要的字符串对象；Why: 这样可以把 labels 和 on/off 两类字符串分开验证。
                LOGINFO("[ui] init_stage:skip_string idx:%u", (unsigned int)string_index);
                if (string_index + 1u >= UI_STRING_COUNT) {
                    UIFinishInitCycle(now_tick_ms);
                } else {
                    ui_runtime.init_stage = (UIInitStage_e)(ui_runtime.init_stage + 1);
                }
                break;
            }

            UIBuildAllStrings(UI_Graph_ADD, &snapshot.indicator_state);
            LOGINFO("[ui] init_stage:draw_string idx:%u", (unsigned int)string_index);
            UICharRefresh(&referee_recv_info->referee_id, ui_strings[string_index]);

            if (string_index + 1u >= UI_STRING_COUNT) {
                UIFinishInitCycle(now_tick_ms);
            } else {
                ui_runtime.init_stage = (UIInitStage_e)(ui_runtime.init_stage + 1);
            }
        } else {
            UIFinishInitCycle(now_tick_ms);
        }
        break;
    }

    ui_runtime.last_packet_tick_ms = now_tick_ms;
}

static uint8_t UIStringInitStageEnabled(uint8_t string_index)
{
    if (string_index <= UI_STRING_PITCH_LABEL) {
        // What: 前四个字符串属于标签层；Why: 恢复模式需要先验证静态标签是否稳定落图。
        return (uint8_t)UI_RECOVERY_ENABLE_LABEL_STRINGS;
    }

    // What: 其余字符串属于状态 on/off 层；Why: 这些对象要与标签层分离开，便于定位状态字符串是否会触发客户端拒收。
    return (uint8_t)UI_RECOVERY_ENABLE_STATE_STRINGS;
}

static void UIBuildDisplaySnapshot(UIDisplaySnapshot_t *snapshot)
{
    float clamped_pitch = 0.0f;
    float shoot_speed_mps = 0.0f;
    float power_w = 0.0f;
    float buffer_ratio = 0.0f;

    if (snapshot == NULL) {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));

    // What: 让 qiang 固定朝上并用底盘相对云台偏角驱动 body；Why: 用户要求显示的是云台与底盘的相对位置，而不是两者在世界坐标中的各自朝向。
    snapshot->body_relative_angle_deg = theta_format(-interactive_data->chassis_gimbal_offset_deg);
    snapshot->qiang_relative_angle_deg = 0.0f;

    // What: 把云台真实 pitch 严格约束到机构限位；Why: 数字 UI 也必须与实机上限一致，避免显示越界角度。
    clamped_pitch = UIClampFloat(interactive_data->gimbal_pitch_deg, (float)PITCH_MIN_ANGLE, (float)PITCH_MAX_ANGLE);
    snapshot->pitch_deg = clamped_pitch;
    // What: 浮点数字严格按协议缩放到 milli 单位；Why: 2026 协议规定客户端显示值等于输入 int32 / 1000。
    snapshot->pitch_value_milli_deg = UIRoundFloatToInt(clamped_pitch * 1000.0f);

    if (referee_recv_info->last_shoot_data_tick_ms != 0u &&
        (HAL_GetTick() - referee_recv_info->last_shoot_data_tick_ms) <= UI_SHOOT_SPEED_STALE_MS) {
        // What: 仅在最近确实有 0x0207 射击数据时显示弹速；Why: 停火后若不超时清零，选手端会长期残留上一发的旧速度。
        shoot_speed_mps = referee_recv_info->ShootData.initial_speed;
    }
    snapshot->bullet_speed_milli_mps = UIRoundFloatToInt(shoot_speed_mps * 1000.0f);

    power_w = interactive_data->chassis_power_w;
    snapshot->power_milli_w = UIRoundFloatToInt(power_w * 1000.0f);
    snapshot->bullet_num = (int32_t)referee_recv_info->ProjectileAllowance.projectile_allowance_17mm;

    buffer_ratio = (float)referee_recv_info->PowerHeatData.buffer_energy / UI_BUFFER_FULL_SCALE_J;
    buffer_ratio = UIClampFloat(buffer_ratio, 0.0f, 1.0f);
    snapshot->buffer_end_x = UIClampCoord((int32_t)(UI_BUFFER_START_X + buffer_ratio * (float)(UI_BUFFER_END_X_MAX - UI_BUFFER_START_X)));

    snapshot->indicator_state.rfid_on = UIIsRFIDActive();
    snapshot->indicator_state.fric_on = (interactive_data->friction_on != 0u) ? 1u : 0u;
    snapshot->indicator_state.cap_on = (interactive_data->cap_on != 0u) ? 1u : 0u;
}

static void UIBuildStaticFigures(void)
{
    // What: 仅保留准星横线这一项静态图元；Why: 用户明确要求取消 pitch 导轨和滑块，静态区不再保留该槽体。
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_CROSSHAIR], "jzx", UI_Graph_ADD, UI_LAYER_MAIN, UI_Color_Yellow, UI_CROSSHAIR_WIDTH,
               UI_CROSSHAIR_START_X, UI_CROSSHAIR_START_Y, UI_CROSSHAIR_END_X, UI_CROSSHAIR_END_Y);
}

static void UIBuildMoveFigures(uint32_t operate_type, const UIDisplaySnapshot_t *snapshot)
{
    static const float body_fill_center_x[4] = { -24.0f, -8.0f, 8.0f, 24.0f }; // What: 将 4 条填充线横向再大步外扩；Why: 用户要求更激进地放大 body，需要让车身宽度一眼可见地继续增加。
    int32_t body_start_x[4];
    int32_t body_start_y[4];
    int32_t body_end_x[4];
    int32_t body_end_y[4];
    int32_t qiang_start_x = 0;
    int32_t qiang_start_y = 0;
    int32_t qiang_end_x = 0;
    int32_t qiang_end_y = 0;
    uint8_t i = 0u;

    if (snapshot == NULL) {
        return;
    }

    // What: 将 body 改成 4 条并排填充线组成的近似实心方块；Why: 裁判客户端不支持旋转实心矩形，只能用可旋转粗线去逼近参考图效果。
    for (i = 0u; i < 4u; i++) {
        if (UI_RECOVERY_ENABLE_BODY_ROTATION != 0u) {
            UIRotateRelativePoint(body_fill_center_x[i], -UI_BODY_HALF_SIZE, snapshot->body_relative_angle_deg, &body_start_x[i], &body_start_y[i]);
            UIRotateRelativePoint(body_fill_center_x[i], UI_BODY_HALF_SIZE, snapshot->body_relative_angle_deg, &body_end_x[i], &body_end_y[i]);
        } else {
            body_start_x[i] = UIRoundFloatToInt(body_fill_center_x[i]);
            body_start_y[i] = UIRoundFloatToInt(-UI_BODY_HALF_SIZE);
            body_end_x[i] = UIRoundFloatToInt(body_fill_center_x[i]);
            body_end_y[i] = UIRoundFloatToInt(UI_BODY_HALF_SIZE);
        }
    }

    for (i = 0u; i < 4u; i++) {
        UILineDraw(&ui_move_figures[UI_MOVE_FIGURE_BODY_0 + i], (char[4]){ 'b', '0' + (char)i, '0', '\0' },
                   operate_type, UI_LAYER_MAIN, UI_Color_White, UI_BODY_FILL_WIDTH,
                   UIClampCoord((int32_t)(UI_BODY_CENTER_X + (float)body_start_x[i])),
                   UIClampCoord((int32_t)(UI_BODY_CENTER_Y + (float)body_start_y[i])),
                   UIClampCoord((int32_t)(UI_BODY_CENTER_X + (float)body_end_x[i])),
                   UIClampCoord((int32_t)(UI_BODY_CENTER_Y + (float)body_end_y[i])));
    }

    // What: 枪管固定在云台参考系的正上方；Why: 当前 UI 需要把云台当作参考坐标，让车身只显示相对偏角。
    UIRotateRelativePoint(UI_QIANG_BASE_START_X, UI_QIANG_BASE_START_Y, snapshot->qiang_relative_angle_deg, &qiang_start_x, &qiang_start_y);
    UIRotateRelativePoint(UI_QIANG_BASE_END_X, UI_QIANG_BASE_END_Y, snapshot->qiang_relative_angle_deg, &qiang_end_x, &qiang_end_y);
    UILineDraw(&ui_move_figures[UI_MOVE_FIGURE_QIANG], "qg0", operate_type, UI_LAYER_MAIN, UI_Color_Cyan, UI_QIANG_WIDTH,
               UIClampCoord((int32_t)(UI_BODY_CENTER_X + (float)qiang_start_x)),
               UIClampCoord((int32_t)(UI_BODY_CENTER_Y + (float)qiang_start_y)),
               UIClampCoord((int32_t)(UI_BODY_CENTER_X + (float)qiang_end_x)),
               UIClampCoord((int32_t)(UI_BODY_CENTER_Y + (float)qiang_end_y)));
}

static void UIBuildDataFigures(uint32_t operate_type, const UIDisplaySnapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    // What: 复用现有浮点/整型图元接口承接生成器数值位；Why: 这样无需引入第二套封包器，也能完全落到现有裁判 UI 发包链上。
    UIFloatDraw(&ui_data_figures[UI_DATA_FIGURE_PITCH_VALUE], "pva", operate_type, UI_LAYER_MAIN, UI_Color_Green,
                UI_PITCH_VALUE_FONT, UI_DYNAMIC_FLOAT_DIGIT, UI_PITCH_VALUE_WIDTH, UI_PITCH_VALUE_X, UI_PITCH_VALUE_Y, snapshot->pitch_value_milli_deg);
    UIFloatDraw(&ui_data_figures[UI_DATA_FIGURE_BULLET_SPEED], "bsp", operate_type, UI_LAYER_MAIN, UI_Color_Yellow,
                UI_BULLET_SPEED_FONT, UI_DYNAMIC_FLOAT_DIGIT, UI_BULLET_SPEED_WIDTH, UI_BULLET_SPEED_X, UI_BULLET_SPEED_Y, snapshot->bullet_speed_milli_mps);
    UIIntDraw(&ui_data_figures[UI_DATA_FIGURE_BULLET_NUM], "bnu", operate_type, UI_LAYER_MAIN, UI_Color_Green,
              UI_BULLET_NUM_FONT, UI_BULLET_NUM_WIDTH, UI_BULLET_NUM_X, UI_BULLET_NUM_Y, snapshot->bullet_num);
    UIFloatDraw(&ui_data_figures[UI_DATA_FIGURE_POWER], "pwr", operate_type, UI_LAYER_MAIN, UI_Color_Orange,
                UI_POWER_FONT, UI_DYNAMIC_FLOAT_DIGIT, UI_POWER_WIDTH, UI_POWER_X, UI_POWER_Y, snapshot->power_milli_w);

    // What: 将 buffer 能量条并入数据组；Why: 去掉中间红线后，buffer 正好补进合法 Draw5 包且继续保留动态显示。
    UILineDraw(&ui_data_figures[UI_DATA_FIGURE_BUFFER], "buf", operate_type, UI_LAYER_MAIN, UI_Color_Green, UI_BUFFER_WIDTH,
               UI_BUFFER_START_X, UI_BUFFER_START_Y, snapshot->buffer_end_x, UI_BUFFER_END_Y);
}

static void UIBuildAllStrings(uint32_t operate_type, const UIIndicatorState_t *indicator_state)
{
    const char *rfid_on_text = "  ";
    const char *rfid_off_text = "off";
    const char *fric_on_text = "  ";
    const char *fric_off_text = "off";
    const char *cap_on_text = "  ";
    const char *cap_off_text = "off";

    if (indicator_state == NULL) {
        return;
    }

    if (indicator_state->rfid_on != 0u) {
        rfid_on_text = "on";
        rfid_off_text = "   ";
    }

    if (indicator_state->fric_on != 0u) {
        fric_on_text = "on";
        fric_off_text = "   ";
    }

    if (indicator_state->cap_on != 0u) {
        cap_on_text = "on";
        cap_off_text = "   ";
    }

    // What: labels 保持固定文本和坐标；Why: 这些元素只承担语义定位，不参与任何运行时动画。
    UICharDraw(&ui_strings[UI_STRING_RFID_LABEL], "rfl", operate_type, UI_LAYER_MAIN, UI_Color_White,
               UI_STATUS_LABEL_FONT, UI_STATUS_LABEL_WIDTH, UI_RFID_LABEL_X, UI_RFID_LABEL_Y, "RFID");
    UICharDraw(&ui_strings[UI_STRING_FRIC_LABEL], "frl", operate_type, UI_LAYER_MAIN, UI_Color_White,
               UI_STATUS_LABEL_FONT, UI_STATUS_LABEL_WIDTH, UI_FRIC_LABEL_X, UI_FRIC_LABEL_Y, "fric");
    UICharDraw(&ui_strings[UI_STRING_CAP_LABEL], "cal", operate_type, UI_LAYER_MAIN, UI_Color_White,
               UI_STATUS_LABEL_FONT, UI_STATUS_LABEL_WIDTH, UI_CAP_LABEL_X, UI_CAP_LABEL_Y, "cap");
    UICharDraw(&ui_strings[UI_STRING_PITCH_LABEL], "ptl", operate_type, UI_LAYER_MAIN, UI_Color_Cyan,
               UI_PITCH_LABEL_FONT, UI_PITCH_LABEL_WIDTH, UI_PITCH_LABEL_X, UI_PITCH_LABEL_Y, "pitch");

    // What: on/off 直接切换字符串内容；Why: 这样是真正的“开显示 on、关显示 off”，不会依赖黑色隐藏在不同背景下失效。
    UICharDraw(&ui_strings[UI_STRING_RFID_ON], "ron", operate_type, UI_LAYER_MAIN, UI_Color_Green,
               UI_STATUS_LABEL_FONT, UI_STATUS_LABEL_WIDTH, UI_RFID_ON_X, UI_RFID_ON_Y, "%s", rfid_on_text);
    UICharDraw(&ui_strings[UI_STRING_RFID_OFF], "rof", operate_type, UI_LAYER_MAIN, UI_Color_Main,
               UI_STATUS_LABEL_FONT, UI_STATUS_LABEL_WIDTH, UI_RFID_OFF_X, UI_RFID_OFF_Y, "%s", rfid_off_text);

    UICharDraw(&ui_strings[UI_STRING_FRIC_ON], "fon", operate_type, UI_LAYER_MAIN, UI_Color_Green,
               UI_STATUS_LABEL_FONT, UI_STATUS_LABEL_WIDTH, UI_FRIC_ON_X, UI_FRIC_ON_Y, "%s", fric_on_text);
    UICharDraw(&ui_strings[UI_STRING_FRIC_OFF], "fof", operate_type, UI_LAYER_MAIN, UI_Color_Main,
               UI_STATUS_LABEL_FONT, UI_STATUS_LABEL_WIDTH, UI_FRIC_OFF_X, UI_FRIC_OFF_Y, "%s", fric_off_text);

    UICharDraw(&ui_strings[UI_STRING_CAP_ON], "con", operate_type, UI_LAYER_MAIN, UI_Color_Green,
               UI_STATUS_LABEL_FONT, UI_STATUS_LABEL_WIDTH, UI_CAP_ON_X, UI_CAP_ON_Y, "%s", cap_on_text);
    UICharDraw(&ui_strings[UI_STRING_CAP_OFF], "cof", operate_type, UI_LAYER_MAIN, UI_Color_Main,
               UI_STATUS_LABEL_FONT, UI_STATUS_LABEL_WIDTH, UI_CAP_OFF_X, UI_CAP_OFF_Y, "%s", cap_off_text);
}

static void UIRefreshIndicatorChanges(const UIIndicatorState_t *indicator_state)
{
    uint16_t dirty_mask = 0u;

    if (indicator_state == NULL) {
        return;
    }

    if (indicator_state->rfid_on != ui_runtime.last_indicator_state.rfid_on) {
        dirty_mask |= (uint16_t)((1u << UI_STRING_RFID_ON) | (1u << UI_STRING_RFID_OFF));
    }
    if (indicator_state->fric_on != ui_runtime.last_indicator_state.fric_on) {
        dirty_mask |= (uint16_t)((1u << UI_STRING_FRIC_ON) | (1u << UI_STRING_FRIC_OFF));
    }
    if (indicator_state->cap_on != ui_runtime.last_indicator_state.cap_on) {
        dirty_mask |= (uint16_t)((1u << UI_STRING_CAP_ON) | (1u << UI_STRING_CAP_OFF));
    }

    if (dirty_mask == 0u) {
        return;
    }

    // What: 仅在状态翻转时重建对应字符串；Why: fric/RFID/cap 属于低频状态，没必要跟动画同频刷新占用带宽。
    ui_runtime.last_indicator_state = *indicator_state;
    UIBuildAllStrings(UI_Graph_Change, &ui_runtime.last_indicator_state);
    ui_runtime.dirty_string_mask |= dirty_mask;
}

static void UIProcessRuntimeUpdate(uint32_t now_tick_ms)
{
    UIDisplaySnapshot_t snapshot;
    uint8_t move_dirty = 0u;
    uint8_t data_dirty = 0u;
    UIRuntimePacket_e packet_to_send = UI_RUNTIME_PACKET_NONE;

    UIBuildDisplaySnapshot(&snapshot);
    if (UI_RECOVERY_ENABLE_STATE_STRINGS != 0u) {
        // What: 仅在启用状态字符串层时检查 on/off 翻转；Why: 排障阶段关闭这一层后不应继续制造字符串脏包。
        UIRefreshIndicatorChanges(&snapshot.indicator_state);
    }

    if (UI_RECOVERY_ENABLE_MOVE_PACKET != 0u) {
        move_dirty = UIMovePacketIsDirty(&snapshot);
    }
    if (UI_RECOVERY_ENABLE_DATA_PACKET != 0u) {
        data_dirty = UIDataPacketIsDirty(&snapshot);
    }

    if ((now_tick_ms - ui_runtime.last_packet_tick_ms) < UI_RUNTIME_PACKET_GAP_MS) {
        return;
    }

    packet_to_send = UISelectRuntimePacket(now_tick_ms, move_dirty, data_dirty);

    if (packet_to_send == UI_RUNTIME_PACKET_MOVE) {
        UISendMovePacket(now_tick_ms);
        return;
    }

    if (packet_to_send == UI_RUNTIME_PACKET_DATA) {
        UISendDataPacket(now_tick_ms);
        return;
    }

    if (UI_RECOVERY_ENABLE_STATE_STRINGS != 0u) {
        (void)UISendNextDirtyString(now_tick_ms);
    }
}

static uint8_t UISendNextDirtyString(uint32_t now_tick_ms)
{
    uint8_t string_index = 0u;

    if (ui_runtime.dirty_string_mask == 0u) {
        return 0u;
    }

    for (string_index = 0u; string_index < UI_STRING_COUNT; string_index++) {
        if ((ui_runtime.dirty_string_mask & (uint16_t)(1u << string_index)) != 0u) {
            // What: 每次只发送一个脏字符串；Why: 状态切换低频且单包发送最稳，能把额外负载压到最低。
            UICharRefresh(&referee_recv_info->referee_id, ui_strings[string_index]);
            ui_runtime.dirty_string_mask &= (uint16_t)~(1u << string_index);
            ui_runtime.last_packet_tick_ms = now_tick_ms;
            return 1u;
        }
    }

    return 0u;
}

static void UISendMovePacket(uint32_t now_tick_ms)
{
    // What: 运动组固定打成 5 图元包发送；Why: 这样既能恢复 body 和 qiang 动态，又严格满足协议只支持 1/2/5/7 图元包。
    UIGraphRefresh(&referee_recv_info->referee_id, 5,
                   ui_move_figures[UI_MOVE_FIGURE_BODY_0],
                   ui_move_figures[UI_MOVE_FIGURE_BODY_1],
                   ui_move_figures[UI_MOVE_FIGURE_BODY_2],
                   ui_move_figures[UI_MOVE_FIGURE_BODY_3],
                   ui_move_figures[UI_MOVE_FIGURE_QIANG]);
    memcpy(ui_runtime.last_move_figures, ui_move_figures, sizeof(ui_runtime.last_move_figures));
    do {
        ui_runtime.next_move_due_tick_ms += UI_MOVE_PERIOD_MS;
    } while (ui_runtime.next_move_due_tick_ms <= now_tick_ms);
    ui_runtime.last_packet_tick_ms = now_tick_ms;
}

static void UISendDataPacket(uint32_t now_tick_ms)
{
    // What: 数据组固定打成 5 图元包发送；Why: 协议只支持 1/2/5/7，合成单包后才能把总 0x0301 频率稳稳压在预算内。
    UIGraphRefresh(&referee_recv_info->referee_id, 5,
                   ui_data_figures[UI_DATA_FIGURE_PITCH_VALUE],
                   ui_data_figures[UI_DATA_FIGURE_BULLET_SPEED],
                   ui_data_figures[UI_DATA_FIGURE_BULLET_NUM],
                   ui_data_figures[UI_DATA_FIGURE_POWER],
                   ui_data_figures[UI_DATA_FIGURE_BUFFER]);
    memcpy(ui_runtime.last_data_figures, ui_data_figures, sizeof(ui_runtime.last_data_figures));
    do {
        ui_runtime.next_data_due_tick_ms += UI_DATA_PERIOD_MS;
    } while (ui_runtime.next_data_due_tick_ms <= now_tick_ms);
    ui_runtime.last_packet_tick_ms = now_tick_ms;
}

static uint8_t UIMovePacketIsDirty(const UIDisplaySnapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return 0u;
    }

    UIBuildMoveFigures(UI_Graph_Change, snapshot);
    return (memcmp(ui_runtime.last_move_figures, ui_move_figures, sizeof(ui_runtime.last_move_figures)) != 0) ? 1u : 0u;
}

static uint8_t UIDataPacketIsDirty(const UIDisplaySnapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return 0u;
    }

    UIBuildDataFigures(UI_Graph_Change, snapshot);
    return (memcmp(ui_runtime.last_data_figures, ui_data_figures, sizeof(ui_runtime.last_data_figures)) != 0) ? 1u : 0u;
}

static UIRuntimePacket_e UISelectRuntimePacket(uint32_t now_tick_ms, uint8_t move_dirty, uint8_t data_dirty)
{
    uint8_t move_due = 0u;
    uint8_t data_due = 0u;

    if (now_tick_ms >= ui_runtime.next_move_due_tick_ms) {
        if (move_dirty != 0u) {
            move_due = 1u;
        } else {
            // What: 对“到期但未变化”的组直接跳过一个周期；Why: 避免静止画面仍反复占用 0x0301 频率预算。
            do {
                ui_runtime.next_move_due_tick_ms += UI_MOVE_PERIOD_MS;
            } while (ui_runtime.next_move_due_tick_ms <= now_tick_ms);
        }
    }

    if (now_tick_ms >= ui_runtime.next_data_due_tick_ms) {
        if (data_dirty != 0u) {
            data_due = 1u;
        } else {
            // What: 对无变化的数据组同样滚动下一次截止时间；Why: 这样弹速归零后等静止数据不会继续空发。
            do {
                ui_runtime.next_data_due_tick_ms += UI_DATA_PERIOD_MS;
            } while (ui_runtime.next_data_due_tick_ms <= now_tick_ms);
        }
    }

    if (move_due != 0u && data_due != 0u) {
        // What: 两组同时到期时优先发送更早到期的那组；Why: 用“最早 deadline 优先”能在 30Hz 总预算下尽量逼近 20Hz/10Hz 目标节奏。
        return (ui_runtime.next_move_due_tick_ms <= ui_runtime.next_data_due_tick_ms) ? UI_RUNTIME_PACKET_MOVE : UI_RUNTIME_PACKET_DATA;
    }

    if (move_due != 0u) {
        return UI_RUNTIME_PACKET_MOVE;
    }

    if (data_due != 0u) {
        return UI_RUNTIME_PACKET_DATA;
    }

    return UI_RUNTIME_PACKET_NONE;
}

static uint8_t UIIsRFIDActive(void)
{
    // What: 只要任一 RFID 状态字存在有效位就判定为 on；Why: 你要求选手端只显示开关态，而当前底层并没有更细的业务语义映射表。
    return ((referee_recv_info->RFIDStatus.rfid_status != 0u) || (referee_recv_info->RFIDStatus.rfid_status_2 != 0u)) ? 1u : 0u;
}

static float UIClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static int32_t UIRoundFloatToInt(float value)
{
    // What: 用统一的本地四舍五入把浮点量转成协议整数；Why: 避免不同显示值在 0.1 边界来回抖动导致数字频繁跳字。
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

static uint32_t UIClampCoord(int32_t coord_value)
{
    if (coord_value < 0) {
        return 0u;
    }

    if (coord_value > (int32_t)UI_COORD_MAX) {
        return UI_COORD_MAX;
    }

    return (uint32_t)coord_value;
}

static void UIRotateRelativePoint(float base_x, float base_y, float angle_deg, int32_t *out_x, int32_t *out_y)
{
    const float angle_rad = angle_deg * UI_DEG_TO_RAD;
    const float cos_value = cosf(angle_rad);
    const float sin_value = sinf(angle_rad);
    const float rotated_x = base_x * cos_value - base_y * sin_value;
    const float rotated_y = base_x * sin_value + base_y * cos_value;

    if (out_x != NULL) {
        *out_x = UIRoundFloatToInt(rotated_x);
    }
    if (out_y != NULL) {
        *out_y = UIRoundFloatToInt(rotated_y);
    }
}
