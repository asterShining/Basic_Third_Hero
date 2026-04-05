/**
 * @file referee_task.c
 * @author OpenAI
 * @brief 底盘裁判 UI 任务，使用新的 HUD 布局并继续复用现有裁判发送链路
 *
 * @version 2.0
 * @date 2026-04-04
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
#define UI_STATE_PACKET_GAP_MS 10u
#define UI_STATE_RESYNC_PERIOD_MS 400u
#define UI_MOVE_PERIOD_MS 50u
#define UI_DATA_PERIOD_MS 100u
#define UI_SHOOT_SPEED_STALE_MS 500u

#define UI_BUFFER_FULL_SCALE_J 60.0f
#define UI_DEG_TO_RAD 0.01745329251994329577f
#define UI_COORD_MAX 2047u

// What: 这组坐标直接对齐新布局生成器里的 `on_1/on_2/on_3`；Why: 三个圆环是左侧状态列的视觉锚点，位置错了整套布局会立即走样。
#define UI_ON_1_X 212u
#define UI_ON_1_Y 791u
#define UI_ON_2_X 211u
#define UI_ON_2_Y 716u
#define UI_ON_3_X 211u
#define UI_ON_3_Y 643u
#define UI_ON_RADIUS 27u
#define UI_ON_WIDTH 5u

// What: 这组坐标直接对齐 RMUC HERO 参考里的圆盘式 pitch 刻度；Why: 这次只替换俯仰显示，因此需要把旧滑块彻底换成参考工程的圆盘映射几何。
#define UI_PITCH_GAUGE_CENTER_X 960.0f
#define UI_PITCH_GAUGE_CENTER_Y 540.0f
#define UI_PITCH_GAUGE_OUTER_RADIUS 390.0f
#define UI_PITCH_GAUGE_INNER_RADIUS 345.0f
#define UI_PITCH_NEEDLE_WIDTH 6u
#define UI_PITCH_GAUGE_TOP_ANGLE_DEG 9.0f
#define UI_PITCH_GAUGE_BOTTOM_ANGLE_DEG -34.0f
#define UI_PITCH_SCALE_ZERO_START_X 1350u
#define UI_PITCH_SCALE_ZERO_START_Y 540u
#define UI_PITCH_SCALE_ZERO_END_X 1320u
#define UI_PITCH_SCALE_ZERO_END_Y 540u
#define UI_PITCH_SCALE_TOP_START_X 1345u
#define UI_PITCH_SCALE_TOP_START_Y 478u
#define UI_PITCH_SCALE_TOP_END_X 1315u
#define UI_PITCH_SCALE_TOP_END_Y 483u
#define UI_PITCH_SCALE_BOTTOM_START_X 1285u
#define UI_PITCH_SCALE_BOTTOM_START_Y 757u
#define UI_PITCH_SCALE_BOTTOM_END_X 1258u
#define UI_PITCH_SCALE_BOTTOM_END_Y 741u
#define UI_PITCH_SCALE_NEG10_START_X 1344u
#define UI_PITCH_SCALE_NEG10_START_Y 607u
#define UI_PITCH_SCALE_NEG10_END_X 1314u
#define UI_PITCH_SCALE_NEG10_END_Y 602u
#define UI_PITCH_SCALE_NEG20_START_X 1325u
#define UI_PITCH_SCALE_NEG20_START_Y 674u
#define UI_PITCH_SCALE_NEG20_END_X 1298u
#define UI_PITCH_SCALE_NEG20_END_Y 663u
#define UI_PITCH_SCALE_NEG30_START_X 1297u
#define UI_PITCH_SCALE_NEG30_START_Y 736u
#define UI_PITCH_SCALE_NEG30_END_X 1272u
#define UI_PITCH_SCALE_NEG30_END_Y 720u
#define UI_PITCH_SCALE_WIDTH 4u
#define UI_PITCH_SCALE_FONT 12u
#define UI_PITCH_SCALE_VALUE_30_X 1236u
#define UI_PITCH_SCALE_VALUE_30_Y 715u
#define UI_PITCH_SCALE_VALUE_20_X 1264u
#define UI_PITCH_SCALE_VALUE_20_Y 663u
#define UI_PITCH_SCALE_VALUE_10_X 1272u
#define UI_PITCH_SCALE_VALUE_10_Y 483u
#define UI_PITCH_SCALE_VALUE_WIDTH 1u

// What: 这组坐标来自新布局生成器里的 `body` 矩形；Why: 虽然运行期要把车体改成可旋转粗线组合，但中心点和包络尺寸仍必须继承原布局。
#define UI_BODY_CENTER_X 1585.5f
#define UI_BODY_CENTER_Y 767.5f
#define UI_BODY_HALF_SIZE 35.5f
#define UI_BODY_FILL_WIDTH 12u

// What: 4 条填充线横向分布在 body 矩形内部；Why: 裁判 UI 不支持旋转实心矩形，只能用多条粗线逼近“代码中的车体块”。
#define UI_BODY_FILL_CENTER_0_X -24.0f
#define UI_BODY_FILL_CENTER_1_X -8.0f
#define UI_BODY_FILL_CENTER_2_X 8.0f
#define UI_BODY_FILL_CENTER_3_X 24.0f

// What: 这条线直接继承新布局里的 `shooter`；Why: 你已经确认 shooter 作为参考方向保持静止，不能再跟 body 一起旋转。
#define UI_SHOOTER_START_X 1583u
#define UI_SHOOTER_START_Y 769u
#define UI_SHOOTER_END_X 1582u
#define UI_SHOOTER_END_Y 855u
#define UI_SHOOTER_WIDTH 5u

// What: 这两条线直接继承新布局里的 `body_line_left/right`；Why: 它们是整套 HUD 的静态结构边线，属于纯视觉框架而不是运行期动画。
#define UI_BODY_LINE_LEFT_START_X 548u
#define UI_BODY_LINE_LEFT_START_Y 3u
#define UI_BODY_LINE_LEFT_END_X 713u
#define UI_BODY_LINE_LEFT_END_Y 290u
#define UI_BODY_LINE_LEFT_WIDTH 5u
#define UI_BODY_LINE_RIGHT_START_X 1372u
#define UI_BODY_LINE_RIGHT_START_Y 0u
#define UI_BODY_LINE_RIGHT_END_X 1126u
#define UI_BODY_LINE_RIGHT_END_Y 380u
#define UI_BODY_LINE_RIGHT_WIDTH 5u

// What: 这组坐标直接继承新布局里的 `buffer`；Why: buffer 条的长度变化必须以这条基准线为参考，后续只改终点而不改起点和厚度。
#define UI_BUFFER_START_X 685u
#define UI_BUFFER_START_Y 90u
#define UI_BUFFER_END_X_MAX 1213u
#define UI_BUFFER_END_Y 94u
#define UI_BUFFER_WIDTH 20u

// What: 数值位坐标完全继承生成器；Why: 这几处数字是新布局最敏感的对齐点，不能沿用旧 UI 的坐标或字体配置。
#define UI_BULLET_NUM_X 1359u
#define UI_BULLET_NUM_Y 479u
#define UI_BULLET_NUM_FONT 20u
#define UI_BULLET_NUM_WIDTH 2u
#define UI_PITCH_VALUE_X 1413u
#define UI_PITCH_VALUE_Y 650u
#define UI_PITCH_VALUE_FONT 20u
#define UI_PITCH_VALUE_WIDTH 2u
#define UI_PITCH_VALUE_DIGIT 2u
#define UI_POWER_VALUE_X 1574u
#define UI_POWER_VALUE_Y 465u
#define UI_POWER_VALUE_FONT 20u
#define UI_POWER_VALUE_WIDTH 2u
#define UI_POWER_VALUE_DIGIT 2u
#define UI_BULLET_SPEED_X 1354u
#define UI_BULLET_SPEED_Y 627u
#define UI_BULLET_SPEED_FONT 20u
#define UI_BULLET_SPEED_WIDTH 2u
#define UI_BULLET_SPEED_DIGIT 3u

// What: 这组标签文字坐标直接继承新布局；Why: 左侧状态列和右侧功率区的视觉关系主要靠这些标签建立，偏一点都会显得不像同一套图。
#define UI_LABEL_FRIC_X 202u
#define UI_LABEL_FRIC_Y 808u
#define UI_LABEL_FRIC_FONT 30u
#define UI_LABEL_FRIC_WIDTH 3u
#define UI_LABEL_POWER_X 1590u
#define UI_LABEL_POWER_Y 514u
#define UI_LABEL_POWER_FONT 31u
#define UI_LABEL_POWER_WIDTH 3u
#define UI_LABEL_PITCH_X 1413u
#define UI_LABEL_PITCH_Y 687u
#define UI_LABEL_PITCH_FONT 20u
#define UI_LABEL_PITCH_WIDTH 2u
#define UI_LABEL_ROBOT_X 199u
#define UI_LABEL_ROBOT_Y 733u
#define UI_LABEL_ROBOT_FONT 30u
#define UI_LABEL_ROBOT_WIDTH 3u
#define UI_LABEL_CAP_X 198u
#define UI_LABEL_CAP_Y 661u
#define UI_LABEL_CAP_FONT 31u
#define UI_LABEL_CAP_WIDTH 3u

// What: 这组横线和竖线直接继承参考工程里的 `meter_5/meter_3/ChuiZhi`；Why: 用户这次明确要求把这 3 条静态标尺整合进当前 UI，并且不改原有通信方式。
#define UI_METER_5_START_X 594u
#define UI_METER_5_START_Y 401u
#define UI_METER_5_END_X 1330u
#define UI_METER_5_END_Y 400u
#define UI_METER_5_WIDTH 2u
#define UI_CHUI_ZHI_START_X 972u
#define UI_CHUI_ZHI_START_Y 94u
#define UI_CHUI_ZHI_END_X 967u
#define UI_CHUI_ZHI_END_Y 813u
#define UI_CHUI_ZHI_WIDTH 2u
#define UI_METER_3_START_X 585u
#define UI_METER_3_START_Y 422u
#define UI_METER_3_END_X 1335u
#define UI_METER_3_END_Y 422u
#define UI_METER_3_WIDTH 3u

// What: 这组文字坐标直接继承参考工程里的 `text_5/text_3`；Why: 两个距离标签必须和新增的标尺线严格配对，否则客户端上会只剩横线没有语义。
#define UI_TEXT_5_X 535u
#define UI_TEXT_5_Y 402u
#define UI_TEXT_5_FONT 20u
#define UI_TEXT_5_WIDTH 2u
#define UI_TEXT_3_X 534u
#define UI_TEXT_3_Y 440u
#define UI_TEXT_3_FONT 20u
#define UI_TEXT_3_WIDTH 2u

// What: 亮灯时统一使用绿色；Why: 三个 `on_x` 都是“功能开启”语义，用统一正向颜色最直观。
#define UI_STATE_ON_COLOR UI_Color_Green
// What: 发射机构断电时 `on_1` 统一使用橙色；Why: 这次需要把“摩擦轮想开但裁判判定 shooter 无输出”与普通关闭红色明确区分。
#define UI_STATE_POWER_CUT_COLOR UI_Color_Orange
// What: 熄灭态统一使用固定红色；Why: 用户已经明确要求“关闭为红色、开启为绿色”，且不能跟随主色变蓝。
#define UI_STATE_OFF_COLOR UI_Color_Purplish_red
// What: buffer 断电告警不用 `UI_Color_Main`；Why: 主色在蓝方会变蓝，无法满足“断电就变红”的固定语义。
#define UI_BUFFER_ALERT_COLOR UI_Color_Purplish_red

typedef enum {
    UI_STATIC_FIGURE_SHOOTER = 0,
    UI_STATIC_FIGURE_BODY_LINE_LEFT,
    UI_STATIC_FIGURE_BODY_LINE_RIGHT,
    // What: 这 3 个静态图元直接复用参考工程的距离尺与竖直基准线；Why: 用户当前只要求把这组静态视觉参考并入现有 UI，不引入新的动态数据逻辑。
    UI_STATIC_FIGURE_METER_5,
    UI_STATIC_FIGURE_CHUI_ZHI,
    UI_STATIC_FIGURE_METER_3,
    UI_STATIC_FIGURE_PITCH_SCALE_ZERO_LINE,
    UI_STATIC_FIGURE_PITCH_SCALE_TOP_LINE,
    UI_STATIC_FIGURE_PITCH_SCALE_BOTTOM_LINE,
    UI_STATIC_FIGURE_PITCH_SCALE_NEG10_LINE,
    UI_STATIC_FIGURE_PITCH_SCALE_NEG20_LINE,
    UI_STATIC_FIGURE_PITCH_SCALE_NEG30_LINE,
    UI_STATIC_FIGURE_PITCH_SCALE_VALUE_30,
    UI_STATIC_FIGURE_PITCH_SCALE_VALUE_20,
    UI_STATIC_FIGURE_PITCH_SCALE_VALUE_10,
    UI_STATIC_FIGURE_COUNT,
} UIStaticFigureIndex_e;

typedef enum {
    UI_STATE_FIGURE_ON_1 = 0,
    UI_STATE_FIGURE_ON_2,
    UI_STATE_FIGURE_ON_3,
    UI_STATE_FIGURE_COUNT,
} UIStateFigureIndex_e;

typedef enum {
    UI_MOVE_FIGURE_BODY_0 = 0,
    UI_MOVE_FIGURE_BODY_1,
    UI_MOVE_FIGURE_BODY_2,
    UI_MOVE_FIGURE_BODY_3,
    UI_MOVE_FIGURE_PITCH_NEEDLE,
    UI_MOVE_FIGURE_COUNT,
} UIMoveFigureIndex_e;

typedef enum {
    UI_DATA_FIGURE_BUFFER = 0,
    UI_DATA_FIGURE_POWER_VALUE,
    UI_DATA_FIGURE_PITCH_VALUE,
    UI_DATA_FIGURE_BULLET_NUM,
    UI_DATA_FIGURE_BULLET_SPEED,
    UI_DATA_FIGURE_COUNT,
} UIDataFigureIndex_e;

typedef enum {
    UI_STRING_FRIC = 0,
    UI_STRING_POWER,
    UI_STRING_PITCH,
    UI_STRING_ROBOT,
    UI_STRING_CAP,
    // What: 这 2 个字符串直接对应参考工程里的 `3m/5m`；Why: 新增标尺如果没有文字刻度，操作者很难把横线快速理解成距离参考。
    UI_STRING_TEXT_3,
    UI_STRING_TEXT_5,
    UI_STRING_COUNT,
} UIStringIndex_e;

typedef enum {
    UI_INIT_STAGE_IDLE = 0,
    UI_INIT_STAGE_DELETE_ALL,
    UI_INIT_STAGE_DRAW_STATIC_FIGURES_0,
    UI_INIT_STAGE_DRAW_STATIC_FIGURES_1,
    // What: 静态图元总数从 12 增到 15 后必须新增第三拍；Why: `UIGraphRefresh` 只支持 1/2/5/7，初始化静态层只能改成 7 + 7 + 1 合法分包。
    UI_INIT_STAGE_DRAW_STATIC_FIGURES_2,
    UI_INIT_STAGE_DRAW_STATE_FIGURES_0,
    UI_INIT_STAGE_DRAW_STATE_FIGURES_1,
    UI_INIT_STAGE_DRAW_MOVE_FIGURES,
    UI_INIT_STAGE_DRAW_DATA_FIGURES,
    UI_INIT_STAGE_DRAW_STRING_0,
} UIInitStage_e;

typedef enum {
    UI_RUNTIME_PACKET_NONE = 0,
    UI_RUNTIME_PACKET_STATE,
    UI_RUNTIME_PACKET_MOVE,
    UI_RUNTIME_PACKET_DATA,
} UIRuntimePacket_e;

// What: `on_1` 的显示状态单独抽成三态枚举；Why: fric 现在既要表达正常开关，又要表达“命令已开但 shooter 被裁判切断输出”的异常态。
typedef enum {
    UI_FRIC_INDICATOR_OFF = 0,
    UI_FRIC_INDICATOR_ON,
    UI_FRIC_INDICATOR_POWER_CUT,
} UIFricIndicatorState_e;

typedef struct
{
    // What: `fric_state` 保存 `on_1` 的最终显示语义；Why: 运行期脏检查和初始化建图都要复用同一份三态真值，避免一个地方三态、另一个地方还按布尔判断。
    UIFricIndicatorState_e fric_state;
    uint8_t robot_spin_on;
    uint8_t cap_on;
} UIIndicatorState_t;

typedef struct
{
    float body_relative_angle_deg;
    int32_t pitch_value_milli_deg;
    int32_t bullet_speed_milli_mps;
    int32_t power_milli_w;
    int32_t bullet_num;
    uint32_t pitch_needle_start_x;
    uint32_t pitch_needle_start_y;
    uint32_t pitch_needle_end_x;
    uint32_t pitch_needle_end_y;
    uint32_t buffer_end_x;
    uint32_t buffer_color;
    UIIndicatorState_t indicator_state;
} UIDisplaySnapshot_t;

typedef struct
{
    UIInitStage_e init_stage;
    uint32_t last_packet_tick_ms;
    uint32_t next_state_sync_due_tick_ms;
    uint32_t next_move_due_tick_ms;
    uint32_t next_data_due_tick_ms;
    uint8_t dirty_state_mask;
    uint8_t state_retry_count[UI_STATE_FIGURE_COUNT];
    Graph_Data_t last_move_figures[UI_MOVE_FIGURE_COUNT];
    Graph_Data_t last_data_figures[UI_DATA_FIGURE_COUNT];
    UIIndicatorState_t last_indicator_state;
} UIRuntime_t;

static Referee_Interactive_info_t *interactive_data = NULL; // What: 缓存 UI 实时数据入口；Why: UI 线程只应消费已经整理好的显示量，避免自己跨模块取数造成状态不一致。
static referee_info_t *referee_recv_info = NULL; // What: 缓存裁判接收数据入口；Why: 客户端 ID、弹量、buffer 和弹速都直接依赖裁判原生数据。
uint8_t UI_Seq = 0u; // What: 维护 0x0301 UI 帧序号；Why: 现有 `referee_UI.c` 会直接把它写进协议头，不能丢。

static UIRuntime_t ui_runtime;
static Graph_Data_t ui_static_figures[UI_STATIC_FIGURE_COUNT];
static Graph_Data_t ui_state_figures[UI_STATE_FIGURE_COUNT];
static Graph_Data_t ui_move_figures[UI_MOVE_FIGURE_COUNT];
static Graph_Data_t ui_data_figures[UI_DATA_FIGURE_COUNT];
static String_Data_t ui_strings[UI_STRING_COUNT];
static volatile uint8_t ui_manual_refresh_request = 0u; // What: 锁存外部手动刷新请求；Why: UI 任务与底盘控制任务分离，必须用轻量标志串起重建动作。

static void DetermineRobotID(void);
static void UIRuntimeReset(void);
static void UIStartInitCycle(uint32_t now_tick_ms);
static void UIFinishInitCycle(uint32_t now_tick_ms);
static void UIAdvanceInitStage(uint32_t now_tick_ms);
static void UIBuildDisplaySnapshot(UIDisplaySnapshot_t *snapshot);
static void UIBuildStaticFigures(uint32_t operate_type);
static void UIBuildStateFigures(uint32_t operate_type, const UIIndicatorState_t *indicator_state);
static void UIBuildMoveFigures(uint32_t operate_type, const UIDisplaySnapshot_t *snapshot);
static void UIBuildDataFigures(uint32_t operate_type, const UIDisplaySnapshot_t *snapshot);
static void UIBuildStrings(uint32_t operate_type);
static void UIRefreshStateChanges(const UIIndicatorState_t *indicator_state);
static void UIProcessRuntimeUpdate(uint32_t now_tick_ms);
static uint8_t UISendNextDirtyState(uint32_t now_tick_ms);
static void UISendMovePacket(uint32_t now_tick_ms);
static void UISendDataPacket(uint32_t now_tick_ms);
static uint8_t UIMovePacketIsDirty(const UIDisplaySnapshot_t *snapshot);
static uint8_t UIDataPacketIsDirty(const UIDisplaySnapshot_t *snapshot);
static UIRuntimePacket_e UISelectRuntimePacket(uint32_t now_tick_ms, uint8_t move_dirty, uint8_t data_dirty);
static float UIClampFloat(float value, float min_value, float max_value);
static int32_t UIRoundFloatToInt(float value);
static uint32_t UIClampCoord(int32_t coord_value);
static void UIRotateRelativePoint(float base_x, float base_y, float angle_deg, int32_t *out_x, int32_t *out_y);

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    // What: 初始化裁判接收模块并缓存 UI 数据入口；Why: 仍然复用当前底盘启动链路，不引入新的通信初始化分支。
    referee_recv_info = RefereeInit(referee_usart_handle);
    interactive_data = UI_data;
    referee_recv_info->init_flag = 1u;
    return referee_recv_info;
}

void MyUIInit(void)
{
    if (referee_recv_info == NULL || interactive_data == NULL || referee_recv_info->init_flag == 0u) {
        // What: 数据入口缺失时直接删除 UI 任务；Why: 没有 client_id 和实时数据时继续发包只会制造假象，不利于现场排障。
        vTaskDelete(NULL);
    }

    while (referee_recv_info->GameRobotState.robot_id == 0u) {
        // What: 等待裁判系统分配 robot_id；Why: 客户端 ID 由 robot_id 推导，未拿到前发送的所有 UI 包都会打到错误目标。
        osDelay(100);
    }

    DetermineRobotID();
    UIRuntimeReset();
    UIStartInitCycle(HAL_GetTick());

    LOGINFO("[ui] robot_id:%u client_id:0x%04X",
            (unsigned int)referee_recv_info->referee_id.Robot_ID,
            (unsigned int)referee_recv_info->referee_id.Cilent_ID);
}

void UIRequestRefresh(void)
{
    // What: 对外暴露一次性重绘请求入口；Why: 控制线程只需要触发重建，真正的删图和建图必须继续在 UI 线程里串行执行。
    ui_manual_refresh_request = 1u;
}

void UITask(void)
{
    const uint32_t now_tick_ms = HAL_GetTick();

    if (referee_recv_info == NULL || interactive_data == NULL) {
        return;
    }

    if (ui_manual_refresh_request != 0u) {
        // What: 收到手动重绘后重新走 delete-all 初始化流程；Why: 现有重建路径已经在线下验证稳定，比临时拼局部 add/change 更可靠。
        ui_manual_refresh_request = 0u;
        LOGINFO("[ui] manual_refresh");
        UIStartInitCycle(now_tick_ms);
        return;
    }

    if (ui_runtime.init_stage != UI_INIT_STAGE_IDLE) {
        // What: 初始化阶段继续沿用分时建图；Why: 首次 add 包更容易丢层，拉开间隔能显著提高客户端稳定落图概率。
        if ((now_tick_ms - ui_runtime.last_packet_tick_ms) >= UI_INIT_PACKET_GAP_MS) {
            UIAdvanceInitStage(now_tick_ms);
        }
        return;
    }

    UIProcessRuntimeUpdate(now_tick_ms);
}

static void DetermineRobotID(void)
{
    const uint16_t robot_id = referee_recv_info->GameRobotState.robot_id;

    // What: 直接按当前 2026 协议把 robot_id 映射到客户端 ID；Why: UI 必须只发给本机选手端，不能再沿用旧工程里对颜色和车种的猜测分支。
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
    memset(ui_state_figures, 0, sizeof(ui_state_figures));
    memset(ui_move_figures, 0, sizeof(ui_move_figures));
    memset(ui_data_figures, 0, sizeof(ui_data_figures));
    memset(ui_strings, 0, sizeof(ui_strings));

    UI_Seq = 0u;
    ui_runtime.last_packet_tick_ms = HAL_GetTick();
    ui_runtime.next_state_sync_due_tick_ms = ui_runtime.last_packet_tick_ms + UI_STATE_RESYNC_PERIOD_MS;

    // What: 复位后立刻同步当前显示状态；Why: 初始化建图和运行期基线都必须以最新实测值为起点，避免先闪一帧占位数据。
    UIBuildDisplaySnapshot(&snapshot);
    ui_runtime.last_indicator_state = snapshot.indicator_state;
    // What: 复位时清空状态灯重发计数；Why: 避免旧的脏状态残留到下一轮建图后继续重复发送。
    memset(ui_runtime.state_retry_count, 0, sizeof(ui_runtime.state_retry_count));
}

static void UIStartInitCycle(uint32_t now_tick_ms)
{
    // What: 每次重建都从 delete-all 开始；Why: 只有先把旧对象全部清掉，才能保证新布局不会和旧布局叠在一起。
    ui_runtime.init_stage = UI_INIT_STAGE_DELETE_ALL;
    ui_runtime.last_packet_tick_ms = now_tick_ms - UI_INIT_PACKET_GAP_MS;
    ui_runtime.dirty_state_mask = 0u;
    // What: 开始整页重建时同步清空状态灯重发队列；Why: 初始化阶段会重新 add 正确状态圆环，旧 change 重发已经没有意义。
    memset(ui_runtime.state_retry_count, 0, sizeof(ui_runtime.state_retry_count));
}

static void UIFinishInitCycle(uint32_t now_tick_ms)
{
    UIDisplaySnapshot_t snapshot;

    // What: 建图结束后立即建立运行期的脏检查基线；Why: 否则下一拍会把刚 add 过的对象全部误判成脏数据再发一遍。
    ui_runtime.init_stage = UI_INIT_STAGE_IDLE;
    ui_runtime.last_packet_tick_ms = now_tick_ms;
    ui_runtime.next_state_sync_due_tick_ms = now_tick_ms + UI_STATE_RESYNC_PERIOD_MS;
    ui_runtime.next_move_due_tick_ms = now_tick_ms + UI_MOVE_PERIOD_MS;
    ui_runtime.next_data_due_tick_ms = now_tick_ms + UI_DATA_PERIOD_MS;
    ui_runtime.dirty_state_mask = 0u;
    // What: 初始化收尾时清空状态重发计数；Why: 首轮 add 已经把当前 on/off 状态落图，不需要再补发历史 change。
    memset(ui_runtime.state_retry_count, 0, sizeof(ui_runtime.state_retry_count));

    UIBuildDisplaySnapshot(&snapshot);
    UIBuildMoveFigures(UI_Graph_Change, &snapshot);
    UIBuildDataFigures(UI_Graph_Change, &snapshot);
    memcpy(ui_runtime.last_move_figures, ui_move_figures, sizeof(ui_runtime.last_move_figures));
    memcpy(ui_runtime.last_data_figures, ui_data_figures, sizeof(ui_runtime.last_data_figures));
    ui_runtime.last_indicator_state = snapshot.indicator_state;
}

static void UIAdvanceInitStage(uint32_t now_tick_ms)
{
    UIDisplaySnapshot_t snapshot;
    uint8_t string_index = 0u;

    UIBuildDisplaySnapshot(&snapshot);

    switch (ui_runtime.init_stage) {
    case UI_INIT_STAGE_DELETE_ALL:
        // What: 先清空客户端上已有的所有对象；Why: 新布局已经完全替换旧布局，不能允许旧对象残留。
        UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0u);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_STATIC_FIGURES_0;
        break;

    case UI_INIT_STAGE_DRAW_STATIC_FIGURES_0:
        // What: 第一拍先补车体静态骨架、两条距离横线、竖直基准线和 pitch 零位；Why: 这些都是整页最核心的背景框架，先落图能让页面主体最快成形。
        UIBuildStaticFigures(UI_Graph_ADD);
        UIGraphRefresh(&referee_recv_info->referee_id, 7,
                       ui_static_figures[UI_STATIC_FIGURE_SHOOTER],
                       ui_static_figures[UI_STATIC_FIGURE_BODY_LINE_LEFT],
                       ui_static_figures[UI_STATIC_FIGURE_BODY_LINE_RIGHT],
                       ui_static_figures[UI_STATIC_FIGURE_METER_5],
                       ui_static_figures[UI_STATIC_FIGURE_CHUI_ZHI],
                       ui_static_figures[UI_STATIC_FIGURE_METER_3],
                       ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_ZERO_LINE]);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_STATIC_FIGURES_1;
        break;

    case UI_INIT_STAGE_DRAW_STATIC_FIGURES_1:
        // What: 第二拍补剩余 5 条 pitch 刻度和前 2 个数字标记；Why: 静态图元新增后必须拆成第二个 Draw7 包，才能继续满足底层协议上限。
        UIBuildStaticFigures(UI_Graph_ADD);
        UIGraphRefresh(&referee_recv_info->referee_id, 7,
                       ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_TOP_LINE],
                       ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_BOTTOM_LINE],
                       ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_NEG10_LINE],
                       ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_NEG20_LINE],
                       ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_NEG30_LINE],
                       ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_VALUE_30],
                       ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_VALUE_20]);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_STATIC_FIGURES_2;
        break;

    case UI_INIT_STAGE_DRAW_STATIC_FIGURES_2:
        // What: 第三拍只补最后一个 pitch 数字标记；Why: 15 个静态图元无法刚好拆成两个合法包，只能用 Draw1 收尾。
        UIBuildStaticFigures(UI_Graph_ADD);
        UIGraphRefresh(&referee_recv_info->referee_id, 1,
                       ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_VALUE_10]);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_STATE_FIGURES_0;
        break;

    case UI_INIT_STAGE_DRAW_STATE_FIGURES_0:
        // What: 先把前两个状态圆环打成 Draw2；Why: 当前裁判 UI 封装只支持 1/2/5/7 图元包，状态灯初始化必须按合法分包发送。
        UIBuildStateFigures(UI_Graph_ADD, &snapshot.indicator_state);
        UIGraphRefresh(&referee_recv_info->referee_id, 2,
                       ui_state_figures[UI_STATE_FIGURE_ON_1],
                       ui_state_figures[UI_STATE_FIGURE_ON_2]);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_STATE_FIGURES_1;
        break;

    case UI_INIT_STAGE_DRAW_STATE_FIGURES_1:
        // What: 最后一个状态圆环单独补成 Draw1；Why: 这样既满足底层协议约束，也能保持三个 on/off 灯首帧都按真值创建出来。
        UIBuildStateFigures(UI_Graph_ADD, &snapshot.indicator_state);
        UIGraphRefresh(&referee_recv_info->referee_id, 1,
                       ui_state_figures[UI_STATE_FIGURE_ON_3]);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_MOVE_FIGURES;
        break;

    case UI_INIT_STAGE_DRAW_MOVE_FIGURES:
        // What: 运动组继续固定使用 4 条 body 填充线 + 1 条 pitch 指针；Why: 这样仍然能维持合法 Draw5 包，同时把旧滑块替换成 RMUC 红色指针。
        UIBuildMoveFigures(UI_Graph_ADD, &snapshot);
        UIGraphRefresh(&referee_recv_info->referee_id, 5,
                       ui_move_figures[UI_MOVE_FIGURE_BODY_0],
                       ui_move_figures[UI_MOVE_FIGURE_BODY_1],
                       ui_move_figures[UI_MOVE_FIGURE_BODY_2],
                       ui_move_figures[UI_MOVE_FIGURE_BODY_3],
                       ui_move_figures[UI_MOVE_FIGURE_PITCH_NEEDLE]);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_DATA_FIGURES;
        break;

    case UI_INIT_STAGE_DRAW_DATA_FIGURES:
        // What: 数值组固定使用 buffer + 4 个数字；Why: 同样凑成合法 Draw5 包，并把低频数据层从高频动画层中彻底分离。
        UIBuildDataFigures(UI_Graph_ADD, &snapshot);
        UIGraphRefresh(&referee_recv_info->referee_id, 5,
                       ui_data_figures[UI_DATA_FIGURE_BUFFER],
                       ui_data_figures[UI_DATA_FIGURE_POWER_VALUE],
                       ui_data_figures[UI_DATA_FIGURE_PITCH_VALUE],
                       ui_data_figures[UI_DATA_FIGURE_BULLET_NUM],
                       ui_data_figures[UI_DATA_FIGURE_BULLET_SPEED]);
        ui_runtime.init_stage = UI_INIT_STAGE_DRAW_STRING_0;
        break;

    default:
        if (ui_runtime.init_stage >= UI_INIT_STAGE_DRAW_STRING_0 &&
            ui_runtime.init_stage < (UI_INIT_STAGE_DRAW_STRING_0 + UI_STRING_COUNT)) {
            // What: 字符串初始化阶段每拍只发一个字符串；Why: 字符包单独占一帧，拆开后更容易稳稳落到客户端。
            string_index = (uint8_t)(ui_runtime.init_stage - UI_INIT_STAGE_DRAW_STRING_0);
            UIBuildStrings(UI_Graph_ADD);
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

static void UIBuildDisplaySnapshot(UIDisplaySnapshot_t *snapshot)
{
    float clamped_pitch = 0.0f;
    float pitch_ratio = 0.0f;
    float pitch_gauge_angle_deg = 0.0f;
    float pitch_gauge_angle_rad = 0.0f;
    float shoot_speed_mps = 0.0f;
    float buffer_ratio = 0.0f;
    uint8_t shooter_output_enabled = 0u;

    if (snapshot == NULL) {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));

    // What: body 只显示底盘相对云台的偏角；Why: 你已经确认 shooter 固定不动，因此 body 只能按“相对角”来转才符合视觉语义。
    snapshot->body_relative_angle_deg = theta_format(-interactive_data->chassis_gimbal_offset_deg);

    // What: pitch 必须限制在机构上下限内；Why: 新的圆盘指针和数值都要与当前项目的真实机械行程保持一致，不能显示出越界角。
    clamped_pitch = UIClampFloat(interactive_data->gimbal_pitch_deg, (float)PITCH_MIN_ANGLE, (float)PITCH_MAX_ANGLE);
    snapshot->pitch_value_milli_deg = UIRoundFloatToInt(clamped_pitch * 1000.0f);

    if ((float)PITCH_MAX_ANGLE > (float)PITCH_MIN_ANGLE) {
        // What: 先把当前 pitch 归一化到 0~1 区间；Why: 这样才能把当前项目的角度限位稳定映射到 RMUC 圆盘的视觉角域。
        pitch_ratio = (clamped_pitch - (float)PITCH_MIN_ANGLE) / ((float)PITCH_MAX_ANGLE - (float)PITCH_MIN_ANGLE);
    }
    pitch_ratio = UIClampFloat(pitch_ratio, 0.0f, 1.0f);

    // What: 把当前项目的 `PITCH_MIN_ANGLE ~ PITCH_MAX_ANGLE` 映射到 RMUC pitch 圆盘的上端与下端；Why: 用户要取消旧滑块，但仍要求 pitch 视觉端点继续和真实限位一一对应。
    pitch_gauge_angle_deg = UI_PITCH_GAUGE_TOP_ANGLE_DEG +
                            pitch_ratio * (UI_PITCH_GAUGE_BOTTOM_ANGLE_DEG - UI_PITCH_GAUGE_TOP_ANGLE_DEG);
    pitch_gauge_angle_rad = pitch_gauge_angle_deg * UI_DEG_TO_RAD;
    snapshot->pitch_needle_start_x = UIClampCoord(UIRoundFloatToInt(UI_PITCH_GAUGE_CENTER_X +
                                                                    UI_PITCH_GAUGE_OUTER_RADIUS * cosf(pitch_gauge_angle_rad)));
    snapshot->pitch_needle_start_y = UIClampCoord(UIRoundFloatToInt(UI_PITCH_GAUGE_CENTER_Y -
                                                                    UI_PITCH_GAUGE_OUTER_RADIUS * sinf(pitch_gauge_angle_rad)));
    snapshot->pitch_needle_end_x = UIClampCoord(UIRoundFloatToInt(UI_PITCH_GAUGE_CENTER_X +
                                                                  UI_PITCH_GAUGE_INNER_RADIUS * cosf(pitch_gauge_angle_rad)));
    snapshot->pitch_needle_end_y = UIClampCoord(UIRoundFloatToInt(UI_PITCH_GAUGE_CENTER_Y -
                                                                  UI_PITCH_GAUGE_INNER_RADIUS * sinf(pitch_gauge_angle_rad)));

    if (referee_recv_info->last_shoot_data_tick_ms != 0u &&
        (HAL_GetTick() - referee_recv_info->last_shoot_data_tick_ms) <= UI_SHOOT_SPEED_STALE_MS) {
        // What: 只在最近确实收到 0x0207 时显示弹速；Why: 停火后若不主动超时清零，客户端会长期残留上一发的旧速度。
        shoot_speed_mps = referee_recv_info->ShootData.initial_speed;
    }
    snapshot->bullet_speed_milli_mps = UIRoundFloatToInt(shoot_speed_mps * 1000.0f);

    // What: 数字全部按协议要求缩放到毫单位；Why: 裁判客户端会把 `UIFloatDraw` 的 int32 除以 1000 显示，必须先在固件侧统一处理。
    snapshot->power_milli_w = UIRoundFloatToInt(interactive_data->chassis_power_w * 1000.0f);
    snapshot->bullet_num = (int32_t)referee_recv_info->ProjectileAllowance.projectile_allowance_17mm;

    buffer_ratio = (float)referee_recv_info->PowerHeatData.buffer_energy / UI_BUFFER_FULL_SCALE_J;
    buffer_ratio = UIClampFloat(buffer_ratio, 0.0f, 1.0f);
    snapshot->buffer_end_x = UIClampCoord(UIRoundFloatToInt((float)UI_BUFFER_START_X +
                                                            buffer_ratio * (float)(UI_BUFFER_END_X_MAX - UI_BUFFER_START_X)));
    snapshot->buffer_color = (referee_recv_info->GameRobotState.power_management_chassis_output != 0u) ? UI_Color_Green : UI_BUFFER_ALERT_COLOR;
    shooter_output_enabled = (referee_recv_info->GameRobotState.power_management_shooter_output != 0u) ? 1u : 0u;

    // What: `on_1` 现在不是简单开关，而是“关/正常开/想开但发射机构断电”三态；Why: 用户要求在摩擦轮打开时，如果 shooter 输出被裁判切掉，要立刻显示第三种告警颜色。
    if (interactive_data->friction_on == 0u) {
        // What: 摩擦轮命令本来就是关闭时直接显示关闭态；Why: 这时即便 shooter 总输出也被关掉，也不应误报成告警。
        snapshot->indicator_state.fric_state = UI_FRIC_INDICATOR_OFF;
    } else if (shooter_output_enabled != 0u) {
        // What: 只有在“摩擦轮要求开启且 shooter 输出正常”时才显示绿色；Why: 这样 `on_1` 才真正表达发射链当前可用。
        snapshot->indicator_state.fric_state = UI_FRIC_INDICATOR_ON;
    } else {
        // What: 摩擦轮要求开启但 shooter 总输出断电时改成断电告警态；Why: 这是本轮新增的现场排障信息，目的是让操作者一眼区分“没开”和“开了但被切电”。
        snapshot->indicator_state.fric_state = UI_FRIC_INDICATOR_POWER_CUT;
    }

    // What: 另外两个状态灯仍分别绑定小陀螺和 cap；Why: 这次只扩展 `on_1` 的颜色语义，不能顺带改变其余状态灯的数据来源。
    snapshot->indicator_state.robot_spin_on = (interactive_data->robot_spin_on != 0u) ? 1u : 0u;
    snapshot->indicator_state.cap_on = (interactive_data->cap_on != 0u) ? 1u : 0u;
}

static void UIBuildStaticFigures(uint32_t operate_type)
{
    // What: shooter 永远保持新布局里的静态朝向；Why: 你已经要求 body 反映底盘位置，而 shooter 相对画面静止。
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_SHOOTER], "sht", operate_type, UI_LAYER_MAIN, UI_Color_Yellow, UI_SHOOTER_WIDTH,
               UI_SHOOTER_START_X, UI_SHOOTER_START_Y, UI_SHOOTER_END_X, UI_SHOOTER_END_Y);

    // What: 两条 body 外侧结构线保持静态；Why: 它们属于整套 HUD 的装饰边线，不应跟着底盘姿态一起转。
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_BODY_LINE_LEFT], "bll", operate_type, UI_LAYER_MAIN, UI_Color_Orange, UI_BODY_LINE_LEFT_WIDTH,
               UI_BODY_LINE_LEFT_START_X, UI_BODY_LINE_LEFT_START_Y, UI_BODY_LINE_LEFT_END_X, UI_BODY_LINE_LEFT_END_Y);
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_BODY_LINE_RIGHT], "blr", operate_type, UI_LAYER_MAIN, UI_Color_Orange, UI_BODY_LINE_RIGHT_WIDTH,
               UI_BODY_LINE_RIGHT_START_X, UI_BODY_LINE_RIGHT_START_Y, UI_BODY_LINE_RIGHT_END_X, UI_BODY_LINE_RIGHT_END_Y);

    // What: 这 3 条线直接复用参考工程的距离尺和中心竖直基准；Why: 用户这次只要求把它们并入当前静态层，不能额外改变现有动态 UI 的数据来源。
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_METER_5], "m05", operate_type, UI_LAYER_MAIN, UI_Color_Green, UI_METER_5_WIDTH,
               UI_METER_5_START_X, UI_METER_5_START_Y, UI_METER_5_END_X, UI_METER_5_END_Y);
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_CHUI_ZHI], "cvt", operate_type, UI_LAYER_MAIN, UI_Color_Cyan, UI_CHUI_ZHI_WIDTH,
               UI_CHUI_ZHI_START_X, UI_CHUI_ZHI_START_Y, UI_CHUI_ZHI_END_X, UI_CHUI_ZHI_END_Y);
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_METER_3], "m03", operate_type, UI_LAYER_MAIN, UI_Color_Orange, UI_METER_3_WIDTH,
               UI_METER_3_START_X, UI_METER_3_START_Y, UI_METER_3_END_X, UI_METER_3_END_Y);

    // What: 这几条线和数字共同构成 RMUC pitch 圆盘刻度；Why: 旧的 `pitch_outer` 已经被取消，必须补上新的静态刻度骨架才能让红色指针有明确参照。
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_ZERO_LINE], "pz0", operate_type, UI_LAYER_MAIN, UI_Color_White, UI_PITCH_SCALE_WIDTH,
               UI_PITCH_SCALE_ZERO_START_X, UI_PITCH_SCALE_ZERO_START_Y, UI_PITCH_SCALE_ZERO_END_X, UI_PITCH_SCALE_ZERO_END_Y);
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_TOP_LINE], "ptp", operate_type, UI_LAYER_MAIN, UI_Color_Pink, UI_PITCH_SCALE_WIDTH,
               UI_PITCH_SCALE_TOP_START_X, UI_PITCH_SCALE_TOP_START_Y, UI_PITCH_SCALE_TOP_END_X, UI_PITCH_SCALE_TOP_END_Y);
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_BOTTOM_LINE], "pbt", operate_type, UI_LAYER_MAIN, UI_Color_Pink, UI_PITCH_SCALE_WIDTH,
               UI_PITCH_SCALE_BOTTOM_START_X, UI_PITCH_SCALE_BOTTOM_START_Y, UI_PITCH_SCALE_BOTTOM_END_X, UI_PITCH_SCALE_BOTTOM_END_Y);
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_NEG10_LINE], "p10", operate_type, UI_LAYER_MAIN, UI_Color_White, UI_PITCH_SCALE_WIDTH,
               UI_PITCH_SCALE_NEG10_START_X, UI_PITCH_SCALE_NEG10_START_Y, UI_PITCH_SCALE_NEG10_END_X, UI_PITCH_SCALE_NEG10_END_Y);
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_NEG20_LINE], "p20", operate_type, UI_LAYER_MAIN, UI_Color_White, UI_PITCH_SCALE_WIDTH,
               UI_PITCH_SCALE_NEG20_START_X, UI_PITCH_SCALE_NEG20_START_Y, UI_PITCH_SCALE_NEG20_END_X, UI_PITCH_SCALE_NEG20_END_Y);
    UILineDraw(&ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_NEG30_LINE], "p30", operate_type, UI_LAYER_MAIN, UI_Color_White, UI_PITCH_SCALE_WIDTH,
               UI_PITCH_SCALE_NEG30_START_X, UI_PITCH_SCALE_NEG30_START_Y, UI_PITCH_SCALE_NEG30_END_X, UI_PITCH_SCALE_NEG30_END_Y);
    UIIntDraw(&ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_VALUE_30], "n30", operate_type, UI_LAYER_MAIN, UI_Color_White,
              UI_PITCH_SCALE_FONT, UI_PITCH_SCALE_VALUE_WIDTH, UI_PITCH_SCALE_VALUE_30_X, UI_PITCH_SCALE_VALUE_30_Y, 30);
    UIIntDraw(&ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_VALUE_20], "n20", operate_type, UI_LAYER_MAIN, UI_Color_White,
              UI_PITCH_SCALE_FONT, UI_PITCH_SCALE_VALUE_WIDTH, UI_PITCH_SCALE_VALUE_20_X, UI_PITCH_SCALE_VALUE_20_Y, 20);
    UIIntDraw(&ui_static_figures[UI_STATIC_FIGURE_PITCH_SCALE_VALUE_10], "n10", operate_type, UI_LAYER_MAIN, UI_Color_White,
              UI_PITCH_SCALE_FONT, UI_PITCH_SCALE_VALUE_WIDTH, UI_PITCH_SCALE_VALUE_10_X, UI_PITCH_SCALE_VALUE_10_Y, 10);
}

static void UIBuildStateFigures(uint32_t operate_type, const UIIndicatorState_t *indicator_state)
{
    uint32_t fric_color = UI_STATE_OFF_COLOR;

    if (indicator_state == NULL) {
        return;
    }

    // What: `on_1` 先按三态计算颜色；Why: fric 现在需要同时区分关闭、正常开启和 shooter 断电三种情况，不能继续用二元布尔直出颜色。
    if (indicator_state->fric_state == UI_FRIC_INDICATOR_ON) {
        fric_color = UI_STATE_ON_COLOR;
    } else if (indicator_state->fric_state == UI_FRIC_INDICATOR_POWER_CUT) {
        fric_color = UI_STATE_POWER_CUT_COLOR;
    }

    // What: 三个圆环仍然只通过颜色切换状态；Why: 这样运行期依旧只需要发送 Change 包，不需要频繁增删对象导致客户端闪烁。
    UICircleDraw(&ui_state_figures[UI_STATE_FIGURE_ON_1], "o11", operate_type, UI_LAYER_MAIN,
                 fric_color,
                 UI_ON_WIDTH, UI_ON_1_X, UI_ON_1_Y, UI_ON_RADIUS);
    UICircleDraw(&ui_state_figures[UI_STATE_FIGURE_ON_2], "o22", operate_type, UI_LAYER_MAIN,
                 (indicator_state->robot_spin_on != 0u) ? UI_STATE_ON_COLOR : UI_STATE_OFF_COLOR,
                 UI_ON_WIDTH, UI_ON_2_X, UI_ON_2_Y, UI_ON_RADIUS);
    UICircleDraw(&ui_state_figures[UI_STATE_FIGURE_ON_3], "o33", operate_type, UI_LAYER_MAIN,
                 (indicator_state->cap_on != 0u) ? UI_STATE_ON_COLOR : UI_STATE_OFF_COLOR,
                 UI_ON_WIDTH, UI_ON_3_X, UI_ON_3_Y, UI_ON_RADIUS);
}

static void UIBuildMoveFigures(uint32_t operate_type, const UIDisplaySnapshot_t *snapshot)
{
    static const float body_fill_center_x[4] = {
        UI_BODY_FILL_CENTER_0_X,
        UI_BODY_FILL_CENTER_1_X,
        UI_BODY_FILL_CENTER_2_X,
        UI_BODY_FILL_CENTER_3_X,
    };
    int32_t body_start_x = 0;
    int32_t body_start_y = 0;
    int32_t body_end_x = 0;
    int32_t body_end_y = 0;
    uint8_t i = 0u;

    if (snapshot == NULL) {
        return;
    }

    for (i = 0u; i < 4u; i++) {
        // What: 4 条粗线围绕 body 中心按相对偏角旋转；Why: 这样既能保持“代码里的方块车身感”，又满足裁判协议的图元能力限制。
        UIRotateRelativePoint(body_fill_center_x[i], -UI_BODY_HALF_SIZE, snapshot->body_relative_angle_deg, &body_start_x, &body_start_y);
        UIRotateRelativePoint(body_fill_center_x[i], UI_BODY_HALF_SIZE, snapshot->body_relative_angle_deg, &body_end_x, &body_end_y);

        UILineDraw(&ui_move_figures[UI_MOVE_FIGURE_BODY_0 + i],
                   (char[4]){ 'b', '0' + (char)i, '0', '\0' },
                   operate_type, UI_LAYER_MAIN, UI_Color_White, UI_BODY_FILL_WIDTH,
                   UIClampCoord(UIRoundFloatToInt(UI_BODY_CENTER_X + (float)body_start_x)),
                   UIClampCoord(UIRoundFloatToInt(UI_BODY_CENTER_Y + (float)body_start_y)),
                   UIClampCoord(UIRoundFloatToInt(UI_BODY_CENTER_X + (float)body_end_x)),
                   UIClampCoord(UIRoundFloatToInt(UI_BODY_CENTER_Y + (float)body_end_y)));
    }

    // What: pitch 动态图元改成 RMUC 的红色径向指针；Why: 用户已经明确取消 `pitch_outer/pitch_line`，因此实时 pitch 需要通过圆盘角度映射来表达。
    UILineDraw(&ui_move_figures[UI_MOVE_FIGURE_PITCH_NEEDLE], "ptn", operate_type, UI_LAYER_MAIN, UI_Color_Purplish_red, UI_PITCH_NEEDLE_WIDTH,
               snapshot->pitch_needle_start_x, snapshot->pitch_needle_start_y,
               snapshot->pitch_needle_end_x, snapshot->pitch_needle_end_y);
}

static void UIBuildDataFigures(uint32_t operate_type, const UIDisplaySnapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    // What: buffer 长度严格按 buffer_energy 线性映射；Why: 用户要求通过映射方式直观看到 buffer 条变化，而不是只显示一个数字。
    UILineDraw(&ui_data_figures[UI_DATA_FIGURE_BUFFER], "buf", operate_type, UI_LAYER_MAIN, snapshot->buffer_color, UI_BUFFER_WIDTH,
               UI_BUFFER_START_X, UI_BUFFER_START_Y, snapshot->buffer_end_x, UI_BUFFER_END_Y);

    // What: 功率、pitch、弹速都用浮点图元发送；Why: 这些值需要保留小数部分，才能贴近参考图的读数风格。
    UIFloatDraw(&ui_data_figures[UI_DATA_FIGURE_POWER_VALUE], "pwr", operate_type, UI_LAYER_MAIN, UI_Color_Green,
                UI_POWER_VALUE_FONT, UI_POWER_VALUE_DIGIT, UI_POWER_VALUE_WIDTH, UI_POWER_VALUE_X, UI_POWER_VALUE_Y, snapshot->power_milli_w);
    UIFloatDraw(&ui_data_figures[UI_DATA_FIGURE_PITCH_VALUE], "ptv", operate_type, UI_LAYER_MAIN, UI_Color_Green,
                UI_PITCH_VALUE_FONT, UI_PITCH_VALUE_DIGIT, UI_PITCH_VALUE_WIDTH, UI_PITCH_VALUE_X, UI_PITCH_VALUE_Y, snapshot->pitch_value_milli_deg);
    UIIntDraw(&ui_data_figures[UI_DATA_FIGURE_BULLET_NUM], "bnu", operate_type, UI_LAYER_MAIN, UI_Color_Yellow,
              UI_BULLET_NUM_FONT, UI_BULLET_NUM_WIDTH, UI_BULLET_NUM_X, UI_BULLET_NUM_Y, snapshot->bullet_num);
    UIFloatDraw(&ui_data_figures[UI_DATA_FIGURE_BULLET_SPEED], "bsp", operate_type, UI_LAYER_MAIN, UI_Color_Yellow,
                UI_BULLET_SPEED_FONT, UI_BULLET_SPEED_DIGIT, UI_BULLET_SPEED_WIDTH, UI_BULLET_SPEED_X, UI_BULLET_SPEED_Y, snapshot->bullet_speed_milli_mps);
}

static void UIBuildStrings(uint32_t operate_type)
{
    // What: 原有文本标签继续保留当前布局；Why: 这次只是在现有 UI 上补参考工程的距离刻度文字，其他语义标签不应该被连带改动。
    UICharDraw(&ui_strings[UI_STRING_FRIC], "frc", operate_type, UI_LAYER_MAIN, UI_Color_Yellow,
               UI_LABEL_FRIC_FONT, UI_LABEL_FRIC_WIDTH, UI_LABEL_FRIC_X, UI_LABEL_FRIC_Y, "F");
    UICharDraw(&ui_strings[UI_STRING_POWER], "pow", operate_type, UI_LAYER_MAIN, UI_Color_Yellow,
               UI_LABEL_POWER_FONT, UI_LABEL_POWER_WIDTH, UI_LABEL_POWER_X, UI_LABEL_POWER_Y, "p");
    // What: `pitch` 标签跟随新圆盘区域移动到右侧；Why: 旧滑块删除后，若仍停在左侧会和新的 pitch 数值及指针完全脱节。
    UICharDraw(&ui_strings[UI_STRING_PITCH], "pit", operate_type, UI_LAYER_MAIN, UI_Color_Orange,
               UI_LABEL_PITCH_FONT, UI_LABEL_PITCH_WIDTH, UI_LABEL_PITCH_X, UI_LABEL_PITCH_Y, "pitch");
    UICharDraw(&ui_strings[UI_STRING_ROBOT], "rob", operate_type, UI_LAYER_MAIN, UI_Color_Yellow,
               UI_LABEL_ROBOT_FONT, UI_LABEL_ROBOT_WIDTH, UI_LABEL_ROBOT_X, UI_LABEL_ROBOT_Y, "r");
    UICharDraw(&ui_strings[UI_STRING_CAP], "cap", operate_type, UI_LAYER_MAIN, UI_Color_Yellow,
               UI_LABEL_CAP_FONT, UI_LABEL_CAP_WIDTH, UI_LABEL_CAP_X, UI_LABEL_CAP_Y, "c");

    // What: 这里新增参考工程里的 `3m/5m` 标尺文字；Why: 它们必须和新增的 `meter_3/meter_5` 一起出现，操作者才能直接读懂横线含义。
    UICharDraw(&ui_strings[UI_STRING_TEXT_3], "t03", operate_type, UI_LAYER_MAIN, UI_Color_Orange,
               UI_TEXT_3_FONT, UI_TEXT_3_WIDTH, UI_TEXT_3_X, UI_TEXT_3_Y, "3m");
    UICharDraw(&ui_strings[UI_STRING_TEXT_5], "t05", operate_type, UI_LAYER_MAIN, UI_Color_Green,
               UI_TEXT_5_FONT, UI_TEXT_5_WIDTH, UI_TEXT_5_X, UI_TEXT_5_Y, "5m");
}

static void UIRefreshStateChanges(const UIIndicatorState_t *indicator_state)
{
    uint8_t dirty_mask = 0u;
    uint8_t figure_index = 0u;

    if (indicator_state == NULL) {
        return;
    }

    // What: `on_1` 的脏检查改为比较三态值；Why: 同样是摩擦轮命令开启时，绿色和橙色之间切换也必须触发重发，不能被旧的布尔比较漏掉。
    if (indicator_state->fric_state != ui_runtime.last_indicator_state.fric_state) {
        dirty_mask |= (uint8_t)(1u << UI_STATE_FIGURE_ON_1);
    }
    if (indicator_state->robot_spin_on != ui_runtime.last_indicator_state.robot_spin_on) {
        dirty_mask |= (uint8_t)(1u << UI_STATE_FIGURE_ON_2);
    }
    if (indicator_state->cap_on != ui_runtime.last_indicator_state.cap_on) {
        dirty_mask |= (uint8_t)(1u << UI_STATE_FIGURE_ON_3);
    }

    if (dirty_mask == 0u) {
        return;
    }

    // What: 状态变化时只重建三个圆环图元；Why: on_1/on_2/on_3 都是低频状态，不应该拖着整组动画或数字一起刷新。
    UIBuildStateFigures(UI_Graph_Change, indicator_state);
    ui_runtime.last_indicator_state = *indicator_state;
    ui_runtime.dirty_state_mask |= dirty_mask;
    for (figure_index = 0u; figure_index < UI_STATE_FIGURE_COUNT; figure_index++) {
        if ((dirty_mask & (uint8_t)(1u << figure_index)) != 0u) {
            // What: 每次状态翻转都安排多次重发；Why: 单个 change 包若偶发丢失，后续两次补发还能把 on/off 最终拉回正确状态。
            ui_runtime.state_retry_count[figure_index] = 3u;
        }
    }
}

static void UIProcessRuntimeUpdate(uint32_t now_tick_ms)
{
    UIDisplaySnapshot_t snapshot;
    uint8_t figure_index = 0u;
    uint8_t move_dirty = 0u;
    uint8_t data_dirty = 0u;
    UIRuntimePacket_e packet_to_send = UI_RUNTIME_PACKET_NONE;

    UIBuildDisplaySnapshot(&snapshot);
    UIRefreshStateChanges(&snapshot.indicator_state);

    if (ui_runtime.dirty_state_mask == 0u && now_tick_ms >= ui_runtime.next_state_sync_due_tick_ms) {
        // What: 周期性把三个状态灯按当前真值重新标脏一次；Why: 若客户端偶发漏掉某次 change 包，后续自愈重发能把卡住的旧颜色拉回正确状态。
        UIBuildStateFigures(UI_Graph_Change, &snapshot.indicator_state);
        ui_runtime.dirty_state_mask = (uint8_t)((1u << UI_STATE_FIGURE_COUNT) - 1u);
        for (figure_index = 0u; figure_index < UI_STATE_FIGURE_COUNT; figure_index++) {
            // What: 周期自愈每个圆环只安排一次补发；Why: 常态下只需低成本纠偏，不需要像真正状态翻转那样连续重试三次。
            ui_runtime.state_retry_count[figure_index] = 1u;
        }
        do {
            ui_runtime.next_state_sync_due_tick_ms += UI_STATE_RESYNC_PERIOD_MS;
        } while (ui_runtime.next_state_sync_due_tick_ms <= now_tick_ms);
    }

    if (ui_runtime.dirty_state_mask != 0u) {
        // What: 状态灯变化走更短的发送间隔；Why: 用户直接盯着 on/off 灯看，若还跟数字和动画共用 25ms 节流会显得反应偏慢。
        if ((now_tick_ms - ui_runtime.last_packet_tick_ms) >= UI_STATE_PACKET_GAP_MS) {
            (void)UISendNextDirtyState(now_tick_ms);
        }
        return;
    }

    if ((now_tick_ms - ui_runtime.last_packet_tick_ms) < UI_RUNTIME_PACKET_GAP_MS) {
        // What: move/data 两类常规包继续受 25ms 总间隔限制；Why: 高频动画和数字更新仍要共享同一条 0x0301 通道，必须先守住总预算。
        return;
    }

    move_dirty = UIMovePacketIsDirty(&snapshot);
    data_dirty = UIDataPacketIsDirty(&snapshot);
    packet_to_send = UISelectRuntimePacket(now_tick_ms, move_dirty, data_dirty);

    if (packet_to_send == UI_RUNTIME_PACKET_MOVE) {
        UISendMovePacket(now_tick_ms);
        return;
    }

    if (packet_to_send == UI_RUNTIME_PACKET_DATA) {
        UISendDataPacket(now_tick_ms);
    }
}

static uint8_t UISendNextDirtyState(uint32_t now_tick_ms)
{
    uint8_t figure_index = 0u;

    for (figure_index = 0u; figure_index < UI_STATE_FIGURE_COUNT; figure_index++) {
        if ((ui_runtime.dirty_state_mask & (uint8_t)(1u << figure_index)) != 0u) {
            // What: 每拍只发送一个状态圆环；Why: 3 个状态同时变化的概率极低，拆开发能把瞬时带宽占用压到最低。
            UIGraphRefresh(&referee_recv_info->referee_id, 1, ui_state_figures[figure_index]);
            if (ui_runtime.state_retry_count[figure_index] > 0u) {
                ui_runtime.state_retry_count[figure_index]--;
            }
            // What: 只有在既发送成功又耗尽重发次数后才清掉脏位；Why: 这样即便首个 change 包漏掉，也会自动继续补发到稳定为止。
            if (ui_runtime.state_retry_count[figure_index] == 0u) {
                ui_runtime.dirty_state_mask &= (uint8_t)~(1u << figure_index);
            }
            ui_runtime.last_packet_tick_ms = now_tick_ms;
            return 1u;
        }
    }

    return 0u;
}

static void UISendMovePacket(uint32_t now_tick_ms)
{
    // What: 高频动画组仍固定打成 Draw5；Why: 4 条 body 线和 1 条 pitch 指针仍然刚好组成合法动态包，不需要改通信节奏。
    UIGraphRefresh(&referee_recv_info->referee_id, 5,
                   ui_move_figures[UI_MOVE_FIGURE_BODY_0],
                   ui_move_figures[UI_MOVE_FIGURE_BODY_1],
                   ui_move_figures[UI_MOVE_FIGURE_BODY_2],
                   ui_move_figures[UI_MOVE_FIGURE_BODY_3],
                   ui_move_figures[UI_MOVE_FIGURE_PITCH_NEEDLE]);

    memcpy(ui_runtime.last_move_figures, ui_move_figures, sizeof(ui_runtime.last_move_figures));
    do {
        ui_runtime.next_move_due_tick_ms += UI_MOVE_PERIOD_MS;
    } while (ui_runtime.next_move_due_tick_ms <= now_tick_ms);
    ui_runtime.last_packet_tick_ms = now_tick_ms;
}

static void UISendDataPacket(uint32_t now_tick_ms)
{
    // What: 数值组同样固定打成 Draw5；Why: 这样 buffer 和 4 个数字就能共用一个低频包，不需要为每个数字单独占一帧。
    UIGraphRefresh(&referee_recv_info->referee_id, 5,
                   ui_data_figures[UI_DATA_FIGURE_BUFFER],
                   ui_data_figures[UI_DATA_FIGURE_POWER_VALUE],
                   ui_data_figures[UI_DATA_FIGURE_PITCH_VALUE],
                   ui_data_figures[UI_DATA_FIGURE_BULLET_NUM],
                   ui_data_figures[UI_DATA_FIGURE_BULLET_SPEED]);

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
            // What: 到期但无变化时直接滚动到下一个周期；Why: 车体和 pitch 都静止时不应继续空发高频动画包。
            do {
                ui_runtime.next_move_due_tick_ms += UI_MOVE_PERIOD_MS;
            } while (ui_runtime.next_move_due_tick_ms <= now_tick_ms);
        }
    }

    if (now_tick_ms >= ui_runtime.next_data_due_tick_ms) {
        if (data_dirty != 0u) {
            data_due = 1u;
        } else {
            // What: 到期但无变化时同样跳过；Why: 弹速归零且数值稳定后，没有必要继续浪费低频数据预算。
            do {
                ui_runtime.next_data_due_tick_ms += UI_DATA_PERIOD_MS;
            } while (ui_runtime.next_data_due_tick_ms <= now_tick_ms);
        }
    }

    if (move_due != 0u && data_due != 0u) {
        // What: 同时到期时优先发截止更早的那组；Why: 这样在共享总带宽下更接近 20Hz/10Hz 的目标节奏。
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
    // What: 统一使用四舍五入把浮点量转成协议整数；Why: 可以减少边界小抖动导致的数字来回跳字。
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
