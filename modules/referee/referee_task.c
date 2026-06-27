/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

// UI调试开关: 1=忽略真实数据,自动循环切换各模式以检查UI/裁判系统链路; 0=显示chassis喂入的真实机器状态(比赛用)
#define UI_DEBUG_MODE_TEST 0

// 缓冲能量条参数: 条长正比于剩余缓冲能量。满缓冲(未超功率)=满条;超功率消耗缓冲则条变短,提醒别再超
#define BUFFER_ENERGY_MAX  60.0f // 缓冲能量满值(J),裁判系统默认60
#define UI_PWR_BAR_X_START 720u  // 能量条/方框左端x(空)
#define UI_PWR_BAR_X_END   1220u // 方框右端x(满)
#define UI_PWR_BAR_Y       160u  // 能量条y(方框中线)

// 相机视野标定框:居中矩形,改下面4个值即可调整大小/位置(屏幕1920x1080,中心960,540)
#define CALIB_BOX_CX 960 // 中心x
#define CALIB_BOX_CY 480 // 中心y
#define CALIB_BOX_HW 400 // 半宽(框总宽=2*HW)
#define CALIB_BOX_HH 300 // 半高(框总高=2*HH)

// 底盘通过轨迹线:画面下端两条对称斜边(防撞墙参考线,原点左下角,y向上)
#define TRACK_LINE_CX     960 // 中心x
#define TRACK_LINE_Y_BOT  100 // 斜边底端y(靠屏幕下沿)
#define TRACK_LINE_Y_TOP  400 // 斜边顶端y
#define TRACK_LINE_HW_BOT 400 // 底端半宽(距中心x)
#define TRACK_LINE_HW_TOP 170 // 顶端半宽(距中心x)

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void UIStaticDraw(void);                                           // 清屏并重绘全部图层(首次初始化与一键重置共用)
static void UIForceRefreshAll(Referee_Interactive_info_t *_Interactive_data); // 作废last值,强制动态项刷新到当前真实状态
#if UI_DEBUG_MODE_TEST
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化
#endif

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
#if UI_DEBUG_MODE_TEST
    RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
#endif
    // 一键重置UI(R键): 清屏重绘全部静态图层, 并作废last值让动态项随后刷新回当前真实状态
    if (Interactive_data->ui_reset_request)
    {
        UIStaticDraw();
        UIForceRefreshAll(Interactive_data);
        Interactive_data->ui_reset_request = 0;
    }
    MyUIRefresh(referee_recv_info, Interactive_data);
}

// 作废"上一次"模式记录值,使UIChangeCheck在下一次刷新时判定各模式项均已改变,从而重绘到当前真实状态.
// 配合UIStaticDraw使用: 静态重绘后模式项停留在默认文案(如"zeroforce"), 本函数确保它们被刷新成当前模式.
// 注意: 缓冲能量条故意不在此强制刷新. 其静态重绘已是满格默认, 若在此作废last值会强制画出当前buffer,
//       而裁判系统该字段在重连瞬间/未上线时常为0, 会让能量条被刷成空格. 保持与开机一致, 只在真实变化时更新.
static void UIForceRefreshAll(Referee_Interactive_info_t *_Interactive_data)
{
    _Interactive_data->chassis_last_mode  = (chassis_mode_e)0xFF;
    _Interactive_data->gimbal_last_mode   = (gimbal_mode_e)0xFF;
    _Interactive_data->shoot_last_mode    = (shoot_mode_e)0xFF;
    _Interactive_data->friction_last_mode = (friction_mode_e)0xFF;
    _Interactive_data->lid_last_mode      = (lid_mode_e)0xFF;
    _Interactive_data->vision_last_state  = 0xFF;
}

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_Energy[3];      // 电容能量条
static String_Data_t UI_State_sta[6];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 420};
static Graph_Data_t UI_calib_box;     // 相机视野标定框
static Graph_Data_t UI_track_line[2]; // 底盘通过轨迹线(两条斜边)
static String_Data_t UI_vision_label; // "AUTOAIM" 静态标签
static String_Data_t UI_vision_state; // 视觉目标状态,动态

// 视觉目标状态(Target_State_e: 0=无目标,1=收敛中,2=可开火)对应的显示串(等长10字符,保证change覆盖干净)
static const char *VisionStateStr(uint8_t s)
{
    switch (s)
    {
    case 1:  return "CONVERGING";
    case 2:  return "FIRE READY";
    default: return "NO TARGET ";
    }
}
// 状态对应颜色: 无目标白, 收敛中黄, 可开火绿
static uint32_t VisionStateColor(uint8_t s)
{
    switch (s)
    {
    case 1:  return UI_Color_Yellow;
    case 2:  return UI_Color_Green;
    default: return UI_Color_White;
    }
}

// AUTOAIM状态显示位置(暂置于画面正中心,排除画在视野外的问题; 坐标原点左下角,y向上,屏幕1920x1080,中心960,540)
#define UI_AIM_LABEL_X 800u // "AUTOAIM"标签起始x
#define UI_AIM_STATE_X 1010u // 状态串起始x
#define UI_AIM_Y       900u  // 画面中心y(确认可见后再上移到顶部)

void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    UIStaticDraw(); // 首次绘制全部图层
}

// 清屏并重绘全部静态+动态图层. 首次初始化调用; 一键重置(R键)时再次调用以恢复被裁判系统丢包吞掉的UI
static void UIStaticDraw(void)
{
    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制发射基准线
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);

    // 相机视野标定框(居中大矩形,尺寸由上方CALIB_BOX_*宏控制)
    UIRectangleDraw(&UI_calib_box, "cb0", UI_Graph_ADD, 6, UI_Color_Cyan, 3,
                    CALIB_BOX_CX - CALIB_BOX_HW, CALIB_BOX_CY - CALIB_BOX_HH,
                    CALIB_BOX_CX + CALIB_BOX_HW, CALIB_BOX_CY + CALIB_BOX_HH);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_calib_box);

    // 底盘通过轨迹线:画面下端两条对称斜边(防撞墙;尺寸由TRACK_LINE_*宏控制)
    UILineDraw(&UI_track_line[0], "tl0", UI_Graph_ADD, 6, UI_Color_Green, 3,
               TRACK_LINE_CX - TRACK_LINE_HW_BOT, TRACK_LINE_Y_BOT,
               TRACK_LINE_CX - TRACK_LINE_HW_TOP, TRACK_LINE_Y_TOP);
    UILineDraw(&UI_track_line[1], "tl1", UI_Graph_ADD, 6, UI_Color_Green, 3,
               TRACK_LINE_CX + TRACK_LINE_HW_BOT, TRACK_LINE_Y_BOT,
               TRACK_LINE_CX + TRACK_LINE_HW_TOP, TRACK_LINE_Y_TOP);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_track_line[0], UI_track_line[1]);

    // 画面正上方:AUTOAIM标签(静态) + 视觉目标状态(动态,初始按vision_last_state=0即NO TARGET绘制)
    UICharDraw(&UI_vision_label, "va0", UI_Graph_ADD, 8, UI_Color_Cyan, 20, 2, UI_AIM_LABEL_X, UI_AIM_Y, "AUTOAIM");
    UICharRefresh(&referee_recv_info->referee_id, UI_vision_label);
    UICharDraw(&UI_vision_state, "va1", UI_Graph_ADD, 8, VisionStateColor(0), 20, 2, UI_AIM_STATE_X, UI_AIM_Y, "%s", VisionStateStr(0));
    UICharRefresh(&referee_recv_info->referee_id, UI_vision_state);

    // 绘制车辆状态标志指示
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 150, 750, "chassis:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 700, "gimbal:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 650, "shoot:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 600, "frict:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 550, "lid:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 600, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 550, "open ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

    // 缓冲能量显示，静态(标签+方框)
    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Buffer:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    // 能量条框
    UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // 缓冲能量显示,动态:初始按满缓冲(60J)绘制满条,真实数据到来后由MyUIRefresh更新
    UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, (uint32_t)(BUFFER_ENERGY_MAX * 1000));
    UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Green, 30, UI_PWR_BAR_X_START, UI_PWR_BAR_Y, UI_PWR_BAR_X_END, UI_PWR_BAR_Y);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
}

#if UI_DEBUG_MODE_TEST
// 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
static uint8_t count = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
    count++;
    if (count >= 50)
    {
        count = 0;
        count1++;
    }
    switch (count1 % 4)
    {
    case 0:
    {
        _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        _Interactive_data->Chassis_Power_Data.chassis_power_mx += 3.5;
        if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 18)
            _Interactive_data->Chassis_Power_Data.chassis_power_mx = 0;
        break;
    }
    case 1:
    {
        _Interactive_data->chassis_mode = CHASSIS_ROTATE_CLOCKWISE;
        _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    case 2:
    {
        _Interactive_data->chassis_mode = CHASSIS_NO_FOLLOW;
        _Interactive_data->gimbal_mode = GIMBAL_ANGLE_MODE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        break;
    }
    case 3:
    {
        _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    default:
        break;
    }
}
#endif // UI_DEBUG_MODE_TEST

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        switch (_Interactive_data->chassis_mode)
        {
        case CHASSIS_ZERO_FORCE:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
            break;
        case CHASSIS_ROTATE_CLOCKWISE:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "rotate   ");
            // 此处注意字数对齐问题，字数相同才能覆盖掉
            break;
        case CHASSIS_ROTATE_COUNTERCLOCKWISE:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "rotate   ");
            // 此处注意字数对齐问题，字数相同才能覆盖掉
            break;
        case CHASSIS_NO_FOLLOW:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "nofollow ");
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "follow   ");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
    // gimbal
    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    {
        switch (_Interactive_data->gimbal_mode)
        {
        case GIMBAL_ZERO_FORCE:
        {
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
            break;
        }
        case GIMBAL_FREE_MODE:
        {
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "free     ");
            break;
        }
        case GIMBAL_ANGLE_MODE:
        {
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "angle    ");
            break;
        }
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }
    // shoot
    if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    {
        UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, _Interactive_data->shoot_mode == SHOOT_ON ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    }
    // friction
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, _Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }
    // lid
    if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1)
    {
        UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, _Interactive_data->lid_mode == LID_OPEN ? "open " : "close");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    }
    // buffer energy: 条长正比于剩余缓冲能量。满=未超功率,变短=正在消耗缓冲(超功率),提醒收油门
    if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    {
        float buffer = _Interactive_data->Chassis_Power_Data.chassis_power_mx; // 该字段复用承载缓冲能量(J)
        if (buffer > BUFFER_ENERGY_MAX)
            buffer = BUFFER_ENERGY_MAX; // 夹紧,避免异常值把条画出框外
        if (buffer < 0)
            buffer = 0;
        // 按 剩余/满 比例映射到方框宽度
        uint32_t bar_end = UI_PWR_BAR_X_START + (uint32_t)(buffer / BUFFER_ENERGY_MAX * (UI_PWR_BAR_X_END - UI_PWR_BAR_X_START));
        // 颜色随剩余量预警: 充足绿 / 偏低黄 / 告急红(pink)
        uint32_t bar_color = buffer > 40.0f ? UI_Color_Green : (buffer > 15.0f ? UI_Color_Yellow : UI_Color_Pink);
        UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, bar_color, 18, 2, 2, 750, 230, buffer * 1000);
        UILineDraw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, bar_color, 30, UI_PWR_BAR_X_START, UI_PWR_BAR_Y, bar_end, UI_PWR_BAR_Y);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    }
    // vision: 画面正上方的AUTOAIM目标状态(随状态变色)
    if (_Interactive_data->Referee_Interactive_Flag.vision_flag == 1)
    {
        UICharDraw(&UI_vision_state, "va1", UI_Graph_Change, 8, VisionStateColor(_Interactive_data->vision_state), 20, 2, UI_AIM_STATE_X, UI_AIM_Y, "%s", VisionStateStr(_Interactive_data->vision_state));
        UICharRefresh(&referee_recv_info->referee_id, UI_vision_state);
        _Interactive_data->Referee_Interactive_Flag.vision_flag = 0;
    }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }

    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
        _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    }

    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }

    if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
        _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    }

    if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    {
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
        _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    }

    if (_Interactive_data->vision_state != _Interactive_data->vision_last_state)
    {
        _Interactive_data->Referee_Interactive_Flag.vision_flag = 1;
        _Interactive_data->vision_last_state = _Interactive_data->vision_state;
    }
}
