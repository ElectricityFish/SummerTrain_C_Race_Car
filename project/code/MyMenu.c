#include "MyMenu.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"
#include "Image.h"
#include "Image_Process.h"
#include "Control.h"
#include "Encoder.h"
#include "SpeedControl.h"
#include "SpeedDecision.h"
#include "Kfilter.h"
#include "Wireless.h"
#include "FS-A8S.h"

#define MENU_FONT_WIDTH             (8)
#define MENU_FONT_HEIGHT            (16)
#define MENU_ITEM_START_Y           (24)
#define MENU_VISIBLE_ROWS           (18)
#define MENU_NAME_X                 (16)
#define MENU_VALUE_X                (136)
#define MENU_KEY_REPEAT_TICKS       (100 / MENU_KEY_SCAN_PERIOD_MS)

Menu_Item head;		//创建根节点
Menu_Item *key;		//指向当前光标所在的菜单项

static Menu_Item *image_preview_item = NULL;
static Menu_Item *image_fps_item = NULL;
static Menu_Item *cargo_folder = NULL;
static Menu_Item *base_control_folder = NULL;
static Menu_Item *wireless_control_folder = NULL;
static Menu_Item *check_folder = NULL;
static Menu_Item *protect_folder = NULL;
static Menu_Item *speed_debug_folder = NULL;
static Menu_Item *speed_left_pid_folder = NULL;
static Menu_Item *speed_right_pid_folder = NULL;
static Menu_Item *differential_gain_item = NULL;
static Menu_Item *image_send_folder = NULL;
static Menu_Item *image_send_origin_item = NULL;
static Menu_Item *image_send_processed_item = NULL;
static Menu_Item *image_send_both_item = NULL;
uint16 image_fps_menu_value = 0;
static uint8 cargo_state_menu_value = COMMON_STATE_IDLE;
static uint8 cargo_fault_menu_value = CAR_PROTECTION_REASON_NONE;
static uint8 image_send_status_menu_value = WIRELESS_IMAGE_SEND_NOT_SENT;
static uint8 image_send_action_menu_value = 0;
static uint8 protect_zebra_menu_value = 0U;
static uint8 protect_out_menu_value = 0U;
static uint8 protect_out_control_menu_value = 0U;
static uint8 protect_ok_menu_value = 0U;
static int16 check_encoder_left_menu_value = 0;
static int16 check_encoder_right_menu_value = 0;
static float check_yaw_menu_value = 0.0f;
static float check_pitch_menu_value = 0.0f;
static float check_roll_menu_value = 0.0f;
static uint32 fs_a8s_menu_frame_count = 0U;
static int16 speed_last_left_actual = 0;
static int16 speed_last_right_actual = 0;
static int16 speed_last_left_out = 0;
static int16 speed_last_right_out = 0;
static float speed_last_left_error = 0.0f;
static float speed_last_right_error = 0.0f;

static uint8_t menu_view_first = 0;		//当前页面显示的第一个菜单项编号
static bool menu_refresh_required = true;
static uint8_t key_long_count[KEY_NUMBER];

//取得当前页面第一个需要显示的菜单项
static Menu_Item *menu_get_visible_first(void)
{
	Menu_Item *item;
	uint8_t i;

	if(key == NULL || key->father == NULL)
	{
		return NULL;
	}

	item = key->father->first_son;
	for(i = 0; i < menu_view_first && item != NULL; i++)
	{
		item = item->next_brother;
	}
	return item;
}

//限制字符串显示长度，防止IPS200坐标越界
static void menu_show_text(uint16 x, uint16 y, const char *text, uint8_t max_chars)
{
	uint8_t i;

	if(text == NULL)
	{
		return;
	}

	for(i = 0; i < max_chars && text[i] != '\0'; i++)
	{
		ips200_show_char(x + i * MENU_FONT_WIDTH, y, text[i]);
	}
}

//逐飞浮点显示函数会直接截断小数，这里先加上半个末位单位实现四舍五入
static float menu_round_float_for_display(float value, uint8_t point_num)
{
	float half_unit = 0.5f;

	while(point_num > 0)
	{
		half_unit /= 10.0f;
		point_num--;
	}

	if(value >= 0.0f)
	{
		return value + half_unit;
	}
	return value - half_unit;
}

//判断当前是否已进入图像预览页面
static bool menu_is_image_preview(void)
{
	return (key != NULL && key == image_preview_item);
}

//判断当前显示的是否为Check目录。该页需要原地刷新编码器数值。
static bool menu_is_check_page(void)
{
	return (key != NULL && key->father == check_folder);
}

// 判断当前显示的是否为 Base_Control 目录。该页需要原地刷新状态和故障原因。
static bool menu_is_base_control_page(void)
{
	return (key != NULL && key->father == base_control_folder);
}

// 判断当前显示的是否为 Check/send_img 页面。该页需要实时刷新单帧图传状态。
static bool menu_is_image_send_page(void)
{
	return (key != NULL && key->father == image_send_folder);
}

static bool menu_is_protect_page(void)
{
	return (key != NULL && key->father == protect_folder);
}

static bool menu_is_wireless_control_page(void)
{
	return (key != NULL && key->father == wireless_control_folder);
}

static bool menu_is_speed_debug_page(void)
{
	return (key != NULL)
		&& ((key->father == speed_debug_folder)
			|| (key->father == speed_left_pid_folder)
			|| (key->father == speed_right_pid_folder));
}

// i-BUS 约每 7ms 一帧；菜单按每 4 帧刷新一次，避免屏幕被高频更新占满。
static bool menu_update_fs_a8s_values(void)
{
	uint32 valid_frame_count = fs_a8s_channel_data.valid_frame_count;

	if((uint32)(valid_frame_count - fs_a8s_menu_frame_count) < 4U)
	{
		return false;
	}

	fs_a8s_menu_frame_count = valid_frame_count;
	return true;
}

static bool menu_update_speed_values(void)
{
	bool changed = false;

	if(speed_last_left_actual != speed_left_pid.ActualPulse)
	{
		speed_last_left_actual = speed_left_pid.ActualPulse;
		changed = true;
	}
	if(speed_last_right_actual != speed_right_pid.ActualPulse)
	{
		speed_last_right_actual = speed_right_pid.ActualPulse;
		changed = true;
	}
	if(speed_last_left_out != speed_left_pid.Out)
	{
		speed_last_left_out = speed_left_pid.Out;
		changed = true;
	}
	if(speed_last_right_out != speed_right_pid.Out)
	{
		speed_last_right_out = speed_right_pid.Out;
		changed = true;
	}
	if(speed_last_left_error != speed_left_pid.Error)
	{
		speed_last_left_error = speed_left_pid.Error;
		changed = true;
	}
	if(speed_last_right_error != speed_right_pid.Error)
	{
		speed_last_right_error = speed_right_pid.Error;
		changed = true;
	}

	return changed;
}

static bool menu_update_cargo_values(void)
{
	uint8 state_value = (uint8)common_state;
	uint8 fault_value = car_protection_reason;
	bool changed = false;

	if(cargo_state_menu_value != state_value)
	{
		cargo_state_menu_value = state_value;
		changed = true;
	}
	if(cargo_fault_menu_value != fault_value)
	{
		cargo_fault_menu_value = fault_value;
		changed = true;
	}
	return changed;
}

static bool menu_update_image_send_status(void)
{
	uint8 status_value = wireless_image_send_status;

	if(image_send_status_menu_value == status_value)
	{
		return false;
	}

	image_send_status_menu_value = status_value;
	return true;
}

static bool menu_update_protect_values(void)
{
	uint8 reason = car_stop_reason;
	uint8 zebra_value = ((reason & CAR_STOP_REASON_ZEBRA) != 0U) ? 1U : 0U;
	uint8 out_value = ((reason & CAR_STOP_REASON_OUT_OF_BOUNDS) != 0U) ? 1U : 0U;
	uint8 out_control_value = ((reason & CAR_STOP_REASON_OUT_CONTROL) != 0U) ? 1U : 0U;
	uint8 ok_value = ((reason & CAR_STOP_REASON_OK) != 0U) ? 1U : 0U;
	bool changed = false;

	if(protect_zebra_menu_value != zebra_value)
	{
		protect_zebra_menu_value = zebra_value;
		changed = true;
	}
	if(protect_out_menu_value != out_value)
	{
		protect_out_menu_value = out_value;
		changed = true;
	}
	if(protect_out_control_menu_value != out_control_value)
	{
		protect_out_control_menu_value = out_control_value;
		changed = true;
	}
	if(protect_ok_menu_value != ok_value)
	{
		protect_ok_menu_value = ok_value;
		changed = true;
	}
	return changed;
}

//把中断中更新的脉冲结果同步到菜单绑定变量，返回值表示本次是否有变化。
static bool menu_update_check_values(void)
{
	int16 encoder_left_value = encoder_left_pulse;
	int16 encoder_right_value = encoder_right_pulse;
	float yaw_value = yaw;
	float pitch_value = pitch;
	float roll_value = roll;
	bool changed = false;

	if(check_encoder_left_menu_value != encoder_left_value)
	{
		check_encoder_left_menu_value = encoder_left_value;
		changed = true;
	}
	if(check_encoder_right_menu_value != encoder_right_value)
	{
		check_encoder_right_menu_value = encoder_right_value;
		changed = true;
	}
	if(check_yaw_menu_value != yaw_value)
	{
		check_yaw_menu_value = yaw_value;
		changed = true;
	}
	if(check_pitch_menu_value != pitch_value)
	{
		check_pitch_menu_value = pitch_value;
		changed = true;
	}
	if(check_roll_menu_value != roll_value)
	{
		check_roll_menu_value = roll_value;
		changed = true;
	}
	return changed;
}

//光标移动后，让当前项始终处于可见区域
static void menu_update_view(void)
{
	if(key == NULL)
	{
		menu_view_first = 0;
		return;
	}

	if(key->no < menu_view_first)
	{
		menu_view_first = key->no;
	}
	else if(key->no >= (uint8_t)(menu_view_first + MENU_VISIBLE_ROWS))
	{
		menu_view_first = key->no - MENU_VISIBLE_ROWS + 1;
	}
}

//取得同一级菜单中的最后一项，用于向上首尾循环
static Menu_Item *menu_get_last_brother(Menu_Item *item)
{
	if(item == NULL || item->father == NULL)
	{
		return NULL;
	}

	item = item->father->first_son;
	while(item != NULL && item->next_brother != NULL)
	{
		item = item->next_brother;
	}
	return item;
}

//通过初始化函数在任意节点下添加任意东西
void menu_init(void)
{
	Menu_Item *image_folder;
	Menu_Item *process_folder;
	Menu_Item *pid_folder;
	Menu_Item *servo_pid_folder;
	Menu_Item *differential_folder;
	Menu_Item *item;

	menu_pool_reset();

	//根节点初始化
	head.data = NULL;
	head.father = NULL;
	head.first_son = NULL;
	head.kind = MENU_Folder;
	head.last_brother = NULL;
	head.name = "MENU";
	head.next_brother = NULL;
	head.sons = 0;
	head.no = 0;
	head.select = false;
	head.editable = false;
	head.min_value = 0.0f;
	head.max_value = 0.0f;
	head.step = 0.0f;

	// CarGo 下分为原有菜单控制和无线遥控控制两个目录。
	cargo_folder = create_menu_folder_dynamic(&head, "CarGo");
	if(cargo_folder != NULL)
	{
		base_control_folder = create_menu_folder_dynamic(cargo_folder, "Base_Control");
		if(base_control_folder != NULL)
		{
			create_menu_number_range_dynamic(base_control_folder, "RunCmd", (void *)&car_go_command, uint8_Box, 0.0f, 1.0f, 1.0f);
			item = create_menu_number_dynamic(base_control_folder, "State", &cargo_state_menu_value, uint8_Box);
			if(item != NULL)
			{
				item->editable = false;
			}
			item = create_menu_number_dynamic(base_control_folder, "Fault", &cargo_fault_menu_value, uint8_Box);
			if(item != NULL)
			{
				item->editable = false;
			}
		}

		wireless_control_folder = create_menu_folder_dynamic(cargo_folder, "Wireless_Control");
		if(wireless_control_folder != NULL)
		{
			create_menu_number_range_dynamic(wireless_control_folder, "Enable", (void *)&wireless_control_enabled, uint8_Box, 0.0f, 1.0f, 1.0f);
			item = create_menu_number_dynamic(wireless_control_folder, "CH1", (void *)&fs_a8s_channel_data.channel[0], uint16_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(wireless_control_folder, "CH2", (void *)&fs_a8s_channel_data.channel[1], uint16_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(wireless_control_folder, "CH3", (void *)&fs_a8s_channel_data.channel[2], uint16_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(wireless_control_folder, "CH4", (void *)&fs_a8s_channel_data.channel[3], uint16_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(wireless_control_folder, "CH5_SWB", (void *)&fs_a8s_channel_data.channel[4], uint16_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(wireless_control_folder, "CH6_SWC", (void *)&fs_a8s_channel_data.channel[5], uint16_Box);
			if(item != NULL) item->editable = false;
		}
	}

	// 最近一次停车原因；所有值只读，并在下一次从停车状态发车时统一清零。
	protect_folder = create_menu_folder_dynamic(&head, "protect");
	if(protect_folder != NULL)
	{
		menu_update_protect_values();
		item = create_menu_number_dynamic(protect_folder, "ZEBAR", &protect_zebra_menu_value, uint8_Box);
		if(item != NULL) item->editable = false;
		item = create_menu_number_dynamic(protect_folder, "OUT", &protect_out_menu_value, uint8_Box);
		if(item != NULL) item->editable = false;
		item = create_menu_number_dynamic(protect_folder, "OutControl", &protect_out_control_menu_value, uint8_Box);
		if(item != NULL) item->editable = false;
		item = create_menu_number_dynamic(protect_folder, "OK", &protect_ok_menu_value, uint8_Box);
		if(item != NULL) item->editable = false;
	}

	//图像目录包含采集帧率、处理结果预览和基础巡线参数。
	image_folder = create_menu_folder_dynamic(&head, "Image");
	image_fps_menu_value = image_get_capture_fps();
	image_fps_item = create_menu_number_dynamic(image_folder, "FPS", &image_fps_menu_value, uint16_Box);
	if(image_fps_item != NULL)
	{
		image_fps_item->editable = false;		//帧率是采集统计结果，禁止在菜单中修改
	}
	image_preview_item = create_menu_folder_dynamic(image_folder, "Preview");
	process_folder = create_menu_folder_dynamic(image_folder, "Process");
	if(process_folder != NULL)
	{
		create_menu_number_range_dynamic(process_folder, "RefRows", &image_process_config.reference_rows, uint8_Box, 1.0f, 20.0f, 1.0f);
		create_menu_number_range_dynamic(process_folder, "RefCols", &image_process_config.reference_cols, uint8_Box, 20.0f, 180.0f, 2.0f);
		create_menu_number_range_dynamic(process_folder, "Black", &image_process_config.black_threshold, uint8_Box, 0.0f, 200.0f, 1.0f);
		create_menu_number_range_dynamic(process_folder, "WhiteMin", &image_process_config.white_min_scale, uint8_Box, 1.0f, 10.0f, 1.0f);
		create_menu_number_range_dynamic(process_folder, "WhiteMax", &image_process_config.white_max_scale, uint8_Box, 10.0f, 20.0f, 1.0f);
		create_menu_number_range_dynamic(process_folder, "Contrast", &image_process_config.contrast_threshold, uint8_Box, 1.0f, 100.0f, 1.0f);
		create_menu_number_range_dynamic(process_folder, "Offset", &image_process_config.contrast_offset, uint8_Box, 1.0f, 8.0f, 1.0f);
		create_menu_number_range_dynamic(process_folder, "Range", &image_process_config.search_range, uint8_Box, 1.0f, 60.0f, 1.0f);
		create_menu_number_range_dynamic(process_folder, "WeightRow", &image_process_config.weight_center_row, uint8_Box, 0.0f, 119.0f, 1.0f);
		create_menu_number_range_dynamic(process_folder, "WeightSpan", &image_process_config.weight_span, uint8_Box, 1.0f, 80.0f, 1.0f);
		create_menu_number_range_dynamic(process_folder, "WeightPeak", &image_process_config.weight_peak, uint8_Box, 1.0f, 50.0f, 1.0f);
		create_menu_number_range_dynamic(process_folder, "Smooth", &image_process_config.mid_filter_current, uint8_Box, 0.0f, 100.0f, 1.0f);
		create_menu_number_range_dynamic(process_folder, "CurveBias", &image_process_config.curve_inner_bias, uint8_Box, 0.0f, 20.0f, 1.0f);
	}

	//视觉转向动态 PID 参数。KpNow 是实时计算结果，仅用于观察。
	pid_folder = create_menu_folder_dynamic(&head, "PID");
	servo_pid_folder = create_menu_folder_dynamic(pid_folder, "servo_pid");
	if(servo_pid_folder != NULL)
	{
		create_menu_number_range_dynamic(servo_pid_folder, "KpMin", &servo_pid.KpMin, float_Box, 0.0f, 3.0f, 0.01f);
		create_menu_number_range_dynamic(servo_pid_folder, "KpMax", &servo_pid.KpMax, float_Box, 0.0f, 3.0f, 0.01f);
		create_menu_number_range_dynamic(servo_pid_folder, "ErrFull", &servo_pid.ErrorFull, float_Box, 1.0f, 120.0f, 1.0f);
		create_menu_number_range_dynamic(servo_pid_folder, "ki", &servo_pid.Ki, float_Box, 0.0f, 10.0f, 0.01f);
		create_menu_number_range_dynamic(servo_pid_folder, "kd", &servo_pid.Kd, float_Box, 0.0f, 10.0f, 0.01f);
		create_menu_number_range_dynamic(servo_pid_folder, "kd2", &servo_pid.Kd2, float_Box, 0.0f, 1.0f, 0.01f);
		item = create_menu_number_dynamic(servo_pid_folder, "KpNow", &servo_pid.KpNow, float_Box);
		if(item != NULL)
		{
			item->editable = false;
		}
		item = create_menu_number_dynamic(servo_pid_folder, "YawRate", &servo_pid.YawRate, float_Box);
		if(item != NULL)
		{
			item->editable = false;
		}
	}

	// RUNNING 使用 RunTarget；IDLE 下可用 Enable + Run + 单轮 On 独立调试速度 PI。
	speed_debug_folder = create_menu_folder_dynamic(&head, "Speed");
	if(speed_debug_folder != NULL)
	{
		create_menu_number_range_dynamic(speed_debug_folder, "RunTarget", (void *)&speed_running_target_pulse,
			int16_Box, 0.0f, (float)SPEED_CONTROL_RUN_TARGET_MAX, 10.0f);
		create_menu_number_range_dynamic(speed_debug_folder, "DebugEnable", (void *)&speed_debug_enabled,
			uint8_Box, 0.0f, 1.0f, 1.0f);
		create_menu_number_range_dynamic(speed_debug_folder, "DebugRun", (void *)&speed_debug_run,
			uint8_Box, 0.0f, 1.0f, 1.0f);

		speed_left_pid_folder = create_menu_folder_dynamic(speed_debug_folder, "Left");
		if(speed_left_pid_folder != NULL)
		{
			create_menu_number_range_dynamic(speed_left_pid_folder, "On", (void *)&speed_debug_left_enabled, uint8_Box, 0.0f, 1.0f, 1.0f);
			create_menu_number_range_dynamic(speed_left_pid_folder, "Target", (void *)&speed_left_pid.TargetPulse,
				int16_Box, -(float)SPEED_CONTROL_TARGET_ABS_MAX, (float)SPEED_CONTROL_TARGET_ABS_MAX, 10.0f);
			create_menu_number_range_dynamic(speed_left_pid_folder, "Kp", (void *)&speed_left_pid.Kp, float_Box, 0.0f, 10.0f, 0.01f);
			create_menu_number_range_dynamic(speed_left_pid_folder, "Ki", (void *)&speed_left_pid.Ki, float_Box, 0.0f, 10.0f, 0.01f);
			create_menu_number_range_dynamic(speed_left_pid_folder, "MaxPWM", (void *)&speed_left_pid.OutMax, int16_Box, 0.0f, 8000.0f, 50.0f);
			item = create_menu_number_dynamic(speed_left_pid_folder, "Actual", (void *)&speed_left_pid.ActualPulse, int16_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(speed_left_pid_folder, "Error", (void *)&speed_left_pid.Error, float_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(speed_left_pid_folder, "Out", (void *)&speed_left_pid.Out, int16_Box);
			if(item != NULL) item->editable = false;
		}

		speed_right_pid_folder = create_menu_folder_dynamic(speed_debug_folder, "Right");
		if(speed_right_pid_folder != NULL)
		{
			create_menu_number_range_dynamic(speed_right_pid_folder, "On", (void *)&speed_debug_right_enabled, uint8_Box, 0.0f, 1.0f, 1.0f);
			create_menu_number_range_dynamic(speed_right_pid_folder, "Target", (void *)&speed_right_pid.TargetPulse,
				int16_Box, -(float)SPEED_CONTROL_TARGET_ABS_MAX, (float)SPEED_CONTROL_TARGET_ABS_MAX, 10.0f);
			create_menu_number_range_dynamic(speed_right_pid_folder, "Kp", (void *)&speed_right_pid.Kp, float_Box, 0.0f, 10.0f, 0.01f);
			create_menu_number_range_dynamic(speed_right_pid_folder, "Ki", (void *)&speed_right_pid.Ki, float_Box, 0.0f, 10.0f, 0.01f);
			create_menu_number_range_dynamic(speed_right_pid_folder, "MaxPWM", (void *)&speed_right_pid.OutMax, int16_Box, 0.0f, 8000.0f, 50.0f);
			item = create_menu_number_dynamic(speed_right_pid_folder, "Actual", (void *)&speed_right_pid.ActualPulse, int16_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(speed_right_pid_folder, "Error", (void *)&speed_right_pid.Error, float_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(speed_right_pid_folder, "Out", (void *)&speed_right_pid.Out, int16_Box);
			if(item != NULL) item->editable = false;
		}

		differential_folder = create_menu_folder_dynamic(speed_debug_folder, "Differential");
		if(differential_folder != NULL)
		{
			create_menu_number_range_dynamic(differential_folder, "Enable", (void *)&steering_differential_enabled,
				uint8_Box, 0.0f, 1.0f, 1.0f);
			create_menu_number_range_dynamic(differential_folder, "Dead", (void *)&steering_differential_deadband,
				float_Box, 0.0f, 20.0f, 0.5f);
			differential_gain_item = create_menu_number_range_dynamic(
				differential_folder,
				"Gain",
				(void *)&steering_differential_gain,
				float_Box,
				0.0f,
				1.00f,
				0.001f);
			create_menu_number_range_dynamic(differential_folder, "MaxRatio", (void *)&steering_differential_max_ratio,
				float_Box, 0.0f, 0.80f, 0.01f);
			create_menu_number_range_dynamic(differential_folder, "OuterScale", (void *)&steering_differential_outer_scale,
				float_Box, 0.0f, 1.00f, 0.01f);

			item = create_menu_number_dynamic(differential_folder, "Base", (void *)&speed_decision_base_target_pulse, int16_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(differential_folder, "Active", (void *)&speed_decision_differential_active, uint8_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(differential_folder, "Demand", (void *)&speed_decision_steering_demand, float_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(differential_folder, "Effective", (void *)&speed_decision_effective_demand, float_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(differential_folder, "Ratio", (void *)&speed_decision_differential_ratio, float_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(differential_folder, "ReduceP", (void *)&speed_decision_inner_reduce_pulse, float_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(differential_folder, "PlusP", (void *)&speed_decision_outer_plus_pulse, float_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(differential_folder, "LTarget", (void *)&speed_decision_left_target_pulse, int16_Box);
			if(item != NULL) item->editable = false;
			item = create_menu_number_dynamic(differential_folder, "RTarget", (void *)&speed_decision_right_target_pulse, int16_Box);
			if(item != NULL) item->editable = false;
		}
	}

	//Check目录显示TIM6中断采集到的编码器脉冲，以及姿态解算角度。
	check_folder = create_menu_folder_dynamic(&head, "Check");
	item = create_menu_number_dynamic(check_folder, "Enc_Left", &check_encoder_left_menu_value, int16_Box);
	if(item != NULL)
	{
		item->editable = false;
	}
	item = create_menu_number_dynamic(check_folder, "Yaw", &check_yaw_menu_value, float_Box);
	if(item != NULL)
	{
		item->editable = false;
	}
	item = create_menu_number_dynamic(check_folder, "Pitch", &check_pitch_menu_value, float_Box);
	if(item != NULL)
	{
		item->editable = false;
	}
	item = create_menu_number_dynamic(check_folder, "Roll", &check_roll_menu_value, float_Box);
	if(item != NULL)
	{
		item->editable = false;
	}
	item = create_menu_number_dynamic(check_folder, "Enc_Right", &check_encoder_right_menu_value, int16_Box);
	if(item != NULL)
	{
		item->editable = false;
	}
	// 图传页面：Status 为只读状态，send_O/send_P/send_B 分别发送原图、处理图、两图。
	image_send_status_menu_value = wireless_image_send_status;
	image_send_folder = create_menu_folder_dynamic(check_folder, "send_img");
	if(image_send_folder != NULL)
	{
		item = create_menu_number_dynamic(image_send_folder, "Status", &image_send_status_menu_value, uint8_Box);
		if(item != NULL)
		{
			item->editable = false;
		}
		image_send_origin_item = create_menu_number_dynamic(image_send_folder, "send_O", &image_send_action_menu_value, uint8_Box);
		if(image_send_origin_item != NULL)
		{
			image_send_origin_item->editable = false;
		}
		image_send_processed_item = create_menu_number_dynamic(image_send_folder, "send_P", &image_send_action_menu_value, uint8_Box);
		if(image_send_processed_item != NULL)
		{
			image_send_processed_item->editable = false;
		}
		image_send_both_item = create_menu_number_dynamic(image_send_folder, "send_B", &image_send_action_menu_value, uint8_Box);
		if(image_send_both_item != NULL)
		{
			image_send_both_item->editable = false;
		}
	}

	key = head.first_son;
	menu_view_first = 0;
	memset(key_long_count, 0, sizeof(key_long_count));
	menu_refresh_required = true;

	ips200_set_font(IPS200_8X16_FONT);
	ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

//显示当前光标
static void show_key(void)
{
	uint8_t visible_no;
	uint16 y;

	if(key == NULL || key->no < menu_view_first)
	{
		return;
	}

	visible_no = key->no - menu_view_first;
	if(visible_no < MENU_VISIBLE_ROWS)
	{
		y = MENU_ITEM_START_Y + visible_no * MENU_FONT_HEIGHT;
		ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
		ips200_show_char(0, y, '>');
		ips200_set_color(RGB565_WHITE, RGB565_BLACK);
	}
}

//向下移动，最后一项继续向下时回到第一项
static void key_down(void)
{
	if(key == NULL || key->father == NULL)
	{
		return;
	}

	if(key->next_brother != NULL)
	{
		key = key->next_brother;
	}
	else
	{
		key = key->father->first_son;
	}
	menu_update_view();
	menu_refresh_required = true;
}

//向上移动，第一项继续向上时回到最后一项
static void key_up(void)
{
	Menu_Item *last;

	if(key == NULL)
	{
		return;
	}

	if(key->last_brother != NULL)
	{
		key = key->last_brother;
	}
	else
	{
		last = menu_get_last_brother(key);
		if(last != NULL)
		{
			key = last;
		}
	}
	menu_update_view();
	menu_refresh_required = true;
}

void key_enter(void)
{
	if(key == NULL)
	{
		return;
	}

	// KEY3 在发送行只置发送请求；真正的数据复制与发送由主循环完成。
	if(key == image_send_origin_item)
	{
		wireless_image_request_send(WIRELESS_IMAGE_SEND_ORIGIN);
		menu_refresh_required = true;
		return;
	}
	if(key == image_send_processed_item)
	{
		wireless_image_request_send(WIRELESS_IMAGE_SEND_PROCESSED);
		menu_refresh_required = true;
		return;
	}
	if(key == image_send_both_item)
	{
		wireless_image_request_send(WIRELESS_IMAGE_SEND_BOTH);
		menu_refresh_required = true;
		return;
	}

	if(key->kind == MENU_Folder)
	{
		if(key->first_son != NULL)
		{
			key = key->first_son;
			menu_view_first = 0;
			menu_refresh_required = true;
		}
	}
	else if(key->editable)
	{
		key->select = !key->select;
		menu_refresh_required = true;
	}
}

void key_quit(void)
{
	if(key == NULL || key->father == NULL)
	{
		return;
	}

	//编辑状态下，返回键只退出编辑，不改变所在目录
	if(key->select)
	{
		key->select = false;
		menu_refresh_required = true;
		return;
	}

	//当前目录的父节点不是根节点时，才允许继续返回
	if(key->father->father != NULL)
	{
		key = key->father;
		menu_view_first = 0;
		menu_update_view();
		menu_refresh_required = true;
	}
}

void key_select(void)
{
	if(key != NULL && key->kind != MENU_Folder && key->editable)
	{
		key->select = !key->select;
		menu_refresh_required = true;
	}
}

//按照菜单项的类型、范围和步长调节数值
static bool menu_change_number(int8_t direction)
{
	float old_value;
	float new_value;

	if(key == NULL || key->data == NULL || key->kind == MENU_Folder || !key->editable)
	{
		return false;
	}

	switch(key->kind)
	{
		case uint8_Box:
			old_value = (float)(*(uint8_t *)key->data);
			break;
		case uint16_Box:
			old_value = (float)(*(uint16_t *)key->data);
			break;
		case int8_Box:
			old_value = (float)(*(int8_t *)key->data);
			break;
		case int16_Box:
			old_value = (float)(*(int16_t *)key->data);
			break;
		case float_Box:
			old_value = *(float *)key->data;
			break;
		default:
			return false;
	}

	new_value = old_value + direction * key->step;
	if(new_value > key->max_value)
	{
		new_value = key->max_value;
	}
	if(new_value < key->min_value)
	{
		new_value = key->min_value;
	}

	switch(key->kind)
	{
		case uint8_Box:
			*(uint8_t *)key->data = (uint8_t)new_value;
			break;
		case uint16_Box:
			*(uint16_t *)key->data = (uint16_t)new_value;
			break;
		case int8_Box:
			*(int8_t *)key->data = (int8_t)new_value;
			break;
		case int16_Box:
			*(int16_t *)key->data = (int16_t)new_value;
			break;
		case float_Box:
			*(float *)key->data = new_value;
			break;
		default:
			return false;
	}

	return (new_value != old_value);
}

static void key_plus(void)
{
	if(menu_change_number(1))
	{
		menu_refresh_required = true;
	}
}

static void key_sub(void)
{
	if(menu_change_number(-1))
	{
		menu_refresh_required = true;
	}
}

void key_1(void)
{
	if(key == NULL)
	{
		return;
	}

	if(!key->select)
	{
		key_up();
	}
	else
	{
		key_plus();
	}
}

void key_2(void)
{
	if(key == NULL)
	{
		return;
	}

	if(!key->select)
	{
		key_down();
	}
	else
	{
		key_sub();
	}
}

//显示当前页中的数值和编辑状态
static void show_number(void)
{
	Menu_Item *item;
	uint8_t row;
	uint8_t point_num;
	uint16 y;

	item = menu_get_visible_first();
	for(row = 0; row < MENU_VISIBLE_ROWS && item != NULL; row++)
	{
		y = MENU_ITEM_START_Y + row * MENU_FONT_HEIGHT;

		if(item->select)
		{
			ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
			ips200_show_char(8, y, '@');
		}

		if(item->kind == MENU_Folder)
		{
			ips200_set_color(RGB565_GREEN, RGB565_BLACK);
			menu_show_text(200, y, "DIR", 3);
		}
		else
		{
			ips200_set_color(item->select ? RGB565_YELLOW : RGB565_GREEN, RGB565_BLACK);
			if(item == image_send_origin_item || item == image_send_processed_item || item == image_send_both_item)
			{
				menu_show_text(MENU_VALUE_X, y, "KEY3", 4);
			}
			else switch(item->kind)
			{
				case uint8_Box:
					ips200_show_uint(MENU_VALUE_X, y, *(uint8_t *)item->data, 3);
					break;
				case uint16_Box:
					ips200_show_uint(MENU_VALUE_X, y, *(uint16_t *)item->data, 5);
					break;
				case int8_Box:
					ips200_show_int(MENU_VALUE_X, y, *(int8_t *)item->data, 3);
					break;
				case int16_Box:
					ips200_show_int(MENU_VALUE_X, y, *(int16_t *)item->data, 5);
					break;
				case float_Box:
					point_num = (item == differential_gain_item) ? 3U : 2U;
					ips200_show_float(
						MENU_VALUE_X,
						y,
						menu_round_float_for_display(*(float *)item->data, point_num),
						5,
						point_num);
					break;
				default:
					break;
			}
		}

		item = item->next_brother;
	}
	ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

//在Image/Preview页面显示最新一帧处理结果，KEY4可返回上一级菜单
static void menu_show_image_preview(void)
{
	bool new_result = image_process_take_new_result();

	if(menu_refresh_required)
	{
		menu_refresh_required = false;
		ips200_set_font(IPS200_8X16_FONT);
		ips200_set_color(RGB565_WHITE, RGB565_BLACK);
		ips200_clear();
	}

	if(new_result)
	{
		image_process_display();
	}
}

void menu_show(void)
{
	Menu_Item *item;
	const char *folder_name;
	uint8_t row;
	uint16 y;
	uint16 latest_fps;
	bool check_value_changed;
	bool cargo_value_changed;
	bool image_send_status_changed;
	bool protect_values_changed;
	bool fs_a8s_value_changed;
	bool speed_value_changed;

	//每秒结算一次的采集帧率同步到Image/FPS菜单项；只在Image目录中刷新菜单。
	latest_fps = image_get_capture_fps();
	if(latest_fps != image_fps_menu_value)
	{
		image_fps_menu_value = latest_fps;
		if(key != NULL && image_fps_item != NULL && key->father == image_fps_item->father)
		{
			menu_refresh_required = true;
		}
	}

	check_value_changed = menu_update_check_values();
	cargo_value_changed = menu_update_cargo_values();
	image_send_status_changed = menu_update_image_send_status();
	protect_values_changed = menu_update_protect_values();
	fs_a8s_value_changed = menu_update_fs_a8s_values();
	speed_value_changed = menu_update_speed_values();

	//图像预览页面按新帧刷新；普通菜单仍然只在内容变化时刷新
	if(menu_is_image_preview())
	{
		menu_show_image_preview();
		return;
	}

	//没有操作时不刷屏，避免占用智能车主循环时间
	if(!menu_refresh_required)
	{
		//Check页面的名称和光标不变时，仅覆盖数值，避免周期刷新时全屏清除。
		if((menu_is_check_page() && check_value_changed)
			|| (menu_is_base_control_page() && cargo_value_changed)
			|| (menu_is_image_send_page() && image_send_status_changed)
			|| (menu_is_protect_page() && protect_values_changed)
			|| (menu_is_wireless_control_page() && fs_a8s_value_changed)
			|| (menu_is_speed_debug_page() && speed_value_changed))
		{
			ips200_set_font(IPS200_8X16_FONT);
			show_number();
		}
		return;
	}
	menu_refresh_required = false;

	ips200_set_font(IPS200_8X16_FONT);
	ips200_set_color(RGB565_WHITE, RGB565_BLACK);
	ips200_clear();

	if(key == NULL || key->father == NULL)
	{
		ips200_set_color(RGB565_RED, RGB565_BLACK);
		menu_show_text(0, 0, "MENU EMPTY", 30);
		ips200_set_color(RGB565_WHITE, RGB565_BLACK);
		return;
	}

	folder_name = key->father->name;
	ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
	menu_show_text(0, 0, "MENU: ", 6);
	menu_show_text(48, 0, folder_name, 24);
	ips200_draw_line(0, 17, 239, 17, RGB565_GRAY);

	ips200_set_color(RGB565_WHITE, RGB565_BLACK);
	item = menu_get_visible_first();
	for(row = 0; row < MENU_VISIBLE_ROWS && item != NULL; row++)
	{
		y = MENU_ITEM_START_Y + row * MENU_FONT_HEIGHT;
		menu_show_text(MENU_NAME_X, y, item->name, 14);
		item = item->next_brother;
	}

	show_key();
	show_number();
}

//执行一个已经确认的物理按键动作
static void menu_run_key_action(key_index_enum key_index)
{
	switch(key_index)
	{
		case KEY_1:
			key_1();
			break;
		case KEY_2:
			key_2();
			break;
		case KEY_3:
			key_enter();
			break;
		case KEY_4:
			key_quit();
			break;
		default:
			break;
	}
}

//每5ms调用一次：扫描按键，处理短按和限速后的长按连发
void menu_key_task(void)
{
	key_index_enum i;
	key_state_enum state;

	key_scanner();

	for(i = KEY_1; i < KEY_NUMBER; i++)
	{
		state = key_get_state(i);

		if(state == KEY_SHORT_PRESS)
		{
			menu_run_key_action(i);
			key_long_count[i] = 0;
			key_clear_state(i);
		}
		else if(state == KEY_LONG_PRESS)
		{
			if(i == KEY_1 || i == KEY_2)
			{
				if(key_long_count[i] == 0 || key_long_count[i] >= MENU_KEY_REPEAT_TICKS)
				{
					menu_run_key_action(i);
					key_long_count[i] = 1;
				}
				else
				{
					key_long_count[i]++;
				}
			}
			else if(key_long_count[i] == 0)
			{
				//进入和返回键长按时只触发一次
				menu_run_key_action(i);
				key_long_count[i] = 1;
			}
			//长按状态不能在这里清除。保留到松手后，驱动才不会把松手误判成短按
		}
		else
		{
			key_long_count[i] = 0;
		}
	}
}
