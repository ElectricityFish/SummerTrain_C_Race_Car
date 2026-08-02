#include "zf_common_headfile.h"
#include "MyMenu.h"
#include "ServoMotor.h"
#include "Motor.h"
#include "Image.h"
#include "Image_Process.h"
#include "Control.h"
#include "Encoder.h"
#include "SpeedControl.h"
#include "SpeedDecision.h"
#include "SpeedPlanner.h"
#include "MPU6050.h"
#include "Kfilter.h"
#include "Promopt.h"
#include "Wireless.h"
#include "FS-A8S.h"

int main(void)
{
	
	//初始化
	clock_init(SYSTEM_CLOCK_120M);					//初始化芯片时钟，工作频率120MHz
	debug_init();									//初始化默认Debug串口
	
	pit_ms_init(TIM7_PIT, 5);						
	pit_ms_init(TIM6_PIT, 1);
	pit_ms_init(TIM8_PIT, 1);
	
	
	//IPS200方向必须在初始化屏幕之前设置
	ips200_set_dir(IPS200_PORTAIT);					//单排SPI屏，竖屏240×320
	ips200_init(IPS200_TYPE_SPI);

	key_init(MENU_KEY_SCAN_PERIOD_MS);
	
	servomotor_init();
	motor_init();
	encoder_init();
	speed_control_init();
	speed_decision_init();
	speed_planner_init();
	promopt_init();										//蜂鸣器D7初始化为输出
	wireless_uart_init();								//厂商无线串口：UART6，C6/C7，RTS为C13
	fs_a8s_init();									//FA-A8S i-BUS：UART2，接收引脚D6
	while(mpu6050_module_init())
	{
		ips200_set_color(RGB565_RED, RGB565_BLACK);
		ips200_clear();
		ips200_show_string(0, 0, "MPU6050 INIT ERR");
		system_delay_ms(500);
	}
	kfilter_init();										//初始化Pitch、Roll两组卡尔曼滤波器
	while(image_init())
	{
		ips200_set_color(RGB565_RED, RGB565_BLACK);
		ips200_clear();
		ips200_show_string(0, 0, "MT9V03X INIT ERR");
		system_delay_ms(500);
	}
	image_process_init();
	control_init();
	menu_init();
	menu_show();									//显示初始菜单
	
	//初始化完成
	
	
	while(1)
	{
		car_state_command_task();
		image_update();								//接收DMA采集完成的一帧图像
		if(image_take_new_frame())
		{
			//仅在 IDLE 且菜单已请求时保存刚完成的一帧快照；处理完成后再发送原图或带赛道标记的图像。
			wireless_image_capture_task((common_state == COMMON_STATE_IDLE), image_get_buffer(), MT9V03X_W, MT9V03X_H);
			image_process_frame();
			// 转向 PID 只消费刚完成的一帧结果；两帧之间保持上一条舵机指令。
			if(common_state == COMMON_STATE_RUNNING)
			{
				servo_control();
			}
			// 视觉与本帧舵机指令均已更新后，再生成下一周期的基础速度目标。
			speed_planner_update_from_image();
			wireless_image_send_task();
		}
		control_telemetry_task();					// 无线串口调试信息在主循环发送
		menu_show();								//仅在内容变化时才真正刷新

	}
}





//该中断主要负责按键扫描
void TIM7_5ms_PIT(void)
{
	menu_key_task();							
}


//该中断主要负责传感器读取
void TIM6_1ms_PIT(void)
{
	static uint8_t count=0;
	static uint8_t count1=0;
	count1++;
	count++;
	image_fps_1ms_task();							//每1ms计时，按1秒窗口统计实际采集帧率
	image_process_1ms_task();						//统计最近一帧图像处理结果的帧龄
	fs_a8s_1ms_task();							//i-BUS 最后有效帧超时计时
	promopt_tick();
	if(count1>=10)									// 每10ms进行一次姿态解算
	{
		Get_Angle();								//KFILTER_SAMPLE_DT对应10ms
		car_protection_check_attitude();
		count1=0;
	}
	
	if(count>=10)									//每10ms读取一次原始编码器增量并更新速度 PI
	{
		encoder_left_pulse = encoder_left_get_pulse();
		encoder_right_pulse = encoder_right_get_pulse();
		speed_planner_10ms_task();
		speed_decision_10ms_task();
		speed_control_10ms_task(encoder_left_pulse, encoder_right_pulse);
		count=0;
	}
}


//该中断负责运动态控制
void TIM8_1ms_PIT(void)
{
	static uint8_t count=0;
	int16 speed_left_duty;
	int16 speed_right_duty;
	count++;
	
	// i-BUS 失联时，每 1ms 硬急停。CH5 低位由状态机进入闭环 Protect，
	// 不能在这里把其覆盖为直接断电。
	if(wireless_control_enabled && !wireless_control_link_online())
	{
		motor_set_duty(0, 0);
		servomotor_disable();
		count = 0;
		return;
	}

	// 独立速度环调试只允许在本地 IDLE 状态接管电机。
	if((common_state == COMMON_STATE_IDLE) && (wireless_control_enabled == 0U))
	{
		speed_control_debug_get_duty(&speed_left_duty, &speed_right_duty);
		motor_set_duty(speed_left_duty, speed_right_duty);
		count = 0U;
		return;
	}

	// RUNNING、PLAY 与 Protect 均由 10 ms 速度 PI 输出驱动；本中断只取最新 PWM。
	if(speed_control_closed_loop_is_active())
	{
		speed_control_get_closed_loop_duty(&speed_left_duty, &speed_right_duty);
		motor_set_duty(speed_left_duty, speed_right_duty);
	}

	if(count>=20)
	{
		count=0;
		if(common_state == COMMON_STATE_RUNNING)
		{
			// 电机 PWM 已由速度环在上方持续下发；图像帧到来时更新转向 PID。
		}
		else if(common_state == COMMON_STATE_PLAY)
		{
			wireless_control_play_task();
		}
		else
		{
			servomotor_disable();
		}
	}
}
