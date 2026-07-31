#include "zf_common_headfile.h"
#include "MyMenu.h"
#include "ServoMotor.h"
#include "Motor.h"
#include "Image.h"
#include "Image_Process.h"
#include "Control.h"
#include "Encoder.h"
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
		
		image_update();								//接收DMA采集完成的一帧图像
		if(image_take_new_frame())
		{
			//仅在 IDLE 且菜单已请求时保存刚完成的一帧快照；处理完成后再发送原图或带赛道标记的图像。
			wireless_image_capture_task((common_state == COMMON_STATE_IDLE), image_get_buffer(), MT9V03X_W, MT9V03X_H);
			image_process_frame();
			wireless_image_send_task();
		}
		
		car_state_command_task();
		menu_show();								//仅在内容变化时才真正刷新
		
		//运行时进行无线调参
		if(common_state == COMMON_STATE_RUNNING)
		{
			wireless_uart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",servo_pid.KpNow,servo_pid.Actual,
			servo_pid.Target,servo_pid.Error0,servo_pid.Out,servo_pid.Kd2Out);
		}
		

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
	fs_a8s_1ms_task();							//i-BUS 最后有效帧超时计时
	promopt_tick();
	if(count1>=10)									// 每10ms进行一次姿态解算
	{
		Get_Angle();								//KFILTER_SAMPLE_DT对应10ms
		car_protection_check_attitude();
		count1=0;
	}
	
	if(count>=5)									//每5ms进行一次编码器读取
	{
		encoder1=encoder_1_get_pulse();
		encoder2=encoder_2_get_pulse();
		count=0;
	}
}


//该中断负责运动态控制
void TIM8_1ms_PIT(void)
{
	static uint8_t count=0;
	count++;
	
	// 无线模式下 CH5 低位或 i-BUS 失联时，每 1ms 强制关闭执行器。
	if(wireless_control_enabled && !wireless_control_actuators_permitted())
	{
		motor_set_duty(0, 0);
		servomotor_disable();
		count = 0;
		return;
	}

	if(count>=20)
	{
		count=0;
		if(common_state == COMMON_STATE_RUNNING)
		{
			motor_set_duty(2150,2150);
			servo_control();
		}
		else if(common_state == COMMON_STATE_PLAY)
		{
			wireless_control_play_task();
		}
		else
		{
			motor_set_duty(0,0);
			servomotor_disable();
		}
	}
}
