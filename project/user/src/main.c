#include "zf_common_headfile.h"
#include "MyMenu.h"
#include "ServoMotor.h"
#include "Motor.h"
#include "Image.h"
#include "Encoder.h"

int main(void)
{
	clock_init(SYSTEM_CLOCK_120M);					//初始化芯片时钟，工作频率120MHz
	debug_init();									//初始化默认Debug串口
	
	pit_ms_init(TIM7_PIT, 5);						//TIM3改作编码器1，菜单扫描改用空闲TIM7
	pit_ms_init(TIM6_PIT, 1);
	
	
	//IPS200方向必须在初始化屏幕之前设置
	ips200_set_dir(IPS200_PORTAIT);					//单排SPI屏，竖屏240×320
	ips200_init(IPS200_TYPE_SPI);

	//按键扫描周期必须与主循环最后的延时保持一致
	key_init(MENU_KEY_SCAN_PERIOD_MS);
	
	servomotor_init();
	motor_init();
	encoder_init();
	while(image_init())
	{
		ips200_set_color(RGB565_RED, RGB565_BLACK);
		ips200_clear();
		ips200_show_string(0, 0, "MT9V03X INIT ERR");
		system_delay_ms(500);
	}
	menu_init();
	menu_show();									//显示初始菜单
	
	
	
	while(1)
	{
		image_update();								//接收DMA采集完成的一帧图像
		menu_show();								//仅在内容变化时才真正刷新

	}
}



void TIM7_5ms_PIT(void)
{
	menu_key_task();							//扫描并处理四个菜单按键
}


void TIM6_1ms_PIT(void)
{
	static uint8_t count=0;
	count++;
	image_fps_1ms_task();							//每1ms计时，按1秒窗口统计实际采集帧率
	if(count>=20)
	{
		encoder1=encoder_1_get_pulse();
		encoder2=encoder_2_get_pulse();
		count=0;
	}
}


