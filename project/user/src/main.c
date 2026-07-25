#include "zf_common_headfile.h"
#include "MyMenu.h"

int main(void)
{
	clock_init(SYSTEM_CLOCK_120M);					//初始化芯片时钟，工作频率120MHz
	debug_init();									//初始化默认Debug串口
	
	pit_ms_init(TIM1_PIT, 5);
	pit_ms_init(TIM2_PIT, 1);
	
	//IPS200方向必须在初始化屏幕之前设置
	ips200_set_dir(IPS200_PORTAIT);					//单排SPI屏，竖屏240×320
	ips200_init(IPS200_TYPE_SPI);

	//按键扫描周期必须与主循环最后的延时保持一致
	key_init(MENU_KEY_SCAN_PERIOD_MS);

	menu_init();
	menu_show();									//显示初始菜单

	while(1)
	{
		menu_show();								//仅在内容变化时才真正刷新

	}
}

void TIM1_5ms_PIT(void)
{
	menu_key_task();							//扫描并处理四个菜单按键
}

void TIM2_1ms_PIT(void)
{
	
}


