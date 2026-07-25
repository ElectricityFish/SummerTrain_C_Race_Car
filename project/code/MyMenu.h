// 这个文件声明菜单的数据绑定、显示和按键接口
#ifndef __MYMENU_H
#define __MYMENU_H

#include "Menu.h"

#define MENU_KEY_SCAN_PERIOD_MS    (5)

void menu_init(void);
void menu_show(void);
void menu_key_task(void);

void key_1(void);		//上移；编辑状态下增加数值
void key_2(void);		//下移；编辑状态下减少数值
void key_enter(void);	//进入文件夹；数值项进入/退出编辑
void key_quit(void);	//返回上一级；编辑状态下退出编辑
void key_select(void);	//兼容原接口：切换数值项编辑状态

#endif
