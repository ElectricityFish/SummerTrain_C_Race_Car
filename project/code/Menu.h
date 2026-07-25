// 这个文件定义菜单的数据类型，以及菜单项的创建接口
#ifndef __MENU_H
#define __MENU_H

#include "zf_common_typedef.h"

typedef enum MENU_KIND{
	MENU_Folder = 0,
	int16_Box,
	uint16_Box,
	int8_Box,
	uint8_Box,
	float_Box,
}MENU_KIND;

//定义菜单项目
typedef struct  Menu_Item{
	const char *name;			//菜单名字
	
	void *data;					//指向存放的变量
	MENU_KIND kind;				//记录指向数据的属性
	float min_value;			//数值允许的最小值
	float max_value;			//数值允许的最大值
	float step;					//每次按键调节的步长
	
	uint8_t sons;//记录父节点的子节点数量
	uint8_t no;  //记录当前是父节点的第几个子节点

	bool select;	//记录是否被选中
	
	struct  Menu_Item *father;			//指向父节点（上一级菜单）
	struct  Menu_Item *first_son;		//指向第一个子节点（下一级菜单）
	struct  Menu_Item *next_brother;	//指向下一个兄弟节点（同一菜单里的下一项）
	struct  Menu_Item *last_brother;	//指向上一个兄弟节点（同一菜单里的上一项）
	
}Menu_Item;


//复位静态菜单项对象池。每次重新创建整棵菜单树前调用一次
void menu_pool_reset(void);

void Creat_Menu_Folder(Menu_Item *father,Menu_Item *me ,const char name[]);	//参数：父节点，自己，名字
//创建文件（与参数绑定，下面不能再存放东西的）
void Creat_Menu_Number(Menu_Item *father,Menu_Item *me ,const char name[],void *data,MENU_KIND kind);

//动态创建目录，返回创建的目录的地址
Menu_Item *create_menu_folder_dynamic(Menu_Item *father,const char name[]);
//动态创建文件，返回创建的文件的地址
Menu_Item *create_menu_number_dynamic(Menu_Item *father,const char name[],void *data,MENU_KIND kind);

//动态创建带范围和步长的数值菜单项
Menu_Item *create_menu_number_range_dynamic(
	Menu_Item *father,
	const char name[],
	void *data,
	MENU_KIND kind,
	float min_value,
	float max_value,
	float step);

#endif
