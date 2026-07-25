#include "Menu.h"

#define Menu_Item_Max 64							//菜单项最大数量

static Menu_Item menu_item_array[Menu_Item_Max];	//创建菜单项对象(内存)池，最多Menu_Item_Max个菜单项
static uint8_t menu_array_index=0;					//菜单项对象池的索引

//判断类型是否为可以绑定数值的菜单类型
static bool menu_kind_is_number(MENU_KIND kind)
{
	return (kind == int16_Box
		|| kind == uint16_Box
		|| kind == int8_Box
		|| kind == uint8_Box
		|| kind == float_Box);
}

//给数值菜单项设置默认范围，保证旧的创建接口仍然可以继续使用
static void menu_set_default_range(Menu_Item *item)
{
	item->step = 1.0f;

	switch(item->kind)
	{
		case int8_Box:
			item->min_value = -128.0f;
			item->max_value = 127.0f;
			break;
		case uint8_Box:
			item->min_value = 0.0f;
			item->max_value = 255.0f;
			break;
		case int16_Box:
			item->min_value = -32768.0f;
			item->max_value = 32767.0f;
			break;
		case uint16_Box:
			item->min_value = 0.0f;
			item->max_value = 65535.0f;
			break;
		case float_Box:
			item->min_value = -1000000.0f;
			item->max_value = 1000000.0f;
			item->step = 0.1f;
			break;
		default:
			item->min_value = 0.0f;
			item->max_value = 0.0f;
			item->step = 0.0f;
			break;
	}
}

//把用户设置的范围限制在实际数据类型能够保存的范围内
static void menu_limit_range_to_type(
	MENU_KIND kind,
	float *min_value,
	float *max_value,
	float *step)
{
	float type_min;
	float type_max;

	switch(kind)
	{
		case int8_Box:
			type_min = -128.0f;
			type_max = 127.0f;
			break;
		case uint8_Box:
			type_min = 0.0f;
			type_max = 255.0f;
			break;
		case int16_Box:
			type_min = -32768.0f;
			type_max = 32767.0f;
			break;
		case uint16_Box:
			type_min = 0.0f;
			type_max = 65535.0f;
			break;
		case float_Box:
			return;
		default:
			return;
	}

	if(*min_value < type_min)
	{
		*min_value = type_min;
	}
	if(*max_value > type_max)
	{
		*max_value = type_max;
	}
	if(*step < 1.0f)
	{
		*step = 1.0f;
	}
}

//创建带范围的菜单项时，把变量初值限制在所设置的范围内
static void menu_clamp_number_value(Menu_Item *item)
{
	float value;

	switch(item->kind)
	{
		case int8_Box:
			value = (float)(*(int8_t *)item->data);
			break;
		case uint8_Box:
			value = (float)(*(uint8_t *)item->data);
			break;
		case int16_Box:
			value = (float)(*(int16_t *)item->data);
			break;
		case uint16_Box:
			value = (float)(*(uint16_t *)item->data);
			break;
		case float_Box:
			value = *(float *)item->data;
			break;
		default:
			return;
	}

	if(value < item->min_value)
	{
		value = item->min_value;
	}
	if(value > item->max_value)
	{
		value = item->max_value;
	}

	switch(item->kind)
	{
		case int8_Box:
			*(int8_t *)item->data = (int8_t)value;
			break;
		case uint8_Box:
			*(uint8_t *)item->data = (uint8_t)value;
			break;
		case int16_Box:
			*(int16_t *)item->data = (int16_t)value;
			break;
		case uint16_Box:
			*(uint16_t *)item->data = (uint16_t)value;
			break;
		case float_Box:
			*(float *)item->data = value;
			break;
		default:
			break;
	}
}

//创建菜单项，初始化成员
static bool Creat_Menu_Item(
	Menu_Item *father,
	Menu_Item *me,
	const char name[],
	void *data,
	MENU_KIND kind)	//参数：父节点，自己，名字，绑定的变量，属性
{
	if(father == NULL || me == NULL || name == NULL)
	{
		return false;
	}
	if(father->kind != MENU_Folder)
	{
		return false;	//只有文件夹类型项才能有子菜单
	}
	if(kind != MENU_Folder && (!menu_kind_is_number(kind) || data == NULL))
	{
		return false;
	}
	
	me->select = false;
	me->sons=0;
	me->name=name;
	me->father=father;
	me->first_son=NULL;//子节点先定义为空
	me->last_brother=NULL;
	me->next_brother=NULL;
	
	me->kind=kind;
	me->data=data;
	me->no=father->sons;
	menu_set_default_range(me);
	
	if(father->sons==0)//如果父节点还没有子节点
	{
		father->first_son=me;	//就让父节点第一个子节点指向me 
	}
	else//如果有子节点，就让me排在父节点最后一个子节点后面
	{
		Menu_Item *p=father->first_son;
		while(p->next_brother!=NULL)
			p=p->next_brother;
		p->next_brother=me;
		me->last_brother=p;
	}
	father->sons++;//这个记住不要写if-else里面
	return true;
}

//复位菜单项对象池
void menu_pool_reset(void)
{
	menu_array_index = 0;
	memset(menu_item_array, 0, sizeof(menu_item_array));
}


//创建目录
void Creat_Menu_Folder(Menu_Item *father,Menu_Item *me ,const char name[])	//参数：父节点，自己，名字
{
	Creat_Menu_Item(father,me ,name,NULL,MENU_Folder);
}

//创建文件（与参数绑定，下面不能再存放东西的）
void Creat_Menu_Number(Menu_Item *father,Menu_Item *me ,const char name[],void *data,MENU_KIND kind)
{
	Creat_Menu_Item(father,me,name,data,kind);
	
}


//动态创建目录，返回创建的目录的地址
Menu_Item *create_menu_folder_dynamic(Menu_Item *father,const char name[])
{
	Menu_Item *me;

	if(father == NULL || father->kind != MENU_Folder || name == NULL)
	{
		return NULL;
	}
	if(menu_array_index >= Menu_Item_Max)
	{
		return NULL;
	}

	me = &menu_item_array[menu_array_index];	//在菜单项对象池中取一个对象
	if(!Creat_Menu_Item(father,me,name,NULL,MENU_Folder))
	{
		return NULL;
	}
	menu_array_index++;
	return me;	//返回创建的目录的地址
}

//动态创建文件，返回创建的文件的地址
Menu_Item *create_menu_number_dynamic(Menu_Item *father,const char name[],void *data,MENU_KIND kind)
{
	Menu_Item *me;

	if(father == NULL || father->kind != MENU_Folder || name == NULL
		|| data == NULL || !menu_kind_is_number(kind))
	{
		return NULL;
	}
	if(menu_array_index >= Menu_Item_Max)
	{
		return NULL;
	}

	me = &menu_item_array[menu_array_index];	//在菜单项对象池中取一个对象
	if(!Creat_Menu_Item(father,me,name,data,kind))
	{
		return NULL;
	}
	menu_array_index++;
	return me;
}

//动态创建带范围和步长的数值菜单项
Menu_Item *create_menu_number_range_dynamic(
	Menu_Item *father,
	const char name[],
	void *data,
	MENU_KIND kind,
	float min_value,
	float max_value,
	float step)
{
	Menu_Item *me;
	float temp;

	if(step < 0.0f)
	{
		step = -step;
	}
	if(step == 0.0f)
	{
		return NULL;
	}
	if(min_value > max_value)
	{
		temp = min_value;
		min_value = max_value;
		max_value = temp;
	}
	menu_limit_range_to_type(kind, &min_value, &max_value, &step);
	if(min_value > max_value)
	{
		return NULL;
	}

	me = create_menu_number_dynamic(father, name, data, kind);
	if(me != NULL)
	{
		me->min_value = min_value;
		me->max_value = max_value;
		me->step = step;
		menu_clamp_number_value(me);
	}
	return me;
}

