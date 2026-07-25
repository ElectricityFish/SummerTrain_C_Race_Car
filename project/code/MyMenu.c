#include "MyMenu.h"

#include "zf_common_font.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"

#define MENU_FONT_WIDTH             (8)
#define MENU_FONT_HEIGHT            (16)
#define MENU_ITEM_START_Y           (24)
#define MENU_VISIBLE_ROWS           (18)
#define MENU_NAME_X                 (16)
#define MENU_VALUE_X                (136)
#define MENU_KEY_REPEAT_TICKS       (100 / MENU_KEY_SCAN_PERIOD_MS)

Menu_Item head;		//创建根节点
Menu_Item *key;		//指向当前光标所在的菜单项

uint8_t test1 = 10;
uint8_t test2 = 50;
float test_float = 1.50f;

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
	Menu_Item *folder1;
	Menu_Item *folder4;

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
	head.min_value = 0.0f;
	head.max_value = 0.0f;
	head.step = 0.0f;

	folder1 = create_menu_folder_dynamic(&head, "Folder1");
	create_menu_folder_dynamic(&head, "Folder2");
	create_menu_folder_dynamic(&head, "Folder3");
	folder4 = create_menu_folder_dynamic(folder1, "Folder4");

	create_menu_number_range_dynamic(folder1, "test1", &test1, uint8_Box, 0.0f, 100.0f, 1.0f);
	create_menu_number_range_dynamic(folder1, "test_float", &test_float, float_Box, 0.0f, 10.0f, 0.01f);
	create_menu_number_range_dynamic(folder4, "test2", &test2, uint8_Box, 0.0f, 255.0f, 1.0f);

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

	if(key->kind == MENU_Folder)
	{
		if(key->first_son != NULL)
		{
			key = key->first_son;
			menu_view_first = 0;
			menu_refresh_required = true;
		}
	}
	else
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
	if(key != NULL && key->kind != MENU_Folder)
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

	if(key == NULL || key->data == NULL || key->kind == MENU_Folder)
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
			switch(item->kind)
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
					ips200_show_float(MENU_VALUE_X, y, *(float *)item->data, 5, 2);
					break;
				default:
					break;
			}
		}

		item = item->next_brother;
	}
	ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

void menu_show(void)
{
	Menu_Item *item;
	const char *folder_name;
	uint8_t row;
	uint16 y;

	//没有操作时不刷屏，避免占用智能车主循环时间
	if(!menu_refresh_required)
	{
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
