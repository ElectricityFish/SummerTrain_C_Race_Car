#ifndef __WIRELESS_H
#define __WIRELESS_H

#include "zf_device_wireless_uart.h"

//厂商驱动已经提供 wireless_uart_init()，此处直接通过头文件复用，不能重复定义。
//无线printf使用静态格式化缓冲区；单条格式化后的消息不能超过该长度（不含字符串结束符）。
#define WIRELESS_PRINTF_BUFFER_SIZE    (128U)

//用法与printf一致，例如：wireless_uart_printf("pitch = %.2f\r\n", pitch);
//返回值：成功时返回实际发送的字符数；格式化失败、消息过长或无线发送超时返回-1。
int wireless_uart_printf(const char *format, ...);

#endif
