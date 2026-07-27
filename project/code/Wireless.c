#include "Wireless.h"

#include <stdarg.h>
#include <stdio.h>

//提供和printf相同的可变参数调用方式，底层复用厂商无线串口的RTS流控发送函数。
int wireless_uart_printf(const char *format, ...)
{
	static char send_buffer[WIRELESS_PRINTF_BUFFER_SIZE];
	va_list args;
	int format_length;
	uint32 remaining_length;

	if(format == NULL)
	{
		return -1;
	}

	va_start(args, format);
	format_length = vsnprintf(send_buffer, sizeof(send_buffer), format, args);
	va_end(args);

	//vsnprintf返回值大于等于缓冲区长度，说明内容被截断；本函数不发送不完整数据。
	if(format_length < 0 || format_length >= (int)sizeof(send_buffer))
	{
		return -1;
	}

	remaining_length = wireless_uart_send_buffer((const uint8 *)send_buffer, (uint32)format_length);
	if(remaining_length != 0U)
	{
		return -1;
	}

	return format_length;
}
