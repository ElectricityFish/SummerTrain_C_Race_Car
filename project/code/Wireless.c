#include "Wireless.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "zf_device_mt9v03x.h"

#define WIRELESS_IMAGE_VOFA_CHANNEL_ID        (0U)
#define WIRELESS_IMAGE_VOFA_GRAYSCALE8_FORMAT (24U)
#define WIRELESS_IMAGE_VOFA_HEADER_SIZE       (64U)

volatile uint8 wireless_image_send_status = WIRELESS_IMAGE_SEND_NOT_SENT;

// 必须先保存快照：摄像头 DMA 会持续改写 mt9v03x_image，不能在约 2 秒的串口发送期间直接读取它。
static uint8 wireless_image_snapshot[MT9V03X_IMAGE_SIZE];

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

// 此函数可在按键中断中调用，只修改一个 8 位状态标志，不进行耗时操作。
void wireless_image_request_send(void)
{
    if(wireless_image_send_status != WIRELESS_IMAGE_SEND_SENDING)
    {
        wireless_image_send_status = WIRELESS_IMAGE_SEND_SENDING;
    }
}

// 发送 FireWater 图片前导帧，再紧跟完整的 8 位灰度图像数据。
void wireless_image_send_task(bool is_idle, const uint8 *image_addr, uint16 image_width, uint16 image_height)
{
    char image_header[WIRELESS_IMAGE_VOFA_HEADER_SIZE];
    uint32 image_size;
    int header_length;

    if(wireless_image_send_status != WIRELESS_IMAGE_SEND_SENDING || !is_idle)
    {
        return;
    }

    image_size = (uint32)image_width * image_height;
    if(image_addr == NULL || image_width != MT9V03X_W || image_height != MT9V03X_H
        || image_size != MT9V03X_IMAGE_SIZE)
    {
        wireless_image_send_status = WIRELESS_IMAGE_SEND_FAILED;
        return;
    }

    // image_addr 指向刚完成 DMA 的一帧。复制完成后，后续 DMA 改写不会影响本次发送。
    memcpy(wireless_image_snapshot, image_addr, image_size);

    header_length = snprintf(
        image_header,
        sizeof(image_header),
        "image:%u,%lu,%u,%u,%u\n",
        (unsigned int)WIRELESS_IMAGE_VOFA_CHANNEL_ID,
        (unsigned long)image_size,
        (unsigned int)image_width,
        (unsigned int)image_height,
        (unsigned int)WIRELESS_IMAGE_VOFA_GRAYSCALE8_FORMAT);
    if(header_length <= 0 || header_length >= (int)sizeof(image_header)
        || wireless_uart_send_buffer((const uint8 *)image_header, (uint32)header_length) != 0U
        || wireless_uart_send_buffer(wireless_image_snapshot, image_size) != 0U)
    {
        wireless_image_send_status = WIRELESS_IMAGE_SEND_FAILED;
        return;
    }

    wireless_image_send_status = WIRELESS_IMAGE_SEND_SUCCESS;
}
