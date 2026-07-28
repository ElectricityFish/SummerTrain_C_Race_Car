#ifndef __WIRELESS_H
#define __WIRELESS_H

#include "zf_device_wireless_uart.h"

//厂商驱动已经提供 wireless_uart_init()，此处直接通过头文件复用，不能重复定义。
//无线printf使用静态格式化缓冲区；单条格式化后的消息不能超过该长度（不含字符串结束符）。
#define WIRELESS_PRINTF_BUFFER_SIZE    (128U)

// VOFA+ 单帧图像发送状态。1 表示请求已收到，正在等待完整新帧或正在发送。
typedef enum
{
    WIRELESS_IMAGE_SEND_NOT_SENT = 0,
    WIRELESS_IMAGE_SEND_SENDING,
    WIRELESS_IMAGE_SEND_SUCCESS,
    WIRELESS_IMAGE_SEND_FAILED,
} wireless_image_send_state_enum;

typedef enum
{
    WIRELESS_IMAGE_SEND_ORIGIN = 0,
    WIRELESS_IMAGE_SEND_PROCESSED,
    WIRELESS_IMAGE_SEND_BOTH,
} wireless_image_send_type_enum;

extern volatile uint8 wireless_image_send_status;

//用法与printf一致，例如：wireless_uart_printf("pitch = %.2f\n", pitch);
//返回值：成功时返回实际发送的字符数；格式化失败、消息过长或无线发送超时返回-1。
int wireless_uart_printf(const char *format, ...);

// 可从按键中断调用：只记录一次发送请求，不进行复制、格式化或串口阻塞发送。
void wireless_image_request_send(wireless_image_send_type_enum send_type);

// 仅应在获取到一帧完整图像后由主循环调用。
// is_idle 为 false 时，已收到的请求会继续等待；为 true 时复制快照，等待本帧图像处理完成。
void wireless_image_capture_task(bool is_idle, const uint8 *image_addr, uint16 image_width, uint16 image_height);

// 在本帧 image_process_frame() 完成后由主循环调用，发送原图或叠加赛道标记后的图像。
void wireless_image_send_task(void);

#endif
