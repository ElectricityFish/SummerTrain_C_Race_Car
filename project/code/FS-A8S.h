#ifndef __FS_A8S_H
#define __FS_A8S_H

#include "zf_common_typedef.h"

// FA-A8S 的 i-BUS 接在 UART2_RX_D6，发送端无需连接。
#define FS_A8S_CHANNEL_COUNT       (6U)
#define FS_A8S_FRAME_TIMEOUT_MS    (50U)

// 通道值由 UART2 接收中断更新；读取时仅用于显示，暂不参与车辆控制。
typedef struct
{
    volatile uint16 channel[FS_A8S_CHANNEL_COUNT];
    volatile uint32 valid_frame_count;
    volatile uint16 frame_age_ms;
} fs_a8s_channel_data_struct;

extern fs_a8s_channel_data_struct fs_a8s_channel_data;

void fs_a8s_init(void);
void fs_a8s_uart_callback(void);
void fs_a8s_1ms_task(void);
bool fs_a8s_is_online(void);

#endif
