#include "FS-A8S.h"
#include "zf_driver_uart.h"

#define FS_A8S_IBUS_BAUD_RATE       (115200U)
#define FS_A8S_IBUS_FRAME_SIZE      (32U)
#define FS_A8S_IBUS_DATA_SIZE       (30U)
#define FS_A8S_IBUS_FRAME_LENGTH    (0x20U)
#define FS_A8S_IBUS_SERVO_COMMAND   (0x40U)

fs_a8s_channel_data_struct fs_a8s_channel_data;

static uint8 fs_a8s_ibus_frame[FS_A8S_IBUS_FRAME_SIZE];
static uint8 fs_a8s_ibus_frame_index = 0U;

static bool fs_a8s_ibus_check_frame(void)
{
    uint8 index;
    uint16 checksum = 0xFFFFU;
    uint16 received_checksum;

    for(index = 0U; index < FS_A8S_IBUS_DATA_SIZE; index++)
    {
        checksum -= fs_a8s_ibus_frame[index];
    }

    received_checksum = (uint16)fs_a8s_ibus_frame[30]
        | ((uint16)fs_a8s_ibus_frame[31] << 8);
    return (checksum == received_checksum);
}

static void fs_a8s_ibus_update_channels(void)
{
    uint8 channel_index;
    uint8 frame_index;

    for(channel_index = 0U; channel_index < FS_A8S_CHANNEL_COUNT; channel_index++)
    {
        frame_index = 2U + 2U * channel_index;
        fs_a8s_channel_data.channel[channel_index]
            = (uint16)fs_a8s_ibus_frame[frame_index]
            | ((uint16)fs_a8s_ibus_frame[frame_index + 1U] << 8);
    }
    fs_a8s_channel_data.valid_frame_count++;
    fs_a8s_channel_data.frame_age_ms = 0U;
}

// 初始化 FA-A8S 的 i-BUS 接收：D5 仅因 UART 驱动接口要求被配置为 TX，实际不用接线。
void fs_a8s_init(void)
{
    uint8 channel_index;

    for(channel_index = 0U; channel_index < FS_A8S_CHANNEL_COUNT; channel_index++)
    {
        fs_a8s_channel_data.channel[channel_index] = 0U;
    }
    fs_a8s_channel_data.valid_frame_count = 0U;
    fs_a8s_channel_data.frame_age_ms = FS_A8S_FRAME_TIMEOUT_MS;
    fs_a8s_ibus_frame_index = 0U;

    uart_init(UART_2, FS_A8S_IBUS_BAUD_RATE, UART2_TX_D5, UART2_RX_D6);
    uart_rx_interrupt(UART_2, 1U);
}

// 由 UART2_IRQHandler 在每收到一个字节时调用。
void fs_a8s_uart_callback(void)
{
    uint8 data;

    uart_query_byte(UART_2, &data);

    if(0U == fs_a8s_ibus_frame_index)
    {
        if(FS_A8S_IBUS_FRAME_LENGTH == data)
        {
            fs_a8s_ibus_frame[fs_a8s_ibus_frame_index++] = data;
        }
        return;
    }

    if(1U == fs_a8s_ibus_frame_index && FS_A8S_IBUS_SERVO_COMMAND != data)
    {
        // 当前字节若恰好又是帧头，直接作为下一帧起点，提高丢字节后的恢复速度。
        fs_a8s_ibus_frame_index = (FS_A8S_IBUS_FRAME_LENGTH == data) ? 1U : 0U;
        if(1U == fs_a8s_ibus_frame_index)
        {
            fs_a8s_ibus_frame[0] = data;
        }
        return;
    }

    fs_a8s_ibus_frame[fs_a8s_ibus_frame_index++] = data;
    if(FS_A8S_IBUS_FRAME_SIZE == fs_a8s_ibus_frame_index)
    {
        if(fs_a8s_ibus_check_frame())
        {
            fs_a8s_ibus_update_channels();
        }
        fs_a8s_ibus_frame_index = 0U;
    }
}

// 由 1ms 定时任务调用，提供无线失联保护所需的最后有效帧年龄。
void fs_a8s_1ms_task(void)
{
    if(fs_a8s_channel_data.frame_age_ms < FS_A8S_FRAME_TIMEOUT_MS)
    {
        fs_a8s_channel_data.frame_age_ms++;
    }
}

bool fs_a8s_is_online(void)
{
    return (fs_a8s_channel_data.frame_age_ms < FS_A8S_FRAME_TIMEOUT_MS);
}
