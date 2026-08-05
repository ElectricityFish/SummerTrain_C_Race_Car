#ifndef __SPEED_TUNE_SERIAL_H__
#define __SPEED_TUNE_SERIAL_H__

#include "zf_common_typedef.h"

// 临时调试分支专用：COM11/UART6 自动调速协议与地面测试安全联锁。
typedef enum
{
	SPEED_TUNE_STOP_NONE = 0,
	SPEED_TUNE_STOP_COMMAND,
	SPEED_TUNE_STOP_TIME,
	SPEED_TUNE_STOP_DISTANCE,
	SPEED_TUNE_STOP_HEARTBEAT,
	SPEED_TUNE_STOP_REMOTE,
	SPEED_TUNE_STOP_ATTITUDE,
	SPEED_TUNE_STOP_OVERSPEED,
	SPEED_TUNE_STOP_DIRECTION,
	SPEED_TUNE_STOP_STATE,
} Speed_Tune_Stop_Reason_t;

void speed_tune_serial_init(void);

// 主循环任务：接收命令、发送 ACK、遥测与停止事件。
void speed_tune_serial_task(void);

// 由 1 ms 定时任务调用：运行时限、心跳、CH5 和状态机联锁。
void speed_tune_serial_1ms_task(void);

// 由速度 PI 更新完成后的 10 ms 任务调用：采样、距离和异常检测。
void speed_tune_serial_10ms_task(void);

bool speed_tune_serial_is_running(void);

#endif
