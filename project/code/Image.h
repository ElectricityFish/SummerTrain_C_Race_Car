#ifndef __IMAGE_H
#define __IMAGE_H

#include "zf_device_mt9v03x.h"

//初始化MT9V03X摄像头。返回0表示成功，1表示失败。
uint8 image_init(void);

//在主循环中调用，接收底层DMA已经采集完成的新图像。
void image_update(void);

//由DMA1通道4完成中断调用，记录一次实际完成的图像采集。
void image_dma_finish_handler(void);

//由摄像头VSYNC外部中断调用，记录一次摄像头帧同步事件。
void image_vsync_event_handler(void);

//每完成一帧图像处理后调用，记录一次实际完成的图像处理。
void image_process_finish_handler(void);

//由TIM6的1ms定时中断调用，跟踪帧间隔并每1000ms结算帧率和最大间隔。
void image_fps_1ms_task(void);

//获取最近一个完整1秒统计窗口内的实际图像采集帧率。
uint16 image_get_capture_fps(void);

//获取最近一个完整1秒统计窗口内的实际图像处理帧率。
uint16 image_get_process_fps(void);

//获取最近一个完整1秒统计窗口内观察到的最大采集帧间隔，单位为ms。
uint16 image_get_capture_max_gap_ms(void);

//获取最近一个完整1秒统计窗口内观察到的最大VSYNC间隔，单位为ms。
uint16 image_get_vsync_max_gap_ms(void);

//获取最近一个完整1秒统计窗口内观察到的最大处理帧间隔，单位为ms。
uint16 image_get_process_max_gap_ms(void);

//获取一帧未处理的新图像标志。调用后会清除该标志。
bool image_take_new_frame(void);

//获取灰度图像缓冲区首地址，图像尺寸由MT9V03X_W和MT9V03X_H定义。
const uint8 *image_get_buffer(void);

#endif
