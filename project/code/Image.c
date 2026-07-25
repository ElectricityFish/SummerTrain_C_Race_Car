#include "Image.h"

static bool image_new_frame = false;
static vuint16 image_dma_frame_count = 0;
static uint16 image_capture_fps = 0;
static uint16 image_fps_time_ms = 0;

//初始化底层摄像头和DMA采集链路
uint8 image_init(void)
{
	image_new_frame = false;
	image_dma_frame_count = 0;
	image_capture_fps = 0;
	image_fps_time_ms = 0;
	mt9v03x_finish_flag = 0;
	return mt9v03x_init();
}

//将底层DMA完成标志转换为应用层的新帧标志
void image_update(void)
{
	if(mt9v03x_finish_flag)
	{
		mt9v03x_finish_flag = 0;
		image_new_frame = true;
	}
}

//每次DMA完整搬运完一帧188x120图像时调用，因此该计数就是实际采集帧数。
void image_dma_finish_handler(void)
{
	image_dma_frame_count++;
}

//TIM6每1ms调用一次。每经过完整的1秒，将该秒内的DMA完成次数保存为采集帧率。
void image_fps_1ms_task(void)
{
	image_fps_time_ms++;
	if(image_fps_time_ms >= 1000)
	{
		image_capture_fps = image_dma_frame_count;
		image_dma_frame_count = 0;
		image_fps_time_ms = 0;
	}
}

//返回最近一个完整统计周期内的实际采集帧率，单位为帧/秒。
uint16 image_get_capture_fps(void)
{
	return image_capture_fps;
}

//取走一帧新图像的通知。图像数据仍保存在底层全局缓冲区中。
bool image_take_new_frame(void)
{
	if(!image_new_frame)
	{
		return false;
	}

	image_new_frame = false;
	return true;
}

//返回二维灰度图像数组的首地址，供显示或图像处理模块使用
const uint8 *image_get_buffer(void)
{
	return (const uint8 *)&mt9v03x_image[0][0];
}
