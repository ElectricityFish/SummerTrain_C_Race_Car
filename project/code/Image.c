#include "Image.h"

#include "zf_driver_dma.h"

static bool image_new_frame = false;
static vuint16 image_dma_frame_count = 0;
static vuint32 image_vsync_total = 0;
static vuint32 image_capture_frame_total = 0;
static vuint32 image_process_frame_total = 0;
static uint32 image_process_frame_total_last = 0;
static uint16 image_capture_fps = 0;
static uint16 image_process_fps = 0;
static uint16 image_fps_time_ms = 0;
static uint32 image_vsync_gap_total_last = 0;
static uint32 image_capture_gap_total_last = 0;
static uint32 image_process_gap_total_last = 0;
static uint16 image_vsync_gap_age_ms = 0;
static uint16 image_capture_gap_age_ms = 0;
static uint16 image_process_gap_age_ms = 0;
static uint16 image_vsync_max_gap_current_ms = 0;
static uint16 image_capture_max_gap_current_ms = 0;
static uint16 image_process_max_gap_current_ms = 0;
static uint16 image_vsync_max_gap_ms = 0;
static uint16 image_capture_max_gap_ms = 0;
static uint16 image_process_max_gap_ms = 0;
static bool image_vsync_gap_started = false;
static bool image_capture_gap_started = false;
static bool image_process_gap_started = false;
static uint8 image_capture_buffer_secondary[MT9V03X_H][MT9V03X_W];
static uint8 * volatile image_capture_write_buffer = &mt9v03x_image[0][0];
static const uint8 * volatile image_latest_frame = NULL;
static const uint8 * volatile image_processing_buffer = NULL;
static volatile bool image_capture_in_progress = false;

//由1ms任务根据帧完成总数计算帧间隔；统计误差不超过一个定时周期。
static void image_gap_1ms_update(
	uint32 frame_total,
	uint32 *last_total,
	bool *started,
	uint16 *gap_age_ms,
	uint16 *max_gap_ms)
{
	if(!(*started))
	{
		if(frame_total != *last_total)
		{
			*last_total = frame_total;
			*started = true;
		}
		return;
	}

	if(*gap_age_ms < 0xFFFFU)
	{
		(*gap_age_ms)++;
	}
	if(*gap_age_ms > *max_gap_ms)
	{
		*max_gap_ms = *gap_age_ms;
	}

	if(frame_total != *last_total)
	{
		*last_total = frame_total;
		*gap_age_ms = 0U;
	}
}

//初始化底层摄像头和DMA采集链路
uint8 image_init(void)
{
	image_new_frame = false;
	image_dma_frame_count = 0;
	image_vsync_total = 0;
	image_capture_frame_total = 0;
	image_process_frame_total = 0;
	image_process_frame_total_last = 0;
	image_capture_fps = 0;
	image_process_fps = 0;
	image_fps_time_ms = 0;
	image_vsync_gap_total_last = 0;
	image_capture_gap_total_last = 0;
	image_process_gap_total_last = 0;
	image_vsync_gap_age_ms = 0;
	image_capture_gap_age_ms = 0;
	image_process_gap_age_ms = 0;
	image_vsync_max_gap_current_ms = 0;
	image_capture_max_gap_current_ms = 0;
	image_process_max_gap_current_ms = 0;
	image_vsync_max_gap_ms = 0;
	image_capture_max_gap_ms = 0;
	image_process_max_gap_ms = 0;
	image_vsync_gap_started = false;
	image_capture_gap_started = false;
	image_process_gap_started = false;
	image_capture_write_buffer = &mt9v03x_image[0][0];
	image_latest_frame = NULL;
	image_processing_buffer = NULL;
	image_capture_in_progress = false;
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
	image_capture_in_progress = false;
	image_latest_frame = image_capture_write_buffer;
	image_dma_frame_count++;
	image_capture_frame_total++;
}

//每次摄像头VSYNC到来时调用，用于区分摄像头输出间隔与DMA完成间隔。
void image_vsync_event_handler(void)
{
	uint8 *next_write_buffer;

	// ISR 中本函数先于厂商 camera_vsync_handler() 调用。先关闭通道并切换目标，
	// 随后的厂商处理函数只重装计数并启动DMA，因此两块缓冲区会逐帧交替写入。
	dma_disable(MT9V03X_DMA_CH);
	next_write_buffer =
		(image_capture_write_buffer == &mt9v03x_image[0][0])
		? &image_capture_buffer_secondary[0][0]
		: &mt9v03x_image[0][0];
	// 算法或Preview仍持有首选缓冲区时，继续复用另一块写缓冲，绝不覆盖读帧。
	if(next_write_buffer == image_processing_buffer)
	{
		next_write_buffer = image_capture_write_buffer;
	}
	image_capture_write_buffer = next_write_buffer;
	dma_set_destination(MT9V03X_DMA_CH, (uint32)image_capture_write_buffer);
	image_capture_in_progress = true;
	image_vsync_total++;
}

//每完成一帧图像处理时调用，因此该计数就是主循环实际完成的处理帧数。
void image_process_finish_handler(void)
{
	image_process_frame_total++;
}

//TIM6每1ms调用一次。持续跟踪帧间隔，并每1秒保存帧率与最大间隔。
void image_fps_1ms_task(void)
{
	image_gap_1ms_update(
		image_vsync_total,
		&image_vsync_gap_total_last,
		&image_vsync_gap_started,
		&image_vsync_gap_age_ms,
		&image_vsync_max_gap_current_ms);
	image_gap_1ms_update(
		image_capture_frame_total,
		&image_capture_gap_total_last,
		&image_capture_gap_started,
		&image_capture_gap_age_ms,
		&image_capture_max_gap_current_ms);
	image_gap_1ms_update(
		image_process_frame_total,
		&image_process_gap_total_last,
		&image_process_gap_started,
		&image_process_gap_age_ms,
		&image_process_max_gap_current_ms);

	image_fps_time_ms++;
	if(image_fps_time_ms >= 1000)
	{
		uint32 process_frame_total_now = image_process_frame_total;

		image_capture_fps = image_dma_frame_count;
		image_process_fps = (uint16)(process_frame_total_now - image_process_frame_total_last);
		image_process_frame_total_last = process_frame_total_now;
		image_dma_frame_count = 0;
		image_vsync_max_gap_ms = image_vsync_max_gap_current_ms;
		image_capture_max_gap_ms = image_capture_max_gap_current_ms;
		image_process_max_gap_ms = image_process_max_gap_current_ms;
		//若当前正处于长时间无新帧状态，跨窗口后继续保留已经累计的间隔。
		image_vsync_max_gap_current_ms = image_vsync_gap_age_ms;
		image_capture_max_gap_current_ms = image_capture_gap_age_ms;
		image_process_max_gap_current_ms = image_process_gap_age_ms;
		image_fps_time_ms = 0;
	}
}

//返回最近一个完整统计周期内的实际采集帧率，单位为帧/秒。
uint16 image_get_capture_fps(void)
{
	return image_capture_fps;
}

//返回最近一个完整统计周期内的实际图像处理帧率，单位为帧/秒。
uint16 image_get_process_fps(void)
{
	return image_process_fps;
}

//返回最近一个完整1秒窗口内观察到的最大采集帧间隔，单位为ms。
uint16 image_get_capture_max_gap_ms(void)
{
	return image_capture_max_gap_ms;
}

//返回最近一个完整1秒窗口内观察到的最大VSYNC间隔，单位为ms。
uint16 image_get_vsync_max_gap_ms(void)
{
	return image_vsync_max_gap_ms;
}

//返回最近一个完整1秒窗口内观察到的最大处理帧间隔，单位为ms。
uint16 image_get_process_max_gap_ms(void)
{
	return image_process_max_gap_ms;
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

//先发布读锁，再复查DMA状态；即使VSYNC恰好在中间发生，也不会返回正在写入的帧。
const uint8 *image_acquire_latest_frame(void)
{
	const uint8 *frame = image_latest_frame;

	if(frame == NULL)
	{
		return NULL;
	}
	image_processing_buffer = frame;
	if(image_capture_in_progress && frame == image_capture_write_buffer)
	{
		image_processing_buffer = NULL;
		return NULL;
	}
	return frame;
}

void image_release_frame(const uint8 *frame)
{
	if(frame != NULL && image_processing_buffer == frame)
	{
		image_processing_buffer = NULL;
	}
}
