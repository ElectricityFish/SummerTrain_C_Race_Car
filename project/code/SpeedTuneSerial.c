#include "SpeedTuneSerial.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "Control.h"
#include "Motor.h"
#include "SpeedControl.h"
#include "Wireless.h"
#include "zf_common_interrupt.h"

#define SPEED_TUNE_PROTOCOL_VERSION          (1U)
#define SPEED_TUNE_RX_CHUNK_SIZE             (64U)
#define SPEED_TUNE_LINE_SIZE                 (64U)
#define SPEED_TUNE_SAMPLE_BUFFER_SIZE        (64U)
#define SPEED_TUNE_TELEMETRY_BURST           (4U)

#define SPEED_TUNE_HEARTBEAT_TIMEOUT_MS      (300U)
#define SPEED_TUNE_ARM_TIMEOUT_MS            (30000U)
#define SPEED_TUNE_RUN_TIME_MIN_MS           (100U)
#define SPEED_TUNE_RUN_TIME_MAX_MS           (1500U)
#define SPEED_TUNE_PULSE_LIMIT_MIN           (500U)
#define SPEED_TUNE_PULSE_LIMIT_MAX           (25000U)
#define SPEED_TUNE_OVERSPEED_MARGIN          (150)
#define SPEED_TUNE_OVERSPEED_MIN             (250)
#define SPEED_TUNE_DIRECTION_REVERSE_LIMIT   (-30)
#define SPEED_TUNE_FAULT_CONFIRM_SAMPLES     (2U)

typedef struct
{
	uint16 run_id;
	uint16 tick;
	Speed_Tune_Snapshot_t speed;
} Speed_Tune_Sample_t;

static Speed_Tune_Config_t speed_tune_config;
static volatile uint8 speed_tune_configured;
static volatile uint8 speed_tune_armed;
static volatile uint8 speed_tune_running;
static volatile uint8 speed_tune_low_seen;
static volatile uint16 speed_tune_arm_age_ms;
static volatile uint16 speed_tune_heartbeat_age_ms;
static volatile uint16 speed_tune_run_elapsed_ms;
static volatile uint16 speed_tune_run_limit_ms;
static volatile uint16 speed_tune_run_tick;
static volatile uint16 speed_tune_run_id;
static volatile uint32 speed_tune_pulse_sum;
static volatile uint32 speed_tune_pulse_limit;
static volatile uint8 speed_tune_fault_count;

static Speed_Tune_Sample_t speed_tune_samples[SPEED_TUNE_SAMPLE_BUFFER_SIZE];
static volatile uint8 speed_tune_sample_head;
static volatile uint8 speed_tune_sample_tail;
static volatile uint16 speed_tune_sample_dropped;

static volatile uint8 speed_tune_stop_event_pending;
static volatile Speed_Tune_Stop_Reason_t speed_tune_stop_reason;
static volatile uint16 speed_tune_stop_run_id;
static volatile uint16 speed_tune_stop_elapsed_ms;
static volatile uint32 speed_tune_stop_pulse_sum;
static volatile uint16 speed_tune_stop_dropped;

static char speed_tune_line[SPEED_TUNE_LINE_SIZE];
static uint8 speed_tune_line_length;
static uint8 speed_tune_line_overflow;

static int32 speed_tune_abs_int16(int16 value)
{
	return (value >= 0) ? (int32)value : -(int32)value;
}

static int16 speed_tune_limit_int16(int32 value)
{
	if(value > 32767)
	{
		return 32767;
	}
	if(value < -32767)
	{
		return -32767;
	}
	return (int16)value;
}

static uint8 speed_tune_hex_value(char value)
{
	if(value >= '0' && value <= '9')
	{
		return (uint8)(value - '0');
	}
	if(value >= 'A' && value <= 'F')
	{
		return (uint8)(value - 'A' + 10);
	}
	if(value >= 'a' && value <= 'f')
	{
		return (uint8)(value - 'a' + 10);
	}
	return 0xFFU;
}

static bool speed_tune_check_checksum(char *line)
{
	char *star;
	uint8 checksum = 0U;
	uint8 high;
	uint8 low;
	char *cursor;

	star = strrchr(line, '*');
	if(star == NULL || star[1] == '\0' || star[2] == '\0' || star[3] != '\0')
	{
		return false;
	}

	high = speed_tune_hex_value(star[1]);
	low = speed_tune_hex_value(star[2]);
	if(high > 0x0FU || low > 0x0FU)
	{
		return false;
	}

	for(cursor = line; cursor < star; cursor++)
	{
		checksum ^= (uint8)(*cursor);
	}
	if(checksum != (uint8)((high << 4) | low))
	{
		return false;
	}

	*star = '\0';
	return true;
}

static bool speed_tune_parse_long(const char *text, int32 *value)
{
	char *end;
	long parsed;

	if(text == NULL || value == NULL || *text == '\0')
	{
		return false;
	}
	parsed = strtol(text, &end, 10);
	if(*end != '\0')
	{
		return false;
	}
	*value = (int32)parsed;
	return true;
}

static const char *speed_tune_reason_text(Speed_Tune_Stop_Reason_t reason)
{
	switch(reason)
	{
		case SPEED_TUNE_STOP_COMMAND: return "COMMAND";
		case SPEED_TUNE_STOP_TIME: return "TIME";
		case SPEED_TUNE_STOP_DISTANCE: return "DISTANCE";
		case SPEED_TUNE_STOP_HEARTBEAT: return "HEARTBEAT";
		case SPEED_TUNE_STOP_REMOTE: return "REMOTE";
		case SPEED_TUNE_STOP_ATTITUDE: return "ATTITUDE";
		case SPEED_TUNE_STOP_OVERSPEED: return "OVERSPEED";
		case SPEED_TUNE_STOP_DIRECTION: return "DIRECTION";
		case SPEED_TUNE_STOP_STATE: return "STATE";
		default: return "NONE";
	}
}

static void speed_tune_clear_samples(void)
{
	uint32 primask = interrupt_global_disable();
	speed_tune_sample_head = 0U;
	speed_tune_sample_tail = 0U;
	speed_tune_sample_dropped = 0U;
	interrupt_global_enable(primask);
}

static void speed_tune_stop(Speed_Tune_Stop_Reason_t reason)
{
	uint8 was_running;
	uint32 primask = interrupt_global_disable();

	was_running = speed_tune_running;
	speed_tune_running = 0U;
	speed_tune_armed = 0U;
	speed_tune_fault_count = 0U;
	speed_control_tune_stop();
	motor_set_duty(0, 0);
	control_speed_tune_exit();

	if(was_running != 0U)
	{
		speed_tune_stop_reason = reason;
		speed_tune_stop_run_id = speed_tune_run_id;
		speed_tune_stop_elapsed_ms = speed_tune_run_elapsed_ms;
		speed_tune_stop_pulse_sum = speed_tune_pulse_sum;
		speed_tune_stop_dropped = speed_tune_sample_dropped;
		speed_tune_stop_event_pending = 1U;
	}
	interrupt_global_enable(primask);
}

static void speed_tune_push_sample(const Speed_Tune_Sample_t *sample)
{
	uint8 next_head;

	if(sample == NULL)
	{
		return;
	}
	next_head = (uint8)((speed_tune_sample_head + 1U) % SPEED_TUNE_SAMPLE_BUFFER_SIZE);
	if(next_head == speed_tune_sample_tail)
	{
		if(speed_tune_sample_dropped < 65535U)
		{
			speed_tune_sample_dropped++;
		}
		return;
	}
	speed_tune_samples[speed_tune_sample_head] = *sample;
	speed_tune_sample_head = next_head;
}

static bool speed_tune_pop_sample(Speed_Tune_Sample_t *sample)
{
	uint32 primask;

	if(sample == NULL)
	{
		return false;
	}
	primask = interrupt_global_disable();
	if(speed_tune_sample_tail == speed_tune_sample_head)
	{
		interrupt_global_enable(primask);
		return false;
	}
	*sample = speed_tune_samples[speed_tune_sample_tail];
	speed_tune_sample_tail = (uint8)((speed_tune_sample_tail + 1U)
		% SPEED_TUNE_SAMPLE_BUFFER_SIZE);
	interrupt_global_enable(primask);
	return true;
}

static void speed_tune_send_error(uint16 sequence, const char *reason)
{
	wireless_uart_printf("ERR,%u,%s\n", sequence, reason);
}

static bool speed_tune_parse_sequence(char *token, uint16 *sequence)
{
	int32 value;
	if(!speed_tune_parse_long(token, &value) || value < 0 || value > 65535)
	{
		return false;
	}
	*sequence = (uint16)value;
	return true;
}

static void speed_tune_command_cfg(uint16 sequence, char *tokens[], uint8 token_count)
{
	int32 values[8];
	uint8 index;
	Speed_Tune_Config_t config;

	if(token_count != 10U)
	{
		speed_tune_send_error(sequence, "FIELDS");
		return;
	}
	if(speed_tune_running != 0U || speed_tune_armed != 0U)
	{
		speed_tune_send_error(sequence, "BUSY");
		return;
	}
	for(index = 0U; index < 8U; index++)
	{
		if(!speed_tune_parse_long(tokens[index + 2U], &values[index]))
		{
			speed_tune_send_error(sequence, "NUMBER");
			return;
		}
	}
	if(values[0] < 0 || values[0] > 10000
		|| values[1] < 0 || values[1] > 10000
		|| values[2] < 0 || values[2] > 10000
		|| values[3] < 0 || values[3] > 10000
		|| values[4] < 0 || values[4] > SPEED_CONTROL_RUN_TARGET_MAX
		|| values[5] < 0 || values[5] > SPEED_CONTROL_RUN_TARGET_MAX
		|| values[6] < 0 || values[6] > 8000
		|| values[7] < 0 || values[7] > 8000)
	{
		speed_tune_send_error(sequence, "RANGE");
		return;
	}

	config.left_kp = (float)values[0] / 1000.0f;
	config.left_ki = (float)values[1] / 1000.0f;
	config.right_kp = (float)values[2] / 1000.0f;
	config.right_ki = (float)values[3] / 1000.0f;
	config.left_target = (int16)values[4];
	config.right_target = (int16)values[5];
	config.left_out_max = (int16)values[6];
	config.right_out_max = (int16)values[7];
	speed_tune_config = config;
	speed_tune_configured = 1U;
	wireless_uart_printf("ACK,%u,CFG\n", sequence);
}

static void speed_tune_command_arm(uint16 sequence, uint8 token_count)
{
	if(token_count != 2U)
	{
		speed_tune_send_error(sequence, "FIELDS");
		return;
	}
	if(speed_tune_configured == 0U)
	{
		speed_tune_send_error(sequence, "NO_CFG");
		return;
	}
	if(common_state != COMMON_STATE_IDLE || wireless_control_enabled != 0U)
	{
		speed_tune_send_error(sequence, "STATE");
		return;
	}
	if(speed_control_debug_is_active())
	{
		speed_tune_send_error(sequence, "LOCAL_DEBUG");
		return;
	}
	if(speed_tune_low_seen == 0U || !control_remote_kill_released())
	{
		speed_tune_send_error(sequence, "CH5_LOW_REQUIRED");
		return;
	}

	speed_tune_low_seen = 0U;
	speed_tune_armed = 1U;
	speed_tune_arm_age_ms = 0U;
	speed_tune_heartbeat_age_ms = 0U;
	wireless_uart_printf("ACK,%u,ARM\n", sequence);
}

static void speed_tune_command_run(uint16 sequence, char *tokens[], uint8 token_count)
{
	int32 run_time;
	int32 pulse_limit;
	uint32 primask;

	if(token_count != 4U
		|| !speed_tune_parse_long(tokens[2], &run_time)
		|| !speed_tune_parse_long(tokens[3], &pulse_limit))
	{
		speed_tune_send_error(sequence, "FIELDS");
		return;
	}
	if(run_time < SPEED_TUNE_RUN_TIME_MIN_MS || run_time > SPEED_TUNE_RUN_TIME_MAX_MS
		|| pulse_limit < SPEED_TUNE_PULSE_LIMIT_MIN
		|| pulse_limit > SPEED_TUNE_PULSE_LIMIT_MAX)
	{
		speed_tune_send_error(sequence, "RANGE");
		return;
	}
	if(speed_tune_armed == 0U || speed_tune_running != 0U)
	{
		speed_tune_send_error(sequence, "NOT_ARMED");
		return;
	}
	if(!control_remote_kill_permitted())
	{
		speed_tune_send_error(sequence, "CH5_HIGH_REQUIRED");
		return;
	}

	primask = interrupt_global_disable();
	if(!control_speed_tune_enter())
	{
		interrupt_global_enable(primask);
		speed_tune_send_error(sequence, "STATE");
		return;
	}
	if(!speed_control_tune_start(&speed_tune_config))
	{
		control_speed_tune_exit();
		interrupt_global_enable(primask);
		speed_tune_send_error(sequence, "CONFIG");
		return;
	}

	speed_tune_run_id = sequence;
	speed_tune_run_limit_ms = (uint16)run_time;
	speed_tune_pulse_limit = (uint32)pulse_limit;
	speed_tune_run_elapsed_ms = 0U;
	speed_tune_run_tick = 0U;
	speed_tune_pulse_sum = 0U;
	speed_tune_fault_count = 0U;
	speed_tune_heartbeat_age_ms = 0U;
	speed_tune_stop_event_pending = 0U;
	speed_tune_clear_samples();
	speed_tune_running = 1U;
	interrupt_global_enable(primask);
	wireless_uart_printf("ACK,%u,RUN\n", sequence);
}

static void speed_tune_process_line(char *line)
{
	char *tokens[12];
	char *token;
	uint8 token_count = 0U;
	uint16 sequence = 0U;

	if(!speed_tune_check_checksum(line))
	{
		speed_tune_send_error(0U, "CHECKSUM");
		return;
	}

	token = strtok(line, ",");
	while(token != NULL && token_count < 12U)
	{
		tokens[token_count++] = token;
		token = strtok(NULL, ",");
	}
	if(token_count < 2U || !speed_tune_parse_sequence(tokens[1], &sequence))
	{
		speed_tune_send_error(0U, "SEQUENCE");
		return;
	}

	if(strcmp(tokens[0], "HELLO") == 0)
	{
		wireless_uart_printf("HELLO,%u,SPEED_TUNE,%u,115200\n",
			sequence, SPEED_TUNE_PROTOCOL_VERSION);
	}
	else if(strcmp(tokens[0], "CFG") == 0)
	{
		speed_tune_command_cfg(sequence, tokens, token_count);
	}
	else if(strcmp(tokens[0], "ARM") == 0)
	{
		speed_tune_command_arm(sequence, token_count);
	}
	else if(strcmp(tokens[0], "RUN") == 0)
	{
		speed_tune_command_run(sequence, tokens, token_count);
	}
	else if(strcmp(tokens[0], "HB") == 0)
	{
		speed_tune_heartbeat_age_ms = 0U;
		wireless_uart_printf("ACK,%u,HB\n", sequence);
	}
	else if(strcmp(tokens[0], "STOP") == 0)
	{
		speed_tune_stop(SPEED_TUNE_STOP_COMMAND);
		wireless_uart_printf("ACK,%u,STOP\n", sequence);
	}
	else if(strcmp(tokens[0], "GET") == 0)
	{
		wireless_uart_printf("STATE,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
			sequence,
			speed_tune_configured,
			speed_tune_armed,
			speed_tune_running,
			control_remote_kill_released() ? 1U : 0U,
			control_remote_kill_permitted() ? 1U : 0U,
			(uint16)(speed_tune_config.left_kp * 1000.0f + 0.5f),
			(uint16)(speed_tune_config.left_ki * 1000.0f + 0.5f),
			(uint16)(speed_tune_config.right_kp * 1000.0f + 0.5f),
			(uint16)(speed_tune_config.right_ki * 1000.0f + 0.5f),
			speed_tune_config.left_target,
			speed_tune_config.right_target,
			speed_tune_config.left_out_max,
			speed_tune_config.right_out_max);
	}
	else
	{
		speed_tune_send_error(sequence, "COMMAND");
	}
}

void speed_tune_serial_init(void)
{
	memset(&speed_tune_config, 0, sizeof(speed_tune_config));
	speed_tune_configured = 0U;
	speed_tune_armed = 0U;
	speed_tune_running = 0U;
	speed_tune_low_seen = 0U;
	speed_tune_arm_age_ms = 0U;
	speed_tune_heartbeat_age_ms = SPEED_TUNE_HEARTBEAT_TIMEOUT_MS;
	speed_tune_run_elapsed_ms = 0U;
	speed_tune_run_limit_ms = 0U;
	speed_tune_run_tick = 0U;
	speed_tune_run_id = 0U;
	speed_tune_pulse_sum = 0U;
	speed_tune_pulse_limit = 0U;
	speed_tune_fault_count = 0U;
	speed_tune_stop_event_pending = 0U;
	speed_tune_stop_reason = SPEED_TUNE_STOP_NONE;
	speed_tune_line_length = 0U;
	speed_tune_line_overflow = 0U;
	speed_tune_clear_samples();
}

void speed_tune_serial_1ms_task(void)
{
	if(control_remote_kill_released())
	{
		speed_tune_low_seen = 1U;
	}

	if(speed_tune_armed != 0U && speed_tune_running == 0U)
	{
		if(speed_tune_arm_age_ms < SPEED_TUNE_ARM_TIMEOUT_MS)
		{
			speed_tune_arm_age_ms++;
		}
		else
		{
			speed_tune_armed = 0U;
		}
	}

	if(speed_tune_running == 0U)
	{
		return;
	}

	if(speed_tune_heartbeat_age_ms < 65535U)
	{
		speed_tune_heartbeat_age_ms++;
	}
	if(speed_tune_run_elapsed_ms < 65535U)
	{
		speed_tune_run_elapsed_ms++;
	}

	if(!control_remote_kill_permitted())
	{
		speed_tune_stop(SPEED_TUNE_STOP_REMOTE);
	}
	else if(!control_speed_tune_is_active())
	{
		speed_tune_stop((car_protection_reason != CAR_PROTECTION_REASON_NONE)
			? SPEED_TUNE_STOP_ATTITUDE : SPEED_TUNE_STOP_STATE);
	}
	else if(speed_tune_heartbeat_age_ms > SPEED_TUNE_HEARTBEAT_TIMEOUT_MS)
	{
		speed_tune_stop(SPEED_TUNE_STOP_HEARTBEAT);
	}
	else if(speed_tune_run_elapsed_ms >= speed_tune_run_limit_ms)
	{
		speed_tune_stop(SPEED_TUNE_STOP_TIME);
	}
}

void speed_tune_serial_10ms_task(void)
{
	Speed_Tune_Sample_t sample;
	int32 left_abs;
	int32 right_abs;
	int32 left_limit;
	int32 right_limit;
	bool overspeed;
	bool wrong_direction;

	if(speed_tune_running == 0U || !control_speed_tune_is_active())
	{
		return;
	}

	sample.run_id = speed_tune_run_id;
	sample.tick = speed_tune_run_tick++;
	speed_control_tune_get_snapshot(&sample.speed);
	// 防止遥测中的积分项转换溢出；正式控制量不受影响。
	sample.speed.left_i_out = speed_tune_limit_int16(sample.speed.left_i_out);
	sample.speed.right_i_out = speed_tune_limit_int16(sample.speed.right_i_out);
	speed_tune_push_sample(&sample);

	left_abs = speed_tune_abs_int16(sample.speed.left_actual);
	right_abs = speed_tune_abs_int16(sample.speed.right_actual);
	speed_tune_pulse_sum += (uint32)((left_abs + right_abs) / 2);

	left_limit = (int32)sample.speed.left_target + SPEED_TUNE_OVERSPEED_MARGIN;
	right_limit = (int32)sample.speed.right_target + SPEED_TUNE_OVERSPEED_MARGIN;
	if(left_limit < SPEED_TUNE_OVERSPEED_MIN) left_limit = SPEED_TUNE_OVERSPEED_MIN;
	if(right_limit < SPEED_TUNE_OVERSPEED_MIN) right_limit = SPEED_TUNE_OVERSPEED_MIN;
	overspeed = (left_abs > left_limit) || (right_abs > right_limit);
	wrong_direction = ((sample.speed.left_target > 0)
			&& (sample.speed.left_actual < SPEED_TUNE_DIRECTION_REVERSE_LIMIT))
		|| ((sample.speed.right_target > 0)
			&& (sample.speed.right_actual < SPEED_TUNE_DIRECTION_REVERSE_LIMIT));

	if(overspeed || wrong_direction)
	{
		if(speed_tune_fault_count < SPEED_TUNE_FAULT_CONFIRM_SAMPLES)
		{
			speed_tune_fault_count++;
		}
	}
	else
	{
		speed_tune_fault_count = 0U;
	}

	if(speed_tune_fault_count >= SPEED_TUNE_FAULT_CONFIRM_SAMPLES)
	{
		speed_tune_stop(wrong_direction
			? SPEED_TUNE_STOP_DIRECTION : SPEED_TUNE_STOP_OVERSPEED);
	}
	else if(speed_tune_pulse_sum >= speed_tune_pulse_limit)
	{
		speed_tune_stop(SPEED_TUNE_STOP_DISTANCE);
	}
}

bool speed_tune_serial_is_running(void)
{
	return (speed_tune_running != 0U);
}

void speed_tune_serial_task(void)
{
	uint8 receive_buffer[SPEED_TUNE_RX_CHUNK_SIZE];
	uint32 received;
	uint32 index;
	uint8 sent = 0U;
	Speed_Tune_Sample_t sample;

	received = wireless_uart_read_buffer(receive_buffer, sizeof(receive_buffer));
	for(index = 0U; index < received; index++)
	{
		char value = (char)receive_buffer[index];
		if(value == '\r')
		{
			continue;
		}
		if(value == '\n')
		{
			if(speed_tune_line_overflow != 0U)
			{
				speed_tune_send_error(0U, "LINE_TOO_LONG");
			}
			else if(speed_tune_line_length > 0U)
			{
				speed_tune_line[speed_tune_line_length] = '\0';
				speed_tune_process_line(speed_tune_line);
			}
			speed_tune_line_length = 0U;
			speed_tune_line_overflow = 0U;
			continue;
		}
		if(speed_tune_line_length + 1U < SPEED_TUNE_LINE_SIZE)
		{
			speed_tune_line[speed_tune_line_length++] = value;
		}
		else
		{
			speed_tune_line_overflow = 1U;
		}
	}

	while(sent < SPEED_TUNE_TELEMETRY_BURST && speed_tune_pop_sample(&sample))
	{
		wireless_uart_printf(
			"D,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			sample.run_id,
			sample.tick,
			sample.speed.left_target,
			sample.speed.left_actual,
			sample.speed.left_out,
			sample.speed.left_error,
			sample.speed.left_i_out,
			sample.speed.right_target,
			sample.speed.right_actual,
			sample.speed.right_out,
			sample.speed.right_error,
			sample.speed.right_i_out);
		sent++;
	}

	if(speed_tune_stop_event_pending != 0U
		&& speed_tune_sample_head == speed_tune_sample_tail)
	{
		wireless_uart_printf("DONE,%u,%s,%u,%lu,%u\n",
			speed_tune_stop_run_id,
			speed_tune_reason_text(speed_tune_stop_reason),
			speed_tune_stop_elapsed_ms,
			(unsigned long)speed_tune_stop_pulse_sum,
			speed_tune_stop_dropped);
		speed_tune_stop_event_pending = 0U;
	}
}
