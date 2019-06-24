/*
* =====================================================================================
*
*  Copyright (C) 2016. Huami Ltd, unpublished work. This computer program includes
*  Confidential, Proprietary Information and is a Trade Secret of Huami Ltd.
*  All use, disclosure, and/or reproduction is prohibited unless authorized in writing.
*  All Rights Reserved.
*
*
* =====================================================================================
*/
#include "alg_config.h"
#if BUILD_ALG_MOVEMENT == 1
#include "alg_math.h"
#include "alg_movement.h"
#include "algorithm.h"

#define MedianFilter3Int16 MedianFilter3int16_t
#define median3_getScore_int16 median3_getScore_int16_t

static uint8_t basic_ret = 0;				//basic layer return
static uint8_t start_flag = 0;				//start detect flag
static uint32_t current_index = 0;			//the index of current data
static uint8_t dynamic_start_flag = 0;		//dynamic status start flag
static uint8_t upper_status_cnt = 0;		//upper layer dynamic status count
static uint32_t pre_index = 0;					//satisfied point previous index
static uint32_t cur_index = 0;					//satisfied point current index
bool pre_upper_ret;								//previous upper layer return value
static movement_detect movement_t;
static movement_feature movement_fea;
static movement_feature movement_fea2;
static MedianFilter3Int16 median_filter_x;	//the median filter
static MedianFilter3Int16 median_filter_y;	//the median filter
static MedianFilter3Int16 median_filter_z;	//the median filter

// dectect basic layer state
uint8_t detect_movement(movement_feature *move_fea, uint8_t length)
{
	uint8_t move_info = 0;
	int32_t avg_amp = move_fea->sum_amp / length;
	uint32_t var_amp = move_fea->square_sum_amp / length - avg_amp * avg_amp;

	memset(move_fea, 0, sizeof(movement_feature));

	if (var_amp >= VAR_AMP_THRESHOLD)
	{
		movement_t.start_flag = 1;
		movement_t.satisfied_std_cnt++;
	}
	if (movement_t.start_flag)
	{
		movement_t.window_cnt++;
		if (movement_t.window_cnt < WINDOWS_CNT_MAX)
		{
			if (movement_t.satisfied_std_cnt > SATISFIED_VAR_SIZE)
			{
				memset(&movement_t, 0, sizeof(movement_detect));
				move_info = IS_MOVE;
				pre_index = cur_index;
				cur_index = current_index;
			}
		}
		else
		{
			memset(&movement_t, 0, sizeof(movement_detect));
			move_info = IS_NOT_MOVE;
		}
	}
	else
	{
		move_info = IS_NOT_MOVE;
	}
	return move_info;
}
//get movement state
uint8_t get_movement(uint16_t amp_t)
{
	uint32_t var_amp = 0;
	uint32_t avg_amp = 0;
	uint8_t movement_info = 0;

	movement_fea.sum_amp += amp_t;
	movement_fea.square_sum_amp += (uint32_t)(amp_t) * (uint32_t)(amp_t);
	movement_fea.raw_data_cnt++;

	if ((movement_fea.raw_data_cnt > (WINDOW_LENGTH >> 1)) || (start_flag == 1))
	{
		movement_fea2.sum_amp += amp_t;
		movement_fea2.square_sum_amp += (uint32_t)(amp_t) * (uint32_t)(amp_t);
		movement_fea2.raw_data_cnt++;
		start_flag = 1;
	}
	if (movement_fea.raw_data_cnt >= WINDOW_LENGTH)
	{
		movement_info = detect_movement(&movement_fea, WINDOW_LENGTH);
	}
	if (movement_fea2.raw_data_cnt >= WINDOW_LENGTH)
	{
		movement_info = detect_movement(&movement_fea2, WINDOW_LENGTH);
	}

	return movement_info;
}
// acc filter
uint16_t get_smoothed_amp(int16_t* acc_buf)
{
	uint16_t amp_temp;
	int16_t acc_x = median3_getScore_int16(acc_buf[0], &median_filter_x);
	int16_t acc_y = median3_getScore_int16(acc_buf[1], &median_filter_y);
	int16_t acc_z = median3_getScore_int16(acc_buf[2], &median_filter_z);
	amp_temp = sqrt32(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

	return amp_temp;
}

/* initialize the movement parameters */
void movement_initialize(void)
{
	upper_status_cnt = 0;
	dynamic_start_flag = 0;
	pre_upper_ret = false;
	memset(&movement_fea, 0, sizeof(movement_feature));
	memset(&movement_fea2, 0, sizeof(movement_feature));
	memset(&movement_t, 0, sizeof(movement_detect));
	memset(&median_filter_x, 0, sizeof(MedianFilter3Int16));
	memset(&median_filter_y, 0, sizeof(MedianFilter3Int16));
	memset(&median_filter_z, 0, sizeof(MedianFilter3Int16));
}

/* main interface of movement state */
bool movement_receive_acc(const int16_t* acc_buf)
{
	uint16_t amp = 0;
	bool upper_ret = false;
	amp = get_smoothed_amp(acc_buf);
	if (current_index > 0)
	{
		basic_ret += get_movement(amp);
	}
	//get upper layer state
	if (current_index % UPPER_LAYER_SIZE == 0)// 1s 25Hz
	{
		if (basic_ret)
		{
			upper_status_cnt++;
			if ((upper_status_cnt == 5) || (dynamic_start_flag == 1))// when upper layer dynamic state continuous 5s or last state  is dynamic state (pre_upper_ret = true)
			{
				dynamic_start_flag = 1;
				upper_ret = true;
				upper_status_cnt = 0;
			}

			if ((basic_ret > 1) || (cur_index - pre_index == 15))
				basic_ret = 1;
			else
				basic_ret = 0;
			cur_index = 0;
		}
		else
		{
			dynamic_start_flag = 0;
			upper_status_cnt = 0;
			upper_ret = false;
		}
		pre_upper_ret = upper_ret;
	}
	current_index++;
	return pre_upper_ret;
}


static ALG_ERROR_T movement_enable()
{
	movement_initialize();
	return ALG_OK;
}

static ALG_ERROR_T movement_process(const sensor_data_t *data, alg_result_t *result)
{
	if (data->type == DATA_TYPE_ACC) {
		uint16_t i = 0;
		bool final_move = false;
		for (i = 0; i < data->count; i++) {
			short acc[3] = { 0 };
			acc[0] = data->acc_data[i].x;
			acc[1] = data->acc_data[i].y;
			acc[2] = data->acc_data[i].z;
			bool move = movement_receive_acc(&acc[0]);
			if (move) {
				final_move = true;
			}
		}
		//log_printf("movement data size:%d,move:%d\r\n", data->count,final_move);
		if (result) {
			result->updated = 1;
			result->type = ALG_TYPE_MOVEMENT;
			result->move.move = final_move;
		}
	}

	return ALG_OK;
}

static alg_descriptor_t g_desc = {
	.base = {
		.name = "movement",
		.version = "V1.0",
		.type = ALG_TYPE_MOVEMENT,
		.delay = 0,

		.enable = movement_enable,
		.disable = 0,
	},
	.data_type = DATA_TYPE_ACC,

	.process = movement_process,
};

void alg_movement_init()
{
	alg_register(&g_desc);
}
#endif
