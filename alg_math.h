/*=====================================================================================================
sensorhub_algo_math.h
=====================================================================================================

include some math calculate methods

Date			Author          Notes
2015/07/17	Haocheng Wu		Initial create

=====================================================================================================*/

#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef __ALG_MATH_H__
#define __ALG_MATH_H__

/************************** public math const ********************/

#define PI                      (3.1415926535897932384626433832795)


/********************** math initialize ****************************/

/* initialize static values, @return occupiedByte count */
uint16_t math_initialize(void);

/******************** Methods by simple MACROs ***********************/

#define MAX(a,b)							(((a) > (b)) ? (a) : (b))
#define MIN(a,b)							(((a) < (b)) ? (a) : (b))
#define ABS(x)								((x) >= 0 ? (x) : -(x))
#define ABS_MINUS(a,b)						(((a) > (b)) ? ((a)-(b)) : ((b)-(a)))


/******************** Methods for single value ***********************/

/* get fixed point sqrt */
uint16_t sqrt32(uint32_t m);

/* get fixed point sqrt for 64 bit */
uint32_t sqrt64(uint64_t val);

/* get fixed point pow */
int64_t pow64(int64_t x, int8_t pow);


/******************** Methods for similarity *************************/

/* get the distance between two values, (abs(v1-v2)/(abs(v1)+abs(v2)))^2 */
int32_t getValueDistance(int16_t value1, int16_t value2);



/******************** Methods for array ******************************/

/* get the max value in the values array */
uint16_t getMaxValue(uint16_t* values, uint8_t length);
/* get max value of uint16_t */
uint16_t get_max_u16(const uint16_t *data, uint16_t size);
/* get min value of uint16_t */
uint16_t get_min_u16(const uint16_t *data, uint16_t size);

/*get the max value forward*/
int16_t get_max_i16(int16_t *data, uint16_t size);

uint16_t get_max_u32(uint32_t *data, uint16_t size);

int16_t get_min_i16(int16_t *data, uint16_t size);
/* get min value of int32_t */
int32_t get_min_i32(int32_t *data, uint16_t size);
/* get max value of int32_t */
int32_t get_max_i32(int32_t *data, uint16_t size);
/* get Variance of acc data */
uint32_t accVar_u16(const uint16_t *data, uint16_t size);

uint32_t var_i16(int16_t *data, uint16_t size);

uint16_t mean_u16(const uint16_t *data, uint16_t size);


/******************** Methods for datet time *************************/

#define START_YEAR					(1980)

/* convert the time to day (from START_YEAR.START_MONTH.START_DAY) */
uint64_t convertToBaseDay(uint16_t year, uint8_t month, uint8_t day);

/* convert the time to milisecond (from START_YEAR.START_MONTH.START_DAY, 00:00:00) */
uint64_t convertToBaseMiliSecond(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, float sec);


/******************** Methods based on cordic methods ****************/

#define MULTIPLY_FP_RESOLUTION_BITS			(15)

#define IS_USE_CORDIC_Q28
#ifdef IS_USE_CORDIC_Q28
#define MATH_CORDIC_CALIBRATION_VALUE	(268435456)
#else
#define MATH_CORDIC_CALIBRATION_VALUE	(65536)
#endif

int64_t CordicAtan2_64(int64_t y, int64_t x);
int32_t CordicAtan2(int32_t y, int32_t x);
void CordicUnitVec(int32_t *cos, int32_t *sin, int32_t theta);
void CordicUnitVec64(int64_t *cos, int64_t *sin, int64_t theta);

/* fixed point atan2 */
int16_t atan2_fixPoint(int16_t y_fp, int16_t x_fp);

/* the cordic (high accurate) method to calculate atan2, the value is maximized by 10000 times  */
int32_t atan2_fixPoint_cordic(int16_t y_fp, int16_t x_fp);

int32_t mult_32(int32_t var1, int32_t var2, int32_t q);

/******************** Methods of FFT *********************************/

#define FFT_N			(256)
#define FFT_N_LOG2N   (8)

/*get the frequency domain information by using FFT_128*/
void rfft_power_i16(int32_t *target, int16_t fft_n,
	uint8_t start, uint8_t end, uint32_t *power);

#endif
