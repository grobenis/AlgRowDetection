#ifndef ALG_ROW_DETECTION_H
#define ALG_ROW_DETECTION_H

#define MAX_LEN 100
#define MAX_RS_LEN 100
#define WAVELEN 50

#include    <stdio.h>
#include    <stdlib.h>
#include    <stdint.h>
#include	<stdbool.h>
#include    "alg_math.h"


//RESULT
typedef struct ResultStruct
{
	int32_t rowCounts;	//划次
	float avgRowDuration; //平均时间
	int16_t avgRowCountPerMin; //分均划频 :: 最后展示
	int16_t avgRowCountPerMinNow; //实时划频 :: 当前展示
	float avgPaddleTime;	//拉桨时间
	float avgReturnPaddleTime;  //回桨时间
}RowResultStruct;
typedef struct RowGroupInfo_t
{
	int32_t mCount;	//Row Number
	int32_t mTime; //Time
	float mCalories;
	int32_t mAvgHeartRate;
	float mAvgCadence;
}RowGroupInfo_t;

//Real-Time Data:Used as the input of program
typedef struct RealTimeData
{
	int16_t acc[3];
	int16_t ang[3];
}RealTimeData;

//Data Stack :Save a certain number of points
typedef struct RawDataStack
{
	RealTimeData* Data;
	int16_t DataStackNum;
}RawDataStack;
typedef struct DataStack
{
	int16_t* Data;
	int16_t DataNum;
}DataStack; //Information extracted from the original data after filtering

//Intermediate variables
typedef struct TempStack
{
	int8_t trend; //Point`s G
	int32_t	IsolatedPointsNumber; //Number of isolated points
	int32_t Count1;
	int32_t CycleLimit;
	float SimiLimit; //The lower limit of Waveform similarity 
	int8_t hand;

	int32_t PeakLoc;
	int32_t LastPeakLoc;
	int32_t ValleyLoc;
	int32_t LastValleyLoc;

	float CycleTime[3]; //Average cycle time
	float PaddleTime[3]; //Used to calculate the average Paddle time
	int16_t PeakValueStack[3];
	int16_t PeaksLocStack[3];

	int16_t Wave[WAVELEN];

	int16_t windth;
}TempStack;

void row_initial(bool handside); //Initialization parameters
void rowing_receiveAccGyro(int16_t* acc_buf, int16_t* gyro_buf); //Real-time detection

//Update Data in every round.
void UpdateDataStack(RealTimeData OnPoint);
void UpdateMainDataStack();
void UpdateTimeStack(float* CycleTime, float Value);
void UpdatePeakStack(int16_t* Peak, int16_t Value);
void UpdateTrend(int32_t Num); //update the trend of wave in every round

int8_t checkValley(int32_t Num, int32_t sample_num); //check if the current point is a valley\trough

//Get & Print Result or RealTimeData
void getRowingResult(RowResultStruct* result);	//Get the data in real time
void printResult(RowResultStruct Result); //Print result data
void PrintRealTimeData(RealTimeData OnPoint); //Print the data in real time

//Operation On Array
int8_t sign(int16_t a);//Judging the sign of the value
float ComputeVar(int16_t* Array, int16_t length); //Calculate the variance of an array
int16_t GetArrayMax(int16_t* Array, int16_t Num); //Compute the Max Element of a Array
int16_t GetArrayMean(int16_t* Array, int16_t len); //Compute Mean of a Array
float GetFloatArrayMean(float* Array, int16_t len); //Compute the Mean of a Float Array
int16_t GetArrayMaxPosNumLen(int16_t* Array, int16_t start, int16_t end); //Count the number of positive values between two peaks

//Compute Similarity
float ComputeSimilarity(float* Array, int Num1, int16_t* Wave); //Compute the similarity of two signal
void SeqResample(int16_t* data, int n, int des_len, float* rseq);
float inter_linear(float x0, float x1, float y0, float y1, float x);
float interp1(float x[], int16_t y[], int n, float px);
void MultiPointInterp(float x[], int16_t y[], int n, float px[], int m, float py[]);
#endif // ROWDETECTION_H
