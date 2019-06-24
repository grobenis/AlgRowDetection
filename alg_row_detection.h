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

//结果
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
	int32_t mCount;	//划次
	int32_t mTime;//用时
	float mCalories;
	int32_t mAvgHeartRate;
	float mAvgCadence;
}RowGroupInfo_t;

//实时数据
typedef struct RealTimeData
{
	int16_t acc[3];
	int16_t ang[3];
}RealTimeData;

//临时使用的数据栈  保存最近到来的一定数目的点
typedef struct RawDataStack
{
	RealTimeData* Data;
	int16_t DataStackNum;
}RawDataStack;

//从原始数据中滤波后提取到的信息
typedef struct DataStack
{
	int16_t* Data;
	int16_t DataNum;
}DataStack;

//中间保存的变量
typedef struct TempStack
{
	int8_t trend; //趋势
	int32_t	IsolatedPointsNumber; //孤立点个数
	int32_t Count1;
	int32_t CycleLimit;
	float SimiLimit; //波形相似性下限
	int8_t hand;

	int32_t PeakLoc;
	int32_t LastPeakLoc;
	int32_t ValleyLoc;
	int32_t LastValleyLoc;

	float CycleTime[3]; //用于计算拉桨时间
	float PaddleTime[3]; //用于计算平均时间
	int16_t PeakValueStack[3];
	int16_t PeaksLocStack[3];

	int16_t Wave[WAVELEN];

	int16_t windth;
}TempStack;

void row_initial(bool handside); //初始化参数
void rowing_receiveAccGyro(int16_t* acc_buf, int16_t* gyro_buf); //实时检测划次

//每个轮次更新数据
void UpdateRawDataStack(RealTimeData OnPoint); //根据输入的点 对数据进行更新
void UpdateMainDataStack();
void UpdateTimeStack(float* CycleTime, float Value);
void UpdatePeakStack(int16_t* Peak, int16_t Value);

//获取结果 打印数据
void getRowingResult(RowResultStruct* result);	//实时获取划次数据
void printResult(RowResultStruct Result); //打印结果数据
void PrintRealTimeData(RealTimeData OnPoint);

//对数组的操作
int8_t sign(int16_t a);//判断数值正负号
float ComputeVar(int16_t* Array, int16_t length); //计算一个数组的方差
float ComputeVar2(int16_t* Array, int16_t length); //计算一个数组的方差
int16_t GetArrayMaxPLen(int16_t* Array, int16_t start, int16_t end); //统计两个波峰之间最长的正值
int16_t GetArrayMax(int16_t* Array, int16_t Num); //求数组最大值
int16_t GetArrayMean(int16_t* Array, int16_t len); //求数组的均值
float GetFloatArrayMean(float* Array, int16_t len); //求浮点型数组的均值
int16_t GetArrayMaxPosNumLen(int16_t* Array, int16_t len);

//计算相似性
float ComputeSimilarity(float* Array, int Num1, int16_t* Wave); //计算两个划次之间的相似性
void SeqResample(int16_t* data, int n, int des_len, float* rseq);
float inter_linear(float x0, float x1, float y0, float y1, float x);
float interp1(float x[], int16_t y[], int n, float px);
void MultiPointInterp(float x[], int16_t y[], int n, float px[], int m, float py[]);
#endif // ROWDETECTION_H
