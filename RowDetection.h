#ifndef ROWDETECTION_H
#define ROWDETECTION_H
#ifndef __int8_t_defined
# define __int8_t_defined
typedef signed char             int8_t;
typedef short int               int16_t;
typedef int                     int32_t;
typedef long long int           int64_t;
#endif
#define WAVELEN 50

#include    <stdio.h>
#include    <stdlib.h>
#include    <math.h>

//结果
typedef struct ResultStruct
{
	int rowCounts;	//划次
	float avgRowDuration; //平均时间
	int16_t avgRowCountPerMin; //分均划频
	float avgPaddleTime;	//拉桨时间
	float avgReturnPaddleTime;  //回桨时间
}ResultStruct;
//实时数据
typedef struct RealTimeData
{
	int16_t acc[3];
	int16_t ang[3];
}RealTimeData;
//临时使用的数据栈  保存最近到来的一定长度的点
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
	char trend; //趋势
	int	IsolatedPointsNumber; //孤立点个数
	int Count1;
	int CycleLimit;
	char hand;

	int PeakLoc;
	int LastPeakLoc;
	int ValleyLoc;
	int LastValleyLoc;

	float CycleTime[3]; //用于计算拉桨时间
	float PaddleTime[3]; //用于计算平均时间
	int16_t PeakValueStack[3];
	int16_t PeaksLocStack[3];

	int16_t Wave[WAVELEN];

	int16_t windth;
}TempStack;

void initRowAlgoParameters(char handside); //初始化参数
char rowing_receiveAccGyro(int16_t* acc_buf, int16_t* gyro_buf); //实时检测划次

//每个轮次更新数据
void UpdateRawDataStack(RealTimeData OnPoint); //根据输入的点 对数据进行更新
void UpdateMainDataStack();
void UpdateTimeStack(float* CycleTime, float Value);
void UpdatePeakStack(int16_t* Peak, int16_t Value);

//获取结果 打印数据
void getRowingResult(ResultStruct* result);	//实时获取划次数据
void printResult(ResultStruct Result); //打印结果数据
void PrintRealTimeData(RealTimeData OnPoint);

//对数组的操作
char sign(int16_t a);//判断数值正负号
float ComputeVar(int16_t* Array, int16_t length); //计算一个数组的方差
int16_t GetArrayMaxPLen(int16_t* Array, int16_t start, int16_t end); //统计两个波峰之间最长的正值
int16_t GetArrayMax(int16_t* Array, int16_t Num); //求数组最大值
int16_t GetArrayMean(int16_t* Array, int16_t len); //求数组的均值
int16_t GetArrayMaxPosNumLen(int16_t* Array, int16_t len);

//计算相似性
float ComputeSimilarity(int16_t* Array, int Num1, int16_t* Wave); //计算两个划次之间的相似性
void SeqResample(int16_t* data, int n, int des_len, int16_t* rseq);
float inter_linear(float x0, float x1, float y0, float y1, float x);
float interp1(float x[], float y[], int n, float px);
void MultiPointInterp(float x[], float y[], int n, float px[], int m, float py[]);
#endif // ROWDETECTION_H
