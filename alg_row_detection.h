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

//���
typedef struct ResultStruct
{
	int32_t rowCounts;	//����
	float avgRowDuration; //ƽ��ʱ��
	int16_t avgRowCountPerMin; //�־���Ƶ :: ���չʾ
	int16_t avgRowCountPerMinNow; //ʵʱ��Ƶ :: ��ǰչʾ
	float avgPaddleTime;	//����ʱ��
	float avgReturnPaddleTime;  //�ؽ�ʱ��

}RowResultStruct;

typedef struct RowGroupInfo_t
{
	int32_t mCount;	//����
	int32_t mTime;//��ʱ
	float mCalories;
	int32_t mAvgHeartRate;
	float mAvgCadence;
}RowGroupInfo_t;

//ʵʱ����
typedef struct RealTimeData
{
	int16_t acc[3];
	int16_t ang[3];
}RealTimeData;

//��ʱʹ�õ�����ջ  �������������һ����Ŀ�ĵ�
typedef struct RawDataStack
{
	RealTimeData* Data;
	int16_t DataStackNum;
}RawDataStack;

//��ԭʼ�������˲�����ȡ������Ϣ
typedef struct DataStack
{
	int16_t* Data;
	int16_t DataNum;
}DataStack;

//�м䱣��ı���
typedef struct TempStack
{
	int8_t trend; //����
	int32_t	IsolatedPointsNumber; //���������
	int32_t Count1;
	int32_t CycleLimit;
	float SimiLimit; //��������������
	int8_t hand;

	int32_t PeakLoc;
	int32_t LastPeakLoc;
	int32_t ValleyLoc;
	int32_t LastValleyLoc;

	float CycleTime[3]; //���ڼ�������ʱ��
	float PaddleTime[3]; //���ڼ���ƽ��ʱ��
	int16_t PeakValueStack[3];
	int16_t PeaksLocStack[3];

	int16_t Wave[WAVELEN];

	int16_t windth;
}TempStack;

void row_initial(bool handside); //��ʼ������
void rowing_receiveAccGyro(int16_t* acc_buf, int16_t* gyro_buf); //ʵʱ��⻮��

//ÿ���ִθ�������
void UpdateRawDataStack(RealTimeData OnPoint); //��������ĵ� �����ݽ��и���
void UpdateMainDataStack();
void UpdateTimeStack(float* CycleTime, float Value);
void UpdatePeakStack(int16_t* Peak, int16_t Value);

//��ȡ��� ��ӡ����
void getRowingResult(RowResultStruct* result);	//ʵʱ��ȡ��������
void printResult(RowResultStruct Result); //��ӡ�������
void PrintRealTimeData(RealTimeData OnPoint);

//������Ĳ���
int8_t sign(int16_t a);//�ж���ֵ������
float ComputeVar(int16_t* Array, int16_t length); //����һ������ķ���
float ComputeVar2(int16_t* Array, int16_t length); //����һ������ķ���
int16_t GetArrayMaxPLen(int16_t* Array, int16_t start, int16_t end); //ͳ����������֮�������ֵ
int16_t GetArrayMax(int16_t* Array, int16_t Num); //���������ֵ
int16_t GetArrayMean(int16_t* Array, int16_t len); //������ľ�ֵ
float GetFloatArrayMean(float* Array, int16_t len); //�󸡵�������ľ�ֵ
int16_t GetArrayMaxPosNumLen(int16_t* Array, int16_t len);

//����������
float ComputeSimilarity(float* Array, int Num1, int16_t* Wave); //������������֮���������
void SeqResample(int16_t* data, int n, int des_len, float* rseq);
float inter_linear(float x0, float x1, float y0, float y1, float x);
float interp1(float x[], int16_t y[], int n, float px);
void MultiPointInterp(float x[], int16_t y[], int n, float px[], int m, float py[]);
#endif // ROWDETECTION_H
