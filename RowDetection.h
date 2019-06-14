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

//���
typedef struct ResultStruct
{
	int rowCounts;	//����
	float avgRowDuration; //ƽ��ʱ��
	int16_t avgRowCountPerMin; //�־���Ƶ
	float avgPaddleTime;	//����ʱ��
	float avgReturnPaddleTime;  //�ؽ�ʱ��
}ResultStruct;
//ʵʱ����
typedef struct RealTimeData
{
	int16_t acc[3];
	int16_t ang[3];
}RealTimeData;
//��ʱʹ�õ�����ջ  �������������һ�����ȵĵ�
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
	char trend; //����
	int	IsolatedPointsNumber; //���������
	int Count1;
	int CycleLimit;
	char hand;

	int PeakLoc;
	int LastPeakLoc;
	int ValleyLoc;
	int LastValleyLoc;

	float CycleTime[3]; //���ڼ�������ʱ��
	float PaddleTime[3]; //���ڼ���ƽ��ʱ��
	int16_t PeakValueStack[3];
	int16_t PeaksLocStack[3];

	int16_t Wave[WAVELEN];

	int16_t windth;
}TempStack;

void initRowAlgoParameters(char handside); //��ʼ������
char rowing_receiveAccGyro(int16_t* acc_buf, int16_t* gyro_buf); //ʵʱ��⻮��

//ÿ���ִθ�������
void UpdateRawDataStack(RealTimeData OnPoint); //��������ĵ� �����ݽ��и���
void UpdateMainDataStack();
void UpdateTimeStack(float* CycleTime, float Value);
void UpdatePeakStack(int16_t* Peak, int16_t Value);

//��ȡ��� ��ӡ����
void getRowingResult(ResultStruct* result);	//ʵʱ��ȡ��������
void printResult(ResultStruct Result); //��ӡ�������
void PrintRealTimeData(RealTimeData OnPoint);

//������Ĳ���
char sign(int16_t a);//�ж���ֵ������
float ComputeVar(int16_t* Array, int16_t length); //����һ������ķ���
int16_t GetArrayMaxPLen(int16_t* Array, int16_t start, int16_t end); //ͳ����������֮�������ֵ
int16_t GetArrayMax(int16_t* Array, int16_t Num); //���������ֵ
int16_t GetArrayMean(int16_t* Array, int16_t len); //������ľ�ֵ
int16_t GetArrayMaxPosNumLen(int16_t* Array, int16_t len);

//����������
float ComputeSimilarity(int16_t* Array, int Num1, int16_t* Wave); //������������֮���������
void SeqResample(int16_t* data, int n, int des_len, int16_t* rseq);
float inter_linear(float x0, float x1, float y0, float y1, float x);
float interp1(float x[], float y[], int n, float px);
void MultiPointInterp(float x[], float y[], int n, float px[], int m, float py[]);
#endif // ROWDETECTION_H
