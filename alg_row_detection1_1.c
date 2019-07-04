#include "alg_row_detection.h"

int32_t sample_num;			//Sample Number
int16_t Wave[WAVELEN];		//Wave Shape: For compute similarity
TempStack temp;				//Tempdata
RawDataStack RawData;		//On-time data stack
RowResultStruct RowResult;	//Result 
DataStack Acc;
DataStack Ang;

//init parameter
void row_initial(bool handside)
{
	//parameter:
	//handside:	hadside==true:left hand .else right hand

	sample_num = 0;

	temp.trend = -2;
	temp.IsolatedPointsNumber = 0;
	temp.Count1 = 0;
	temp.CycleLimit = 65;
	temp.SimiLimit = (float)(0.4);
	temp.hand = (handside == true) ? 1 : -1;

	temp.LastPeakLoc = 0;
	temp.PeakLoc = 0;
	temp.ValleyLoc = 0;
	temp.LastValleyLoc = 0;

	for (int16_t i = 0; i < 3; i++)
	{
		temp.CycleTime[i] = 0;
		temp.PaddleTime[i] = 0;
		temp.PeakValueStack[i] = 0;
		temp.PeaksLocStack[i] = 0;
	}

	//init Wave Shape data
	for (int16_t i = 0; i < WAVELEN; i++)
	{
		Wave[i] = 0;
	}

	//init data stack
	temp.windth = 80;
	RawData.DataStackNum = 0;
	RawData.Data = (RealTimeData*)malloc(sizeof(RealTimeData) * temp.windth);
	Acc.DataNum = 0;
	Acc.Data = (int16_t*)malloc(sizeof(int) * temp.windth);
	Ang.DataNum = 0;
	Ang.Data = (int16_t*)malloc(sizeof(int) * temp.windth);

	//init result
	RowResult.rowCounts = 0;
	RowResult.avgRowDuration = 0;
	RowResult.avgPaddleTime = 0;
	RowResult.avgRowCountPerMin = 0;
	RowResult.avgReturnPaddleTime = 0;
	RowResult.avgRowCountPerMinNow = 0;

	return;
}
void rowing_receiveAccGyro(int16_t* acc_buf, int16_t* gyro_buf)
{
	sample_num = sample_num + 1;

	// Update Real-Time Data
	RealTimeData OnPoint = { {acc_buf[0],acc_buf[1],acc_buf[2]},{gyro_buf[0],gyro_buf[1],gyro_buf[2]} };
	UpdateRawDataStack(OnPoint);  //Update raw Data
	UpdateMainDataStack(); //Update Data (Filter) 

	if (Acc.DataNum == 0) return;

	int32_t Num = Acc.DataNum - 1;
	float var = ComputeVar2(Acc.Data, Acc.DataNum); //Compute Var of Data
	if (var > 300000 || var < 8000)
	{
		temp.IsolatedPointsNumber = 0;
		temp.PeakLoc = 0;
		RowResult.avgRowCountPerMinNow = 0;
		UpdateTrend(Num);
		return;
	}

	if (temp.trend == -2)
	{
		RowResult.avgRowCountPerMinNow = 0;
	}		
	else if(temp.trend == -1) //Check if the point is a trough
	{
		if(checkValley(Num,sample_num))
			temp.ValleyLoc = sample_num - 1;
	}
	else if (temp.trend == 1 && Acc.Data[Num] < Acc.Data[Num - 1]) //Wave Peak
	{
		int16_t Peak = Acc.Data[Num - 1];
		int16_t PPdis = sample_num - 1 - temp.LastPeakLoc; //Peak_Peak distance
		float TempCycleTime = (float)(PPdis / 25.0);
		float TempPaddleTime;

		int16_t PeakThresold;	
		PeakThresold = 500;
		
		//PeakThresold = (int16_t)(MAX(GetArrayMean(temp.PeakValueStack, 3) * exp(-PPdis / 80.0)*0.8, 500)); //setting PeakThresold
		if (Peak < PeakThresold || temp.ValleyLoc == 0 || sample_num - 1 - temp.ValleyLoc < 8)
		{
			UpdateTrend(Num);
			return;
		}

		temp.PeakLoc = sample_num - 1;
		float ResampleData[WAVELEN];
		int16_t end = Num - 1;
		int16_t start = end - (temp.PeakLoc - temp.LastPeakLoc);

		UpdatePeakStack(temp.PeaksLocStack, temp.PeakLoc);
		UpdatePeakStack(temp.PeakValueStack, Peak);

		if ((PPdis >= 20 && PPdis <= temp.CycleLimit) || temp.LastPeakLoc == 0)
		{
			temp.IsolatedPointsNumber++;
			if (temp.IsolatedPointsNumber > 3)
			{
				UpdateTimeStack(temp.CycleTime, TempCycleTime);
				RowResult.avgRowDuration = RowResult.avgRowDuration * temp.Count1 / (temp.Count1 + 1) + temp.CycleTime[2] / (temp.Count1 + 1);
				
				int16_t Pnum = GetArrayMaxPLen(Acc.Data, start, end);				
				TempPaddleTime = (float)(Pnum / 25.0);
				UpdateTimeStack(temp.PaddleTime, TempPaddleTime);
				RowResult.avgPaddleTime = TempPaddleTime; //use the last temptime to compute paddletime

				SeqResample(&Acc.Data[start], end - start + 1, WAVELEN, ResampleData);
				for (int16_t i = 0; i < WAVELEN; i++)
				{
					Wave[i] = (int16_t)(Wave[i] * temp.Count1 / (temp.Count1 + 1) + ResampleData[i] / (temp.Count1 + 1));
				}

				temp.Count1++;
				RowResult.rowCounts++;
				RowResult.avgRowCountPerMin = (int16_t)(60 / RowResult.avgRowDuration + 0.5); //Final mean Freq
				RowResult.avgRowCountPerMinNow = (int16_t)(60 / TempCycleTime + 0.5); // now On-Time Freq
				RowResult.avgReturnPaddleTime = TempCycleTime - RowResult.avgPaddleTime; // The last return Paddle-Time

				printf("%4.2f ", sample_num / 25.0);
				printResult(RowResult);
			}
			else if (temp.IsolatedPointsNumber == 3)
			{
				UpdateTimeStack(temp.CycleTime, TempCycleTime);

				int16_t Pnum = GetArrayMaxPLen(Acc.Data, start, end);
				TempPaddleTime = (float)(Pnum / 25.0);
				UpdateTimeStack(temp.PaddleTime, TempPaddleTime);

				SeqResample(&Acc.Data[start], end - start + 1, WAVELEN, ResampleData);
				for (int16_t i = 0; i < WAVELEN; i++)
				{
					Wave[i] = (int16_t)(Wave[i] * temp.Count1 / (temp.Count1 + 1) + ResampleData[i] / (temp.Count1 + 1));
				}
				for (int16_t i = 0; i < 3; i++)
				{
					if (i >= 1)
					{
						RowResult.avgRowDuration = RowResult.avgRowDuration * temp.Count1 / (temp.Count1 + 1) + temp.CycleTime[i] / (temp.Count1 + 1);
						RowResult.avgPaddleTime = RowResult.avgPaddleTime * temp.Count1 / (temp.Count1 + 1) + temp.PaddleTime[i] / (temp.Count1 + 1);
						temp.Count1++;
					}
					RowResult.rowCounts++;
					RowResult.avgRowCountPerMin = (int16_t)(60 / RowResult.avgRowDuration + 0.5);
					RowResult.avgReturnPaddleTime = RowResult.avgRowDuration - RowResult.avgPaddleTime;
				}
				RowResult.avgRowCountPerMinNow = (int16_t)(60 / ((temp.CycleTime[1] + temp.CycleTime[2]) / 2) + 0.5);

				printf("%4.2f ", sample_num / 25.0);
				printResult(RowResult);
			}
			else if (temp.IsolatedPointsNumber == 2)
			{
				SeqResample(&Acc.Data[start], end - start + 1, WAVELEN, ResampleData);

				if (RowResult.rowCounts == 0)
				{
					for (int16_t i = 0; i < WAVELEN; i++)
					{
						Wave[i] = (int16_t)(ResampleData[i]);
					}
					UpdateTimeStack(temp.CycleTime, TempCycleTime);
				}
				else
				{
					float cor = ComputeSimilarity(ResampleData, WAVELEN, Wave);

					if (cor >= temp.SimiLimit)
					{
						UpdateTimeStack(temp.CycleTime, TempCycleTime);

						int16_t Pnum = GetArrayMaxPLen(Acc.Data, start, end);
						TempPaddleTime = (float)(Pnum / 25.0);
						UpdateTimeStack(temp.PaddleTime, TempPaddleTime);
						RowResult.avgPaddleTime = TempPaddleTime;

						for (int16_t i = 0; i < WAVELEN; i++)
						{
							Wave[i] = (int16_t)(Wave[i] * temp.Count1 / (temp.Count1 + 1) + ResampleData[i] / (temp.Count1 + 1));
						}
					}
					else
					{
						temp.IsolatedPointsNumber = 1;
					}
				}
			}
			else
			{
				UpdateTimeStack(temp.CycleTime, TempCycleTime);
			}
		}
		else if (PPdis > temp.CycleLimit)
		{
			temp.IsolatedPointsNumber = 1;
			RowResult.avgRowCountPerMinNow = 0;
		}

		temp.LastPeakLoc = temp.PeakLoc;
		temp.PeakLoc = 0;
		temp.ValleyLoc = 0;
	}
	UpdateTrend(Num);
	return;
}

//Update DataStack in every round
void UpdateMainDataStack()
{
	int16_t AccNum = 0;
	int16_t AngNum = 0;

	if (RawData.DataStackNum < 4)
	{
		return;
	}

	float weigh[9];

	for (int16_t i = 0; i < 5; i++)
	{
		int32_t w = 1;
		weigh[i] = (float)((w << i) / 46.0); //w:left<i
	}

	for (int16_t i = 5; i < 9; i++)
	{
		int32_t w = 1;
		weigh[i] = (float)((w << (8-i)) / 46.0);
	}

	if (4 <= RawData.DataStackNum && RawData.DataStackNum < 9)
	{
		AccNum = RawData.Data[RawData.DataStackNum - 1].acc[0];
		AngNum = RawData.Data[RawData.DataStackNum - 1].ang[1] * temp.hand;
	}
	else
	{
		for (int16_t i = 0; i < 9; i++)
		{
			AccNum += (int16_t)(RawData.Data[RawData.DataStackNum - (9 - i)].acc[0] * weigh[i]);
			AngNum += (int16_t)(RawData.Data[RawData.DataStackNum - (9 - i)].ang[1] * weigh[i]);
		}
	}

	if (Acc.DataNum < temp.windth)
	{
		Acc.Data[Acc.DataNum++] = AccNum;
		Ang.Data[Ang.DataNum++] = AngNum;
	}
	else if (Acc.DataNum == temp.windth)
	{
		for (int16_t j = 1; j < Acc.DataNum; j++)
		{
			Acc.Data[j - 1] = Acc.Data[j];
			Ang.Data[j - 1] = Ang.Data[j];
		}
		Acc.Data[Acc.DataNum - 1] = AccNum;
		Ang.Data[Ang.DataNum - 1] = AngNum;
	}
}
void UpdateRawDataStack(RealTimeData OnPoint)
{
	if (RawData.DataStackNum < temp.windth)
	{
		RawData.Data[RawData.DataStackNum] = OnPoint;
		RawData.DataStackNum++;
	}
	else if (RawData.DataStackNum == temp.windth)
	{
		for (int i = 1; i < RawData.DataStackNum; i++)
		{
			RawData.Data[i - 1] = RawData.Data[i];
		}
		RawData.Data[RawData.DataStackNum - 1] = OnPoint;
	}
	else
	{
		//		printf("A error ourred! num > temp.windth\n");
		//		exit(0);
	}
	return;
}
void UpdateTimeStack(float* Time, float Value)
{
	for (int16_t i = 1; i < 3; i++)
	{
		Time[i - 1] = Time[i];
	}
	Time[2] = Value;
	return;
}
void UpdatePeakStack(int16_t* Peak, int16_t Value)
{
	for (int16_t i = 1; i < 3; i++)
	{
		Peak[i - 1] = Peak[i];
	}
	Peak[2] = Value;
	return;
}
void UpdateTrend(int32_t Num)
{
	if (temp.trend==-2)
	{
		temp.trend = 0;
	}
	else
	{
		temp.trend = sign(Acc.Data[Num]-Acc.Data[Num-1]);
	}
	return;
}

//check
int8_t checkValley(int32_t Num, int32_t sample_num)
{
	if (Acc.Data[Num] > Acc.Data[Num - 1] && (temp.LastPeakLoc == 0 || (sample_num - 1 - temp.LastPeakLoc >= 8)) && Acc.Data[Num - 1] <= -500 && temp.PeakLoc == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
	
}

//Get & Print Result or RealTimeData
void getRowingResult(RowResultStruct* result) {
	*result = RowResult;
}
void printResult(RowResultStruct RowResult)
{
	printf("rowCounts = %d\t", RowResult.rowCounts);
	printf("avgRowDuration = %.2f\t", RowResult.avgRowDuration);
	printf("fre_mean = %d\t", RowResult.avgRowCountPerMin);
	printf("fre_now = %d\t", RowResult.avgRowCountPerMinNow);
	printf("avgPaddleTime= %.2f\t", RowResult.avgPaddleTime);
	printf("avgReturnPaddleTime = %.2f\t", RowResult.avgReturnPaddleTime);
	printf("\n");
}
void PrintRealTimeData(RealTimeData OnPoint)
{
	for (int i = 0; i < 3; i++)
	{
		printf("%d ", OnPoint.acc[i]);
	}
	for (int i = 0; i < 3; i++)
	{
		printf("%d ", OnPoint.ang[i]);
	}
	printf(" \n");
}

//Operation On Array
int8_t sign(int16_t a)
{
	if (a == 0)
	{
		return 0;
	}
	else if (a < 0)
	{
		return -1;
	}
	else if (a > 0)
	{
		return 1;
	}
	return 0;
}
float ComputeVar(int16_t* Array, int16_t length)
{
	if (length <= 0)
		return 0;

	float mean = 0;
	float var;
	float sum = 0;

	for (int i = 0; i < length; i++)
	{
		mean += Array[i];
		sum += Array[i] * Array[i];
	}

	mean /= length;
	sum /= length;
	var = sum - mean * mean;

	var /= length - 1;
	return var;
}
float ComputeVar2(int16_t* Array, int16_t length)
{
	if (length <= 0)
		return 0;

	float mean = 0;
	float var;
	float sum = 0;

	for (int i = 0; i < length; i++)
	{
		mean = Array[i];
	}
	mean/= length;
	for (int i = 0; i < length; i++)
	{
		sum += (Array[i]-mean)* (Array[i] - mean);
	}
	var = sum/length;
	var /= length;
	return var;
}
int16_t GetArrayMaxPLen(int16_t* Array, int16_t start, int16_t end) //Count the number of positive values between two peaks
{ 
	int16_t Pnum = 0;
	int16_t temp = 0;
	for (int16_t i = start; i <= end; i++)
	{
		if (Array[i] > 0)
		{
			temp++;
		}
		else
		{
			if (temp > Pnum)
			{
				Pnum = temp;
			}
			temp = 0;
		}
	}
	return Pnum;
}
int16_t GetArrayMax(int16_t* Array, int16_t Num) //Compute the max element of a Array
{
	if (Num <= 0)
	{
		printf("A error has ocured: The array Nedded to be computed Num <= 0\n");
		return -9999;
	}
	int16_t Maxelement = -1000;
	for (int i = 0; i < Num; i++)
	{
		if (Array[i] > Maxelement)
		{
			Maxelement = Array[i];
		}
	}
	return Maxelement;
}
int16_t GetArrayMean(int16_t* Array, int16_t len) //Compute Mean of a Float Array
{
	int16_t mean = 0;
	for (int16_t i = 0; i < len; i++)
	{
		mean += Array[i];
	}
	mean /= len;
	return mean;
}
float GetFloatArrayMean(float* Array, int16_t len) //Compute the Mean of a Float Array
{
	float mean = 0;
	for (int16_t i = 0; i < len; i++)
	{
		mean += Array[i];
	}
	mean /= len;
	return mean;
}
int16_t GetArrayMaxPosNumLen(int16_t* data, int16_t len)  //Compute Max Number of The length of Positive numbers  of a array
{
	int16_t pmax = 0;
	int temp = 0;
	for (int16_t i = 0; i < len; i++)
	{
		if (data[i] > 0)
		{
			temp++;
		}
		else
		{
			if (temp > pmax)
			{
				pmax = temp;
				temp = 0;
			}
		}
	}

	if (temp > pmax)
	{
		pmax = temp;
		temp = 0;
	}
	return pmax;
}

//Compute Similarity
float ComputeSimilarity(float* Array, int Num1, int16_t* Wave)
{
	float mean1 = 0;
	float mean2 = 0;

	mean1 = GetFloatArrayMean(Array, Num1);
	mean2 = GetArrayMean(Wave, WAVELEN);

	float d1, d2, d3;
	d1 = d2 = d3 = 0;
	for (int i = 0; i < WAVELEN; i++)
	{
		d1 += (Array[i] - mean1) * (Wave[i] - mean2);
		d2 += (Array[i] - mean1) * (Array[i] - mean1);
		d3 += (Wave[i] - mean2) * (Wave[i] - mean2);
	}
	float corr_coe = d1 / (float)(sqrt(d2) * sqrt(d3));
	return corr_coe;
};
void SeqResample(int16_t* data, int n, int des_len, float* rseq)
{
	int i;
	float step;
	float x[MAX_LEN];
	float px[MAX_RS_LEN];

	//x = (float*)malloc(n * sizeof(float));

	for (i = 0; i < n; i++)
		x[i] = (float)i;

	//temp = (len - 1) / (des_len - 1);
	step = (float)(n - 1) / (des_len - 1);
	//xi = 1:temp : len;
	px[0] = x[0];
	for (int16_t i = 1; i < des_len; i++)
		px[i] = px[i - 1] + step;
	//out = interp1(x, data, xi, 'spline');
	MultiPointInterp(x, data, n, px, des_len, rseq);
}
float inter_linear(float x0, float x1, float y0, float y1, float x)
{
	float a0, a1, y;
	a0 = (x - x1) / (x0 - x1);
	a1 = (x - x0) / (x1 - x0);
	y = a0 * y0 + a1 * y1;

	return y;
}
float interp1(float x[], int16_t y[], int n, float px)
{
	float z;
	int i, tmp;

	if (px < x[0])
		z = inter_linear(x[0], x[1], y[0], y[1], px);
	else if (px > x[n - 1])
		z = inter_linear(x[n - 1], x[n], y[n - 1], y[n], px);
	else
	{
		for (i = 0; i < n - 1; i++)
		{
			if ((px >= x[i]) && (px < x[i + 1]))
			{
				tmp = i;
				break;
			}
		}
		if (i == n - 1)
		{
			z = y[i];
			return z;
		}
		z = inter_linear(x[tmp], x[tmp + 1], y[tmp], y[tmp + 1], px);
	}
	return z;
}
void MultiPointInterp(float x[], int16_t y[], int n, float px[], int m, float py[])
{
	int i;
	for (i = 0; i < m; i++)
	{
		py[i] = interp1(x, y, n, px[i]);
	}
}