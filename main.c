#include "stdio.h"
#include <stdlib.h>
#include <string.h>

//#include "RowDetection.h"
#include "ReadCsv.h"
#include "alg_row_detection.h"

FILE* fp_src = NULL;

int main()
{
	FileData* fp_src;
	RealTimeData OnPoint = { {0,0,0},{0,0,0} };
	char* filename;

	//filename = "d:/rowing_pig&data/rowing/2019-04-09_12-15-46/sensor_gyr.csv"; //320´Î×óÓÒ
	//filename = "d:/rowing_pig&data/rowing/2019-04-09_12-15-58/sensor_gyr.csv"; //322
	//filename = "d:/rowing_pig&data/rowing/2019-04-09_12-33-21/sensor_gyr.csv"; //286
	//filename = "d:/rowing_pig&data/rowing/2019-04-09_12-33-38/sensor_gyr.csv"; //284
	//filename = "d:/rowing_pig&data/rowing/1058917799_2019-04-04_12-23-08/sensor_gyr.csv"; //239
	//filename = "d:/rowing_pig&data/rowing/1058917799_2019-04-04_12-39-41/sensor_gyr.csv"; //304
	//filename = "d:/rowing_pig&data/rowing/1058917799_2019-04-08_12-38-47/sensor_gyr.csv"; //297´Î ×óÊÖ fre = 31

	//filename = "d:/rowing_pig&data/rowing/qogir/2019-05-08_12-10-04/sensor_gyr.csv"; //218
	//filename = "d:/rowing_pig&data/rowing/qogir/2019-05-08_12-26-03/sensor_gyr.csv"; //197 --
	//filename = "d:/rowing_pig&data/rowing/qogir/2019-05-08_12-39-10/sensor_gyr.csv"; //239
	//filename = "d:/rowing_pig&data/rowing/qogir/2019-05-08_13-20-17/sensor_gyr.csv"; //310
	//filename = "d:/rowing_pig&data/rowing/qogir/2019-05-08_12-39-10/sensor_gyr.csv"; //251 --
	//filename = "d:/rowing_pig&data/rowing/qogir/2019-05-08_13-32-50/sensor_gyr.csv"; //231 --
	//filename = "d:/rowing_pig&data/rowing/qogir/2019-05-08_13-44-48/sensor_gyr.csv"; //226
	//filename = "d:/rowing_pig&data/rowing/qogir/2019-05-09_09-54-17/sensor_gyr.csv"; //238

	//filename = "d:/rowing_pig&data/rowing/everest/2014-08-21_22-08-36/sensor_gyr.csv"; //222
	//filename = "d:/rowing_pig&data/rowing/everest/2014-08-21_22-24-36/sensor_gyr.csv"; //204
	//filename = "d:/rowing_pig&data/rowing/everest/2014-08-21_22-37-42/sensor_gyr.csv"; //239
	//filename = "d:/rowing_pig&data/rowing/everest/2014-08-21_23-19-35/sensor_gyr.csv"; //310
	//filename = "d:/rowing_pig&data/rowing/everest/2014-08-21_23-31-37/sensor_gyr.csv"; //247 
	//filename = "d:/rowing_pig&data/rowing/everest/2014-08-21_23-43-23/sensor_gyr.csv"; //232
	//filename = "d:/rowing_pig&data/rowing/everest/2014-08-22_05-13-42/sensor_gyr.csv"; //255
	//filename = "d:/rowing_pig&data/rowing/everest/2014-08-22_19-52-50/sensor_gyr.csv"; //237

	//filename = "d:/rowing_pig&data/rowing/everest/2019-05-08_12-10-01/sensor_gyr.csv";
	//filename = "d:/rowing_pig&data/rowing/everest/2019-05-08_12-25-56/sensor_gyr.csv";
	//filename = "d:/rowing_pig&data/rowing/everest/2019-05-08_12-39-06/sensor_gyr.csv";
	//filename = "d:/rowing_pig&data/rowing/everest/2019-05-08_13-21-00/sensor_gyr.csv";
	//filename = "d:/rowing_pig&data/rowing/everest/2019-05-08_13-32-47/sensor_gyr.csv";
	//filename = "d:/rowing_pig&data/rowing/everest/2019-05-08_13-44-41/sensor_gyr.csv";
	//filename = "d:/rowing_pig&data/rowing/everest/2019-05-08_19-15-05/sensor_gyr.csv";
	//filename = "d:/rowing_pig&data/rowing/everest/2019-05-09_09-54-14/sensor_gyr.csv";

	filename = "C:/Users/¹ù Ä/Desktop/sensor_log/3/sensor_gyr.csv"; //20
	filename = "C:\\Users\\¹ù Ä\\Desktop\\sensor_log\\5\\sensor_gyr.csv"; //
	filename = "C:\\Users\\¹ù Ä\\Desktop\\sensor_log\\7\\sensor_gyr.csv"; //20

	extern int16_t sample_num;

	fp_src = readcsvfile(filename);
	if (fp_src == NULL)
		exit(0);

	bool handsetting = 1;// 1:lefthand, -1: righthand    
	row_initial(handsetting);

	int16_t acc[3];
	int16_t ang[3];

	for (int i = 0; i < fp_src->row; i++)
	{
		for (int j = 0; j < fp_src->colum; j++)
		{
			if (j <= 2)
			{
				acc[j] = fp_src->Data[i][j];
			}
			else
			{
				ang[j - 3] = fp_src->Data[i][j];
			}
		}
		rowing_receiveAccGyro(acc, ang);
	}

	RowResultStruct RowResult;
	getRowingResult(&RowResult);
	printf("\nResult:\n");
	printResult(RowResult);

	//int16_t A[6] = {1,2,3,4,5,6};
	////int16_t A[6] = { 1,1,1,1,1,1 };
	//float var = ComputeVar(A, 6);
	//printf("%f \n", var);

	free(fp_src->Data);
	free(fp_src);
	return 0;
}