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

	//filename = "D:/Rowing_Pig&Data/rowing/2019-04-09_12-15-46/sensor_gyr.csv";//300´Î×óÓÒ
	//filename = "D:/Rowing_Pig&Data/rowing/2019-04-09_12-15-58/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/2019-04-09_12-33-21/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/2019-04-09_12-33-38/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/1058917799_2019-04-04_12-23-08/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/1058917799_2019-04-04_12-39-41/sensor_gyr.csv";
	filename = "D:/Rowing_Pig&Data/rowing/1058917799_2019-04-08_12-38-47/sensor_gyr.csv"; //297´Î ×óÊÖ fre = 31

	//filename = "D:/Rowing_Pig&Data/rowing/qogir/2019-05-08_12-10-04/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/qogir/2019-05-08_12-26-03/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/qogir/2019-05-08_12-39-10/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/qogir/2019-05-08_13-20-17/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/qogir/2019-05-08_12-39-10/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/qogir/2019-05-08_13-32-50/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/qogir/2019-05-08_13-44-48/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/qogir/2019-05-09_09-54-17/sensor_gyr.csv";

	//filename = "D:/Rowing_Pig&Data/rowing/everest/2014-08-21_22-08-36/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2014-08-21_22-24-36/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2014-08-21_22-37-42/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2014-08-21_23-19-35/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2014-08-21_23-31-37/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2014-08-21_23-43-23/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2014-08-22_05-13-42/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2014-08-22_19-52-50/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2019-05-08_12-10-01/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2019-05-08_12-25-56/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest//2019-05-08_12-39-06/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2019-05-08_13-21-00/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2019-05-08_13-32-47/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2019-05-08_13-44-41/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2019-05-08_19-15-05/sensor_gyr.csv";
	//filename = "D:/Rowing_Pig&Data/rowing/everest/2019-05-09_09-54-14/sensor_gyr.csv";

	extern int16_t sample_num;

	fp_src = readcsvfile(filename);
	if (fp_src == NULL)
		exit(0);

	char handsetting = 1;// 1:lefthand, -1: righthand    
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
	//printf("%d\n", sample_num);	
	RowResultStruct RowResult;
	getRowingResult(&RowResult);
	printResult(RowResult);

	free(fp_src->Data);
	free(fp_src);
	return 0;
}