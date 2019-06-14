#include "ReadCsv.h"
#include <stdlib.h>
#include <string.h>

FileData* readcsvfile(char* filename)
{
	FileData* fd = NULL;
	FILE* fp = NULL;

	char* line, * record;
	char buffer[1024];

	int Err;
	Err = fopen_s(&fp, filename, "r");
	printf("open %s\n", filename);
	if ( Err == 0 )
	{
		printf("open file successfully!\n");
		fd = (FileData*)malloc(sizeof(FileData));
		fd->colum = 6;

		fd->row = GetFileRowCount(filename);
		printf("%d ", fd->row);
		fd->Data = (int **)malloc(fd->row *sizeof(int*));
		
		for (int i = 0; i < fd->row; i++)
		{
			fd->Data[i] = (int*)malloc(6 * sizeof(int));
		}

		char delims[] = ",";
		char* result = NULL;
		char* next_token = NULL;
		int num;
		int i = 0, j;
		while ((line = fgets(buffer, sizeof(buffer), fp)) != NULL)//当没有读取到文件末尾时循环继续
		{
			j = 0;
			record = strtok_s(line, delims , &next_token);

			while (record != NULL)//读取每一行的数据
			{
				num = (int)atoi(record);
				fd->Data[i][j] = num;
				j++;
				record = strtok_s(NULL, delims , &next_token);
			}
			i++;
		}
		printf("csvfile has Readed\n");
		fclose(fp);
		fp = NULL;
	}
	else
	{
		printf("open failed\n");
		return NULL;
	}
	return fd;
}

int GetFileRowCount(char* filename)
{
	FILE* fp = NULL;
	char* line;
	int Err;
	int row = 0;
	char buffer[1024];

	Err = fopen_s(&fp, filename, "r");
	if (Err == 0)
	{
		printf("open file successfully!\n");
		if (fp)
		{
			while ((line = fgets(buffer, sizeof(buffer), fp)) != NULL)//当没有读取到文件末尾时循环继续
			{
				row++;
			}
		}
	}
	else
	{
		printf("open failed\n");
		exit(0);
	}
	return row;
}