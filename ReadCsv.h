#ifndef READCSV_H
//���һ���ļ������˴�ͷ�ļ���Σ�ʹ�����ַ��������ڵ�һ�α���ʱû�ж���ABC�ĺִ꣬������������У��ڶ���������������ļ�ʱABC�Ѿ������壬�Ͳ����ٱ���
// ʹ������������Ա����ظ����룬
// ��Ҫע�����Ҫʹ��#ifndef���
// ��ͬͷ�ļ��ĺ����Ʊ��벻ͬ
// ͷ�ļ����õĺ�����Ӧ���Ǹ���ͷ�ļ���·��������

#define READCSV_H
#include <stdio.h>
#include <stdlib.h>

typedef struct FileData {
	int **Data;
	int row;
	int colum;
}FileData;

FileData* readcsvfile(char* filename);
int GetFileRowCount(char* filename);

#endif  //READCSV_H