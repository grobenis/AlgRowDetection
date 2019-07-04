#ifndef READCSV_H
//如果一个文件包含了此头文件多次，使用这种方法，即在第一次编译时没有定义ABC的宏，执行了下面的所有，第二次再遇到编译此文件时ABC已经被定义，就不会再编译
// 使用条件编译可以避免重复编译，
// 粗要注意的是要使用#ifndef语句
// 不同头文件的宏名称必须不同
// 头文件产用的宏名称应该是根据头文件的路径得来的

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