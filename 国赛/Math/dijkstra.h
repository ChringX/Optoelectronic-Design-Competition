#ifndef _DIJKSTRA_H_
#define _DIJKSTRA_H_

#include "main.h"

#define numvexs 97

typedef struct
{
	int matrix[numvexs][numvexs];//邻接矩阵
	int numVexs;       //顶点数
	int father[200];    //父节点
	int path[200];      //最短路径 
	int route[200];
}Graph;

extern Graph PhEleTechnology_Test;
extern Graph PhEleTechnology;

void CreatGraph(Graph *G);
int dijk(Graph *G,int head,int target);

#endif
