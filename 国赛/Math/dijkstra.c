#include "dijkstra.h"
#include "map.h"
#include "map_message.h"
#include "usmart.h"
#include "main.h"
#include "stdio.h"

#define inf 65535

Graph PhEleTechnology;
Graph PhEleTechnology_Test;


void CreatGraph(Graph *G)
{
	G->numVexs = numvexs;
	
	//初始化
	for(int i = 0;i < G->numVexs;i++)
	{
		G->path[i] =  0;
		for(int j = 0;j < G->numVexs;j++)
		{
			G->matrix[i][j] = inf;
		}
	}
	
	for(int i = 0;i < G->numVexs;i++)
	{
		for(int j = 0;j < ConnectionNum[i];j++)
		{
			G->matrix[i][Node[j+Address[i]].nodenum] = Node[j+Address[i]].step;
		}
	}
}


//head为源结点，target为目标节点
int	dijk(Graph *G,int head,int target)
{
	int first = 0;
	int num = 0;
	int dis = 0;
	int distance[numvexs]; //head到各点的距离
	int visited[numvexs];  //1为已访问该节点，0为未访问该节点
	
	 //初始化数据
	for(int i = 0;i < G->numVexs;i++)
	{
		visited[i] = 0;
		distance[i] = G->matrix[head][i]; //将与源节点有连线的顶点加上权值
		if(distance[i] != inf  && distance[i] != 1000)
		{
			num++;
		}
	}
	
	visited[head] = 1;
	
	//构建一个循环，在未选择列表中，选出一个离源顶点最近的顶点
	
	for(int a = 0;a < G->numVexs-1;a++)
	{	
		int min = 0;
		int path_Min = inf;
		for(int j = 0;j < G->numVexs;j++)
		{
			if(visited[j] == 0 && distance[j] <= path_Min)
			{ 
          		min = j;
          		path_Min = distance[j];
        	}
		}
		
		visited[min]=1; //最小距离的被选出
		
		if(first < num)
		{
			G->father[min] = head;
			first++;
		}
		
		for(int k = 0;k<G->numVexs;k++)
		{ //根据被选出的顶点更新distance数组
	        if(visited[k] == 0 && distance[k] > distance[min]+G->matrix[min][k])
			{ //因为目前的最小生成树发生变化，以新选出的结点来更新到其他还没有加入到最短生成树的节点的最短路径
	            distance[k]=distance[min]+G->matrix[min][k];
	            G->father[k] = min;
	        }
    	}
	}
	
	dis = distance[target];
	
	int Transition[200] = {0};
    Transition[0] = target;
    int f,g;
    for(f = 1;target != head;f++)
    {
    	Transition[f] = G->father[target];
    	target = G->father[target];
	}
	int p = 0;
	for(g = f-1;g >= 0;g--)
	{
		G->route[p++] = Transition[g];
	}
	G->route[p] = 0XFF;
	
//	for(g = 0;g < f;g++)
//	{
//		printf("%d ",G->route[g]);
//	}
//	printf("\r\n");
	return dis;
}
