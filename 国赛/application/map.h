#ifndef __MAP_H
#define __MAP_H

#include "map_message.h"

#define NO      	(1<<0) 
#define DLEFT 		(1<<1)				//左边横线
#define DRIGHT 		(1<<2)				//右边横线   右半边天
#define RESTMPUZ	(1<<9)			    //陀螺仪校准
#define STOPTURN 	(1<<10)				//停下来转弯
#define SLOWDOWN	(1<<11)    			//减速
#define Corner      (1<<12)             //拐角宝藏点
#define In_Road     (1<<13)             //路中间宝藏点

enum barriers {
	Treasure = 2,
	Platform,
};

enum MapNode {
	A1,A2,L1,A3,A4,A5,L2,A6,A7,P1,          		//9
	L3,B1,A8,A9,A10,A11,A61,L4,A12,A13,     		//19
	B3,A14,A15,A16,B4,L5,A17,B5,A18,A19, 		//29
	A20,A21,L6,A22,L7,B6,A23,A24,A25,B7,   		 //39
	L8,B8,A26,A27,A28,A29,L9,A30,           		//47
	A31,L10,A32,A33,A34,A35,B9,L11,          		 //55
	B10,A36,A37,A38,B11,L12,A39,L13,A40,A41, 	   //65
	A42,A43,B12,A44,L14,B13,A45,A46,A47,B14,  //75
	A48,A49,L15,A62,A50,A51,A52,A53,B16,L16,      //85
	P2,A54,A55,L17,A56,A57,A58,L18,A59,A60,        //95
	A63
};

/**************************************/
//结点信息
//flag 0位寻线方式：0左寻线，1右寻线
//flag 123位到达路口标志：	000最左边打到，001最右边打到，010左边数线，011右边数线，100线数由多变成一条	
//flag 45位，数线数目	
//flag 6位，寻线方式是否要切换，1需要切换，0不需要切换
//flag 7位	需要陀螺仪校正
//flag 8~11	
typedef struct _node{
	u8 nodenum;     //结点名称
	u32  flag;	    //结点标志位
	float angle;	//角度	
	u16	step;		//线长
	float speed;	//寻线速度
	u8 function;    //结点函数
}NODE;

extern NODE Node[205];
/*************************/
//flag 0位：1编码器清零请求，0清零完毕
//flag 1位：启动路口判断
//flag 2位：是否到达路口
//flag 3位：arrive里temp清零
//flag 4位：Z轴置零
//flag 5位：路线处理复位 打到门
//flag 6位：没有门
//flag 7位：红灯
typedef struct _nodesr{
	u8 flag;
	NODE nowNode;	//当前结点
	NODE nextNode;	//下一结点
	NODE lastNode;  //上一个结点
}NODESR;

extern NODESR nodesr;

struct Map_State {
	u16 point;
	u8 routetime;//第几次跑地图
};
extern struct Map_State map;
extern int route[100];
u8 getNextConnectNode(u8 nownode,u8 nextnode);
void mapInit(void);
void Cross(void);
void map_function(u8 fun);
u8 deal_arrive(void);


#endif







