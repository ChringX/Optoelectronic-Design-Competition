#include <stdio.h>
#include <stdint.h>

#define Treasure 2
#define Platform 1
#define NO 5
#define DLEFT 3
#define DRIGHT 4

#define inf 65535
#define numvexs 78

typedef struct _node{
	uint8_t nodenum;     //结点名称
	uint32_t  flag;	    //结点标志位
	float angle;	//角度	
	uint16_t	step;		//线长
	float speed;	//寻线速度
	uint8_t function;    //结点函数
}NODE;

enum MapNode {
	A1,A2,A3,A4,A5,A6,A7,P1,          		//7
	B1,A8,A9,A10,A11,A61,A12,A13,     		//15
	B3,A14,A15,A16,B4,A17,B5,A18,A19, 		//24
	A20,A21,A22,B6,A23,A24,A25,B7,   		 //32
	B8,A26,A27,A28,A29,A30,           		//38
	A31,A32,A33,A34,A35,B9,          		 //44
	B10,A36,A37,A38,B11,A39,A40,A41, 	   //52
	A42,A43,B12,A44,B13,A45,A46,A47,B14,  //61
	A48,A49,A62,A50,A51,A52,A53,B16,      //69
	P2,A54,A55,A56,A57,A58,A59,A60        //77
};

//NODE Node[168]={
///*A1    1*/	{A2,NO,-90,40,1000,1},{B3,NO,180,40,1000,Treasure},
///*A2 	2*/	{A1,DLEFT,90,40,1000,1},{B1,NO,180,40,1000,Treasure},{A3,DRIGHT,-90,40,1000,1},
///*A3 	3*/	{A2,DLEFT,90,40,1000,1},{A9,DRIGHT,180,40,1000,1},{A4,DRIGHT,-90,40,1000,1},
///*A4 	4*/	{A3,DLEFT,90,40,1000,1},{A10,DLEFT,180,40,1000,1}, //9
///*A5	5*/	{A6,DRIGHT,-90,40,1000,1},{A11,DRIGHT|DLEFT,180,40,1000,1},
///*A6	6*/	{A5,DLEFT,90,40,1000,1},{B5,NO,180,40,1000,Treasure},
///*A7	7*/	{P1,NO,-90,40,1000,Platform},{A12,DLEFT,180,40,1000,1},
///*P1	8*/	{A7,DLEFT,90,40,1000,1},
///*B1	9*/	{A2,DLEFT|DRIGHT,0,40,1000,1},
///*A8	10*/{A9,DLEFT,-90,40,1000,1},{A15,DLEFT|DRIGHT,180,40,1000,1}, //19
///*A9	11*/{A3,DLEFT|DRIGHT,0,40,1000,1},{A8,DLEFT,90,40,1000,1},
///*A10	12*/{A4,DLEFT,0,40,1000,1},{A11,DLEFT,-90,40,1000,1},{B4,DLEFT,180,40,1000,Treasure},
///*A11	13*/{A5,DRIGHT,0,40,1000,1},{A10,DRIGHT|DLEFT,90,40,1000,1},{A61,DRIGHT,-90,40,1000,1},
///*A61	14*/{A11,DRIGHT,90,40,1000,1},{A17,DRIGHT,180,40,1000,1},
///*A12	15*/{A7,DRIGHT,0,40,1000,1},{A13,DRIGHT,-90,40,1000,1}, //31
///*A13	16*/{A12,DRIGHT,90,40,1000,1},{A19,DRIGHT,180,40,1000,1},
///*B3	17*/{A1,DRIGHT,0,40,1000,1},
///*A14	18*/{A21,DRIGHT,180,40,1000,1},{A15,DRIGHT|DLEFT,-90,40,1000,1},
///*A15	19*/{A8,DRIGHT,0,40,1000,1},{A14,DLEFT,90,40,1000,1},{A26,DRIGHT,180,40,1000,1},{A16,DRIGHT,-90,40,1000,1}, //40
///*A16	20*/{A15,DRIGHT|DLEFT,90,40,1000,1},{A22,DLEFT,180,40,1000,1},
///*B4	21*/{A10,DRIGHT,0,40,1000,1},{A17,DRIGHT|DLEFT,-90,40,1000,1},
///*A17	22*/{A61,DLEFT,0,40,1000,1},{B4,DRIGHT,90,40,1000,Treasure},{A23,DLEFT,180,40,1000,1},
///*B5	23*/{A6,DLEFT,0,40,1000,1},
///*A18	24*/{A19,DLEFT,-90,40,1000,1},{A25,DRIGHT,180,40,1000,1}, //50
///*A19	25*/{A13,DLEFT,0,40,1000,1},{A18,DLEFT,90,40,1000,1},
///*A20	26*/{A21,DLEFT,-90,40,1000,1},{A31,DLEFT,180,40,1000,1},
///*A21	27*/{A20,DLEFT,90,40,1000,1},{A14,DRIGHT,0,40,1000,1},
///*A22	28*/{A16,DLEFT,0,40,1000,1},{B6,NO,-90,40,1000,Treasure},{A27,DLEFT,180,40,1000,1},
///*B6	29*/{A22,DLEFT|DRIGHT,90,40,1000,1},  //60
///*A23	30*/{A17,DLEFT,0,40,1000,1},{A24,DRIGHT,-90,40,1000,1},
///*A24	31*/{A23,DRIGHT,90,40,1000,1},{A25,DLEFT,-90,40,1000,1},{A29,DLEFT|DRIGHT,180,40,1000,1},
///*A25	32*/{A18,DRIGHT,0,40,1000,1},{A24,DLEFT,90,40,1000,1},
///*B7	33*/{A30,DRIGHT,180,40,1000,1},
///*B8	34*/{A26,DLEFT,-90,40,1000,1},
///*A26	35*/{A15,DLEFT|DRIGHT,0,40,1000,1},{B8,NO,90,40,1000,Treasure},  //71
///*A27	36*/{A22,DRIGHT,0,40,1000,1},{A33,DLEFT|DRIGHT,180,40,1000,1},{A28,DRIGHT,-90,80,1000,1},
///*A28	37*/{A27,DLEFT|DRIGHT,90,80,1000,1},{A34,DRIGHT,180,40,1000,1},{A29,DLEFT,-90,40,1000,1},
///*A29	38*/{A24,DRIGHT|DLEFT,0,40,1000,1},{A28,DLEFT,90,30,1000,1},{A30,DLEFT|DRIGHT,-90,40,1000,1},  //80
///*A30	39*/{B7,NO,0,40,1000,Treasure},{A41,DRIGHT,180,40,1000,1},{A29,DRIGHT,90,40,1000,1},
///*A31	40*/{A20,DRIGHT,0,40,1000,1},{A32,DRIGHT,-90,40,1000,1},{B10,NO,180,40,1000,Treasure},
///*A32	41*/{A31,DRIGHT|DLEFT,90,40,1000,1},{A33,DLEFT,-90,40,1000,1},{A37,DRIGHT/*|DLEFT*/,180,40,1000,1},
///*A33	42*/{A32,DLEFT,90,40,1000,1},{A27,DRIGHT,0,40,1000,1},{A34,DRIGHT|DLEFT,-90,80,1000,1}, //92
///*A34	43*/{A28,DRIGHT|DLEFT,0,40,1000,1},{A33,DRIGHT,90,80,1000,1},{A39,DRIGHT,180,40,1000,1},
///*A35	44*/{B9,NO,-90,40,1000,Treasure},{A46,DRIGHT|DLEFT,180,40,1000,1},
///*B9	45*/{A35,DLEFT,90,40,1000,1},
///*B10	46*/{A31,DRIGHT,0,40,1000,1},
///*A36	47*/{A37,DLEFT,-90,40,1000,1},{A43,DRIGHT,180,40,1000,1},  //101
///*A37	48*/{A32,DLEFT/*|DRIGHT*/,0,40,1000,1},{A36,DLEFT,90,40,1000,1},{A38,DRIGHT,-90,40,1000,1},
///*A38	49*/{A37,DRIGHT,90,40,1000,1},{A44,DLEFT,180,40,1000,1},
///*B11	50*/{A39,DLEFT|DRIGHT,-90,40,1000,1},
///*A39	51*/{A34,DLEFT,0,40,1000,1},{A45,DLEFT,180,40,1000,1},{B11,NO,90,40,1000,1},  //110
///*A40	52*/{A41,DLEFT,-90,40,1000,1},{A47,DRIGHT,180,40,1000,1},
///*A41	53*/{A30,DLEFT,0,40,1000,1},{A40,DLEFT,90,40,1000,1},
///*A42	54*/{A43,DLEFT,-90,40,1000,1},{A48,DLEFT,180,40,1000,1},
///*A43	55*/{A36,DRIGHT,0,40,1000,1},{A42,DLEFT,90,40,1000,1},
///*B12	56*/{A55,DLEFT,180,40,1000,1},
///*A44	57*/{B13,DRIGHT,-90,40,1000,Treasure},{A62,DLEFT,180,40,1000,1},{A38,DLEFT,0,40,1000,1},  //122
///*B13	58*/{A44,DLEFT|DRIGHT,90,40,1000,1},{A51,DRIGHT,180,40,1000,1},
///*A45	59*/{A39,DLEFT,0,40,1000,1},{A46,DLEFT|DRIGHT,90,40,1000,1},
///*A46	60*/{A35,DRIGHT,0,40,1000,1},{A45,DRIGHT,90,40,1000,1},{A47,DLEFT,-90,40,1000,1},{A53,DRIGHT,180,40,1000,1}, //130
///*A47	61*/{A46,DLEFT|DRIGHT,90,40,1000,1},{A40,DRIGHT,0,40,1000,1},
///*B14	62*/{A60,DRIGHT,180,40,1000,1},
///*A48	63*/{A42,DRIGHT,0,40,1000,1},{A49,DRIGHT,-90,40,1000,1},
///*A49	64*/{A48,DRIGHT,90,40,1000,1},{A54,DRIGHT,180,40,1000,1},
///*A62	65*/{A44,DRIGHT,0,40,1000,1},{A50,DRIGHT,-90,40,1000,1},  //139
///*A50	66*/{A62,DRIGHT,90,40,1000,1},{A51,DLEFT|DRIGHT,-90,40,1000,1},{A56,DRIGHT,180,40,1000,1},
///*A51	67*/{B13,DLEFT,0,40,1000,Treasure},{A50,DLEFT,90,40,1000,1},{A57,DLEFT,180,40,1000,1},
///*A52	68*/{A53,DLEFT,-90,40,1000,1},{A58,DLEFT|DRIGHT,180,40,1000,1},
///*A53	69*/{A46,DRIGHT|DLEFT,0,40,1000,1},{A52,DLEFT,90,40,1000,1},
///*B16	70*/{A59,DRIGHT|DLEFT,180,40,1000,1},   //150
///*P2	71*/{A54,DLEFT,-90,40,1000,1},
///*A54	72*/{A49,DLEFT,0,40,1000,1},{P2,NO,90,40,1000,Platform},
///*A55	73*/{B12,NO,0,40,1000,Treasure},{A56,DLEFT,-90,40,1000,1},
///*A56	74*/{A55,DRIGHT,90,40,1000,1},{A50,DLEFT|DRIGHT,0,40,1000,1},
///*A57	75*/{A51,DLEFT,0,40,1000,1},{A58,DLEFT,-90,40,1000,1},
///*A58	76*/{A52,DRIGHT,0,40,1000,1},{A57,DRIGHT,90,40,1000,1},{A59,DLEFT,-90,40,1000,1},
///*A59	77*/{B16,NO,0,40,1000,Treasure},{A58,DRIGHT,90,40,1000,1},{A60,DLEFT,-90,40,1000,1},
///*A60	78*/{B14,NO,0,40,1000,Treasure},{A59,DRIGHT,90,40,1000,1}
//};
#define STOPTURN 1
#define  RESTMPUZ 0
NODE Node[168]={   //A29-A30 B3-A1 5000 其他 7000  到宝藏点距离为无穷
/*A1    1*/	{A2,DRIGHT|STOPTURN,-90,5,7000,1},{B3,NO,180,5,7000,1},
/*A2 	2*/	{A1,DLEFT,90,5,7000,1},{B1,NO,180,5,7000,1},{A3,DRIGHT|RESTMPUZ,-90,5,7000,1},
/*A3 	3*/	{A2,DLEFT|RESTMPUZ|STOPTURN,90,5,7000,1},{A9,DRIGHT,180,5,7000,1},{A4,DRIGHT,-90,5,7000,1},
/*A4 	4*/	{A3,DLEFT,90,5,7000,1},{A10,DLEFT,180,5,7000,1}, //9
/*A5	5*/	{A6,DRIGHT|RESTMPUZ,-90,5,7000,1},{A11,DRIGHT|DLEFT,180,5,7000,1},
/*A6	6*/	{A5,DLEFT|RESTMPUZ,90,5,7000,1},{B5,NO,180,5,7000,1},
/*A7	7*/	{P1,NO,-90,5,7000,Platform},{A12,DLEFT,180,5,7000,1},
/*P1	8*/	{A7,DLEFT,90,5,7000,1},
/*B1	9*/	{A2,DLEFT|DRIGHT,0,5,7000,1},
/*A8	10*/{A9,DLEFT,-90,5,7000,1},{A15,DLEFT|DRIGHT,180,5,7000,1}, //19
/*A9	11*/{A3,DLEFT|DRIGHT,0,5,7000,1},{A8,DLEFT,90,5,7000,1},
/*A10	12*/{A4,DLEFT,0,5,7000,1},{A11,DLEFT,-90,5,7000,1},{B4,DLEFT,180,5,7000,1},
/*A11	13*/{A5,DRIGHT,0,5,7000,1},{A10,DRIGHT|DLEFT,90,5,7000,1},{A61,DRIGHT,-90,5,7000,1},
/*A61	14*/{A11,DRIGHT,90,5,7000,1},{A17,DRIGHT,180,5,7000,1},
/*A12	15*/{A7,DRIGHT,0,5,7000,1},{A13,DRIGHT,-90,5,7000,1}, //31
/*A13	16*/{A12,DRIGHT,90,5,7000,1},{A19,DRIGHT,180,5,7000,1},
/*B3	17*/{A1,DRIGHT,0,5,5000,1},
/*A14	18*/{A21,DRIGHT,180,5,7000,1},{A15,DRIGHT|DLEFT,-90,5,7000,1},
/*A15	19*/{A8,DRIGHT,0,5,7000,1},{A14,DLEFT,90,5,7000,1},{A26,DRIGHT|STOPTURN,180,5,7000,1},{A16,DRIGHT,-90,5,7000,1}, //40
/*A16	20*/{A15,DRIGHT|DLEFT,90,5,7000,1},{A22,DLEFT,180,5,7000,1},
/*B4	21*/{A10,DRIGHT,0,5,7000,1},{A17,DRIGHT|DLEFT,-90,5,7000,1},
/*A17	22*/{A61,DLEFT,0,5,7000,1},{B4,DRIGHT,90,5,7000,1},{A23,DLEFT,180,5,7000,1},
/*B5	23*/{A6,DLEFT,0,5,7000,1},
/*A18	24*/{A19,DLEFT,-90,5,7000,1},{A25,DRIGHT,180,5,7000,1}, //50
/*A19	25*/{A13,DLEFT,0,5,7000,1},{A18,DLEFT,90,5,7000,1},
/*A20	26*/{A21,DLEFT,-90,5,7000,1},{A31,DLEFT,180,5,7000,1},
/*A21	27*/{A20,DLEFT,90,5,7000,1},{A14,DRIGHT,0,5,7000,1},
/*A22	28*/{A16,DLEFT,0,5,7000,1},{B6,NO,-90,5,7000,1},{A27,DLEFT,180,5,7000,1},
/*B6	29*/{A22,DLEFT|DRIGHT,90,5,7000,1},  //60
/*A23	30*/{A17,DLEFT,0,5,7000,1},{A24,DRIGHT,-90,5,7000,1},
/*A24	31*/{A23,DRIGHT,90,5,7000,1},{A25,DLEFT,-90,5,7000,1},{A29,DLEFT|DRIGHT,180,5,7000,1},
/*A25	32*/{A18,DRIGHT,0,5,7000,1},{A24,DLEFT,90,5,7000,1},
/*B7	33*/{A30,DRIGHT,180,5,7000,1},
/*B8	34*/{A26,DLEFT,-90,5,7000,1},
/*A26	35*/{A15,DLEFT|DRIGHT,0,5,7000,1},{B8,NO,90,5,7000,1},  //71
/*A27	36*/{A22,DRIGHT,0,5,7000,1},{A33,DLEFT|DRIGHT,180,5,7000,1},{A28,DRIGHT|RESTMPUZ,-90,5,7000,1},
/*A28	37*/{A27,DLEFT|DRIGHT|RESTMPUZ,90,5,7000,1},{A34,DRIGHT,180,5,7000,1},{A29,DLEFT,-90,5,7000,1},
/*A29	38*/{A24,DRIGHT|DLEFT,0,5,7000,1},{A28,DLEFT,90,5,7000,1},{A30,DLEFT|DRIGHT|RESTMPUZ|STOPTURN,-90,5,5000,1},  //80
/*A30	39*/{B7,NO,0,5,7000,1},{A41,DRIGHT|RESTMPUZ,180,5,7000,1},{A29,DRIGHT|RESTMPUZ,90,5,7000,1},
/*A31	40*/{A20,DRIGHT,0,5,7000,1},{A32,DRIGHT|RESTMPUZ,-90,5,7000,1},{B10,NO,180,5,7000,1},
/*A32	41*/{A31,DRIGHT|DLEFT|RESTMPUZ|STOPTURN,90,5,7000,1},{A33,DLEFT,-90,5,7000,1},{A37,DRIGHT|DLEFT,180,5,7000,1},
/*A33	42*/{A32,DLEFT,90,5,7000,1},{A27,DRIGHT,0,5,7000,1},{A34,DRIGHT|DLEFT|RESTMPUZ,-90,5,7000,1}, //92
/*A34	43*/{A28,DRIGHT|DLEFT,0,5,7000,1},{A33,DRIGHT|RESTMPUZ,90,5,7000,1},{A39,DRIGHT,180,5,7000,1},
/*A35	44*/{B9,NO,-90,5,7000,1},{A46,DRIGHT|DLEFT|RESTMPUZ,180,5,7000,1},
/*B9	45*/{A35,DLEFT,90,5,7000,1},
/*B10	46*/{A31,DRIGHT,0,5,7000,1},
/*A36	47*/{A37,DLEFT,-90,5,7000,1},{A43,DRIGHT,180,5,7000,1},  //101
/*A37	48*/{A32,DLEFT|DRIGHT,0,5,7000,1},{A36,DLEFT,90,5,7000,1},{A38,DRIGHT,-90,5,7000,1},
/*A38	49*/{A37,DRIGHT,90,5,7000,1},{A44,DLEFT,180,5,7000,1},
/*B11	50*/{A39,DLEFT|DRIGHT,-90,5,7000,1},
/*A39	51*/{A34,DLEFT,0,5,7000,1},{A45,DLEFT,180,5,7000,1},{B11,NO,90,5,7000,1},  //110
/*A40	52*/{A41,DLEFT,-90,5,7000,1},{A47,DRIGHT,180,5,7000,1},
/*A41	53*/{A30,DLEFT|RESTMPUZ,0,5,7000,1},{A40,DLEFT,90,5,7000,1},
/*A42	54*/{A43,DLEFT,-90,5,7000,1},{A48,DLEFT,180,5,7000,1},
/*A43	55*/{A36,DRIGHT,0,5,7000,1},{A42,DLEFT,90,5,7000,1},
/*B12	56*/{A55,DLEFT,180,5,7000,1},
/*A44	57*/{B13,DRIGHT,-90,5,7000,1},{A62,DLEFT,180,5,7000,1},{A38,DLEFT,0,5,7000,1},  //122
/*B13	58*/{A44,DLEFT|DRIGHT,90,5,7000,1},{A51,DRIGHT,180,5,7000,1},
/*A45	59*/{A39,DLEFT,0,5,7000,1},{A46,DLEFT|DRIGHT,90,5,7000,1},
/*A46	60*/{A35,DRIGHT|RESTMPUZ|STOPTURN,0,5,7000,1},{A45,DRIGHT,90,5,7000,1},{A47,DLEFT,-90,5,7000,1},{A53,DRIGHT,180,5,7000,1}, //130
/*A47	61*/{A46,DLEFT|DRIGHT,90,5,7000,1},{A40,DRIGHT,0,5,7000,1},
/*B14	62*/{A60,DRIGHT,180,5,7000,1},
/*A48	63*/{A42,DRIGHT,0,5,7000,1},{A49,DRIGHT,-90,5,7000,1},
/*A49	64*/{A48,DRIGHT,90,5,7000,1},{A54,DRIGHT,180,5,7000,1},
/*A62	65*/{A44,DRIGHT,0,5,7000,1},{A50,DRIGHT,-90,5,7000,1},  //139
/*A50	66*/{A62,DRIGHT,90,5,7000,1},{A51,DLEFT|DRIGHT,-90,5,7000,1},{A56,DRIGHT,180,5,7000,1},
/*A51	67*/{B13,DLEFT,0,5,7000,1},{A50,DLEFT,90,5,7000,1},{A57,DLEFT,180,5,7000,1},
/*A52	68*/{A53,DLEFT,-90,5,7000,1},{A58,DLEFT|DRIGHT,180,5,7000,1},
/*A53	69*/{A46,DRIGHT|DLEFT,0,5,7000,1},{A52,DLEFT,90,5,7000,1},
/*B16	70*/{A59,DRIGHT|DLEFT,180,5,7000,1},   //150
/*P2	71*/{A54,DLEFT,-90,5,7000,1},
/*A54	72*/{A49,DLEFT,0,5,7000,1},{P2,NO,90,5,7000,Platform},
/*A55	73*/{B12,NO,0,5,7000,1},{A56,DLEFT|RESTMPUZ,-90,5,7000,1},
/*A56	74*/{A55,DRIGHT|RESTMPUZ,90,5,7000,1},{A50,DLEFT|DRIGHT,0,5,7000,1},
/*A57	75*/{A51,DLEFT,0,5,7000,1},{A58,DLEFT,-90,5,7000,1},
/*A58	76*/{A52,DRIGHT,0,5,7000,1},{A57,DRIGHT,90,5,7000,1},{A59,DLEFT|RESTMPUZ|STOPTURN,-90,5,7000,1},
/*A59	77*/{B16,NO,0,5,7000,1},{A58,DRIGHT|RESTMPUZ,90,5,7000,1},{A60,DLEFT,-90,5,7000,1},
/*A60	78*/{B14,NO,0,5,7000,1},{A59,DRIGHT|STOPTURN,90,5,7000,1}
};
unsigned char ConnectionNum[78] = {
	2, 3, 3, 2, 2, 2, 2, 1, 
	1, 2, 2, 3, 3, 2, 2, 2, 
	1, 2, 4, 2, 2, 3, 1, 2, 2, 
	2, 2, 3, 1, 2, 3, 2, 1, 
	1, 2, 3, 3, 3, 3, 
	3, 3, 3, 3, 2, 1, 
	1, 2, 3, 2, 1, 3, 2, 2, 
	2, 2, 1, 3, 2, 2, 4, 2, 1, 
	2, 2, 2, 3, 3, 2, 2, 1, 
	1, 2, 2, 2, 2, 3, 3, 2
};

unsigned char Address[78] = {
	0, 2, 5, 8, 10, 12, 14, 16, 
	17, 18, 20, 22, 25, 28, 30, 32, 
	34, 35, 37, 41, 43, 45, 48, 49, 51, 
	53, 55, 57, 60, 61, 63, 66, 68, 
	69, 70, 72, 75, 78, 81, 
	84, 87, 90, 93, 96, 98, 
	99, 100, 102, 105, 107, 108, 111, 113, 
	115, 117, 119, 120, 123, 125, 127, 131, 133, 
	134, 136, 138, 140, 143, 146, 148, 150, 
	151, 152, 154, 156, 158, 160, 163, 166
};

typedef struct
{
	int matrix[numvexs][numvexs];//邻接矩阵
	int numVexs;       //顶点数
	int father[200];    //父节点
	int path[200];      //最短路径 
}Graph;

Graph graph;
Graph PhEleTechnology_Test; 
uint8_t TreaPoint[8] = {0};
//创建矩阵并且赋值 
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

int dijk(Graph *G,int head,int target)
{
	int first = 1;
	
	int distance[numvexs]; //head到各点的距离
	int visited[numvexs];  //1为已访问该节点，0为未访问该节点
	
	 //初始化数据
	for(int i = 0;i < G->numVexs;i++)
	{
		visited[i] = 0;
		distance[i] = G->matrix[head][i]; //将与源节点有连线的顶点加上权值
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
		
		if(first == 1)
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
	int dis = distance[target];
//   printf("%d\r\n",distance[target]); 
    
//    int p = 0;
//	int Transition[200] = {0};
//	Transition[0] = target;
//	int f,g;
//    for(f = 1;target != head;f++)
//    {
//    	Transition[f] = G->father[target];
//    	target = G->father[target];
//	}
//	for(g = f-1;g >= 0;g--)
//	{
//		G->path[p++] = Transition[g];
//	}
//	G->path[p] = 0XFF;
//	int route[200];
//	for(p = 0;route[p] != 0XFF;p++)
//	{
//		route[p] = G->path[p];
//	}
//	for(g = 0;g < 200;g++)
//	{
//		printf("%d ",route[g]);
//	}
	
	
    G->path[0] = target;
    int f,g;
    for(f = 1;target != head;f++)
    {
    	G->path[f] = G->father[target];
    	target = G->father[target];
	}
	int p = 0;
	int Transition[200] = {0};
	for(g = f-1;g >= 0;g--)
	{
		Transition[p++] = G->path[g];
	}
	p = 0;
	for(g = 0;g < f;g++)
	{
		G->path[p++] = Transition[g];
	}
	for(g = 0;g < f;g++)
	{
		printf("%d ",G->path[g]);
	}
	printf("  %d",dis);
	printf("\r\n");
	return dis;
}

int Find_Min_Dis(uint8_t nowNodeNum)
{
	int distance[8] = {0};
	uint8_t i = 0;
	uint8_t j = 0;
	for(i = 0;i < 8;i++)
	{
		distance[i] = dijk(&PhEleTechnology_Test,nowNodeNum,TreaPoint[i]);
	}
	int Min_Dis = distance[0];
	for(i = 1;i < 8;i++)
	{
		if(distance[i] < Min_Dis)
		{
			Min_Dis = distance[i];
			j = i;
		}
	}
	printf("\r\n");
	dijk(&graph,nowNodeNum,TreaPoint[j]);
	return TreaPoint[i];
}

int main()
{
	TreaPoint[0] = B3;
	TreaPoint[1] = B1;
	TreaPoint[2] = B5;
	TreaPoint[3] = B7;
	TreaPoint[4] = B10;
	TreaPoint[5] = B12;
	TreaPoint[6] = B9;
	TreaPoint[7] = B16;
	
	CreatGraph(&graph);
	CreatGraph(&PhEleTechnology_Test);
	Find_Min_Dis(P2);
	return 0;
}

