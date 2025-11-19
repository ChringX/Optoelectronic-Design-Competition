#ifndef __BARRIER_H
#define __BARRIER_H

#include "sys.h"

extern uint8_t Start_Color;

void Change_ScanMode(void);
void Route_Set(void);
void Point2data(uint8_t nodenum);
void zhunbei(void);
void Find_Min_Dis(uint8_t nowNodeNum);
void TREASURE(void);
void PLATFORM(void);
void Set_CornerOrInRoad(uint8_t nodenum);
void NoReset_Length(uint8_t nodenum);
void Route_Start(void);
void Route_YesSet(void);
void Confirm_TreaPoint(void);
uint8_t Confirm_FirstTime(void);
uint8_t Long_Trea(void);
uint8_t SymPoint(uint8_t nodenum);
void Set_RoadLength(uint8_t nodenum);
void SYM_Set_RoadLength(uint8_t nodenum);
#endif
