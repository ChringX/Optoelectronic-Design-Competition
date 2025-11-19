#ifndef __BARRIER_H
#define __BARRIER_H

#include "sys.h"

extern uint8_t Start_Color;

void Change_ScanMode(void);
void Route_Set(void);
void Point2data(void);
void zhunbei(void);
void Find_Min_Dis(uint8_t nowNodeNum);
void TREASURE(void);
void PLATFORM(void);
void Route_Start(void);
void Route_YesSet(void);
void Confirm_TreaPoint(void);
uint8_t Confirm_FirstTime(void);
uint8_t Long_Trea(void);
void Repeat_Witefor(void);
void ComfirmSymmetry(void);
void WrongSymmetry(void);
uint8_t SymPoint(uint8_t nodenum);
#endif
