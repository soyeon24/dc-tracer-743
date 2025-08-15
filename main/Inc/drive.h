/*
 * drive.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#ifndef MAIN_INC_DRIVE_H_
#define MAIN_INC_DRIVE_H_
#include <stdint.h>

#include "../../MDK-ARM/Inc/main.h"
void Drive_First(void);
void Drive_Second(void);
void Drive_Third(void);
void Drive_Forth(void);
void Drive_Start();
void Drive_Test_Without_Motor();
void Drive_LPTIM5_IRQ();
void First_Drive_Mark_Debug();
void Drive_Test_Menu();
void Drive_Second();
//void Drive_Test_Menu();
void Mark_Debug();
void Drive_Test_Menu();
void Buzzer_Stop();
void Drive_First_Pit_In_Correct();

extern volatile float targetVelocity;
extern volatile float currentVelocity;
extern volatile float targetVelocitySetting;
extern volatile float accel;
extern volatile float accelSetting;
extern volatile float decelSetting;
extern volatile float decel;
extern volatile float pitInLine;
extern volatile float curveDecel;
extern volatile float curveRate;
extern volatile float peakVelocity;
extern volatile float targetVelocityPitinSetting;
extern float_t saveCentiMeter;
extern uint8_t changeingSafety;

#endif /* MAIN_INC_DRIVE_H_ */

