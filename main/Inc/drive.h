/*
 * drive.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#ifndef MAIN_INC_DRIVE_H_
#define MAIN_INC_DRIVE_H_
#include <stdint.h>
#include <stdbool.h>

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
void targeV3();
void targeV31();
void targeV32();
void targeV33();
void SecD_Menu_Print();
void Drive_Second_fast_menu();
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
extern float pitInDecel;
void Change_pitin_decel();

extern bool windowDeadZone;

#endif /* MAIN_INC_DRIVE_H_ */

