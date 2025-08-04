/*
 * drive.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#ifndef MAIN_INC_DRIVE_H_
#define MAIN_INC_DRIVE_H_
#include <stdint.h>
#include "main.h"
void Drive_First(void);
void Drive_Second(void);
void Drive_Third(void);
void Drive_Forth(void);
void Drive_Start();
void Drive_Test_Without_Motor();
void Drive_LPTIM5_IRQ();
void First_Drive_Mark_Debug();
void Drive_Test_Menu();

extern volatile float targetVelocity;
extern volatile float currentVelocity;
extern volatile float targetVelocitySetting;
extern volatile float accel;
extern volatile float accelSetting;
extern volatile float decelSetting;
extern volatile float decel;
extern volatile float pitInLine;
#endif /* MAIN_INC_DRIVE_H_ */
