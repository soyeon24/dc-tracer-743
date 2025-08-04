/*
 * motor.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#include "main.h"
#include <stdint.h>
#ifndef MAIN_INC_MOTOR_H_
#define MAIN_INC_MOTOR_H_

void Motor_Test_Menu(void);
void Motor_LPTIM4_IRQ();

typedef struct {
	volatile uint16_t CurrEncVal; //현재 엔코더
	volatile uint16_t PastEncVal; //이전 엔코더
	volatile int16_t EncDiff;	// 현재 - 이전 엔코더 == 거리 차이
	volatile float EncV; //앤코더 변화 속도 (=각속도)
	volatile float EncD; // 엔코더 변화 거리(=속도의 시간대비 적분값)
	volatile float ComV; //커맨드 속도 (=목표 속도)
	volatile float ComD; //커맨드 위치 (=목표 거리)
	volatile float v; //바퀴속도
	volatile int32_t Duty;
	volatile float ErrV;
	volatile float CurPI;
	volatile float VoltPI;
	volatile float Integral;
	volatile float gainP;
	volatile float gainI;


} motor;

extern motor MotorL;
extern motor MotorR;
void Motor_Left_Gain_P();
void Motor_Right_Gain_P();
void Encoder_Test();
void Left_Motor_Test_Phase();
//void Both_Motor_Test_Phase();
void Motor_velocity_change();
void Motor_Start();
void Motor_Stop();
void Motor_Test_76EHWAN();
void Motor_Left_Gain_I();
void Motor_Right_Gain_I();
void  Motor_Speed_Test();
void Motor_Speed_Change();
void Left_PI_Both_Change();

void Motor_Left_Gain_Both();
void Motor_Right_Gain_Both();
#endif /* MAIN_INC_MOTOR_H_ */
