/*
 * motor.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#include "motor.h"
#include "math.h"
#include "drive.h"
#include "sensor.h"
#include "tim.h"
#include "gpio.h"
#include "lptim.h"
#include "init.h"
#include "lcd.h"
#include <stdbool.h>
#include "custom_switch.h"
#include "dwt_delay.h"
#include "setting.h"

#define ABS(x) ((x>0) ? x:(-x))
#define MIN(a, b) ((a > b) ? b : a )
#define ENCODER_PULSE_PER_REVOLUTION 2048.f
//ENCODER_RESOLUTION
//ENCO
#define GEAR_RATIO (63.f/17.f)
#define WHEEL_DIAMETER 0.035f //4cm라 가정
#define DT 0.0005
//TICK_PER_REV * REV_PER_METER
//TICK_PER_REV / METER_PER_REV

#define TICK_PER_METER (ENCODER_PULSE_PER_REVOLUTION/(M_PI*WHEEL_DIAMETER)*GEAR_RATIO)

//#define TICK_PER_METER (GEAR_RATIO/(WHEEL_RADIUS*M_PI))
#define RADIAN_PER_TICK (1.0f/ENCODER_PULSE_PER_REVOLUTION)
//#define ANGLE_PER_METER (TICK_PER_METER * ENCODER_PULSE_PER_REVOLUTION)
//#define RADIAN_PER_SEC (2.f * M_PI / DT / ENCODER_PULSE_PER_REVOLUTION)
#define MOTOR_RESISTANCE 0.68f//모터 저항
#define MOTOR_KT 0.0146f//모터 토크상수  // 14.6 mNm A-1
#define MOTOR_KE 68.486719848257492598485625755493f //모터 속도상수 //(rad/sec)/V    //654 min-1 V-1

#define MOTOR_BATTERY_MAX 18.f

#define MOTOR_TIM		&htim8
#define MOTORL_CHANNEL	TIM_CHANNEL_2
#define MOTORR_CHANNEL	TIM_CHANNEL_1
#define MOTORL_ENCODER_TIMER &hlptim2
#define MOTORR_ENCODER_TIMER &hlptim1

#define MOTORL_PH_GPIO_Port  GPIOD
#define MOTORL_PH_Pin        GPIO_PIN_14
#define MOTORR_PH_GPIO_Port  GPIOA
#define MOTORR_PH_Pin        GPIO_PIN_9

motor MotorL;
motor MotorR;

float EncLRadperSec;
float EncRRadperSec;
int motorTickL = 0;
int motorTickR = 0;
//1,8 모터 low power timer 1,2 encoder

void Motor_Start() { //Time 0.0005 L 0.175 420, R 0.23 400
	MotorL.gainP = 0.225f; //0.48f;//1.2f;//0.48f;//0.005f;//1.23f; //1.46
	MotorL.gainI = 170.0f; //102.4f;//300.0f;//0.1f; //300.0f;

	MotorR.gainP = 0.3; //0.9f;//0.8f;// 0.8f; //0.97
	MotorR.gainI = 350.f; //0.0f;//

	MotorL.CurrEncVal = 0; //현재 엔코더
	MotorL.PastEncVal = 0; //이전 엔코더
	MotorL.EncDiff = 0;	// 현재 - 이전 엔코더 == 거리 차이
	MotorL.rad_per_sec = 0; //앤코더 변화 속도 (=각속도)
	MotorL.EncD = 0; // 엔코더 변화 거리(=속도의 시간대비 적분값)
	MotorL.rad_per_sec_cmd = 0; //커맨드 속도 (=목표 속도)
	MotorL.ComD = 0; //커맨드 위치 (=목표 거리)
	MotorL.mps_cmd = 0; //바퀴속도
	MotorL.duty_ratio = 0;
	MotorL.pwm_timer_val = 0;
	MotorL.rad_per_sec_err = 0;
	MotorL.current = 0;
	MotorL.voltage = 0;
	MotorL.Integral = 0;

	MotorR.CurrEncVal = 0; //현재 엔코더
	MotorR.PastEncVal = 0; //이전 엔코더
	MotorR.EncDiff = 0;	// 현재 - 이전 엔코더 == 거리 차이
	MotorR.rad_per_sec = 0; //앤코더 변화 속도 (=각속도)
	MotorR.EncD = 0; // 엔코더 변화 거리(=속도의 시간대비 적분값)
	MotorR.rad_per_sec_cmd = 0; //커맨드 속도 (=목표 속도)
	MotorR.ComD = 0; //커맨드 위치 (=목표 거리)
	MotorR.mps_cmd = 0; //바퀴속도
	MotorR.duty_ratio = 0;
	MotorR.pwm_timer_val = 0;
	MotorR.rad_per_sec_err = 0;
	MotorR.current = 0;
	MotorR.voltage = 0;
	MotorR.Integral = 0;
	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535); //왼쪽 엔코더
	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535); //오른쪽 엔코더

	//	DRVOFF 핀 설정 0: 주행 / 1: 정지
	HAL_GPIO_WritePin(L_DRVOFF_GPIO_Port, L_DRVOFF_Pin, 0);
	HAL_GPIO_WritePin(R_DRVOFF_GPIO_Port, R_DRVOFF_Pin, 0);

	//	nSLEEP 핀 설정 0: 절전 모드 / 1: 동작 모드
	HAL_GPIO_WritePin(L_nSLEEP_GPIO_Port, L_nSLEEP_Pin, 1);
	HAL_GPIO_WritePin(R_nSLEEP_GPIO_Port, R_nSLEEP_Pin, 1);

	//	모터드라이버 안정화
	HAL_Delay(1);

	//	모터가 Sleep 상태에서 Active 상태로 바뀔 때, 메인 보드에 동작 가능한 지 알려주기 위해 nFault 핀을 강제로 low로 내려보낸다.
	//	nFault 핀이 low면 OUT1, 2가 HI-Z로 변해 모터가 돌지 않는다. 따라서 Active 하기 위해선 nFault를 reset해야 한다.
	//	Reset 방법은 nSleep 핀에 pulse를 넣으면 되는데 pulse의 low 시간에 따라 동작이 바뀐다.
	//	주석 읽었으면 이거

	//	t_RESET min: 5us / t_RESET max: 20us
	//	t_SLEEP min: 40us / t_SLEEP max: 140

	//	pulse의 low time 세팅
	//	0 				~	t_RESET min	: Clear Fault -> NO 			/ Sleep -> NO
	//	t_RESET min		~	t_RESET max : Clear Fault -> Indeterminate 	/ Sleep -> NO
	//	t_RESET max		~	t_SLEEP min : Clear Fault -> YES			/ Sleep -> NO
	//	t_SLEEP min		~	t_SLEEP max : Clear Fault -> YES			/ Sleep -> Indeterminate
	//	t_SLEEP max		~				: Clear Fault -> YES			/ Sleep -> YES

	//	따라서 nFault에 알맞는 low 시간은 20us 이상 40us 미만이다. 따라서 nSleep 핀을 low로 보낸 뒤, 30us 뒤에 high로 놓으면 된다.
	bool nFault = true;

	//	nSleep핀 low로 보내기
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_SET);
	while (nFault) {
		nFault = !(HAL_GPIO_ReadPin(L_nFAULT_GPIO_Port, L_nFAULT_Pin))
				| !(HAL_GPIO_ReadPin(L_nFAULT_GPIO_Port, L_nFAULT_Pin));
		HAL_GPIO_WritePin(L_nSLEEP_GPIO_Port, L_nSLEEP_Pin, 0);
		HAL_GPIO_WritePin(R_nSLEEP_GPIO_Port, R_nSLEEP_Pin, 0);

		//	30us 기다리기
		DWT_Delay_us(30);

		//	30us 이후 다시 nSleep핀 high로 보내기
		HAL_GPIO_WritePin(L_nSLEEP_GPIO_Port, L_nSLEEP_Pin, 1);
		HAL_GPIO_WritePin(R_nSLEEP_GPIO_Port, R_nSLEEP_Pin, 1);

		HAL_Delay(1);
	}

	//	문제 해결
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Start_IT(MOTOR_TIM, MOTORR_CHANNEL);
	HAL_TIM_PWM_Start_IT(MOTOR_TIM, MOTORL_CHANNEL);

	__HAL_TIM_SET_COMPARE(MOTOR_TIM, MOTORR_CHANNEL, 0);
	__HAL_TIM_SET_COMPARE(MOTOR_TIM, MOTORL_CHANNEL, 0);

//
//
//	HAL_GPIO_WritePin(L_nSLEEP_GPIO_Port, L_nSLEEP_Pin, 1);
//	HAL_GPIO_WritePin(R_nSLEEP_GPIO_Port, R_nSLEEP_Pin, 1);
//	HAL_GPIO_WritePin(L_DRVOFF_GPIO_Port, L_DRVOFF_Pin, 0);
//	HAL_GPIO_WritePin(R_DRVOFF_GPIO_Port, R_DRVOFF_Pin, 0);
//
////	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
////	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
//
	HAL_LPTIM_Counter_Start_IT(&hlptim4, 0);

}

void Motor_Stop() {

	HAL_GPIO_WritePin(L_DRVOFF_GPIO_Port, L_DRVOFF_Pin, 1);
	HAL_GPIO_WritePin(R_DRVOFF_GPIO_Port, R_DRVOFF_Pin, 1);
	HAL_GPIO_WritePin(L_nSLEEP_GPIO_Port, L_nSLEEP_Pin, 0);
	HAL_GPIO_WritePin(R_nSLEEP_GPIO_Port, R_nSLEEP_Pin, 0);

	HAL_LPTIM_Encoder_Stop(MOTORR_ENCODER_TIMER);
	HAL_LPTIM_Encoder_Stop(MOTORL_ENCODER_TIMER);

	HAL_LPTIM_Counter_Stop_IT(&hlptim4);
	HAL_LPTIM_Counter_Stop_IT(&hlptim3);

}

menu_t motor_menu[] = { { "speed T", Motor_Speed_Change }, { "1.left gain ",
		Motor_Left_Gain_Both }, { "2.Right gain", Motor_Right_Gain_Both}, { "encoder",
				Encoder_Test }, { "PWM Test", Motor_Test_76EHWAN }, { "1.left gI ",
		Motor_Left_Gain_I }, { "2.Right gI", Motor_Right_Gain_I }, {
		"back menu", Back_To_Menu } };

uint32_t MOTOR_MENU_CNT = (sizeof(motor_menu) / sizeof(menu_t));

void Motor_Menu_Print(uint32_t index) {
	for (uint32_t i = 0; i < MOTOR_MENU_CNT; i++) {
		Set_Color(index, i);
		Custom_LCD_Printf(0, i + 1, "%s", (motor_menu + i)->name);

	}
}

void Motor_Test_Menu() {
	uint32_t numOfMotorMenu = sizeof(motor_menu) / sizeof(menu_t);
	uint32_t selected_index = 0;
	uint32_t sw = CUSTOM_JS_NONE;
	while (1) {
		POINT_COLOR = WHITE;
		BACK_COLOR = BLACK;
		Custom_LCD_Printf(0, 0, "Motor M");
		while (1) {
			Set_Color(numOfMotorMenu, selected_index);
			Motor_Menu_Print(selected_index);
			sw = Custom_Switch_Read();
			if (sw == CUSTOM_JS_U_TO_D) {
				selected_index++;
				selected_index = (selected_index % 8);
			} else if (sw == CUSTOM_JS_D_TO_U) {
				selected_index--;
				selected_index = (selected_index % 8);
			} else if (sw == CUSTOM_JS_L_TO_R) {
				Custom_LCD_Clear();
				(motor_menu + selected_index)->func();
			}
			Set_Color(numOfMotorMenu, selected_index);

		}
	}
}
int kkkk=0;
// 두 모터의 속도를 PI제어하는 함수
void Motor_LPTIM4_IRQ() {
kkkk++;

	//input
	MotorL.PastEncVal = MotorL.CurrEncVal;
	MotorR.PastEncVal = MotorR.CurrEncVal;

	MotorL.CurrEncVal = (uint16_t) LPTIM2->CNT;	//1은 왼쪽
	MotorR.CurrEncVal = (uint16_t) -LPTIM1->CNT;	//2는 오른쪽 감소

	MotorL.EncDiff = (int16_t) MotorL.CurrEncVal - (int16_t) MotorL.PastEncVal;
	MotorR.EncDiff = (int16_t) MotorR.CurrEncVal - (int16_t) MotorR.PastEncVal;

	MotorL.rad_per_sec = ((float) MotorL.EncDiff) / DT * RADIAN_PER_TICK; //rad/sec
	MotorR.rad_per_sec = ((float) MotorR.EncDiff) / DT * RADIAN_PER_TICK;

	MotorL.rad_per_sec_cmd = MotorL.mps_cmd
			* (TICK_PER_METER * 2 * M_PI / ENCODER_PULSE_PER_REVOLUTION);
	MotorR.rad_per_sec_cmd = MotorR.mps_cmd
			* (TICK_PER_METER * 2 * M_PI / ENCODER_PULSE_PER_REVOLUTION);

	MotorL.rad_per_sec_err = MotorL.rad_per_sec - MotorL.rad_per_sec_cmd;
	MotorR.rad_per_sec_err = MotorR.rad_per_sec - MotorR.rad_per_sec_cmd;

	// I
	MotorL.rad_err += MotorL.rad_per_sec_err * DT;
	MotorR.rad_err += MotorR.rad_per_sec_err * DT;
	/*주기 2KHz */
	MotorL.current = -(MotorL.rad_err * MotorL.gainI + MotorL.rad_per_sec_err * MotorL.gainP);
	MotorR.current = -(MotorR.rad_err * MotorR.gainI + MotorR.rad_per_sec_err * MotorR.gainP);

	MotorL.voltage = MotorL.current * MOTOR_RESISTANCE
			+ MotorL.rad_per_sec_cmd / MOTOR_KE;
	MotorR.voltage = MotorR.current * MOTOR_RESISTANCE
			+ MotorR.rad_per_sec_cmd / MOTOR_KE;

	bool DirL = MotorL.voltage > 0;
	bool DirR = MotorR.voltage > 0;

//	MotorL.voltage = MIN(ABS(MotorL.voltage), MOTOR_BATTERY_MAX);
//	MotorL.voltage = MIN(batteryVolt, MotorL.voltage);
//	MotorR.voltage = MIN(ABS(MotorR.voltage), MOTOR_BATTERY_MAX);
//	MotorR.voltage = MIN(batteryVolt, MotorR.voltage);

	MotorL.duty_ratio = MIN(ABS(MotorL.voltage) / batteryVolt, 1);
	MotorR.duty_ratio = MIN(ABS(MotorR.voltage) / batteryVolt, 1);

	MotorL.pwm_timer_val = MotorL.duty_ratio * TIM8->ARR;
	MotorR.pwm_timer_val = MotorR.duty_ratio * TIM8->ARR;

	// 정방향: PH = 0, 역방향: PH = 1
	HAL_GPIO_WritePin(MOTORL_PH_GPIO_Port, MOTORL_PH_Pin, !DirL);
	HAL_GPIO_WritePin(MOTORR_PH_GPIO_Port, MOTORR_PH_Pin, DirR);

	__HAL_TIM_SET_COMPARE(MOTOR_TIM, MOTORL_CHANNEL, MotorL.pwm_timer_val);
	__HAL_TIM_SET_COMPARE(MOTOR_TIM, MOTORR_CHANNEL, MotorR.pwm_timer_val);
}

void Encoder_Test() {
	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535);
	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535);
	uint8_t sw = 0;
	while (1) {
		sw = Custom_Switch_Read();
		Custom_LCD_Printf(0, 0, "L %5d", LPTIM2->CNT);
		Custom_LCD_Printf(0, 2, "R %5u", LPTIM1->CNT);
		if (sw == CUSTOM_JS_L_TO_R) {
			break;
		}

	}
}

//void Left_PI_Both_Change() {
//	uint8_t sw = CUSTOM_JS_NONE;
//	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535);
//	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535);
//	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
//	Battery_Start();
//	Motor_Start();
//	while (1) {
//		sw = Custom_Switch_Read();
////		Custom_LCD_Printf(0, 0, "%9f", MotorL.gainP);
////		Custom_LCD_Printf(0, 1, "%9f", MotorL.gainI);
//		Custom_LCD_Printf(0, 3, "%f", MotorL.CurrEncVal);
//		Custom_LCD_Printf(0, 4, "%9d ", MotorL.EncDiff); //0
//		Custom_LCD_Printf(0, 5, "%f", MotorL.rad_per_sec); //0
//		Custom_LCD_Printf(0, 6, "%9f", MotorL.rad_per_sec_cmd);
//		Custom_LCD_Printf(0, 7, "%9d", MotorL.Duty);
//		Custom_LCD_Printf(0, 8, "%9d", LPTIM2->CNT);
//		Custom_LCD_Printf(0, 9, "%f", batteryVolt);
////		Custom_LCD_Printf(0, 9, "down to back");
//		if (sw == CUSTOM_JS_L_TO_R) {
//			MotorL.gainP += 0.01;
//		} else if (sw == CUSTOM_JS_R_TO_L) {
//			MotorL.gainP -= 0.01;
//		} else if (sw == CUSTOM_JS_D_TO_U) {
//			MotorL.gainI += 1;
//		} else if (sw == CUSTOM_JS_U_TO_D) {
//			MotorL.gainI -= 1;
//		}
//
//	}
//
//	Motor_Stop();
//	Battery_Stop();
//	HAL_LPTIM_Encoder_Stop(MOTORR_ENCODER_TIMER);
//	HAL_LPTIM_Encoder_Stop(MOTORL_ENCODER_TIMER);
//}
void Motor_Left_Gain_Both() {
	uint8_t sw = CUSTOM_JS_NONE;
	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535);
	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
	Battery_Start();
	Motor_Start();
	while (1) {
		sw = Custom_Switch_Read();
		Custom_LCD_Printf(0, 0, "%9f", MotorL.gainP);
		Custom_LCD_Printf(0, 1, "%f", MotorL.gainI);
		Custom_LCD_Printf(0, 2, "encV %5d", MotorL.CurrEncVal);
		Custom_LCD_Printf(0, 3, "%9d ", MotorL.EncDiff); //0
		Custom_LCD_Printf(0, 4, "%f", MotorL.rad_per_sec); //0
		Custom_LCD_Printf(0, 5, "%9f", MotorL.rad_per_sec_cmd);
		Custom_LCD_Printf(0, 6, "%9d", MotorL.duty_ratio);
		Custom_LCD_Printf(0, 7, "%9d", LPTIM2->CNT);
		Custom_LCD_Printf(0, 8, "%f", batteryVolt);
		Custom_LCD_Printf(0, 9, "down to back");
		if (sw == CUSTOM_JS_L_TO_R) {
			MotorL.gainP += 0.01;
		} else if (sw == CUSTOM_JS_R_TO_L) {
			MotorL.gainP -= 0.005;
		} else if (sw == CUSTOM_JS_D_TO_U) {
			MotorL.gainI += 1;
		} else if (sw == CUSTOM_JS_U_TO_D) {
			MotorL.gainI -= 1;
		}

	}

	Motor_Stop();
	Battery_Stop();
	HAL_LPTIM_Encoder_Stop(MOTORR_ENCODER_TIMER);
	HAL_LPTIM_Encoder_Stop(MOTORL_ENCODER_TIMER);
}
void Motor_Left_Gain_P() {
	uint8_t sw = CUSTOM_JS_NONE;
	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535);
	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
	Battery_Start();
	Motor_Start();
	while (1) {
		sw = Custom_Switch_Read();
		Custom_LCD_Printf(0, 0, "%9f", MotorL.gainP);
		Custom_LCD_Printf(0, 1, "%f", MotorL.gainI);
		Custom_LCD_Printf(0, 2, "encV %5d", MotorL.CurrEncVal);
		Custom_LCD_Printf(0, 3, "%9d ", MotorL.EncDiff); //0
		Custom_LCD_Printf(0, 4, "%f", MotorL.rad_per_sec); //0
		Custom_LCD_Printf(0, 5, "%9f", MotorL.rad_per_sec_cmd);
		Custom_LCD_Printf(0, 6, "%9d", MotorL.duty_ratio);
		Custom_LCD_Printf(0, 7, "%9d", LPTIM2->CNT);
		Custom_LCD_Printf(0, 8, "%f", batteryVolt);
		Custom_LCD_Printf(0, 9, "down to back");
		if (sw == CUSTOM_JS_L_TO_R) {
			MotorL.gainP += 0.5;
		} else if (sw == CUSTOM_JS_R_TO_L) {
			MotorL.gainP -= 0.5;
		}
//			else if (sw == CUSTOM_JS_D_TO_U) {
//			MotorL.gainI += 1;
//		} else if (sw == CUSTOM_JS_U_TO_D) {
//			MotorL.gainI -= 1;
//		}

	}

	Motor_Stop();
	Battery_Stop();
	HAL_LPTIM_Encoder_Stop(MOTORR_ENCODER_TIMER);
	HAL_LPTIM_Encoder_Stop(MOTORL_ENCODER_TIMER);
}

void Motor_Left_Gain_I() {
	uint8_t sw = CUSTOM_JS_NONE;
	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535);
	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
	Battery_Start();
	Motor_Start();
	while (1) {
		sw = Custom_Switch_Read();
		Custom_LCD_Printf(0, 0, "%9f", MotorL.gainI);
		Custom_LCD_Printf(0, 1, "encV %5d", MotorL.CurrEncVal);
		Custom_LCD_Printf(0, 2, "%9d ", MotorL.EncDiff); //0
		Custom_LCD_Printf(0, 3, "%f", MotorL.rad_per_sec); //0
		Custom_LCD_Printf(0, 4, "%9f", MotorL.rad_per_sec_cmd);
		Custom_LCD_Printf(0, 5, "%9d", MotorL.duty_ratio);
		Custom_LCD_Printf(0, 6, "%9d", LPTIM2->CNT);
		Custom_LCD_Printf(0, 7, "%f", batteryVolt);
		Custom_LCD_Printf(0, 8, "down to back");
		if (sw == CUSTOM_JS_L_TO_R) {
			MotorL.gainI += 10.0f;
		} else if (sw == CUSTOM_JS_R_TO_L) {

			MotorL.gainI -= 10.0f;

		} else if (sw == CUSTOM_JS_U_TO_D) {
			Custom_LCD_Clear();
			break;
		}

	}

	Motor_Stop();
	Battery_Stop();
	HAL_LPTIM_Encoder_Stop(MOTORR_ENCODER_TIMER);
	HAL_LPTIM_Encoder_Stop(MOTORL_ENCODER_TIMER);
}

void Motor_Right_Gain_P() {
	uint8_t sw = CUSTOM_JS_NONE;
	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535);
	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
	Battery_Start();
	Motor_Start();
	while (1) {
		sw = Custom_Switch_Read();
		Custom_LCD_Printf(0, 0, "%9f", MotorR.gainP);
		Custom_LCD_Printf(0, 1, "encV %5d", MotorR.CurrEncVal);
		Custom_LCD_Printf(0, 2, "%9d ", MotorR.EncDiff); //0
		Custom_LCD_Printf(0, 3, "%f", MotorR.rad_per_sec); //0
		Custom_LCD_Printf(0, 4, "%9f", MotorR.rad_per_sec_cmd);
		Custom_LCD_Printf(0, 5, "%9d", MotorR.duty_ratio);
		Custom_LCD_Printf(0, 6, "%9d", MotorR.CurrEncVal);
		Custom_LCD_Printf(0, 7, "%f", batteryVolt);
		Custom_LCD_Printf(0, 8, "down to back");
		if (sw == CUSTOM_JS_L_TO_R) {
			MotorR.gainP += 0.01;
		} else if (sw == CUSTOM_JS_R_TO_L) {
			MotorR.gainP -= 0.01;
		} else if (sw == CUSTOM_JS_U_TO_D) {
			Custom_LCD_Clear();
			break;
		}

	}

	Motor_Stop();
	Battery_Stop();
	HAL_LPTIM_Encoder_Stop(MOTORR_ENCODER_TIMER);
	HAL_LPTIM_Encoder_Stop(MOTORL_ENCODER_TIMER);
}

void Motor_Right_Gain_I() {
	uint8_t sw = CUSTOM_JS_NONE;
	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535);
	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
	Battery_Start();
	Motor_Start();
	while (1) {
		sw = Custom_Switch_Read();
		Custom_LCD_Printf(0, 0, "%9f", MotorR.gainI);
		Custom_LCD_Printf(0, 1, "%5d", MotorR.CurrEncVal);
		Custom_LCD_Printf(0, 2, "%9d", MotorR.EncDiff); //0
		Custom_LCD_Printf(0, 3, "%9f", MotorR.rad_per_sec); //0
		Custom_LCD_Printf(0, 4, "%9f", MotorR.EncD);
		Custom_LCD_Printf(0, 5, "%9d", MotorR.duty_ratio);
		Custom_LCD_Printf(0, 6, "%9d", LPTIM1->CNT);
		Custom_LCD_Printf(0, 7, "%f", batteryVolt);
		Custom_LCD_Printf(0, 8, "down to back");
		if (sw == CUSTOM_JS_L_TO_R) {
			MotorR.gainI += 10.0f;
		} else if (sw == CUSTOM_JS_R_TO_L) {
			if (MotorR.gainI > 0) {
				MotorR.gainI -= 2.0f;
			}
		} else if (sw == CUSTOM_JS_U_TO_D) {
			Custom_LCD_Clear();
			break;
		}

	}

	Motor_Stop();
	Battery_Stop();
	HAL_LPTIM_Encoder_Stop(MOTORR_ENCODER_TIMER);
	HAL_LPTIM_Encoder_Stop(MOTORL_ENCODER_TIMER);
}
void Motor_Right_Gain_Both() {
	uint8_t sw = CUSTOM_JS_NONE;
	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535);
	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
	Battery_Start();
	Motor_Start();
	while (1) {
		sw = Custom_Switch_Read();
		Custom_LCD_Printf(0, 0, "%9f", MotorR.gainP);
		Custom_LCD_Printf(0, 1, "%f", MotorR.gainI);
		Custom_LCD_Printf(0, 2, "encV %5d", MotorR.CurrEncVal);
		Custom_LCD_Printf(0, 3, "%9d ", MotorR.EncDiff); //0
		Custom_LCD_Printf(0, 4, "%f", MotorR.rad_per_sec); //0
		Custom_LCD_Printf(0, 5, "%9f", MotorR.rad_per_sec_cmd);
		Custom_LCD_Printf(0, 6, "%9d", MotorL.duty_ratio);
		Custom_LCD_Printf(0, 7, "%9d", LPTIM1->CNT);
		Custom_LCD_Printf(0, 8, "%f", batteryVolt);
		Custom_LCD_Printf(0, 9, "down to back");
		if (sw == CUSTOM_JS_L_TO_R) {
			MotorR.gainP += 0.05;
		} else if (sw == CUSTOM_JS_R_TO_L) {
			MotorR.gainP -= 0.01;
		} else if (sw == CUSTOM_JS_D_TO_U) {
			MotorR.gainI += 10;
		} else if (sw == CUSTOM_JS_U_TO_D) {
			MotorR.gainI -= 1;
		}

	}

	Motor_Stop();
	Battery_Stop();
	HAL_LPTIM_Encoder_Stop(MOTORR_ENCODER_TIMER);
	HAL_LPTIM_Encoder_Stop(MOTORL_ENCODER_TIMER);
}

void Motor_Speed_Change() {
	currentVelocity = 0;
	uint8_t sw = CUSTOM_JS_NONE;
	targetVelocity = 0.5f;
	accel = accelSetting;
	decel = decelSetting;

	Battery_Start();
	Motor_Start();

	Drive_Start();

	while ((sw = Custom_Switch_Read()) != CUSTOM_JS_U_TO_D) {
		if (sw == CUSTOM_JS_L_TO_R) {
			targetVelocity += 0.1f;
		} else if (sw == CUSTOM_JS_R_TO_L) {
			targetVelocity -= 0.1f;
		}
		Custom_LCD_Printf(0, 2, " %d", MotorL.CurrEncVal);
		Custom_LCD_Printf(0, 3, " %d", MotorL.PastEncVal);
		Custom_LCD_Printf(0, 4, "%d", MotorL.EncDiff);
		Custom_LCD_Printf(0, 5, "EV %f", MotorL.rad_per_sec);
		Custom_LCD_Printf(0, 6, "%f", MotorL.EncD);
		Custom_LCD_Printf(0, 7, " %f", MotorL.rad_per_sec_cmd);
		Custom_LCD_Printf(0, 8, " %f", MotorL.duty_ratio);
		Custom_LCD_Printf(0, 9, "R %f", MotorL.rad_per_sec_err);
	}

	Motor_Stop();
	Battery_Stop();
}

void Motor_Speed_Test() {
	bool flag = 0;
	currentVelocity = 0.3;
	targetVelocity = 1.0f;
	Battery_Start();
	Motor_Start();
	Drive_Start();
	accel = accelSetting;
	decel = decelSetting;

	while (1) {
//		Custom_LCD_Printf(0, 0,"cv %f", currentVelocity);
//		Custom_LCD_Printf(0, 1, "tv %f", targetVelocity);
		if (flag) {
			targetVelocity += 0.01;
		} else {
			targetVelocity -= 0.01;
		}
		if (targetVelocity >= 2)
			flag = 0;
		else if (targetVelocity <= 1.0)
			flag = 1;

	}

}

void Motor_Test_76EHWAN() {

	Custom_LCD_Clear();

	uint8_t sw = CUSTOM_JS_NONE;

	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535);
	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535);

//	DRVOFF 핀 설정 0: 주행 / 1: 정지
	HAL_GPIO_WritePin(L_DRVOFF_GPIO_Port, L_DRVOFF_Pin, 0);
	HAL_GPIO_WritePin(R_DRVOFF_GPIO_Port, R_DRVOFF_Pin, 0);

//	nSLEEP 핀 설정 0: 절전 모드 / 1: 동작 모드
	HAL_GPIO_WritePin(L_nSLEEP_GPIO_Port, L_nSLEEP_Pin, 1);
	HAL_GPIO_WritePin(R_nSLEEP_GPIO_Port, R_nSLEEP_Pin, 1);

//	모터드라이버 안정화
	HAL_Delay(1);

//	모터가 Sleep 상태에서 Active 상태로 바뀔 때, 메인 보드에 동작 가능한 지 알려주기 위해 nFault 핀을 강제로 low로 내려보낸다.
//	nFault 핀이 low면 OUT1, 2가 HI-Z로 변해 모터가 돌지 않는다. 따라서 Active 하기 위해선 nFault를 reset해야 한다.
//	Reset 방법은 nSleep 핀에 pulse를 넣으면 되는데 pulse의 low 시간에 따라 동작이 바뀐다.
//	주석 읽었으면 이거

//	t_RESET min: 5us / t_RESET max: 20us
//	t_SLEEP min: 40us / t_SLEEP max: 140

//	pulse의 low time 세팅
//	0 				~	t_RESET min	: Clear Fault -> NO 			/ Sleep -> NO
//	t_RESET min		~	t_RESET max : Clear Fault -> Indeterminate 	/ Sleep -> NO
//	t_RESET max		~	t_SLEEP min : Clear Fault -> YES			/ Sleep -> NO
//	t_SLEEP min		~	t_SLEEP max : Clear Fault -> YES			/ Sleep -> Indeterminate
//	t_SLEEP max		~				: Clear Fault -> YES			/ Sleep -> YES

//	따라서 nFault에 알맞는 low 시간은 20us 이상 40us 미만이다. 따라서 nSleep 핀을 low로 보낸 뒤, 30us 뒤에 high로 놓으면 된다.
	bool nFault = true;

//	nSleep핀 low로 보내기
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_SET);
	while (nFault) {
		nFault = !(HAL_GPIO_ReadPin(L_nFAULT_GPIO_Port, L_nFAULT_Pin))
				| !(HAL_GPIO_ReadPin(L_nFAULT_GPIO_Port, L_nFAULT_Pin));
		HAL_GPIO_WritePin(L_nSLEEP_GPIO_Port, L_nSLEEP_Pin, 0);
		HAL_GPIO_WritePin(R_nSLEEP_GPIO_Port, R_nSLEEP_Pin, 0);

//	30us 기다리기
		DWT_Delay_us(30);

//	30us 이후 다시 nSleep핀 high로 보내기
		HAL_GPIO_WritePin(L_nSLEEP_GPIO_Port, L_nSLEEP_Pin, 1);
		HAL_GPIO_WritePin(R_nSLEEP_GPIO_Port, R_nSLEEP_Pin, 1);

		HAL_Delay(1);
	}

//	문제 해결
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Start_IT(MOTOR_TIM, MOTORR_CHANNEL);
	HAL_TIM_PWM_Start_IT(MOTOR_TIM, MOTORL_CHANNEL);

	__HAL_TIM_SET_COMPARE(MOTOR_TIM, MOTORR_CHANNEL, 3000);
	__HAL_TIM_SET_COMPARE(MOTOR_TIM, MOTORL_CHANNEL, 3000);

	while (1) {
		Custom_LCD_Printf(0, 0, "%5d", MOTORR_ENCODER_TIMER.Instance->CNT);
		Custom_LCD_Printf(0, 1, "%5d", MOTORL_ENCODER_TIMER.Instance->CNT);
		Custom_LCD_Printf(0, 3, "%f", MotorL.rad_per_sec * MOTOR_KT);

		sw = Custom_Switch_Read();
		if (sw == CUSTOM_JS_D_TO_U) {

			HAL_GPIO_WritePin(L_DRVOFF_GPIO_Port, L_DRVOFF_Pin, 1);
			HAL_GPIO_WritePin(R_DRVOFF_GPIO_Port, R_DRVOFF_Pin, 1);
			HAL_GPIO_WritePin(L_nSLEEP_GPIO_Port, L_nSLEEP_Pin, 0);
			HAL_GPIO_WritePin(R_nSLEEP_GPIO_Port, R_nSLEEP_Pin, 0);
			break;

		}

	}
	HAL_LPTIM_Encoder_Stop(MOTORR_ENCODER_TIMER);
	HAL_LPTIM_Encoder_Stop(MOTORL_ENCODER_TIMER);
}

void Left_Motor_Test_Phase() {

	HAL_LPTIM_Encoder_Start(MOTORR_ENCODER_TIMER, 65535);
	HAL_LPTIM_Encoder_Start(MOTORL_ENCODER_TIMER, 65535);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
	Battery_Start();
	Motor_Start();
	while (1) {
//		HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
		uint8_t sw = Custom_Switch_Read();
		while (1) {
			sw = Custom_Switch_Read();
			if (sw == CUSTOM_JS_D_TO_U) {
				break;
			} else if (sw == CUSTOM_JS_L_TO_R) {
				MotorL.gainP * 2;
			} else if (sw == CUSTOM_JS_R_TO_L) {
				MotorL.gainP / 2;
			}
			Custom_LCD_Printf(0, 0, "%9f", MotorL.gainP);
			Custom_LCD_Printf(0, 1, "%5d", MotorL.CurrEncVal);
			Custom_LCD_Printf(0, 2, "%9d", MotorL.EncDiff); //0
			Custom_LCD_Printf(0, 3, "%9f", MotorL.rad_per_sec); //0
			Custom_LCD_Printf(0, 4, "%9f", MotorL.voltage);
			Custom_LCD_Printf(0, 5, "%9d", MotorL.duty_ratio);
			Custom_LCD_Printf(0, 6, "%f", batteryVolt);
			Custom_LCD_Printf(0, 7, "%f", MotorL.rad_per_sec * MOTOR_KT);
		}

		HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 1);
	}

	Motor_Stop();
	Battery_Stop();
	HAL_LPTIM_Encoder_Stop(MOTORR_ENCODER_TIMER);
	HAL_LPTIM_Encoder_Stop(MOTORL_ENCODER_TIMER);
}
