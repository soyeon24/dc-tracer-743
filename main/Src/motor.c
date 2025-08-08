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
#include "init.h"
#include "lcd.h"
#include <stdbool.h>
#include "dwt_delay.h"
#include "setting.h"
#include "../../MDK-ARM/Inc/custom_switch.h"
#include "../../MDK-ARM/Inc/gpio.h"
#include "../../MDK-ARM/Inc/lptim.h"
#include "../../MDK-ARM/Inc/tim.h"

#define ABS(x) ((x>0) ? x:(-x))
#define MIN(a, b) ((a > b) ? b : a )
#define  ENCODER_PULSE_PER_REVOLUTION 4096.f
#define GEAR (63.f/17.f)
#define WHEEL 0.035f //4cm라 가정
#define RADIUS (WHEEL/2)
#define TIME 0.0005
#define PI M_PI
#define TICK_PER_METER (GEAR/(WHEEL*PI))	// 바퀴랑 곱해서 틱으로 변환
#define ANGLE_PER_TICK (1.0f/ ENCODER_PULSE_PER_REVOLUTION)
#define ANGLE_PER_METER (TICK_PER_METER * ANGLE_PER_TICK)
#define RADIAN_PER_SEC (2.f * PI / TIME / ENCODER_PULSE_PER_REVOLUTION)
#define MOTOR_RES 0.68f//모터래지스터
#define MOTOR_KE 0.0146f//모터역기전력 토크상수
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

void Motor_Start() {
	MotorL.gainP = 0.45f; //0.22; //0.13f;//0.175f;//0.48f;//1.2f;//0.48f;//0.005f;//1.23f; //1.46
	MotorL.gainI =500.0f; //390.0f;//420.0f;//102.4f;//300.0f;//0.1f; //300.0f;

	MotorR.gainP = 0.4f; //0.23;//0.9f;//0.8f;// 0.8f; //0.97
	MotorR.gainI = 500.0f; //400.0f;//0.0f;//

	MotorL.CurrEncVal = 0; //현재 엔코더
	MotorL.PastEncVal = 0; //이전 엔코더
	MotorL.EncDiff = 0;	// 현재 - 이전 엔코더 == 거리 차이
	MotorL.EncV = 0; //앤코더 변화 속도 (=각속도)
	MotorL.EncD = 0; // 엔코더 변화 거리(=속도의 시간대비 적분값)
	MotorL.ComV = 0; //커맨드 속도 (=목표 속도)
	MotorL.ComD = 0; //커맨드 위치 (=목표 거리)
	MotorL.v = 0; //바퀴속도
	MotorL.Duty = 0;
	MotorL.ErrV = 0;
	MotorL.CurPI = 0;
	MotorL.VoltPI = 0;
	MotorL.Integral = 0;
	MotorL.currentTick = 0;

	MotorR.CurrEncVal = 0; //현재 엔코더
	MotorR.PastEncVal = 0; //이전 엔코더
	MotorR.EncDiff = 0;	// 현재 - 이전 엔코더 == 거리 차이
	MotorR.EncV = 0; //앤코더 변화 속도 (=각속도)
	MotorR.EncD = 0; // 엔코더 변화 거리(=속도의 시간대비 적분값)
	MotorR.ComV = 0; //커맨드 속도 (=목표 속도)
	MotorR.ComD = 0; //커맨드 위치 (=목표 거리)
	MotorR.v = 0; //바퀴속도
	MotorR.Duty = 0;
	MotorR.ErrV = 0;
	MotorR.CurPI = 0;
	MotorR.VoltPI = 0;
	MotorR.Integral = 0;
	MotorR.currentTick = 0;
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

menu_t motor_menu[] = { { "enc", Encoder_Test },
		{ "speed T", Motor_Speed_Change },
		{ "1.left g ", Motor_Left_Gain_Both }, { "2.Right PI",
				Motor_Right_Gain_Both }, { "Left PI", Motor_Left_Gain_P }, {
				"PWM Test", Motor_Test_76EHWAN }, { "1.left gI ",
				Motor_Left_Gain_I }, { "back menu", Back_To_Menu } };

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
//int kkk = 0;
void Motor_LPTIM4_IRQ() {
//	kkk++;

	//input
	MotorL.CurrEncVal = (uint16_t) LPTIM2->CNT;	//1은 왼쪽
	MotorR.CurrEncVal = (uint16_t) -LPTIM1->CNT;	//2는 오른쪽 감소

	MotorL.EncDiff = (int16_t) MotorL.CurrEncVal - (int16_t) MotorL.PastEncVal;
	MotorR.EncDiff = (int16_t) MotorR.CurrEncVal - (int16_t) MotorR.PastEncVal;

	MotorL.PastEncVal = MotorL.CurrEncVal;
	MotorR.PastEncVal = MotorR.CurrEncVal;

	MotorL.EncV = ((float) MotorL.EncDiff) / TIME * ANGLE_PER_TICK;
	MotorR.EncV = ((float) MotorR.EncDiff) / TIME * ANGLE_PER_TICK; //각속도

	MotorL.ComV = MotorL.v * TICK_PER_METER;
	MotorR.ComV = MotorR.v * TICK_PER_METER; // tick per sec

//	MotorL.wheel_rad_per_sec_cmd = MotorL.v / RADIUS;
//	MotorR.wheel_rad_per_sec_cmd = MotorR.v / RADIUS;

	MotorL.ErrV = MotorL.ComV - MotorL.EncV;
	MotorR.ErrV = MotorR.ComV - MotorR.EncV;

	// I
	MotorL.Integral += MotorL.ErrV * TIME;
	MotorR.Integral += MotorR.ErrV * TIME;
	/*주기를 2KHz에서 1KHz로 바꾸었다. 주기가 너무 빨라서 떨리는 현상이 발생할 수 있다. 다시 2kHz로 바꿈*/
	MotorL.CurPI = MotorL.Integral * MotorL.gainI + MotorL.ErrV * MotorL.gainP;
	MotorR.CurPI = MotorR.Integral * MotorR.gainI + MotorR.ErrV * MotorR.gainP;

	MotorL.VoltPI = MotorL.CurPI * MOTOR_RES;
	MotorR.VoltPI = MotorR.CurPI * MOTOR_RES;

	bool DirL = MotorL.VoltPI > 0;
	bool DirR = MotorR.VoltPI > 0;

	MotorL.VoltPI = MIN(ABS(MotorL.VoltPI), MOTOR_BATTERY_MAX);
	MotorL.VoltPI = MIN(batteryVolt, MotorL.VoltPI);
	MotorR.VoltPI = MIN(ABS(MotorR.VoltPI), MOTOR_BATTERY_MAX);
	MotorR.VoltPI = MIN(batteryVolt, MotorR.VoltPI);

	MotorL.Duty = TIM8->ARR * MotorL.VoltPI / batteryVolt;
	MotorR.Duty = TIM8->ARR * MotorR.VoltPI / batteryVolt;

	MotorL.currentTick += (int32_t) MotorL.EncDiff;
	MotorR.currentTick += (int32_t) MotorR.EncDiff;

	// 정방향: PH = 0, 역방향: PH = 1
	HAL_GPIO_WritePin(MOTORL_PH_GPIO_Port, MOTORL_PH_Pin, !DirL);
	HAL_GPIO_WritePin(MOTORR_PH_GPIO_Port, MOTORR_PH_Pin, DirR);

	__HAL_TIM_SET_COMPARE(MOTOR_TIM, MOTORL_CHANNEL, MotorL.Duty);
	__HAL_TIM_SET_COMPARE(MOTOR_TIM, MOTORR_CHANNEL, MotorR.Duty);

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
//		Custom_LCD_Printf(0, 5, "%f", MotorL.EncV); //0
//		Custom_LCD_Printf(0, 6, "%9f", MotorL.ComV);
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
		Custom_LCD_Printf(0, 4, "%f", MotorL.EncV); //0
		Custom_LCD_Printf(0, 5, "%9f", MotorL.ComV);
		Custom_LCD_Printf(0, 6, "%9d", MotorL.Duty);
		Custom_LCD_Printf(0, 7, "%9d", LPTIM2->CNT);
		Custom_LCD_Printf(0, 8, "%f", batteryVolt);
		Custom_LCD_Printf(0, 9, "down to back");
		if (sw == CUSTOM_JS_L_TO_R) {
			MotorL.gainP *= 2;
		} else if (sw == CUSTOM_JS_R_TO_L) {
			MotorL.gainP /=3;
		} else if (sw == CUSTOM_JS_D_TO_U) {
			MotorL.gainI *=2;
		} else if (sw == CUSTOM_JS_U_TO_D) {
			MotorL.gainI +=10;
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
		Custom_LCD_Printf(0, 4, "%f", MotorL.EncV); //0
		Custom_LCD_Printf(0, 5, "%9f", MotorL.ComV);
		Custom_LCD_Printf(0, 6, "%9d", MotorL.Duty);
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
		Custom_LCD_Printf(0, 3, "%f", MotorL.EncV); //0
		Custom_LCD_Printf(0, 4, "%9f", MotorL.ComV);
		Custom_LCD_Printf(0, 5, "%9d", MotorL.Duty);
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
		Custom_LCD_Printf(0, 3, "%f", MotorR.EncV); //0
		Custom_LCD_Printf(0, 4, "%9f", MotorR.ComV);
		Custom_LCD_Printf(0, 5, "%9d", MotorR.Duty);
		Custom_LCD_Printf(0, 6, "%9d", MotorR.CurrEncVal);
		Custom_LCD_Printf(0, 7, "%f", batteryVolt);
		Custom_LCD_Printf(0, 8, "down to back");
		if (sw == CUSTOM_JS_L_TO_R) {
			MotorR.gainP += 0.001;
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
		Custom_LCD_Printf(0, 3, "%9f", MotorR.EncV); //0
		Custom_LCD_Printf(0, 4, "%9f", MotorR.EncD);
		Custom_LCD_Printf(0, 5, "%9d", MotorR.Duty);
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
		Custom_LCD_Printf(0, 4, "%f", MotorR.EncV); //0
		Custom_LCD_Printf(0, 5, "%9f", MotorR.ComV);
		Custom_LCD_Printf(0, 6, "%9d", MotorL.Duty);
		Custom_LCD_Printf(0, 7, "%9d", LPTIM1->CNT);
		Custom_LCD_Printf(0, 8, "%f", batteryVolt);
		Custom_LCD_Printf(0, 9, "down to back");
		if (sw == CUSTOM_JS_L_TO_R) {
					MotorR.gainP *=2;
				} else if (sw == CUSTOM_JS_R_TO_L) {
					MotorR.gainP /=3;
				} else if (sw == CUSTOM_JS_D_TO_U) {
					MotorR.gainI *=2;
				} else if (sw == CUSTOM_JS_U_TO_D) {
					MotorR.gainI +=10;
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
		Custom_LCD_Printf(0, 0, "tv %f", targetVelocity);
		Custom_LCD_Printf(0, 1, "cv %f", currentVelocity);
		Custom_LCD_Printf(0, 2, "L %f", MotorL.v);
		Custom_LCD_Printf(0, 3, "EV %f", MotorL.EncV);
		Custom_LCD_Printf(0, 4, "R %f", MotorR.v);
		Custom_LCD_Printf(0, 5, "EV %f", MotorR.EncV);
//		Custom_LCD_Printf(0, 6, "%d", kkk);
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
		Custom_LCD_Printf(0, 3, "%f", MotorL.EncV * MOTOR_KE);

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

