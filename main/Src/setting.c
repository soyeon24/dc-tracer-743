/*
 * setting.c
 *
 *  Created on: Aug 1, 2025
 *      Author: psybe
 */
#include "init.h"
#include "lcd.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"
#include "setting.h"

#include "../../MDK-ARM/Inc/custom_switch.h"
#include "../../MDK-ARM/Inc/lptim.h"
#include "../../MDK-ARM/Inc/tim.h"

#define SETTING_MENU_CNT 8

menu_t settingMenu[] = { {"1. target V",Change_Target_Velocity }, {"2. threshold", Change_Threshold},{"3. pit in",Change_Pit_In},{"6.drive M",Drive_Test_Menu}, { "7.S menu ", Sensor_Test_Menu }, { "8.M menu ",
		Motor_Test_Menu } ,{"back", Back_To_Menu}};

void target1_3pitin_58() {
	Custom_LCD_Clear();
	uint8_t sw = CUSTOM_JS_NONE;
	while (1) {
		Custom_LCD_Printf(0, 0, "tv %f",targetVelocitySetting);
		Custom_LCD_Printf(0, 1,"pit in 0.058");
		Custom_LCD_Printf(0, 3, "down");
		sw = Custom_Switch_Read();
		if(sw == CUSTOM_JS_L_TO_R){
			targetVelocitySetting = 1.3;
						pitInLine =0.058;
			break;
		}
	}
}

void target1_5pitin_48() {
	Custom_LCD_Clear();
	uint8_t sw = CUSTOM_JS_NONE;
	while (1) {
		Custom_LCD_Printf(0, 0, "tv %f",targetVelocitySetting);
		Custom_LCD_Printf(0, 1,"pit in 0.048");
		Custom_LCD_Printf(0, 3, "down");
		sw = Custom_Switch_Read();
		if(sw == CUSTOM_JS_L_TO_R){
			targetVelocitySetting = 1.5;
						pitInLine =0.048;
			break;
		}
	}
}

void target1_8pitin_6() {
	Custom_LCD_Clear();
	uint8_t sw = CUSTOM_JS_NONE;
	while (1) {
		Custom_LCD_Printf(0, 0, "tv %f",targetVelocitySetting);
		Custom_LCD_Printf(0, 1,"pit in 0.006");
		Custom_LCD_Printf(0, 3, "down");
		sw = Custom_Switch_Read();
		if(sw == CUSTOM_JS_L_TO_R){
			targetVelocitySetting = 1.8;
						pitInLine =0.006;
			break;
		}
	}
}
void target1_7pitin_38() {
	Custom_LCD_Clear();
	uint8_t sw = CUSTOM_JS_NONE;
	while (1) {
		Custom_LCD_Printf(0, 0, "tv %f",targetVelocitySetting);
		Custom_LCD_Printf(0, 1,"pit in 0.038");
		Custom_LCD_Printf(0, 3, "down");
		sw = Custom_Switch_Read();
		 if(sw == CUSTOM_JS_L_TO_R){
			targetVelocitySetting = 1.7;
						pitInLine =0.038;
			break;
		}
	}
}





void Change_curve_rate(){
	Custom_LCD_Clear();
	uint8_t sw = CUSTOM_JS_NONE;
	while (1) {
		Custom_LCD_Printf(0, 0, "%f", curveRate);
		sw = Custom_Switch_Read();
		if(sw== CUSTOM_JS_D_TO_U){
			curveRate+= 0.000001f;
		}
		else if(sw ==CUSTOM_JS_U_TO_D){
			curveRate -= 0.000001f;
		}
		else if(sw == CUSTOM_JS_L_TO_R){
			break;
		}
	}
}

void Change_peak_v(){
	Custom_LCD_Clear();
		uint8_t sw = CUSTOM_JS_NONE;
		while (1) {
			Custom_LCD_Printf(0, 0, "%f", peakVelocity);
			sw = Custom_Switch_Read();
			if(sw== CUSTOM_JS_D_TO_U){
				peakVelocity+=0.5;
			}
			else if(sw ==CUSTOM_JS_U_TO_D){
				peakVelocity-= 0.1f;
			}
			else if(sw == CUSTOM_JS_L_TO_R){
				break;
			}
		}
}
void Change_curveDecel(){
	Custom_LCD_Clear();
		uint8_t sw = CUSTOM_JS_NONE;
		while (1) {
			Custom_LCD_Printf(0, 0, "%f", curveDecel);
			sw = Custom_Switch_Read();
			if(sw== CUSTOM_JS_D_TO_U){
				curveRate+= 500.f;
			}
			else if(sw ==CUSTOM_JS_U_TO_D){
				curveRate -=1000.f;
			}
			else if(sw == CUSTOM_JS_L_TO_R){
				break;
			}
		}
}


void Change_accelSetting(){
	Custom_LCD_Clear();
	uint8_t sw = CUSTOM_JS_NONE;
	while (1) {
		Custom_LCD_Printf(0, 0, "%f", targetVelocitySetting);
		sw = Custom_Switch_Read();
		if(sw== CUSTOM_JS_D_TO_U){
			accelSetting+= 0.1;
		}
		else if(sw ==CUSTOM_JS_U_TO_D){
			accelSetting -= 0.1;
		}
		else if(sw == CUSTOM_JS_L_TO_R){
			break;
		}
	}
}

void Change_decelSetting(){
	Custom_LCD_Clear();
	uint8_t sw = CUSTOM_JS_NONE;
	while (1) {
		Custom_LCD_Printf(0, 0, "%f", decelSetting);
		sw = Custom_Switch_Read();
		if(sw== CUSTOM_JS_D_TO_U){
			decelSetting+= 0.1;
		}
		else if(sw ==CUSTOM_JS_U_TO_D){
			decelSetting -= 0.1;
		}
		else if(sw == CUSTOM_JS_L_TO_R){
			break;
		}
	}
}

void Change_Target_Velocity() {
	Custom_LCD_Clear();
	uint8_t sw = CUSTOM_JS_NONE;
	while (1) {
		Custom_LCD_Printf(0, 0, "%f", targetVelocitySetting);
		sw = Custom_Switch_Read();
		if(sw== CUSTOM_JS_D_TO_U){
			targetVelocitySetting += 0.1;
		}
		else if(sw ==CUSTOM_JS_U_TO_D){
			targetVelocitySetting -= 0.1;
		}
		else if(sw == CUSTOM_JS_L_TO_R){
			break;
		}
	}
}

void Change_Threshold() {
	Custom_LCD_Clear();
	uint8_t sw = CUSTOM_JS_NONE;
	while (1) {
		Custom_LCD_Printf(0, 0, "%d",sensorThreshold);
		sw = Custom_Switch_Read();
		if(sw== CUSTOM_JS_D_TO_U){
			sensorThreshold += 1;
		}
		else if(sw ==CUSTOM_JS_U_TO_D){
			sensorThreshold -= 1;
		}
		else if(sw == CUSTOM_JS_L_TO_R){
			break;
		}
	}
}


void Change_Pit_In() {
	Custom_LCD_Clear();
	uint8_t sw = CUSTOM_JS_NONE;
	while (1) {
		Custom_LCD_Printf(0, 0, "%f",pitInLine);
		Custom_LCD_Printf(0, 1, "up 0.001");

		Custom_LCD_Printf(0, 2, "down -0.001");
		Custom_LCD_Printf(0, 3, "left +0.005");
//		Custom_LCD_Printf(0,4 , "");
		sw = Custom_Switch_Read();
		if(sw== CUSTOM_JS_D_TO_U){
			pitInLine += 0.001;
		}
		else if(sw ==CUSTOM_JS_U_TO_D){
			pitInLine -= 0.001f;
		}
		else if(sw == CUSTOM_JS_L_TO_R){
			break;
		}
		else if(sw == CUSTOM_JS_R_TO_L){
			pitInLine +=0.005f;
				}
	}
}



void Setting_Menu_Print(uint32_t index) {

	for (uint32_t i = 0; i < SETTING_MENU_CNT; i++) {
		Set_Color(index, i);
		Custom_LCD_Printf(0, i + 1, "%s", (settingMenu + i)->name);

	}
}

void Back_To_Menu() {
	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
		Custom_LCD_Printf(0, 0, "want to");
		Custom_LCD_Printf(0, 1, "go menu?");
		Custom_LCD_Printf(0, 3, "yes:");
		Custom_LCD_Printf(0, 4, "press");
	}

	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
	}

	Custom_LCD_Clear();
	Init();
}

void Setting_Menu() {


	uint32_t selected_index = 0;
	uint32_t sw = CUSTOM_JS_NONE;
	uint32_t numOfMenu = sizeof(settingMenu) / sizeof(menu_t);

	while (1) {
		POINT_COLOR = WHITE;
		BACK_COLOR = BLACK;
		while (1) {
			Custom_LCD_Printf(0, 0, "Main Menu");
			Battery_LCD_Picture();
			Set_Color(numOfMenu, selected_index);
			Setting_Menu_Print(selected_index);
			sw = Custom_Switch_Read();
			if (sw == CUSTOM_JS_U_TO_D) {
				selected_index++;
				selected_index = (selected_index % 8);
			} else if (sw == CUSTOM_JS_D_TO_U) {
				selected_index--;
				selected_index = (selected_index % 8);
			} else if (sw == CUSTOM_JS_L_TO_R) {
				Custom_LCD_Clear();
				Battery_Stop();
				(settingMenu + selected_index)->func();
			}
			Set_Color(numOfMenu, selected_index);

		}

	}
}


//void Buzzer_TIM15_IRQ(){
//	if(sensorState&~Window.center){
//		__HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2, TIM15->ARR/5 );
//	}
//}
//void Buzzer_Start(){
//	HAL_TIM_PWM_Start_IT(&htim15, TIM_CHANNEL_2);
//}
