/*
 * setting.c
 *
 *  Created on: Aug 1, 2025
 *      Author: psybe
 */
#include "init.h"
#include "lcd.h"
#include "tim.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"
#include "lptim.h"
#include "setting.h"

#include "custom_switch.h"

#define SETTING_MENU_CNT 8

menu_t settingMenu[] = { {"target V",Change_Target_Velocity },{"Pit in",Change_Pit_In}, {"threshold", Change_Threshold},{"back", Back_To_Menu}};

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
		Custom_LCD_Printf(0, 0, "%d",pitInLine);
		sw = Custom_Switch_Read();
		if(sw== CUSTOM_JS_D_TO_U){
			pitInLine += 0.1f;
		}
		else if(sw ==CUSTOM_JS_U_TO_D){
			pitInLine -= 0.1f;
		}
		else if(sw == CUSTOM_JS_L_TO_R){
			break;
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
