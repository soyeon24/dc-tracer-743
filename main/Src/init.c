/*
 * init.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
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

#define MAIN_MENU_CNT 8
//
//#define CUSTOM_JS_NONE 0x00
//#define CUSTOM_JS_L_TO_R    0x01
//#define CUSTOM_JS_R_TO_L    0x02
//#define CUSTOM_JS_D_TO_U    0x04
//#define CUSTOM_JS_U_TO_D    0x08

ST7735_Object_t hLCD;

#define CUSTOM_FLASH_SECTOR_ADDR  0x08040000  // 원하는 sector 시작 주소 (예: Bank 1, Sector 4)
#define CUSTOM_FLASH_SECTOR_SIZE  0x20000     // 128KB


void Setting() {

}

menu_t menu[] = { { "Cali ",
		Sensor_Calibration }, { "3.first ", Drive_First }, { "left gain ",
		Motor_Left_Gain_Both }, { "settings",Setting_Menu }, { "6.Setting ",
		Setting }, { "7.S menu ", Sensor_Test_Menu }, { "8.M menu ",
		Motor_Test_Menu } };

void Button_Test() {
	int i = 0;
	while (1) {
		Custom_LCD_Printf(0, 0, "%d", i);
		uint8_t sw = Custom_Switch_Read();
		if (sw == CUSTOM_JS_R_TO_L) {
			i--;

			Custom_LCD_Clear();
		} else if (sw == CUSTOM_JS_L_TO_R) {
			i++;
			Custom_LCD_Clear();
		} else if (sw == CUSTOM_JS_U_TO_D) {
			i = i / 2;
			Custom_LCD_Clear();
		} else if (sw == CUSTOM_JS_D_TO_U) {
			i = i * 2;
			Custom_LCD_Clear();
		}
	}

}

void Main_Menu_Print(uint32_t index) {

	for (uint32_t i = 0; i < MAIN_MENU_CNT; i++) {
		Set_Color(index, i);
		Custom_LCD_Printf(0, i + 1, "%s", (menu + i)->name);

	}
}

uint32_t percentColor = 0;
void Battery_LCD_Picture() {
	float batteryPercent = (batteryVolt) / 25.2;
	if (batteryPercent > 0.75) {
		percentColor = 0x06A0;
	} else if (batteryPercent > 0.3) {
		percentColor = 0xFFE0;
	} else {
		percentColor = 0xF800;
	}
	Custom_LCD_Printf(1, 9, "%.0f%%", batteryPercent * 100);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 41, 145, 1, 14, 0xFFFF);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 41, 145, 35, 1, 0xFFFF);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 41, 158, 35, 1, 0xFFFF);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 75, 145, 1, 14, 0xFFFF);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 76, 149, 1, 6, 0xFFFF);
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 43, 147,
			(uint32_t) 31 * batteryPercent, 10, percentColor);
	ST7735_LCD_Driver.FillRect(&st7735_pObj,
			44 + (uint32_t) (31 * batteryPercent), 147,
			31 - (uint32_t) (31 * batteryPercent), 10, BLACK);
}

uint32_t get_selected_idx(uint32_t item_cnt) {
	return ((hlptim1.Instance->CNT + 128) / 256) % item_cnt;
}

void Switch_Test() {
	int32_t i = 0;
	while (1) {
		Custom_LCD_Printf(0, 0, "%d", i);
		uint8_t sw = Custom_Switch_Read();
		if (sw == CUSTOM_JS_R_TO_L) {
			i--;

			Custom_LCD_Clear();
		} else if (sw == CUSTOM_JS_L_TO_R) {
			i++;
			Custom_LCD_Clear();
		} else if (sw == CUSTOM_JS_U_TO_D) {
			i = i / 2;
			Custom_LCD_Clear();
		} else if (sw == CUSTOM_JS_D_TO_U) {
			i = i * 2;
			Custom_LCD_Clear();
		}
	}
}

void Init() {

	Battery_Start();
//	Button_Test();
	uint32_t selected_index = 0;
	uint32_t sw = CUSTOM_JS_NONE;
	uint32_t numOfMenu = sizeof(menu) / sizeof(menu_t);

	while (1) {
		POINT_COLOR = WHITE;
		BACK_COLOR = BLACK;
		while (1) {
			Custom_LCD_Printf(0, 0, "Main Menu");
			Battery_LCD_Picture();
			Set_Color(numOfMenu, selected_index);
			Main_Menu_Print(selected_index);
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
				(menu + selected_index)->func();
			}
			Set_Color(numOfMenu, selected_index);

		}
		//Custom_LCD_Printf(0, 0, "Main Menu");
//

//		HAL_LPTIM_Encoder_Stop(&hlptim1);
//
//		HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
//		HAL_LPTIM_Encoder_Start(&hlptim1, 65535);

	}
}

