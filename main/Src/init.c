/*
 * init.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#include "init.h"
#include "lcd.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"
#include "setting.h"
#include "LSM6DS3TR.h"

#include "../../MDK-ARM/Inc/custom_switch.h"
#include "../../MDK-ARM/Inc/lptim.h"
#include "../../MDK-ARM/Inc/tim.h"

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

//menu_t menu2[] = { { "1. Cali ",
//		Sensor_Calibration }, { "2. first ", Drive_First },  { "3. settings",Setting_Menu },{ "tv 1.3 ",
//				target1_3pitin_58 }, { "tv 1.5 ",
//						target1_5pitin_48},{ "1.7",target1_7pitin_38 },{"tv 1.8",target1_8pitin_6},
//,

menu_t menu[] = { { "1. cali  ", Sensor_Calibration }, { "2. first  ", Drive_First },
		{ "zyro", LSM6DS3TR_C_Init }, { "4. secD Menu   ",
				Drive_Second_fast_menu }, { "5. pit in", Change_Pit_In }, { "6.curve  ",
				Change_curve_rate }, { "final slow", safety_targetV1 }, {
				"setting", Setting_Menu } };

void safety_targetV1() {
	uint8_t sw = 0;
	while (1) {
		sw = Custom_Switch_Read();
		Custom_LCD_Printf(0,1,"tv 1");
		if (sw == CUSTOM_JS_D_TO_U) {
		pitInLine = 0.12;
		targetVelocitySetting = 1.0;

		} else if (sw == CUSTOM_JS_U_TO_D) {

		} else if (sw == CUSTOM_JS_L_TO_R) {
			break;
		}
	}
}

void PresetTV35() {
	uint8_t sw = 0;
	while (1) {
		sw = Custom_Switch_Read();
		if (sw == CUSTOM_JS_L_TO_R) {
			break;
		}

	}
	targetVelocitySetting = 3.5f;
//	curveRate =0.000064f;

}

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
	float batteryPercent = (batteryVolt-19.2f) / 8.f;
	if (batteryPercent > 0.75) {
		percentColor = 0x06A0;
	} else if (batteryPercent > 0.3) {
		percentColor = 0xFFE0;
	} else {
		percentColor = 0xF800;
	}
	Custom_LCD_Printf(1, 9, "%f", batteryVolt);
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
//	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
//	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	Battery_Start();
//	Button_Test();
	uint32_t selected_index = 0;
	uint32_t sw = CUSTOM_JS_NONE;
	uint32_t numOfMenu = sizeof(menu) / sizeof(menu_t);
//	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	while (1) {
//		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
//			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
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
				Battery_Start();
			}
			Set_Color(numOfMenu, selected_index);

		}
		//Custom_LCD_Printf(0, 0, "Main Menu");
//

//		HAL_LPTIM_Encoder_Stop(&hlptim1);
//
		HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
//		HAL_LPTIM_Encoder_Start(&hlptim1, 65535);

	}
}
