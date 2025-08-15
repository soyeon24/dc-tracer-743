/*
 * init.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#ifndef MAIN_INC_INIT_H_
#define MAIN_INC_INIT_H_

#include <stdint.h>
#include "main.h"

void Init();
void Custom_LCD_Clear();
void Custom_LCD_Printf(int x, int y, const char *text, ...);
void Battery_LCD_Picture();

void PresetTV35();
void safety_targetV1();
typedef struct {
	char name[10];
	void (*func)(void);
} menu_t;

uint32_t get_selected_idx(uint32_t item_cnt);

#endif /* MAIN_INC_INIT_H_ */
