/*
 * custom_switch.h
 *
 * Modified on: July 22, 2025
 *      Author: Joonho Gwon
 *    Modifier: Soyeon Park
 */

#ifndef INC_CUSTOM_SWITCH_H_
#define INC_CUSTOM_SWITCH_H_

#include "stm32h7xx_hal.h"
#include "custom_delay.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * User Settings: 스위치가 연결된 포트와 핀
 * (active-low, 버튼 누르면 GND → GPIO_PIN_RESET)
 */
#define SWL_PORT GPIO_PIN_
#define SW1_PIN  GPIO_PIN_4
#define SW2_PORT GPIOB
#define SW2_PIN  GPIO_PIN_5

#define CUSTOM_JS_NONE 0x00
#define CUSTOM_JS_L_TO_R    0x01
#define CUSTOM_JS_R_TO_L    0x02
#define CUSTOM_JS_D_TO_U    0x04
#define CUSTOM_JS_U_TO_D    0x08

uint8_t Custom_Switch_Read(void);

#endif /* INC_CUSTOM_SWITCH_H_ */
