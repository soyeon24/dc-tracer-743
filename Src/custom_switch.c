/*
 * custom_switch.h
 *
 * Modified on: July 22, 2025
 *      Author: Joonho Gwon
 *    Modifier: Soyeon Park
 */

#include "custom_switch.h"

#define CUSTOM_SW_NONE 0x00
#define CUSTOM_SW_U    0x02
#define CUSTOM_SW_D    0x01
#define CUSTOM_SW_L    0x08
#define CUSTOM_SW_R    0x04

#define LONG_OFF   0x01
#define SHORT_ON   0x02
#define LONG_ON    0x04
#define SHORT_OFF  0x08

#define TIME_SHORT 80   // ms
#define TIME_LONG  300  // ms

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;       // HAL 핀 정의 (e.g. GPIO_PIN_4)
    uint32_t      timer;
    uint32_t      prevTick;
    uint8_t       state;
} ButtonState_t;

static void Custom_Switch_Init_ButtonState(ButtonState_t *State,
                                           GPIO_TypeDef *GPIOx, uint16_t Pin)
{
    State->port     = GPIOx;
    State->pin      = Pin;
    State->timer    = 0;
    State->prevTick = Custom_Delay_Get_SysTick();
    State->state    = LONG_OFF;
}

static uint8_t Custom_Switch_State_Machine(ButtonState_t *State)
{
    bool     currentPushed = (HAL_GPIO_ReadPin(State->port, State->pin) == GPIO_PIN_RESET);
    uint8_t  pushEvent     = 0;
    uint32_t currTick      = Custom_Delay_Get_SysTick();
    uint32_t elapsed       = currTick - State->prevTick;

    switch (State->state) {
        case LONG_OFF:
            if (currentPushed) {
                State->state = SHORT_ON;
                State->timer = TIME_SHORT;
            }
            break;

        case SHORT_ON:
            if (elapsed >= State->timer) {
                pushEvent     = 1;
                State->state  = LONG_ON;
                State->timer  = TIME_LONG;
            } else {
                State->timer -= elapsed;
            }
            break;

        case LONG_ON:
            if (!currentPushed) {
                State->state = SHORT_OFF;
                State->timer = TIME_SHORT;
                break;
            }
            if (elapsed >= State->timer) {
                pushEvent    = 1;
                State->timer = TIME_LONG;
            } else {
                State->timer -= elapsed;
            }
            break;

        case SHORT_OFF:
            if (elapsed >= State->timer) {
                State->state = LONG_OFF;
            } else {
                State->timer -= elapsed;
            }
            break;
    }

    State->prevTick = currTick;
    return pushEvent;
}

ButtonState_t btnSWU;
ButtonState_t btnSWD;
ButtonState_t btnSWL;
ButtonState_t btnSWR;

uint8_t Custom_Switch_Read(void)
{
    static bool initialized = false;
    static ButtonState_t swu, swd, swl, swr;

    if (!initialized) {
        initialized = true;
        Custom_Switch_Init_ButtonState(&swu, GPIOD, GPIO_PIN_1);  // SWU
        Custom_Switch_Init_ButtonState(&swl, GPIOD, GPIO_PIN_3);  // SWL
        Custom_Switch_Init_ButtonState(&swr, GPIOD, GPIO_PIN_4);  // SWR
        Custom_Switch_Init_ButtonState(&swd, GPIOD, GPIO_PIN_5);  // SWD
    }

    uint8_t evU = Custom_Switch_State_Machine(&swu);
    uint8_t evD = Custom_Switch_State_Machine(&swd);
    uint8_t evL = Custom_Switch_State_Machine(&swl);
    uint8_t evR = Custom_Switch_State_Machine(&swr);

    uint8_t result = CUSTOM_SW_NONE;
    if (evU) result |= CUSTOM_SW_U;
    if (evD) result |= CUSTOM_SW_D;
    if (evL) result |= CUSTOM_SW_L;
    if (evR) result |= CUSTOM_SW_R;

    return result;
}
