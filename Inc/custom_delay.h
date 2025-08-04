/**
 * custom_delay.h
 * STM32H7 HAL 기반 Delay 헤더
 */

#ifndef CUSTOM_DELAY_H_
#define CUSTOM_DELAY_H_

#include "stm32h7xx_hal.h"

/**
 * @brief SysTick은 HAL_Init() 시 자동으로 1 ms 증가하도록 설정되므로
 *        별도 초기화는 필요 없습니다.
 */
static inline void Custom_Delay_Init_SysTick(void)
{
    // HAL_Init()에서 이미 초기화됨
}

/**
 * @brief  경과 시간(ms) 반환
 * @retval ms 단위 타임스탬프
 */
static inline uint32_t Custom_Delay_Get_SysTick(void)
{
    return HAL_GetTick();
}

/**
 * @brief  ms 지연
 * @param  millisecond 지연 시간(ms)
 */
static inline void Custom_Delay_ms(uint32_t millisecond)
{
    HAL_Delay(millisecond);
}

#endif /* CUSTOM_DELAY_H_ */
