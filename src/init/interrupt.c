#include <stm32f103rc_gpio.h>
#include <stm32f103rc_rcc.h>
#include <stm32f103rc_usart.h>
#include <stm32f103rc_tim.h>

extern int gJiffies;

/*
 * TIM6_IRQHandler - Timer6
 */
void TIM6_IRQHandler(void) {
    if (TIM_SR_UIF & TIM6->SR) {
        TIM6->SR &= ~TIM_SR_UIF;
        gJiffies++;
    }
}
