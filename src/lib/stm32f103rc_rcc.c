#include <stm32f103rc_rcc.h>

/*
 * rcc_switch_apb2_periph_clock - 切换APB2外设的时钟
 *
 * @p: 外设,支持位选
 * @on: 0-关闭,非零-打开
 */
void rcc_switch_apb2_periph_clock(uint32 p, int on) {
    assert(is_apb2(p));

    if (on)
        RCC->APB2ENR |= p;
    else
        RCC->APB2ENR &= ~p;
}

/*
 * rcc_switch_apb1_periph_clock - 切换APB1外设的时钟
 *
 * @p: 外设,支持位选
 * @on: 0-关闭,非零-打开
 */
void rcc_switch_apb1_periph_clock(uint32 p, int on) {
    assert(is_apb1(p));

    if (0 == on)
        RCC->APB1ENR &= ~p;
    else
        RCC->APB1ENR |= p;
}

/************************************************/
/* 系统时钟状态
*/
static uint8 AHBAPBPrescTable[16] = { 0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9 };
static uint8 ADCPrescTable[4] = { 2, 4, 6, 8 };
/*
 * rcc_get_sysclk_freq - 获取系统时钟
 *
 * return: 系统时钟Hz
 */
uint32 rcc_get_sysclk_freq(void) {
    uint32 src, pllmul, pllsrc;

    src = RCC->CFGR & RCC_CFGR_SWS;
    switch (src) {
    case RCC_CFGR_SWS_HSE:
        return HSE_FREQ;
    case RCC_CFGR_SWS_HSI:
        return HSI_FREQ;
    case RCC_CFGR_SWS_PLL:
        pllmul = RCC->CFGR & RCC_CFGR_PLLMULL;
        pllmul = (pllmul >> 18) + 2;
        pllsrc = RCC->CFGR & RCC_CFGR_PLLSRC_Mask;

        switch (pllsrc) {
        case RCC_CFGR_PLLSRC_HSIDiv2:
            return (HSI_FREQ >> 1) * pllmul;
        case RCC_CFGR_PLLSRC_HSEDIV1:
            return HSE_FREQ * pllmul;
        case RCC_CFGR_PLLSRC_HSEDIV2:
            return (HSE_FREQ >> 1) * pllmul;
        default:
            return HSI_FREQ;
        }
    default:
        return HSI_FREQ;
    }
}
/*
 * rcc_get_clocks_freq - 获取系统各种时钟
 *
 * @clocks: 填充数据结构
 */
void rcc_get_clocks_freq(rcc_clocks_t *clocks) {
    uint32 tmp, presc;

    clocks->SYSCLK = rcc_get_sysclk_freq();

    tmp = RCC->CFGR & RCC_CFGR_HPRE;
    tmp = tmp >> 4;
    presc = AHBAPBPrescTable[tmp];
    clocks->HCLK = clocks->SYSCLK >> presc;

    tmp = RCC->CFGR & RCC_CFGR_PPRE1;
    tmp = tmp >> 8;
    presc = AHBAPBPrescTable[tmp];
    clocks->PCLK1 = clocks->HCLK >> presc;

    tmp = RCC->CFGR & RCC_CFGR_PPRE2;
    tmp = tmp >> 11;
    presc = AHBAPBPrescTable[tmp];
    clocks->PCLK2 = clocks->HCLK >> presc;

    tmp = RCC->CFGR & RCC_CFGR_ADCPRE;
    tmp = tmp >> 14;
    presc = ADCPrescTable[tmp];
    clocks->ADCCLK = clocks->PCLK2 >> presc;
}
