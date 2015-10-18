#include <stm32f103rc_rcc.h>
#include <stm32f103rc_flash.h>

/*
 * SystemInit - 系统初始化,由汇编文件startup调用
 *
 * HSE经PLL时钟源, SYSCLK = 72MHz
 * HCLK = 72MHz, PCLK2 = 72MHz, PCLK1 = 36MHz
 * 打开闪存预读取缓存,设置闪存读取间隔
 *
 * 注: 这里外部时钟为 8MHz
 */
void SystemInit(void)
{
    RCC->CR |= RCC_CR_HSION;
    // 重置配置、控制、中断寄存器
    RCC->CFGR &= ~(RCC_CFGR_SW | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_ADCPRE | RCC_CFGR_MCO
                 | RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL | RCC_CFGR_USBPRE);
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON | RCC_CR_HSEBYP);
    RCC->CIR = (RCC_CIR_ALLC & (~RCC_CIE_ALLE));
    // 开外部高速时钟
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    // 打开闪存预读取缓存,设置闪存读取周期
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;
    // 设置PLL时钟源、各个时钟分频
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSEDIV1 | RCC_CFGR_PLLMULL9;
    // 打开PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    // 以PLL输出作为系统时钟源 SYSCLK = 9 * HSE
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
