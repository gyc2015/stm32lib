#include <stm32f103rc_rcc.h>
#include <stm32f103rc_flash.h>

/*
 * SystemInit - ϵͳ��ʼ��,�ɻ���ļ�startup����
 *
 * HSE��PLLʱ��Դ, SYSCLK = 72MHz
 * HCLK = 72MHz, PCLK2 = 72MHz, PCLK1 = 36MHz
 * ������Ԥ��ȡ����,���������ȡ���
 *
 * ע: �����ⲿʱ��Ϊ 8MHz
 */
void SystemInit(void)
{
    RCC->CR |= RCC_CR_HSION;
    // �������á����ơ��жϼĴ���
    RCC->CFGR &= ~(RCC_CFGR_SW | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_ADCPRE | RCC_CFGR_MCO
                 | RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL | RCC_CFGR_USBPRE);
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON | RCC_CR_HSEBYP);
    RCC->CIR = (RCC_CIR_ALLC & (~RCC_CIE_ALLE));
    // ���ⲿ����ʱ��
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    // ������Ԥ��ȡ����,���������ȡ����
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;
    // ����PLLʱ��Դ������ʱ�ӷ�Ƶ
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSEDIV1 | RCC_CFGR_PLLMULL9;
    // ��PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    // ��PLL�����Ϊϵͳʱ��Դ SYSCLK = 9 * HSE
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
