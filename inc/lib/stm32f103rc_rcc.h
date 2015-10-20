/*
* �������������¼�֮һ���ᵼ��ϵͳ��λ
* 1. NRST���ŵĵ͵�ƽ(�ⲿ��λ�ź�)
* 2. ���ڿ��Ź���λ(WWDG)
* 3. �������Ź���λ(IWDG)
* 4. �����λ(SW)
* 5. �͵�������(Low-power management reset)
* ����/״̬(Control/Status, RCC_CSR)�Ĵ�����־�˸�λԴ
*/

/*
* ��������ʱ�ӿ�������ϵͳʱ��(system clock, SYSCLK):
* - �ڲ����پ���(HSI oscillator clock)
* - �ⲿ���پ���(HSE oscillator clock)
* - ���໷ʱ��(PLL clock)
*
* �豸�������������ָ���ʱ��:
* - 40kHz�����ڲ�RC��ʱ��(LSI RC),�����������Ź������ڴ�Stop/Standbyģʽ�Զ����ѵ�RTC
* - 32.768kHz�����ⲿ����(LSE crystal),����real-time clock(RTCCLK)
*
* Ϊ��ʡ��,ÿ��ʱ�Ӷ����Զ����Ŀ���
*
* �������µ�����(Peripheral)���е�����ʱ�Ӷ�����SYSCLK�ṩ:
* - �����̽ӿ�ʱ��(flash memory programming interface clock, FLITFCLK),��HSI����
* - USB OTG FS 48MHz��PLL VCO(2*PLLCK)�Ϳɱ�̷�Ƶ��(divide by 3 or 2)����
* - I2S2��I2S3
* - ��̫����
*/


#ifndef STM32F103RC_RCC_H
#define STM32F103RC_RCC_H

#include <stm32f103rc.h>

typedef struct {
    volatile uint32 CR;         /* ʱ�ӿ���, Clock Control */
    volatile uint32 CFGR;       /* ʱ������, Clock Configuration */
    volatile uint32 CIR;        /* ʱ���ж�, Clock Interrupt */
    volatile uint32 APB2RSTR;   /* APB2���踴λ, APB2 Peripheral Reset */
    volatile uint32 APB1RSTR;   /* APB1���踴λ, APB1 Peripheral Reset */
    volatile uint32 AHBENR;     /* AHB����ʱ��ʹ��, AHB Peripheral clock enable */
    volatile uint32 APB2ENR;    /* APB2����ʱ��ʹ��, APB2 Peripheral clock enable */
    volatile uint32 APB1ENR;    /* APB1����ʱ��ʹ��, APB1 Peripheral clock enable */
    volatile uint32 BDCR;       /* ����, Backup Domain */
    volatile uint32 CSR;        /* ����/״̬�Ĵ���, Contrl/Status */
} rcc_regs_t;

/* RCC�Ĵ�����ַӳ�� */
#define RCC_BASE (AHB_BASE + 0x1000)
/* RCC�Ĵ���ָ����� */
#define RCC ((rcc_regs_t *)RCC_BASE)

/*
 * ʱ�ӿ��ƼĴ��� RCC_CR
 * ƫ�Ƶ�ַ: 0x00
 * ��λֵ: 0x0000 XX83
 * ����: �޵ȴ�״̬, word/half-word/byte����
 */
#define RCC_CR_RESET_VALUE  ((uint32)0x00000083)        /* ��λֵ           */
#define RCC_CR_HSION        ((uint32)0x00000001)        /* �ڲ�����ʱ�� ʹ�� */
#define RCC_CR_HSIRDY       ((uint32)0x00000002)        /* �ڲ�����ʱ�� ���� */
#define RCC_CR_HSITRIM      ((uint32)0x000000F8)        /* �ڲ�����ʱ�� ���� */
#define RCC_CR_HSICAL       ((uint32)0x0000FF00)        /* �ڲ�����ʱ�� У׼ */
#define RCC_CR_HSEON        ((uint32)0x00010000)        /* �ⲿ����ʱ�� ʹ�� */
#define RCC_CR_HSERDY       ((uint32)0x00020000)        /* �ⲿ����ʱ�� ���� */
#define RCC_CR_HSEBYP       ((uint32)0x00040000)        /* �ⲿ����ʱ�� ��· */
#define RCC_CR_CSSON        ((uint32)0x00080000)        /* ʱ�Ӱ�ȫϵͳ ʹ�� */
#define RCC_CR_PLLON        ((uint32)0x01000000)        /* PLL         ʹ�� */
#define RCC_CR_PLLRDY       ((uint32)0x02000000)        /* PLL         ���� */
/*
 * ʱ�����üĴ��� RCC_CFGR
 * ƫ�Ƶ�ַ: 0x04
 * ��λֵ: 0x0000 0000
 * ����: �����ʷ�����ʱ���л�ʱ����1���������ȴ�����, word/half-word/byte����
 */
#define RCC_CFGR_RESET_VALUE    ((uint32)0x00000000)    /* ��λֵ */

#define RCC_CFGR_SW             ((uint32)0x00000003)    /* ϵͳʱ��Դѡ��λ SW[1:0]*/
#define RCC_CFGR_SW_HSI         ((uint32)0x00000000)    /* �ڲ�����ʱ�� */
#define RCC_CFGR_SW_HSE         ((uint32)0x00000001)    /* �ⲿ����ʱ�� */
#define RCC_CFGR_SW_PLL         ((uint32)0x00000002)    /* PLL */

#define RCC_CFGR_SWS            ((uint32)0x0000000C)    /* ��ǰϵͳʱ�� SWS[1:0] */
#define RCC_CFGR_SWS_HSI        ((uint32)0x00000000)    /* �ڲ�����ʱ�� */
#define RCC_CFGR_SWS_HSE        ((uint32)0x00000004)    /* �ⲿ����ʱ�� */
#define RCC_CFGR_SWS_PLL        ((uint32)0x00000008)    /* PLL */

#define RCC_CFGR_HPRE           ((uint32)0x000000F0)    /* AHB��Ƶ HPRE[3:0],��SYSCLK,HCLK */
#define RCC_CFGR_HPRE_DIV1      ((uint32)0x00000000)    /* 1��Ƶ */
#define RCC_CFGR_HPRE_DIV2      ((uint32)0x00000080)    /* 2��Ƶ */
#define RCC_CFGR_HPRE_DIV4      ((uint32)0x00000090)    /* 4��Ƶ */
#define RCC_CFGR_HPRE_DIV8      ((uint32)0x000000A0)    /* 8��Ƶ */
#define RCC_CFGR_HPRE_DIV16     ((uint32)0x000000B0)    /* 16��Ƶ */
#define RCC_CFGR_HPRE_DIV64     ((uint32)0x000000C0)    /* 64��Ƶ */
#define RCC_CFGR_HPRE_DIV128    ((uint32)0x000000D0)    /* 128��Ƶ */
#define RCC_CFGR_HPRE_DIV256    ((uint32)0x000000E0)    /* 256��Ƶ */
#define RCC_CFGR_HPRE_DIV512    ((uint32)0x000000F0)    /* 512��Ƶ */

#define RCC_CFGR_PPRE1          ((uint32)0x00000700)    /* ��������(APB1)��Ƶ PRE1[2:0],��HCLK,���Ϊ36MHz,PCLK1 */
#define RCC_CFGR_PPRE1_DIV1     ((uint32)0x00000000)    /* 1��Ƶ */
#define RCC_CFGR_PPRE1_DIV2     ((uint32)0x00000400)    /* 2��Ƶ */
#define RCC_CFGR_PPRE1_DIV4     ((uint32)0x00000500)    /* 4��Ƶ */
#define RCC_CFGR_PPRE1_DIV8     ((uint32)0x00000600)    /* 8��Ƶ */
#define RCC_CFGR_PPRE1_DIV16    ((uint32)0x00000700)    /* 16��Ƶ */

#define RCC_CFGR_PPRE2          ((uint32)0x00003800)    /* ��������(APB2)��Ƶ PRE2[2:0],��HCLK,���Ϊ72MHz,PCLK2 */
#define RCC_CFGR_PPRE2_DIV1     ((uint32)0x00000000)    /* 1��Ƶ */
#define RCC_CFGR_PPRE2_DIV2     ((uint32)0x00002000)    /* 2��Ƶ */
#define RCC_CFGR_PPRE2_DIV4     ((uint32)0x00002800)    /* 4��Ƶ */
#define RCC_CFGR_PPRE2_DIV8     ((uint32)0x00003000)    /* 8��Ƶ */
#define RCC_CFGR_PPRE2_DIV16    ((uint32)0x00003800)    /* 16��Ƶ */

#define RCC_CFGR_ADCPRE         ((uint32)0x0000C000)    /* ADC��Ƶ ADCPRE[1:0], ��PCLK2,���Ϊ14MHz, ADCCLK */
#define RCC_CFGR_ADCPRE_DIV2    ((uint32)0x00000000)    /* 2 */
#define RCC_CFGR_ADCPRE_DIV4    ((uint32)0x00004000)    /* 4 */
#define RCC_CFGR_ADCPRE_DIV6    ((uint32)0x00008000)    /* 6 */
#define RCC_CFGR_ADCPRE_DIV8    ((uint32)0x0000C000)    /* 8 */

#define RCC_CFGR_PLLSRC         ((uint32)0x00010000)    /* PLLʱ��Դ,0:HSI��2��Ƶ��,1:HSE */
#define RCC_CFGR_PLLXTPRE       ((uint32)0x00020000)    /* 0:HSE����Ƶ,1:2��Ƶ */
#define RCC_CFGR_PLLSRC_Mask    ((uint32)0x00030000)
#define RCC_CFGR_PLLSRC_HSIDiv2 ((uint32)0x00000000)    /* HSI��2��Ƶ������PLL */
#define RCC_CFGR_PLLSRC_HSEDIV1 ((uint32)0x00010000)    /* HSE����Ƶ����PLL */
#define RCC_CFGR_PLLSRC_HSEDIV2 ((uint32)0x00030000)    /* HSE��2��Ƶ������PLL */

#define RCC_CFGR_PLLMULL        ((uint32)0x003C0000)    /* PLL��Ƶ PLLMUL[3:0] */
#define RCC_CFGR_PLLMULL2       ((uint32)0x00000000)    /* 2 */
#define RCC_CFGR_PLLMULL3       ((uint32)0x00040000)    /* 3 */
#define RCC_CFGR_PLLMULL4       ((uint32)0x00080000)    /* 4 */
#define RCC_CFGR_PLLMULL5       ((uint32)0x000C0000)    /* 5 */
#define RCC_CFGR_PLLMULL6       ((uint32)0x00100000)    /* 6 */
#define RCC_CFGR_PLLMULL7       ((uint32)0x00140000)    /* 7 */
#define RCC_CFGR_PLLMULL8       ((uint32)0x00180000)    /* 8 */
#define RCC_CFGR_PLLMULL9       ((uint32)0x001C0000)    /* 9 */
#define RCC_CFGR_PLLMULL10      ((uint32)0x00200000)    /* 10 */
#define RCC_CFGR_PLLMULL11      ((uint32)0x00240000)    /* 11 */
#define RCC_CFGR_PLLMULL12      ((uint32)0x00280000)    /* 12 */
#define RCC_CFGR_PLLMULL13      ((uint32)0x002C0000)    /* 13 */
#define RCC_CFGR_PLLMULL14      ((uint32)0x00300000)    /* 14 */
#define RCC_CFGR_PLLMULL15      ((uint32)0x00340000)    /* 15 */
#define RCC_CFGR_PLLMULL16      ((uint32)0x00380000)    /* 16 */

#define RCC_CFGR_USBPRE         ((uint32)0x00400000)    /* USB�豸��Ƶ */
#define RCC_CFGR_USBPRE_Div1_5  ((uint32)0x00000000)    /* PLLʱ�ӵ�1.5��Ƶ */
#define RCC_CFGR_USBPRE_DIV1    ((uint32)0x00400000)    /* PLLʱ�ӵ�1��Ƶ */

#define RCC_CFGR_MCO            ((uint32)0x07000000)    /* MCOʱ�����ѡ��,MCO[2:0],��ΪGPIO���ŵ�����,������Ƶ��Ϊ50MHz */
#define RCC_CFGR_MCO_NOCLOCK    ((uint32)0x00000000)    /* ����� */
#define RCC_CFGR_MCO_SYSCLK     ((uint32)0x04000000)    /* SYSCLK */
#define RCC_CFGR_MCO_HSI        ((uint32)0x05000000)    /* HSIʱ�� */
#define RCC_CFGR_MCO_HSE        ((uint32)0x06000000)    /* HSEʱ�� */
#define RCC_CFGR_MCO_PLL        ((uint32)0x07000000)    /* PLLʱ��2��Ƶ */
 /*
  * ʱ���жϼĴ��� RCC_CIR
  * ƫ�Ƶ�ַ: 0x08
  * ��λֵ: 0x0000 0000
  * ����: û�еȴ�״̬, word/half-word/byte����
  */
#define RCC_CIR_LSIRDYF         ((uint32)0x00000001)        /* LSIʱ�Ӿ����ж�,LSIRDYIE=1ʱ��Ӳ����1,���д1��LSIRDYCλ��� */
#define RCC_CIR_LSERDYF         ((uint32)0x00000002)        /* LSEʱ�Ӿ����ж�,LSERDYIE=1ʱ��Ӳ����1,���д1��LSERDYCλ��� */
#define RCC_CIR_HSIRDYF         ((uint32)0x00000004)        /* HSIʱ�Ӿ����ж�,HSIRDYIE=1ʱ��Ӳ����1,���д1��HSIRDYCλ��� */
#define RCC_CIR_HSERDYF         ((uint32)0x00000008)        /* HSEʱ�Ӿ����ж�,HSERDYIE=1ʱ��Ӳ����1,���д1��HSERDYCλ��� */
#define RCC_CIR_PLLRDYF         ((uint32)0x00000010)        /* PLLʱ�Ӿ����ж�,PLLRDYIE=1ʱ��Ӳ����1,���д1��PLLRDYCλ��� */
#define RCC_CIR_CSSF            ((uint32)0x00000080)        /* ʱ�Ӱ�ȫϵͳ�жϱ�־,HSEʧЧʱӲ����1,���д1��CSSCλ��� */
#define RCC_CIR_LSIRDYIE        ((uint32)0x00000100)        /* LSI�����ж�ʹ�� */
#define RCC_CIR_LSERDYIE        ((uint32)0x00000200)        /* LSE�����ж�ʹ�� */
#define RCC_CIR_HSIRDYIE        ((uint32)0x00000400)        /* HSI�����ж�ʹ�� */
#define RCC_CIR_HSERDYIE        ((uint32)0x00000800)        /* HSE�����ж�ʹ�� */
#define RCC_CIR_PLLRDYIE        ((uint32)0x00001000)        /* PLL�����ж�ʹ�� */
#define RCC_CIR_LSIRDYC         ((uint32)0x00010000)        /* ���LSI�����ж� */
#define RCC_CIR_LSERDYC         ((uint32)0x00020000)        /* ���LSE�����ж� */
#define RCC_CIR_HSIRDYC         ((uint32)0x00040000)        /* ���HSI�����ж� */
#define RCC_CIR_HSERDYC         ((uint32)0x00080000)        /* ���HSE�����ж� */
#define RCC_CIR_PLLRDYC         ((uint32)0x00100000)        /* ���PLL�����ж� */
#define RCC_CIR_CSSC            ((uint32)0x00800000)        /* ���ʱ�Ӱ�ȫϵͳ�ж� */

#define RCC_CIE_ALLE            ((uint32)0x00001F00)        /* �������ж� */
#define RCC_CIR_ALLC            ((uint32)0x009F0000)        /* ��������жϱ�־ */
 /*
  * ��������(APB2)��λ�Ĵ��� APB2RSTR
  * ƫ�Ƶ�ַ: 0x0C
  * ��λֵ: 0x0000 0000
  * ����: û�еȴ�״̬, word/half-word/byte����
  */
#define APB2RSTR_AFIORST    ((uint32)0x00000001)        /* ��������I/O��λ */
#define APB2RSTR_IOPARST    ((uint32)0x00000004)        /* IO�˿�A��λ */
#define APB2RSTR_IOPBRST    ((uint32)0x00000008)        /* IO�˿�B��λ */
#define APB2RSTR_IOPCRST    ((uint32)0x00000010)        /* IO�˿�C��λ */
#define APB2RSTR_IOPDRST    ((uint32)0x00000020)        /* IO�˿�D��λ */
#define APB2RSTR_IOPERST    ((uint32)0x00000040)        /* IO�˿�E��λ */
#define APB2RSTR_IOPFRST    ((uint32)0x00000080)        /* IO�˿�F��λ */
#define APB2RSTR_IOPGRST    ((uint32)0x00000100)        /* IO�˿�G��λ */
#define APB2RSTR_ADC1RST    ((uint32)0x00000200)        /* ADC1 */
#define APB2RSTR_ADC2RST    ((uint32)0x00000400)        /* ADC2 */
#define APB2RSTR_TIM1RST    ((uint32)0x00000800)        /* TIM1 */
#define APB2RSTR_SPI1RST    ((uint32)0x00001000)        /* SPI1 */
#define APB2RSTR_TIM8RST    ((uint32)0x00002000)        /* TIM8 */
#define APB2RSTR_USART1RST  ((uint32)0x00004000)        /* USART1 */
#define APB2RSTR_ADC3RST    ((uint32)0x00008000)        /* ADC3 */
#define APB2RSTR_TIM9RST    ((uint32)0x00080000)        /* TIM9 */
#define APB2RSTR_TIM10RST   ((uint32)0x00100000)        /* TIM10 */
#define APB2RSTR_TIM11RST   ((uint32)0x00200000)        /* TIM11 */
 /*
  * ��������(APB1)��λ�Ĵ��� APB1RSTR
  * ƫ�Ƶ�ַ: 0x10
  * ��λֵ: 0x0000 0000
  * ����: û�еȴ�״̬, word/half-word/byte����
  */
#define APB1RSTR_TIM2RST    ((uint32)0x00000001)        /* TIM2 */
#define APB1RSTR_TIM3RST    ((uint32)0x00000002)        /* TIM3 */
#define APB1RSTR_TIM4RST    ((uint32)0x00000004)        /* TIM4 */
#define APB1RSTR_TIM5RST    ((uint32)0x00000008)        /* TIM5 */
#define APB1RSTR_TIM6RST    ((uint32)0x00000010)        /* TIM6 */
#define APB1RSTR_TIM7RST    ((uint32)0x00000020)        /* TIM7 */
#define APB1RSTR_TIM12RST   ((uint32)0x00000040)        /* TIM12 */
#define APB1RSTR_TIM13RST   ((uint32)0x00000080)        /* TIM13 */
#define APB1RSTR_TIM14RST   ((uint32)0x00000100)        /* TIM14 */
#define APB1RSTR_WWDGRST    ((uint32)0x00000800)        /* WWDG */
#define APB1RSTR_SPI2RST    ((uint32)0x00004000)        /* SPI2 */
#define APB1RSTR_SPI3RST    ((uint32)0x00008000)        /* SPI3 */
#define APB1RSTR_USART2RST  ((uint32)0x00020000)        /* USART2 */
#define APB1RSTR_USART3RST  ((uint32)0x00040000)        /* USART3 */
#define APB1RSTR_USART4RST  ((uint32)0x00080000)        /* USART4 */
#define APB1RSTR_UART5RST   ((uint32)0x00100000)        /* UART5 */
#define APB1RSTR_I2C1RST    ((uint32)0x00200000)        /* I2C1 */
#define APB1RSTR_I2C2RST    ((uint32)0x00400000)        /* I2C2 */
#define APB1RSTR_USBRST     ((uint32)0x00800000)        /* USB */
#define APB1RSTR_CANRST     ((uint32)0x02000000)        /* CAN */
#define APB1RSTR_BKPRST     ((uint32)0x08000000)        /* Backup interface */
#define APB1RSTR_PWRRST     ((uint32)0x10000000)        /* Power interface */
#define APB1RSTR_DACRST     ((uint32)0x20000000)        /* DAC interface */
 /*
  * ��������(AHB)ʱ��ʹ�ܼĴ��� RCC_AHBENR
  * ƫ�Ƶ�ַ: 0x14
  * ��λֵ: 0x0000 0014
  * ����: û�еȴ�״̬, word/half-word/byte����
  */
#define RCC_AHBENR_DMA1EN       ((uint16)0x0001)            /* DMA1 */
#define RCC_AHBENR_DMA2EN       ((uint16)0x0002)            /* DMA2 */
#define RCC_AHBENR_SRAMEN       ((uint16)0x0004)            /* SRAM */
#define RCC_AHBENR_FLITFEN      ((uint16)0x0010)            /* FLITF */
#define RCC_AHBENR_CRCEN        ((uint16)0x0040)            /* CRC */
#define RCC_AHBENR_FSMCEN       ((uint16)0x0100)            /* FSMC */
#define RCC_AHBENR_SDIOEN       ((uint16)0x0400)            /* SDIO */
 /*
  * ������������(APB2)ʱ��ʹ�ܼĴ��� APB2ENR
  * ƫ�Ƶ�ַ: 0x18
  * ��λֵ: 0x0000 0000
  * ����: û�еȴ�״̬, word/half-word/byte����
  */
#define APB2ENR_AFIOEN      ((uint32)0x00000001)        /* ����I/Oʱ��ʹ�� */
#define APB2ENR_IOPAEN      ((uint32)0x00000004)        /* A */
#define APB2ENR_IOPBEN      ((uint32)0x00000008)        /* B */
#define APB2ENR_IOPCEN      ((uint32)0x00000010)        /* C */
#define APB2ENR_IOPDEN      ((uint32)0x00000020)        /* D */
#define APB2ENR_IOPEEN      ((uint32)0x00000040)        /* E */
#define APB2ENR_IOPFEN      ((uint32)0x00000080)        /* F */
#define APB2ENR_IOPGEN      ((uint32)0x00000100)        /* G */
#define APB2ENR_ADC1EN      ((uint32)0x00000200)        /* ADC1 */
#define APB2ENR_ADC2EN      ((uint32)0x00000400)        /* ADC2 */
#define APB2ENR_TIM1EN      ((uint32)0x00000800)        /* TIM1 */
#define APB2ENR_SPI1EN      ((uint32)0x00001000)        /* SPI1 */
#define APB2ENR_TIM8EN      ((uint32)0x00002000)        /* TIM8 */
#define APB2ENR_USART1EN    ((uint32)0x00004000)        /* USART1 */
#define APB2ENR_ADC3EN      ((uint32)0x00008000)        /* ADC3 */
#define APB2ENR_TIM9EN      ((uint32)0x00080000)        /* TIM9 */
#define APB2ENR_TIM10EN     ((uint32)0x00100000)        /* TIM10 */
#define APB2ENR_TIM11EN     ((uint32)0x00200000)        /* TIM11 */
/*
 * ������������(APB1)ʱ��ʹ�ܼĴ��� APB1ENR
 * ƫ�Ƶ�ַ: 0x1C
 * ��λֵ: 0x0000 0000
 * ����: û�еȴ�״̬, word/half-word/byte����
 */
#define APB1ENR_TIM2EN      ((uint32)0x00000001)        /* TIM2 */
#define APB1ENR_TIM3EN      ((uint32)0x00000002)        /* TIM3 */
#define APB1ENR_TIM4EN      ((uint32)0x00000004)        /* TIM4 */
#define APB1ENR_TIM5EN      ((uint32)0x00000008)        /* TIM5 */
#define APB1ENR_TIM6EN      ((uint32)0x00000010)        /* TIM6 */
#define APB1ENR_TIM7EN      ((uint32)0x00000020)        /* TIM7 */
#define APB1ENR_TIM12EN     ((uint32)0x00000040)        /* TIM12 */
#define APB1ENR_TIM13EN     ((uint32)0x00000080)        /* TIM13 */
#define APB1ENR_TIM14EN     ((uint32)0x00000100)        /* TIM14 */
#define APB1ENR_WWDGEN      ((uint32)0x00000800)        /* WWDG */
#define APB1ENR_SPI2EN      ((uint32)0x00004000)        /* SPI2 */
#define APB1ENR_SPI3EN      ((uint32)0x00008000)        /* SPI3 */
#define APB1ENR_USART2EN    ((uint32)0x00020000)        /* USART2 */
#define APB1ENR_USART3EN    ((uint32)0x00040000)        /* USART3 */
#define APB1ENR_UART4EN     ((uint32)0x00080000)        /* UART4 */
#define APB1ENR_UART5EN     ((uint32)0x00100000)        /* UART5 */ 
#define APB1ENR_I2C1EN      ((uint32)0x00200000)        /* I2C1 */
#define APB1ENR_I2C2EN      ((uint32)0x00400000)        /* I2C2 */
#define APB1ENR_USBEN       ((uint32)0x00800000)        /* USB */
#define APB1ENR_CANEN       ((uint32)0x02000000)        /* CAN */
#define APB1ENR_BKPEN       ((uint32)0x08000000)        /* Backup interface */
#define APB1ENR_PWREN       ((uint32)0x10000000)        /* Power interface */
#define APB1ENR_DACEN       ((uint32)0x20000000)        /* DAC interface */
/*
 * ���ݿ���(Backup Domain Control, BDC)�Ĵ��� RCC_BDCR
 * ƫ�Ƶ�ַ: 0x20
 * ��λֵ: 0x0000 0000
 * ����: 0��3���ȴ�����, word/half-word/byte����
 */
#define RCC_BDCR_LSEON          ((uint32)0x00000001)        /* �����ⲿ��������LSE */
#define RCC_BDCR_LSERDY         ((uint32)0x00000002)        /* LSE���� */
#define RCC_BDCR_LSEBYP         ((uint32)0x00000004)        /* LSE��·���� */

#define RCC_BDCR_RTCSEL         ((uint32)0x00000300)        /* RTCʱ��Դѡ�� RTCSEL[1:0] */
#define RCC_BDCR_RTCSEL_NOCLOCK ((uint32)0x00000000)        /* ����,˭������ */
#define RCC_BDCR_RTCSEL_LSE     ((uint32)0x00000100)        /* LSE */
#define RCC_BDCR_RTCSEL_LSI     ((uint32)0x00000200)        /* LSI */
#define RCC_BDCR_RTCSEL_HSE     ((uint32)0x00000300)        /* HSE��128��Ƶ�� */

#define RCC_BDCR_RTCEN          ((uint32)0x00008000)        /* RTCʱ��ʹ�� */
#define RCC_BDCR_BDRST          ((uint32)0x00010000)        /* Backup domain software reset  */
/*
 * ����/״̬(Control/Status, CS)�Ĵ��� RCC_CSR
 * ƫ�Ƶ�ַ: 0x24
 * ��λֵ: 0x0C00 0000
 * ����: 0��3���ȴ�����, word/half-word/byte����
 */
#define RCC_CSR_LSION           ((uint32)0x00000001)        /* ����LSI */
#define RCC_CSR_LSIRDY          ((uint32)0x00000002)        /* LSI���� */
#define RCC_CSR_RMVF            ((uint32)0x01000000)        /* �����λ��־ */
#define RCC_CSR_PINRSTF         ((uint32)0x04000000)        /* NRST���Ÿ�λ��־,Ӳ����1,���д1��RMVF��� */
#define RCC_CSR_PORRSTF         ((uint32)0x08000000)        /* �ϵ�/���縴λ��־,Ӳ����1,���д1��RMVF��� */
#define RCC_CSR_SFTRSTF         ((uint32)0x10000000)        /* �����λ��־,Ӳ����1,���д1��RMVF��� */
#define RCC_CSR_IWDGRSTF        ((uint32)0x20000000)        /* �������Ź���λ��־,Ӳ����1,���д1��RMVF��� */
#define RCC_CSR_WWDGRSTF        ((uint32)0x40000000)        /* ���ڿ��Ź���λ��־,Ӳ����1,���д1��RMVF��� */
#define RCC_CSR_LPWRRSTF        ((uint32)0x80000000)        /* �͹��ĸ�λ��־,Ӳ����1,���д1��RMVF���  */
/*
 * APH����
 */
#define AHB_DMA1        ((uint16)0x0001)
#define AHB_DMA2        ((uint16)0x0002)
#define AHB_SRAM        ((uint16)0x0004)
#define AHB_FLITF       ((uint16)0x0010)
#define AHB_CRC         ((uint16)0x0040)
#define AHB_FSMC        ((uint16)0x0100)
#define AHB_SDIO        ((uint16)0x0400)
#define AHB_ALL         ((uint16)0x0557)
#define is_ahb(p) ((0 != p) && (0 == ((p) & (~AHB_ALL))))
/*
 * APB2����
 */
#define APB2_AFIO       ((uint32)0x00000001)
#define APB2_GPIOA      ((uint32)0x00000004)
#define APB2_GPIOB      ((uint32)0x00000008)
#define APB2_GPIOC      ((uint32)0x00000010)
#define APB2_GPIOD      ((uint32)0x00000020)
#define APB2_GPIOE      ((uint32)0x00000040)
#define APB2_GPIOF      ((uint32)0x00000080)
#define APB2_GPIOG      ((uint32)0x00000100)
#define APB2_ADC1       ((uint32)0x00000200)
#define APB2_ADC2       ((uint32)0x00000400)
#define APB2_TIM1       ((uint32)0x00000800)
#define APB2_SPI1       ((uint32)0x00001000)
#define APB2_TIM8       ((uint32)0x00002000)
#define APB2_USART1     ((uint32)0x00004000)
#define APB2_ADC3       ((uint32)0x00008000)
#define APB2_TIM9       ((uint32)0x00080000)
#define APB2_TIM10      ((uint32)0x00100000)
#define APB2_TIM11      ((uint32)0x00200000)
#define APB2_ALL        ((uint32)0x0038FFFD)
#define is_apb2(p) ((0 != p) && (0 == ((p) & (~APB2_ALL))))
/*
 * APB1����
 */
#define APB1_TIM2       ((uint32)0x00000001)
#define APB1_TIM3       ((uint32)0x00000002)
#define APB1_TIM4       ((uint32)0x00000004)
#define APB1_TIM5       ((uint32)0x00000008)
#define APB1_TIM6       ((uint32)0x00000010)
#define APB1_TIM7       ((uint32)0x00000020)
#define APB1_TIM12      ((uint32)0x00000040)
#define APB1_TIM13      ((uint32)0x00000080)
#define APB1_TIM14      ((uint32)0x00000100)
#define APB1_WWDG       ((uint32)0x00000800)
#define APB1_SPI2       ((uint32)0x00004000)
#define APB1_SPI3       ((uint32)0x00008000)
#define APB1_USART2     ((uint32)0x00020000)
#define APB1_USART3     ((uint32)0x00040000)
#define APB1_UART4      ((uint32)0x00080000)
#define APB1_UART5      ((uint32)0x00100000)
#define APB1_I2C1       ((uint32)0x00200000)
#define APB1_I2C2       ((uint32)0x00400000)
#define APB1_USB        ((uint32)0x00800000)
#define APB1_CAN1       ((uint32)0x02000000)
#define APB1_BKP        ((uint32)0x08000000)
#define APB1_PWR        ((uint32)0x10000000)
#define APB1_DAC        ((uint32)0x20000000)
#define APB1_CEC        ((uint32)0x40000000)
#define APB1_ALL        ((uint32)0x7AFEC9FF)
#define is_apb1(p) ((0 != p) && (0 == ((p) & (~APB1_ALL))))

/*
 * rcc_switch_apb2_periph_clock - �л�APB2�����ʱ��
 *
 * @p: ����,֧��λѡ
 * @on: 0-�ر�,����-��
 */
void rcc_switch_apb2_periph_clock(uint32 p, int on);
/*
 * rcc_switch_apb1_periph_clock - �л�APB1�����ʱ��
 *
 * @p: ����,֧��λѡ
 * @on: 0-�ر�,����-��
 */
void rcc_switch_apb1_periph_clock(uint32 p, int on);

/************************************************/
/* ϵͳʱ��״̬
 */

/*
 * rcc_get_sysclk_freq - ��ȡϵͳʱ��
 *
 * return: ϵͳʱ��Hz
 */
uint32 rcc_get_sysclk_freq(void);

typedef struct {
    uint32 SYSCLK;  /* ϵͳʱ��Ƶ��,Hz */
    uint32 HCLK;    /* AHB��Ƶ���Ƶ��, Hz */
    uint32 PCLK1;   /* ��������APB1Ƶ��, Hz */
    uint32 PCLK2;   /* ��������APB2Ƶ��, Hz */
    uint32 ADCCLK;  /* ADת��Ƶ��, Hz */
} rcc_clocks_t;

/*
 * rcc_get_clocks_freq - ��ȡϵͳ����ʱ��
 *
 * @clocks: ������ݽṹ
 */
void rcc_get_clocks_freq(rcc_clocks_t *clocks);


#endif
