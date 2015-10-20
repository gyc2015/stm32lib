/*
* 发生以下五种事件之一都会导致系统复位
* 1. NRST引脚的低电平(外部复位信号)
* 2. 窗口看门狗复位(WWDG)
* 3. 独立看门狗复位(IWDG)
* 4. 软件复位(SW)
* 5. 低电量管理(Low-power management reset)
* 控制/状态(Control/Status, RCC_CSR)寄存器标志了复位源
*/

/*
* 以下三种时钟可以用作系统时钟(system clock, SYSCLK):
* - 内部高速晶振(HSI oscillator clock)
* - 外部高速晶振(HSE oscillator clock)
* - 锁相环时钟(PLL clock)
*
* 设备还具有以下两种辅助时钟:
* - 40kHz低速内部RC振荡时钟(LSI RC),驱动独立看门狗和用于从Stop/Standby模式自动唤醒的RTC
* - 32.768kHz低速外部晶振(LSE crystal),驱动real-time clock(RTCCLK)
*
* 为了省电,每种时钟都可以独立的开关
*
* 除了如下的外设(Peripheral)所有的外设时钟都是由SYSCLK提供:
* - 闪存编程接口时钟(flash memory programming interface clock, FLITFCLK),由HSI驱动
* - USB OTG FS 48MHz由PLL VCO(2*PLLCK)和可编程分频器(divide by 3 or 2)驱动
* - I2S2和I2S3
* - 以太网卡
*/


#ifndef STM32F103RC_RCC_H
#define STM32F103RC_RCC_H

#include <stm32f103rc.h>

typedef struct {
    volatile uint32 CR;         /* 时钟控制, Clock Control */
    volatile uint32 CFGR;       /* 时钟配置, Clock Configuration */
    volatile uint32 CIR;        /* 时钟中断, Clock Interrupt */
    volatile uint32 APB2RSTR;   /* APB2外设复位, APB2 Peripheral Reset */
    volatile uint32 APB1RSTR;   /* APB1外设复位, APB1 Peripheral Reset */
    volatile uint32 AHBENR;     /* AHB外设时钟使能, AHB Peripheral clock enable */
    volatile uint32 APB2ENR;    /* APB2外设时钟使能, APB2 Peripheral clock enable */
    volatile uint32 APB1ENR;    /* APB1外设时钟使能, APB1 Peripheral clock enable */
    volatile uint32 BDCR;       /* 备份, Backup Domain */
    volatile uint32 CSR;        /* 控制/状态寄存器, Contrl/Status */
} rcc_regs_t;

/* RCC寄存器地址映射 */
#define RCC_BASE (AHB_BASE + 0x1000)
/* RCC寄存器指针访问 */
#define RCC ((rcc_regs_t *)RCC_BASE)

/*
 * 时钟控制寄存器 RCC_CR
 * 偏移地址: 0x00
 * 复位值: 0x0000 XX83
 * 访问: 无等待状态, word/half-word/byte访问
 */
#define RCC_CR_RESET_VALUE  ((uint32)0x00000083)        /* 复位值           */
#define RCC_CR_HSION        ((uint32)0x00000001)        /* 内部高速时钟 使能 */
#define RCC_CR_HSIRDY       ((uint32)0x00000002)        /* 内部高速时钟 就绪 */
#define RCC_CR_HSITRIM      ((uint32)0x000000F8)        /* 内部高速时钟 调整 */
#define RCC_CR_HSICAL       ((uint32)0x0000FF00)        /* 内部高速时钟 校准 */
#define RCC_CR_HSEON        ((uint32)0x00010000)        /* 外部高速时钟 使能 */
#define RCC_CR_HSERDY       ((uint32)0x00020000)        /* 外部高速时钟 就绪 */
#define RCC_CR_HSEBYP       ((uint32)0x00040000)        /* 外部高速时钟 旁路 */
#define RCC_CR_CSSON        ((uint32)0x00080000)        /* 时钟安全系统 使能 */
#define RCC_CR_PLLON        ((uint32)0x01000000)        /* PLL         使能 */
#define RCC_CR_PLLRDY       ((uint32)0x02000000)        /* PLL         就绪 */
/*
 * 时钟配置寄存器 RCC_CFGR
 * 偏移地址: 0x04
 * 复位值: 0x0000 0000
 * 访问: 当访问发生在时钟切换时会有1或者两个等待周期, word/half-word/byte访问
 */
#define RCC_CFGR_RESET_VALUE    ((uint32)0x00000000)    /* 复位值 */

#define RCC_CFGR_SW             ((uint32)0x00000003)    /* 系统时钟源选择位 SW[1:0]*/
#define RCC_CFGR_SW_HSI         ((uint32)0x00000000)    /* 内部高速时钟 */
#define RCC_CFGR_SW_HSE         ((uint32)0x00000001)    /* 外部高速时钟 */
#define RCC_CFGR_SW_PLL         ((uint32)0x00000002)    /* PLL */

#define RCC_CFGR_SWS            ((uint32)0x0000000C)    /* 当前系统时钟 SWS[1:0] */
#define RCC_CFGR_SWS_HSI        ((uint32)0x00000000)    /* 内部高速时钟 */
#define RCC_CFGR_SWS_HSE        ((uint32)0x00000004)    /* 外部高速时钟 */
#define RCC_CFGR_SWS_PLL        ((uint32)0x00000008)    /* PLL */

#define RCC_CFGR_HPRE           ((uint32)0x000000F0)    /* AHB分频 HPRE[3:0],对SYSCLK,HCLK */
#define RCC_CFGR_HPRE_DIV1      ((uint32)0x00000000)    /* 1分频 */
#define RCC_CFGR_HPRE_DIV2      ((uint32)0x00000080)    /* 2分频 */
#define RCC_CFGR_HPRE_DIV4      ((uint32)0x00000090)    /* 4分频 */
#define RCC_CFGR_HPRE_DIV8      ((uint32)0x000000A0)    /* 8分频 */
#define RCC_CFGR_HPRE_DIV16     ((uint32)0x000000B0)    /* 16分频 */
#define RCC_CFGR_HPRE_DIV64     ((uint32)0x000000C0)    /* 64分频 */
#define RCC_CFGR_HPRE_DIV128    ((uint32)0x000000D0)    /* 128分频 */
#define RCC_CFGR_HPRE_DIV256    ((uint32)0x000000E0)    /* 256分频 */
#define RCC_CFGR_HPRE_DIV512    ((uint32)0x000000F0)    /* 512分频 */

#define RCC_CFGR_PPRE1          ((uint32)0x00000700)    /* 低速外设(APB1)分频 PRE1[2:0],对HCLK,最大为36MHz,PCLK1 */
#define RCC_CFGR_PPRE1_DIV1     ((uint32)0x00000000)    /* 1分频 */
#define RCC_CFGR_PPRE1_DIV2     ((uint32)0x00000400)    /* 2分频 */
#define RCC_CFGR_PPRE1_DIV4     ((uint32)0x00000500)    /* 4分频 */
#define RCC_CFGR_PPRE1_DIV8     ((uint32)0x00000600)    /* 8分频 */
#define RCC_CFGR_PPRE1_DIV16    ((uint32)0x00000700)    /* 16分频 */

#define RCC_CFGR_PPRE2          ((uint32)0x00003800)    /* 高速外设(APB2)分频 PRE2[2:0],对HCLK,最大为72MHz,PCLK2 */
#define RCC_CFGR_PPRE2_DIV1     ((uint32)0x00000000)    /* 1分频 */
#define RCC_CFGR_PPRE2_DIV2     ((uint32)0x00002000)    /* 2分频 */
#define RCC_CFGR_PPRE2_DIV4     ((uint32)0x00002800)    /* 4分频 */
#define RCC_CFGR_PPRE2_DIV8     ((uint32)0x00003000)    /* 8分频 */
#define RCC_CFGR_PPRE2_DIV16    ((uint32)0x00003800)    /* 16分频 */

#define RCC_CFGR_ADCPRE         ((uint32)0x0000C000)    /* ADC分频 ADCPRE[1:0], 对PCLK2,最大为14MHz, ADCCLK */
#define RCC_CFGR_ADCPRE_DIV2    ((uint32)0x00000000)    /* 2 */
#define RCC_CFGR_ADCPRE_DIV4    ((uint32)0x00004000)    /* 4 */
#define RCC_CFGR_ADCPRE_DIV6    ((uint32)0x00008000)    /* 6 */
#define RCC_CFGR_ADCPRE_DIV8    ((uint32)0x0000C000)    /* 8 */

#define RCC_CFGR_PLLSRC         ((uint32)0x00010000)    /* PLL时钟源,0:HSI经2分频后,1:HSE */
#define RCC_CFGR_PLLXTPRE       ((uint32)0x00020000)    /* 0:HSE不分频,1:2分频 */
#define RCC_CFGR_PLLSRC_Mask    ((uint32)0x00030000)
#define RCC_CFGR_PLLSRC_HSIDiv2 ((uint32)0x00000000)    /* HSI经2分频后输入PLL */
#define RCC_CFGR_PLLSRC_HSEDIV1 ((uint32)0x00010000)    /* HSE不分频输入PLL */
#define RCC_CFGR_PLLSRC_HSEDIV2 ((uint32)0x00030000)    /* HSE经2分频后输入PLL */

#define RCC_CFGR_PLLMULL        ((uint32)0x003C0000)    /* PLL倍频 PLLMUL[3:0] */
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

#define RCC_CFGR_USBPRE         ((uint32)0x00400000)    /* USB设备分频 */
#define RCC_CFGR_USBPRE_Div1_5  ((uint32)0x00000000)    /* PLL时钟的1.5分频 */
#define RCC_CFGR_USBPRE_DIV1    ((uint32)0x00400000)    /* PLL时钟的1分频 */

#define RCC_CFGR_MCO            ((uint32)0x07000000)    /* MCO时钟输出选择,MCO[2:0],因为GPIO引脚的限制,最大输出频率为50MHz */
#define RCC_CFGR_MCO_NOCLOCK    ((uint32)0x00000000)    /* 不输出 */
#define RCC_CFGR_MCO_SYSCLK     ((uint32)0x04000000)    /* SYSCLK */
#define RCC_CFGR_MCO_HSI        ((uint32)0x05000000)    /* HSI时钟 */
#define RCC_CFGR_MCO_HSE        ((uint32)0x06000000)    /* HSE时钟 */
#define RCC_CFGR_MCO_PLL        ((uint32)0x07000000)    /* PLL时钟2分频 */
 /*
  * 时钟中断寄存器 RCC_CIR
  * 偏移地址: 0x08
  * 复位值: 0x0000 0000
  * 访问: 没有等待状态, word/half-word/byte访问
  */
#define RCC_CIR_LSIRDYF         ((uint32)0x00000001)        /* LSI时钟就绪中断,LSIRDYIE=1时由硬件置1,软件写1到LSIRDYC位清除 */
#define RCC_CIR_LSERDYF         ((uint32)0x00000002)        /* LSE时钟就绪中断,LSERDYIE=1时由硬件置1,软件写1到LSERDYC位清除 */
#define RCC_CIR_HSIRDYF         ((uint32)0x00000004)        /* HSI时钟就绪中断,HSIRDYIE=1时由硬件置1,软件写1到HSIRDYC位清除 */
#define RCC_CIR_HSERDYF         ((uint32)0x00000008)        /* HSE时钟就绪中断,HSERDYIE=1时由硬件置1,软件写1到HSERDYC位清除 */
#define RCC_CIR_PLLRDYF         ((uint32)0x00000010)        /* PLL时钟就绪中断,PLLRDYIE=1时由硬件置1,软件写1到PLLRDYC位清除 */
#define RCC_CIR_CSSF            ((uint32)0x00000080)        /* 时钟安全系统中断标志,HSE失效时硬件置1,软件写1到CSSC位清除 */
#define RCC_CIR_LSIRDYIE        ((uint32)0x00000100)        /* LSI就绪中断使能 */
#define RCC_CIR_LSERDYIE        ((uint32)0x00000200)        /* LSE就绪中断使能 */
#define RCC_CIR_HSIRDYIE        ((uint32)0x00000400)        /* HSI就绪中断使能 */
#define RCC_CIR_HSERDYIE        ((uint32)0x00000800)        /* HSE就绪中断使能 */
#define RCC_CIR_PLLRDYIE        ((uint32)0x00001000)        /* PLL就绪中断使能 */
#define RCC_CIR_LSIRDYC         ((uint32)0x00010000)        /* 清除LSI就绪中断 */
#define RCC_CIR_LSERDYC         ((uint32)0x00020000)        /* 清除LSE就绪中断 */
#define RCC_CIR_HSIRDYC         ((uint32)0x00040000)        /* 清除HSI就绪中断 */
#define RCC_CIR_HSERDYC         ((uint32)0x00080000)        /* 清除HSE就绪中断 */
#define RCC_CIR_PLLRDYC         ((uint32)0x00100000)        /* 清除PLL就绪中断 */
#define RCC_CIR_CSSC            ((uint32)0x00800000)        /* 清除时钟安全系统中断 */

#define RCC_CIE_ALLE            ((uint32)0x00001F00)        /* 打开所有中断 */
#define RCC_CIR_ALLC            ((uint32)0x009F0000)        /* 清除所有中断标志 */
 /*
  * 高速外设(APB2)复位寄存器 APB2RSTR
  * 偏移地址: 0x0C
  * 复位值: 0x0000 0000
  * 访问: 没有等待状态, word/half-word/byte访问
  */
#define APB2RSTR_AFIORST    ((uint32)0x00000001)        /* 辅助功能I/O复位 */
#define APB2RSTR_IOPARST    ((uint32)0x00000004)        /* IO端口A复位 */
#define APB2RSTR_IOPBRST    ((uint32)0x00000008)        /* IO端口B复位 */
#define APB2RSTR_IOPCRST    ((uint32)0x00000010)        /* IO端口C复位 */
#define APB2RSTR_IOPDRST    ((uint32)0x00000020)        /* IO端口D复位 */
#define APB2RSTR_IOPERST    ((uint32)0x00000040)        /* IO端口E复位 */
#define APB2RSTR_IOPFRST    ((uint32)0x00000080)        /* IO端口F复位 */
#define APB2RSTR_IOPGRST    ((uint32)0x00000100)        /* IO端口G复位 */
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
  * 低速外设(APB1)复位寄存器 APB1RSTR
  * 偏移地址: 0x10
  * 复位值: 0x0000 0000
  * 访问: 没有等待状态, word/half-word/byte访问
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
  * 总线外设(AHB)时钟使能寄存器 RCC_AHBENR
  * 偏移地址: 0x14
  * 复位值: 0x0000 0014
  * 访问: 没有等待状态, word/half-word/byte访问
  */
#define RCC_AHBENR_DMA1EN       ((uint16)0x0001)            /* DMA1 */
#define RCC_AHBENR_DMA2EN       ((uint16)0x0002)            /* DMA2 */
#define RCC_AHBENR_SRAMEN       ((uint16)0x0004)            /* SRAM */
#define RCC_AHBENR_FLITFEN      ((uint16)0x0010)            /* FLITF */
#define RCC_AHBENR_CRCEN        ((uint16)0x0040)            /* CRC */
#define RCC_AHBENR_FSMCEN       ((uint16)0x0100)            /* FSMC */
#define RCC_AHBENR_SDIOEN       ((uint16)0x0400)            /* SDIO */
 /*
  * 高速总线外设(APB2)时钟使能寄存器 APB2ENR
  * 偏移地址: 0x18
  * 复位值: 0x0000 0000
  * 访问: 没有等待状态, word/half-word/byte访问
  */
#define APB2ENR_AFIOEN      ((uint32)0x00000001)        /* 辅助I/O时钟使能 */
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
 * 低速总线外设(APB1)时钟使能寄存器 APB1ENR
 * 偏移地址: 0x1C
 * 复位值: 0x0000 0000
 * 访问: 没有等待状态, word/half-word/byte访问
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
 * 备份控制(Backup Domain Control, BDC)寄存器 RCC_BDCR
 * 偏移地址: 0x20
 * 复位值: 0x0000 0000
 * 访问: 0到3个等待周期, word/half-word/byte访问
 */
#define RCC_BDCR_LSEON          ((uint32)0x00000001)        /* 开启外部低速振荡器LSE */
#define RCC_BDCR_LSERDY         ((uint32)0x00000002)        /* LSE就绪 */
#define RCC_BDCR_LSEBYP         ((uint32)0x00000004)        /* LSE旁路输入 */

#define RCC_BDCR_RTCSEL         ((uint32)0x00000300)        /* RTC时钟源选择 RTCSEL[1:0] */
#define RCC_BDCR_RTCSEL_NOCLOCK ((uint32)0x00000000)        /* 别闹,谁都不用 */
#define RCC_BDCR_RTCSEL_LSE     ((uint32)0x00000100)        /* LSE */
#define RCC_BDCR_RTCSEL_LSI     ((uint32)0x00000200)        /* LSI */
#define RCC_BDCR_RTCSEL_HSE     ((uint32)0x00000300)        /* HSE经128分频后 */

#define RCC_BDCR_RTCEN          ((uint32)0x00008000)        /* RTC时钟使能 */
#define RCC_BDCR_BDRST          ((uint32)0x00010000)        /* Backup domain software reset  */
/*
 * 控制/状态(Control/Status, CS)寄存器 RCC_CSR
 * 偏移地址: 0x24
 * 复位值: 0x0C00 0000
 * 访问: 0到3个等待周期, word/half-word/byte访问
 */
#define RCC_CSR_LSION           ((uint32)0x00000001)        /* 开启LSI */
#define RCC_CSR_LSIRDY          ((uint32)0x00000002)        /* LSI就绪 */
#define RCC_CSR_RMVF            ((uint32)0x01000000)        /* 清除复位标志 */
#define RCC_CSR_PINRSTF         ((uint32)0x04000000)        /* NRST引脚复位标志,硬件置1,软件写1到RMVF清除 */
#define RCC_CSR_PORRSTF         ((uint32)0x08000000)        /* 上电/掉电复位标志,硬件置1,软件写1到RMVF清除 */
#define RCC_CSR_SFTRSTF         ((uint32)0x10000000)        /* 软件复位标志,硬件置1,软件写1到RMVF清除 */
#define RCC_CSR_IWDGRSTF        ((uint32)0x20000000)        /* 独立看门狗复位标志,硬件置1,软件写1到RMVF清除 */
#define RCC_CSR_WWDGRSTF        ((uint32)0x40000000)        /* 窗口看门狗复位标志,硬件置1,软件写1到RMVF清除 */
#define RCC_CSR_LPWRRSTF        ((uint32)0x80000000)        /* 低功耗复位标志,硬件置1,软件写1到RMVF清除  */
/*
 * APH外设
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
 * APB2外设
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
 * APB1外设
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
 * rcc_switch_apb2_periph_clock - 切换APB2外设的时钟
 *
 * @p: 外设,支持位选
 * @on: 0-关闭,非零-打开
 */
void rcc_switch_apb2_periph_clock(uint32 p, int on);
/*
 * rcc_switch_apb1_periph_clock - 切换APB1外设的时钟
 *
 * @p: 外设,支持位选
 * @on: 0-关闭,非零-打开
 */
void rcc_switch_apb1_periph_clock(uint32 p, int on);

/************************************************/
/* 系统时钟状态
 */

/*
 * rcc_get_sysclk_freq - 获取系统时钟
 *
 * return: 系统时钟Hz
 */
uint32 rcc_get_sysclk_freq(void);

typedef struct {
    uint32 SYSCLK;  /* 系统时钟频率,Hz */
    uint32 HCLK;    /* AHB分频后的频率, Hz */
    uint32 PCLK1;   /* 低速外设APB1频率, Hz */
    uint32 PCLK2;   /* 高速外设APB2频率, Hz */
    uint32 ADCCLK;  /* AD转换频率, Hz */
} rcc_clocks_t;

/*
 * rcc_get_clocks_freq - 获取系统各种时钟
 *
 * @clocks: 填充数据结构
 */
void rcc_get_clocks_freq(rcc_clocks_t *clocks);


#endif
