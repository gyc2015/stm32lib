/*
 * 对于Flash的编程，实际上可以实现对程序的烧写
 * 具体可以参考ST的官方文档
 * STM32F10XXX Flash memory microcontrollers Programming Manual(PM0075)
 */

#ifndef STM32F103RC_FLASH_H
#define STM32F103RC_FLASH_H

#include <stm32f103rc.h>

typedef struct {
    volatile uint32 ACR;        /* 闪存访问控制,Access Control Register */
    volatile uint32 KEYR;
    volatile uint32 OPTKEYR;
    volatile uint32 SR;
    volatile uint32 CR;
    volatile uint32 AR;
    volatile uint32 RESERVED;
    volatile uint32 OBR;
    volatile uint32 WRPR;
} flash_regs_t;

/* Flash接口寄存器地址映射 */
#define FLASH_INTERFACE_BASE (AHB_BASE + 0x2000)
/* Flash接口寄存器指针访问 */
#define FLASH ((flash_regs_t *)FLASH_INTERFACE_BASE)

/*
 * 闪存访问控制寄存器 FLASH_ACR
 * 偏移地址: 0x00
 * 复位值: 0x0000 0030
 */
#define FLASH_ACR_LATENCY       ((uint8)0x03)     /* 相对于系统时钟SYSCLK,闪存访问时间间隔 */
#define FLASH_ACR_LATENCY_0     ((uint8)0x00)     /* 无等待,    0 < SYSCLK <= 24MHz */
#define FLASH_ACR_LATENCY_1     ((uint8)0x01)     /* 1周期, 24MHz < SYSCLK <= 48MHz */
#define FLASH_ACR_LATENCY_2     ((uint8)0x02)     /* 2周期, 48MHz < SYSCLK <= 72MHz */

#define FLASH_ACR_HLFCYA        ((uint8)0x08)     /* 使能闪存半周期访问 */
#define FLASH_ACR_PRFTBE        ((uint8)0x10)     /* 使能预读取缓存 */
#define FLASH_ACR_PRFTBS        ((uint8)0x20)     /* 预读取缓存状态,0:失能,1:使能 */


#endif
