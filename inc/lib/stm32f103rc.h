#ifndef STM32F103RC_H
#define STM32F103RC_H

#include <types.h>

#define FLASH_BASE  ((uint32)0x08000000)
#define SRAM_BASE   ((uint32)0x20000000)
#define PERIPH_BASE ((uint32)0x40000000)
#define FSMC_BASE   ((uint32)0x60000000)
#define FSMC_R_BASE ((uint32)0xA0000000)

#define APB1_BASE   PERIPH_BASE
#define APB2_BASE   (PERIPH_BASE + 0x10000)
#define AHB_BASE    (PERIPH_BASE + 0x20000)

/************************************************/
/* 硬件时钟环境
 */
#define HSI_FREQ    ((uint32)8000000)
#define HSE_FREQ    ((uint32)8000000)

/************************************************/
/* Cortex-M3的内存映射中有两个区域支持位映射(bit-band)
 * 在STM32F10xxx系列的芯片中这个区域分别对应SRAM的64kB空间和外设地址空间
 */
#define SRAM_BB_BASE    ((uint32)0x22000000) /* SRAM位映射基地址 */
#define PERIPH_BB_BASE  ((uint32)0x42000000) /* 外设地址位映射基地址 */
/*
 * BIT_WORD_ADDR - 位映射地址公式
 *
 * e.g.: 0x22006008 = 0x22000000 + (0x300 * 32) + (2 * 4)
 * 访问字节0x22006008等效于访问SRAM中第300字节的第2位
 *
 * @base: 位映射基地址
 * @offset: 别名空间中的偏移量(byte)
 * @number: 目标位在字节中的位编号(0-7)
 */
#define BIT_WORD_ADDR(base, offset, number) ((base) + ((offset) * 32) + ((number) * 4))

/************************************************/
/* 断言函数
 */
#define assert(e) do {} while(0)

#endif
