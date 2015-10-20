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
/* Ӳ��ʱ�ӻ���
 */
#define HSI_FREQ    ((uint32)8000000)
#define HSE_FREQ    ((uint32)8000000)

/************************************************/
/* Cortex-M3���ڴ�ӳ��������������֧��λӳ��(bit-band)
 * ��STM32F10xxxϵ�е�оƬ���������ֱ��ӦSRAM��64kB�ռ�������ַ�ռ�
 */
#define SRAM_BB_BASE    ((uint32)0x22000000) /* SRAMλӳ�����ַ */
#define PERIPH_BB_BASE  ((uint32)0x42000000) /* �����ַλӳ�����ַ */
/*
 * BIT_WORD_ADDR - λӳ���ַ��ʽ
 *
 * e.g.: 0x22006008 = 0x22000000 + (0x300 * 32) + (2 * 4)
 * �����ֽ�0x22006008��Ч�ڷ���SRAM�е�300�ֽڵĵ�2λ
 *
 * @base: λӳ�����ַ
 * @offset: �����ռ��е�ƫ����(byte)
 * @number: Ŀ��λ���ֽ��е�λ���(0-7)
 */
#define BIT_WORD_ADDR(base, offset, number) ((base) + ((offset) * 32) + ((number) * 4))

/************************************************/
/* ���Ժ���
 */
#define assert(e) do {} while(0)

#endif
