/*
 * ����Flash�ı�̣�ʵ���Ͽ���ʵ�ֶԳ������д
 * ������Բο�ST�Ĺٷ��ĵ�
 * STM32F10XXX Flash memory microcontrollers Programming Manual(PM0075)
 */

#ifndef STM32F103RC_FLASH_H
#define STM32F103RC_FLASH_H

#include <stm32f103rc.h>

typedef struct {
    volatile uint32 ACR;        /* ������ʿ���,Access Control Register */
    volatile uint32 KEYR;
    volatile uint32 OPTKEYR;
    volatile uint32 SR;
    volatile uint32 CR;
    volatile uint32 AR;
    volatile uint32 RESERVED;
    volatile uint32 OBR;
    volatile uint32 WRPR;
} flash_regs_t;

/* Flash�ӿڼĴ�����ַӳ�� */
#define FLASH_INTERFACE_BASE (AHB_BASE + 0x2000)
/* Flash�ӿڼĴ���ָ����� */
#define FLASH ((flash_regs_t *)FLASH_INTERFACE_BASE)

/*
 * ������ʿ��ƼĴ��� FLASH_ACR
 * ƫ�Ƶ�ַ: 0x00
 * ��λֵ: 0x0000 0030
 */
#define FLASH_ACR_LATENCY       ((uint8)0x03)     /* �����ϵͳʱ��SYSCLK,�������ʱ���� */
#define FLASH_ACR_LATENCY_0     ((uint8)0x00)     /* �޵ȴ�,    0 < SYSCLK <= 24MHz */
#define FLASH_ACR_LATENCY_1     ((uint8)0x01)     /* 1����, 24MHz < SYSCLK <= 48MHz */
#define FLASH_ACR_LATENCY_2     ((uint8)0x02)     /* 2����, 48MHz < SYSCLK <= 72MHz */

#define FLASH_ACR_HLFCYA        ((uint8)0x08)     /* ʹ����������ڷ��� */
#define FLASH_ACR_PRFTBE        ((uint8)0x10)     /* ʹ��Ԥ��ȡ���� */
#define FLASH_ACR_PRFTBS        ((uint8)0x20)     /* Ԥ��ȡ����״̬,0:ʧ��,1:ʹ�� */


#endif
