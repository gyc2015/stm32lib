#ifndef CORE_M3_NVIC_H
#define CORE_M3_NVIC_H

#include <core_m3.h>

typedef struct {
    volatile uint32 ISER[8];    /*!< Offset: 0x000  Interrupt Set Enable Register           */
    uint32 RESERVED0[24];
    volatile uint32 ICER[8];    /*!< Offset: 0x080  Interrupt Clear Enable Register         */
    uint32 RSERVED1[24];
    volatile uint32 ISPR[8];    /*!< Offset: 0x100  Interrupt Set Pending Register          */
    uint32 RESERVED2[24];
    volatile uint32 ICPR[8];    /*!< Offset: 0x180  Interrupt Clear Pending Register        */
    uint32 RESERVED3[24];
    volatile uint32 IABR[8];    /*!< Offset: 0x200  Interrupt Active bit Register           */
    uint32 RESERVED4[56];
    volatile uint8  IP[240];    /*!< Offset: 0x300  Interrupt Priority Register (8Bit wide) */
    uint32 RESERVED5[644];
    volatile uint32 STIR;       /*!< Offset: 0xE00  Software Trigger Interrupt Register     */
}  nvic_regs_t;

/* NVIC�Ĵ�����ַӳ�� */
#define NVIC_BASE   (SCS_BASE + 0x0100)
/* NVIC�Ĵ���ָ����� */
#define NVIC    ((nvic_regs_t*)NVIC_BASE)


/************************************************************************************/
/* ���ȼ�����
 * ʵ����,�������ȼ��������SCB(System Control Block)�Ĵ�����PRIGROUPλ
 * ����ʹ����,�Ƕ��жϵ�����,��������TI�ٷ�����趨,��ǰ׺NVIC���
 */
#define NVIC_PriGroup_0    ((uint32)0x00000700)    /* 0b.yyyy, 0λ��ռ���ȼ� */
#define NVIC_PriGroup_1    ((uint32)0x00000600)    /* 0bx.yyy, 1λ��ռ���ȼ�  */
#define NVIC_PriGroup_2    ((uint32)0x00000500)    /* 0bxx.yy, 2λ��ռ���ȼ�  */
#define NVIC_PriGroup_3    ((uint32)0x00000400)    /* 0bxxx.y, 3λ��ռ���ȼ�  */
#define NVIC_PriGroup_4    ((uint32)0x00000300)    /* 0bxxxx., 4λ��ռ���ȼ�  */
#define is_nvic_prigroup(p) (((p) == NVIC_PriGroup_0) || \
                             ((p) == NVIC_PriGroup_1) || \
                             ((p) == NVIC_PriGroup_2) || \
                             ((p) == NVIC_PriGroup_3) || \
                             ((p) == NVIC_PriGroup_4))
/*
 * nvic_config_prigroup - ����Ƕ���жϷ���
 *
 * @conf: ����ѡ��
 *        NVIC_PriGroup_0: 0b.yyyy, 0λ��ռ���ȼ�
 *        NVIC_PriGroup_1: 0bx.yyy, 1λ��ռ���ȼ�
 *        NVIC_PriGroup_2: 0bxx.yy, 2λ��ռ���ȼ�
 *        NVIC_PriGroup_3: 0bxxx.y, 3λ��ռ���ȼ�
 *        NVIC_PriGroup_4: 0bxxxx., 4λ��ռ���ȼ�
 */
void nvic_config_prigroup(uint32 conf);

/************************************************/
/* �ж�����
 */
typedef struct {
    uint8 IRQn;
    uint8 pre_prior;
    uint8 sub_prior;
    uint8 enale;
} nvic_conf_t;

void nvic_init(const nvic_conf_t *conf);

#endif
