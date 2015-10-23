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

/* NVIC寄存器地址映射 */
#define NVIC_BASE   (SCS_BASE + 0x0100)
/* NVIC寄存器指针访问 */
#define NVIC    ((nvic_regs_t*)NVIC_BASE)


/************************************************************************************/
/* 优先级分组
 * 实际上,控制优先级分组的是SCB(System Control Block)寄存器的PRIGROUP位
 * 由于使用上,是对中断的配置,这里沿用TI官方库的设定,以前缀NVIC标记
 */
#define NVIC_PriGroup_0    ((uint32)0x00000700)    /* 0b.yyyy, 0位抢占优先级 */
#define NVIC_PriGroup_1    ((uint32)0x00000600)    /* 0bx.yyy, 1位抢占优先级  */
#define NVIC_PriGroup_2    ((uint32)0x00000500)    /* 0bxx.yy, 2位抢占优先级  */
#define NVIC_PriGroup_3    ((uint32)0x00000400)    /* 0bxxx.y, 3位抢占优先级  */
#define NVIC_PriGroup_4    ((uint32)0x00000300)    /* 0bxxxx., 4位抢占优先级  */
#define is_nvic_prigroup(p) (((p) == NVIC_PriGroup_0) || \
                             ((p) == NVIC_PriGroup_1) || \
                             ((p) == NVIC_PriGroup_2) || \
                             ((p) == NVIC_PriGroup_3) || \
                             ((p) == NVIC_PriGroup_4))
/*
 * nvic_config_prigroup - 配置嵌套中断分组
 *
 * @conf: 配置选项
 *        NVIC_PriGroup_0: 0b.yyyy, 0位抢占优先级
 *        NVIC_PriGroup_1: 0bx.yyy, 1位抢占优先级
 *        NVIC_PriGroup_2: 0bxx.yy, 2位抢占优先级
 *        NVIC_PriGroup_3: 0bxxx.y, 3位抢占优先级
 *        NVIC_PriGroup_4: 0bxxxx., 4位抢占优先级
 */
void nvic_config_prigroup(uint32 conf);

/************************************************/
/* 中断配置
 */
typedef struct {
    uint8 IRQn;
    uint8 pre_prior;
    uint8 sub_prior;
    uint8 enale;
} nvic_conf_t;

void nvic_init(const nvic_conf_t *conf);

#endif
