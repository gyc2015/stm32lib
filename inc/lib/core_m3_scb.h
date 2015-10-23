#ifndef CORE_M3_SCB_H
#define CORE_M3_SCB_H

#include <core_m3.h>

typedef struct {
    volatile uint32 CPUID;      /* 0x00  CPU ID Base Register                                  */
    volatile uint32 ICSR;       /* 0x04  Interrupt Control State Register                      */
    volatile uint32 VTOR;       /* 0x08  Vector Table Offset Register                          */
    volatile uint32 AIRCR;      /* 0x0C  应用中断与复位控制 */
    volatile uint32 SCR;        /* 0x10  System Control Register                               */
    volatile uint32 CCR;        /* 0x14  Configuration Control Register                        */
    volatile uint8  SHP[12];    /* 0x18  System Handlers Priority Registers (4-7, 8-11, 12-15) */
    volatile uint32 SHCSR;      /* 0x24  System Handler Control and State Register             */
    volatile uint32 CFSR;       /* 0x28  Configurable Fault Status Register                    */
    volatile uint32 HFSR;       /* 0x2C  Hard Fault Status Register                            */
    volatile uint32 DFSR;       /* 0x30  Debug Fault Status Register                           */
    volatile uint32 MMFAR;      /* 0x34  Mem Manage Address Register                           */
    volatile uint32 BFAR;       /* 0x38  Bus Fault Address Register                            */
    volatile uint32 AFSR;       /* 0x3C  Auxiliary Fault Status Register                       */
    volatile uint32 PFR[2];     /* 0x40  Processor Feature Register                            */
    volatile uint32 DFR;        /* 0x48  Debug Feature Register                                */
    volatile uint32 ADR;        /* 0x4C  Auxiliary Feature Register                            */
    volatile uint32 MMFR[4];    /* 0x50  Memory Model Feature Register                         */
    volatile uint32 ISAR[5];    /* 0x60  ISA Feature Register                                  */
} scb_regs_t;

/* SCB寄存器地址映射 */
#define SCB_BASE   (SCS_BASE + 0x0D00)
/* SCB寄存器指针访问 */
#define SCB    ((scb_regs_t*)SCB_BASE)

/*
 * 应用中断和复位控制, AIRCR
 * 偏移地址: 0x00
 * 复位值: 0x0000
 */
#define SCB_AIRCR_VECTKEYSTAT   ((uint32)0xFFFF0000)
#define SCB_AIRCR_VECTKEY       ((uint32)0xFFFF0000)
#define SCB_AIRCR_ENDIANESS     ((uint32)0x00008000)
#define SCB_AIRCR_PRIGROUP      ((uint32)0x00000700)
#define SCB_AIRCR_SYSRESETREQ   ((uint32)0x00000004)
#define SCB_AIRCR_VECTCLRACTIVE ((uint32)0x00000002)
#define SCB_AIRCR_VECTRESET     ((uint32)0x00000001)

/* 在对SCB_AIRCR修改时,必须在高16位添加这个key,否则写入无效 */
#define SCB_AIRCR_Vectkey    ((uint32)0x05FA0000)



#endif
