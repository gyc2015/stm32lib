#ifndef STM32F103RC_USART_H
#define STM32F103RC_USART_H

#include <stm32f103rc.h>

typedef struct {
    volatile uint16 SR;     /* 状态寄存器, Status Register */
    uint16  RESERVED0;
    volatile uint16 DR;      /* 数据寄存器, Data Register */
    uint16  RESERVED1;
    volatile uint16 BRR;    /* 波特率寄存器, Baud Rate Register */
    uint16  RESERVED2;
    volatile uint16 CR1;    /* 控制寄存器1, Control Register 1 */
    uint16  RESERVED3;
    volatile uint16 CR2;    /* 控制寄存器2, Control Register 2 */
    uint16  RESERVED4;
    volatile uint16 CR3;    /* 控制寄存器3, Control Rergister 3 */
    uint16  RESERVED5;
    volatile uint16 GTPR;   /* 保护时间和预分频寄存器, Guard time and prescaler register */
    uint16  RESERVED6;
} usart_regs_t;

/* USART寄存器地址映射 */
#define USART1_BASE     (APB2_BASE + 0x3800)
#define USART2_BASE     (APB1_BASE + 0x4400)
#define USART3_BASE     (APB1_BASE + 0x4800)
#define UART4_BASE      (APB1_BASE + 0x4C00)
#define UART5_BASE      (APB1_BASE + 0x5000)
/* USART寄存器指针访问 */
#define USART1  ((usart_regs_t *) USART1_BASE)
#define USART2  ((usart_regs_t *) USART2_BASE)
#define USART3  ((usart_regs_t *) USART3_BASE)
#define UART4   ((usart_regs_t *) UART4_BASE)
#define UART5   ((usart_regs_t *) UART5_BASE)
#define is_usart(p) (((p) == USART1) || \
                            ((p) == USART2) || \
                            ((p) == USART3) || \
                            ((p) == UART4) || \
                            ((p) == UART5))
/*
 * 状态寄存器 USART_SR
 * 偏移地址: 0x00
 * 复位值: 0x00C0
 */
#define USART_SR_PE     ((uint16)0x0001)      /* 校验错误,Parity Error,硬件置位,依次读USART_SR和USART_DR清零 */
#define USART_SR_FE     ((uint16)0x0002)      /* 帧错误,Framing Error,硬件置位,依次读USART_SR和USART_DR清零 */
#define USART_SR_NE     ((uint16)0x0004)      /* 噪声错误,Noise Error,硬件置位,依次读USART_SR和USART_DR清零 */
#define USART_SR_ORE    ((uint16)0x0008)      /* 过载错误,OverRun Error,硬件置位,依次读USART_SR和USART_CR清零,数据传送过快,接收缓存尚未读取新数据就到来了 */
#define USART_SR_IDLE   ((uint16)0x0010)      /* 总线空闲,硬件置位,依次读USART_SR和USART_DR清零 */
#define USART_SR_RXNE   ((uint16)0x0020)      /* 接收数据非空,硬件置位,读USART_DR清零 */
#define USART_SR_TC     ((uint16)0x0040)      /* 发送完成,一帧数据发送完后,TXE=1时,硬件置位,读USART_SR写USART_DR清除 */
#define USART_SR_TXE    ((uint16)0x0080)      /* 发送数据空,硬件置位,写USART_DR清零 */
#define USART_SR_LBD    ((uint16)0x0100)      /* LIN断开检测标志,硬件置位,写0清零 */
#define USART_SR_CTS    ((uint16)0x0200)      /* CTS标志,对串口UART4和UART5无效 */
/*
 * 波特率寄存器 USART_BRR
 * 偏移地址: 0x08
 * 复位值: 0x0000
 */
#define USART_BRR_DIV_Fraction ((uint16)0x000F)  /*!< Fraction of USARTDIV */
#define USART_BRR_DIV_Mantissa ((uint16)0xFFF0)  /*!< Mantissa of USARTDIV */
/*
 * 控制寄存器1 USART_CR1
 * 偏移地址: 0x0C
 * 复位值: 0x0000
 */
#define USART_CR1_SBK       ((uint16)0x0001)    /* 发送断开帧 Send Break */
#define USART_CR1_RWU       ((uint16)0x0002)    /* 接收唤醒 Receiver WakeUp */
#define USART_CR1_RE        ((uint16)0x0004)    /* 接收使能 Receiver Enable */
#define USART_CR1_TE        ((uint16)0x0008)    /* 发送使能 Transmitter Enable */
#define USART_CR1_IDLEIE    ((uint16)0x0010)    /* 空闲中断使能 IDLE Interrupt Enable */
#define USART_CR1_RXNEIE    ((uint16)0x0020)    /* 接收缓冲非空中断使能 RXNE Interrupt Enable */
#define USART_CR1_TCIE      ((uint16)0x0040)    /* 发送完成中断 Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE     ((uint16)0x0080)    /* 发送缓冲区空中断使能 TXE Interrupt Enable */
#define USART_CR1_PEIE      ((uint16)0x0100)    /* 校验错误中断使能 PE Interrupt Enable */
#define USART_CR1_PS        ((uint16)0x0200)    /* 检验选择 Parity Selection,0:偶Even检验,1:奇odd校验 */
#define USART_CR1_PCE       ((uint16)0x0400)    /* 硬件校验控制使能 Parity Control Enable,是否由硬件负责产生或者检测校验位 */
#define USART_CR1_WAKE      ((uint16)0x0800)    /* 唤醒方式 Wakeup method,0:空闲总线,1:地址标记 */
#define USART_CR1_M         ((uint16)0x1000)    /* 字长 Word length,是否添加校验位 */
#define USART_CR1_UE        ((uint16)0x2000)    /* 开串口 USART Enable */
/*
 * 控制寄存器2 USART_CR2
 * 偏移地址: 0x10
 * 复位值: 0x0000
 */
#define USART_CR2_ADD       ((uint16)0x000F)    /* 地址,Add[3:0] Address of the USART node */
#define USART_CR2_LBDL      ((uint16)0x0020)    /* LIN断开检测长度,LIN Break Detection Length,0:10位,1:11位 */
#define USART_CR2_LBDIE     ((uint16)0x0040)    /* LIN断开检测中断使能,LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL      ((uint16)0x0100)    /* 最后一个时钟脉冲,Last Bit Clock pulse,同步模式,UART4和UART5不存在 */
#define USART_CR2_CPHA      ((uint16)0x0200)    /* 时钟相位,Clock Phase,同步模式,UART4和UART5不存在 */
#define USART_CR2_CPOL      ((uint16)0x0400)    /* 时钟极性,Clock Polarity,同步模式,UART4和UART5不存在 */
#define USART_CR2_CLKEN     ((uint16)0x0800)    /* 时钟使能,Clock Enable,UART4和UART5不存在 */
#define USART_CR2_STOP      ((uint16)0x3000)    /* 停止位位数,STOP[1:0] bits (STOP bits) */
#define USART_CR2_LINEN     ((uint16)0x4000)    /* LIN模式使能,LIN mode enable */
/*
 * 控制寄存器3 USART_CR3
 * 偏移地址: 0x14
 * 复位值: 0x0000
 */
#define USART_CR3_EIE       ((uint16)0x0001)    /* 错误中断使能, Error Interrupt Enable,多缓冲区通信模式 */
#define USART_CR3_IREN      ((uint16)0x0002)    /* 红外模式使能, IrDA mode Enable */
#define USART_CR3_IRLP      ((uint16)0x0004)    /* 红外低功耗, IrDA Low-Power */
#define USART_CR3_HDSEL     ((uint16)0x0008)    /* 半双工选择, Half-Duplex Selection */
#define USART_CR3_NACK      ((uint16)0x0010)    /* 智能卡NACK模式, Smartcard NACK enable */
#define USART_CR3_SCEN      ((uint16)0x0020)    /* 智能卡模式, Smartcard mode enable */
#define USART_CR3_DMAR      ((uint16)0x0040)    /* DMA接收使能, DMA Enable Receiver */
#define USART_CR3_DMAT      ((uint16)0x0080)    /* DMA发送使能, DMA Enable Transmitter */
#define USART_CR3_RTSE      ((uint16)0x0100)    /* RTS使能, RTS Enable, UART4和UART5不存在 */
#define USART_CR3_CTSE      ((uint16)0x0200)    /* CTS使能, CTS Enable */
#define USART_CR3_CTSIE     ((uint16)0x0400)    /* CTS中断使能, CTS Interrupt Enable */
/*
 * 保护时间和预分频寄存器 USART_GTPR
 * 偏移地址: 0x18
 * 复位值: 0x0000
 */
#define USART_GTPR_PSC      ((uint16)0x00FF)    /* 预分频器, PSC[7:0] bits (Prescaler value) */
#define USART_GTPR_GT       ((uint16)0xFF00)    /* 保护时间, Guard time value */

/************************************************/
/* 串口的同步模式时钟设置,对控制寄存器2(USART_CR2)的配置
 * 对于串口UART4和UART5不适用,一般给零即可.
 */
#define USART_Clock_Disable     ((uint16)0x0000)    /* USART_CR2_CLKEN */
#define USART_Clock_Enable      ((uint16)0x0800)
#define USART_CPOL_Low          ((uint16)0x0000)    /* USART_CR2_CPOL */
#define USART_CPOL_High         ((uint16)0x0400)
#define USART_CPHA_1Edge        ((uint16)0x0000)    /* USART_CR2_CPHA */
#define USART_CPHA_2Edge        ((uint16)0x0200)
#define USART_LastBit_Disable   ((uint16)0x0000)    /* USART_CR2_LBCL */
#define USART_LastBit_Enable    ((uint16)0x0100)
#define USART_Clock_Mask        ((uint16)0x0F00)

/*
 * usart_init_clock - 初始化串口同步模式时钟
 *
 * 不工作在同步模式时,建议清零
 *
 * @USARTx: 串口寄存器指针访问
 * @conf: 同步时钟配置
 */
void usart_init_clock(usart_regs_t *USARTx, uint16 conf);

/************************************************/
/* 串口的通用配置
 */
typedef struct {
    uint32 baud_rate;               /* 波特率 */
    uint16 word_length;             /* 字长,8|9位 */
    uint16 stop_bits;               /* 停止位数 */
    uint16 parity;                  /* 奇偶校验 */
    uint16 mode;                    /* 收发模式 */
    uint16 hardwareFlowControl;     /* 硬件控制流 */
} usart_conf_t;

/* 字长 */
#define USART_WordLength_8  ((uint16)0x0000)    /* 8位字长 */
#define USART_WordLength_9  ((uint16)0x1000)    /* 9 */
#define is_usart_wordlength(p) (((p) == USART_WordLength_8) || ((p) == USART_WordLength_9))
/* 停止位 */
#define USART_StopBits_1   ((uint16)0x0000)     /* 1个停止位 */
#define USART_StopBits_0_5 ((uint16)0x1000)     /* 0.5个 */
#define USART_StopBits_2   ((uint16)0x2000)     /* 2 */
#define USART_StopBits_1_5 ((uint16)0x3000)     /* 1.5 */
#define is_usart_stop_bits(p) (((p) == USART_StopBits_1) || \
                               ((p) == USART_StopBits_0_5) || \
                               ((p) == USART_StopBits_2) || \
                               ((p) == USART_StopBits_1_5))
/* 奇偶校验 */
#define USART_Parity_No     ((uint16)0x0000)
#define USART_Parity_Even   ((uint16)0x0400)
#define USART_Parity_Odd    ((uint16)0x0600) 
#define is_usart_parity(p) (((p) == USART_Parity_No) || \
                            ((p) == USART_Parity_Even) || \
                            ((p) == USART_Parity_Odd))
/*  收发模式 */
#define USART_Mode_Rx       ((uint16)0x0004)
#define USART_Mode_Tx       ((uint16)0x0008)
#define USART_Mode_Both     ((uint32)0x000C)
#define is_usart_mode(p) ((0 != p) && (0 == ((p) & (~USART_Mode_Both))))
/* 硬件控制流 */
#define USART_HardwareFlowControl_None       ((uint16)0x0000)
#define USART_HardwareFlowControl_RTS        ((uint16)0x0100)
#define USART_HardwareFlowControl_CTS        ((uint16)0x0200)
#define USART_HardwareFlowControl_RTS_CTS    ((uint16)0x0300)
#define is_usart_hardware_flow_control(p)\
                              (((p) == USART_HardwareFlowControl_None) || \
                               ((p) == USART_HardwareFlowControl_RTS) || \
                               ((p) == USART_HardwareFlowControl_CTS) || \
                               ((p) == USART_HardwareFlowControl_RTS_CTS))

/*
* usart_init - 初始化串口
*
* @USARTx: 目标串口
* @conf: 配置项
*/
void usart_init(usart_regs_t *USARTx, const usart_conf_t *conf);

/*
 * usart_enable - 使能串口
 *
 * @USARTx: 目标串口
 * @enable: 是否使能
 */
void usart_switch(usart_regs_t *USARTx, uint16 enable);

/************************************************/
/* 串口的中断配置
 */
#define USART_IT_PE     ((uint16)0x0028)
#define USART_IT_TXE    ((uint16)0x0727)
#define USART_IT_TC     ((uint16)0x0626)
#define USART_IT_RXNE   ((uint16)0x0525)
#define USART_IT_IDLE   ((uint16)0x0424)
#define USART_IT_LBD    ((uint16)0x0846)
#define USART_IT_CTS    ((uint16)0x096A)
#define USART_IT_ERR    ((uint16)0x0060)
#define USART_IT_ORE    ((uint16)0x0360)
#define USART_IT_NE     ((uint16)0x0260)
#define USART_IT_FE     ((uint16)0x0160)
#define is_usart_conf_it(it) (((it) == USART_IT_PE) || ((it) == USART_IT_TXE) || \
                               ((it) == USART_IT_TC) || ((it) == USART_IT_RXNE) || \
                               ((it) == USART_IT_IDLE) || ((it) == USART_IT_LBD) || \
                               ((it) == USART_IT_CTS) || ((it) == USART_IT_ERR))
#define is_usart_get_it(it) (((it) == USART_IT_PE) || ((it) == USART_IT_TXE) || \
                            ((it) == USART_IT_TC) || ((it) == USART_IT_RXNE) || \
                            ((it) == USART_IT_IDLE) || ((it) == USART_IT_LBD) || \
                            ((it) == USART_IT_CTS) || ((it) == USART_IT_ORE) || \
                            ((it) == USART_IT_NE) || ((it) == USART_IT_FE))
#define is_usart_clear_it(it) (((it) == USART_IT_TC) || ((it) == USART_IT_RXNE) || \
                               ((it) == USART_IT_LBD) || ((it) == USART_IT_CTS))

/*
 * usart_it_config - 设置串口中断
 *
 * @USARTx: 目标串口地址访问
 * @it: 配置中断项
 * @enable: 是否使能
 */
void usart_it_config(usart_regs_t *USARTx, uint16 it, uint16 enable);

/************************************************/
/* 串口的状态
 */
#define is_usart_flag(FLAG) (((FLAG) == USART_SR_PE) || ((FLAG) == USART_SR_TXE) || \
                             ((FLAG) == USART_SR_TC) || ((FLAG) == USART_SR_RXNE) || \
                             ((FLAG) == USART_SR_IDLE) || ((FLAG) == USART_SR_LBD) || \
                             ((FLAG) == USART_SR_CTS) || ((FLAG) == USART_SR_ORE) || \
                             ((FLAG) == USART_SR_NE) || ((FLAG) == USART_SR_FE))

/*
 * usart_get_flag_status - 获取串口标记位状态
 *
 * @USARTx: 目标串口
 * @flag: 标识
 */
uint16 usart_get_flag_status(usart_regs_t* USARTx, uint16 flag);

#endif
