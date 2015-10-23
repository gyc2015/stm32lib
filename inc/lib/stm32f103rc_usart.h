#ifndef STM32F103RC_USART_H
#define STM32F103RC_USART_H

#include <stm32f103rc.h>

typedef struct {
    volatile uint16 SR;     /* ״̬�Ĵ���, Status Register */
    uint16  RESERVED0;
    volatile uint16 DR;      /* ���ݼĴ���, Data Register */
    uint16  RESERVED1;
    volatile uint16 BRR;    /* �����ʼĴ���, Baud Rate Register */
    uint16  RESERVED2;
    volatile uint16 CR1;    /* ���ƼĴ���1, Control Register 1 */
    uint16  RESERVED3;
    volatile uint16 CR2;    /* ���ƼĴ���2, Control Register 2 */
    uint16  RESERVED4;
    volatile uint16 CR3;    /* ���ƼĴ���3, Control Rergister 3 */
    uint16  RESERVED5;
    volatile uint16 GTPR;   /* ����ʱ���Ԥ��Ƶ�Ĵ���, Guard time and prescaler register */
    uint16  RESERVED6;
} usart_regs_t;

/* USART�Ĵ�����ַӳ�� */
#define USART1_BASE     (APB2_BASE + 0x3800)
#define USART2_BASE     (APB1_BASE + 0x4400)
#define USART3_BASE     (APB1_BASE + 0x4800)
#define UART4_BASE      (APB1_BASE + 0x4C00)
#define UART5_BASE      (APB1_BASE + 0x5000)
/* USART�Ĵ���ָ����� */
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
 * ״̬�Ĵ��� USART_SR
 * ƫ�Ƶ�ַ: 0x00
 * ��λֵ: 0x00C0
 */
#define USART_SR_PE     ((uint16)0x0001)      /* У�����,Parity Error,Ӳ����λ,���ζ�USART_SR��USART_DR���� */
#define USART_SR_FE     ((uint16)0x0002)      /* ֡����,Framing Error,Ӳ����λ,���ζ�USART_SR��USART_DR���� */
#define USART_SR_NE     ((uint16)0x0004)      /* ��������,Noise Error,Ӳ����λ,���ζ�USART_SR��USART_DR���� */
#define USART_SR_ORE    ((uint16)0x0008)      /* ���ش���,OverRun Error,Ӳ����λ,���ζ�USART_SR��USART_CR����,���ݴ��͹���,���ջ�����δ��ȡ�����ݾ͵����� */
#define USART_SR_IDLE   ((uint16)0x0010)      /* ���߿���,Ӳ����λ,���ζ�USART_SR��USART_DR���� */
#define USART_SR_RXNE   ((uint16)0x0020)      /* �������ݷǿ�,Ӳ����λ,��USART_DR���� */
#define USART_SR_TC     ((uint16)0x0040)      /* �������,һ֡���ݷ������,TXE=1ʱ,Ӳ����λ,��USART_SRдUSART_DR��� */
#define USART_SR_TXE    ((uint16)0x0080)      /* �������ݿ�,Ӳ����λ,дUSART_DR���� */
#define USART_SR_LBD    ((uint16)0x0100)      /* LIN�Ͽ�����־,Ӳ����λ,д0���� */
#define USART_SR_CTS    ((uint16)0x0200)      /* CTS��־,�Դ���UART4��UART5��Ч */
/*
 * �����ʼĴ��� USART_BRR
 * ƫ�Ƶ�ַ: 0x08
 * ��λֵ: 0x0000
 */
#define USART_BRR_DIV_Fraction ((uint16)0x000F)  /*!< Fraction of USARTDIV */
#define USART_BRR_DIV_Mantissa ((uint16)0xFFF0)  /*!< Mantissa of USARTDIV */
/*
 * ���ƼĴ���1 USART_CR1
 * ƫ�Ƶ�ַ: 0x0C
 * ��λֵ: 0x0000
 */
#define USART_CR1_SBK       ((uint16)0x0001)    /* ���ͶϿ�֡ Send Break */
#define USART_CR1_RWU       ((uint16)0x0002)    /* ���ջ��� Receiver WakeUp */
#define USART_CR1_RE        ((uint16)0x0004)    /* ����ʹ�� Receiver Enable */
#define USART_CR1_TE        ((uint16)0x0008)    /* ����ʹ�� Transmitter Enable */
#define USART_CR1_IDLEIE    ((uint16)0x0010)    /* �����ж�ʹ�� IDLE Interrupt Enable */
#define USART_CR1_RXNEIE    ((uint16)0x0020)    /* ���ջ���ǿ��ж�ʹ�� RXNE Interrupt Enable */
#define USART_CR1_TCIE      ((uint16)0x0040)    /* ��������ж� Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE     ((uint16)0x0080)    /* ���ͻ��������ж�ʹ�� TXE Interrupt Enable */
#define USART_CR1_PEIE      ((uint16)0x0100)    /* У������ж�ʹ�� PE Interrupt Enable */
#define USART_CR1_PS        ((uint16)0x0200)    /* ����ѡ�� Parity Selection,0:żEven����,1:��oddУ�� */
#define USART_CR1_PCE       ((uint16)0x0400)    /* Ӳ��У�����ʹ�� Parity Control Enable,�Ƿ���Ӳ������������߼��У��λ */
#define USART_CR1_WAKE      ((uint16)0x0800)    /* ���ѷ�ʽ Wakeup method,0:��������,1:��ַ��� */
#define USART_CR1_M         ((uint16)0x1000)    /* �ֳ� Word length,�Ƿ����У��λ */
#define USART_CR1_UE        ((uint16)0x2000)    /* ������ USART Enable */
/*
 * ���ƼĴ���2 USART_CR2
 * ƫ�Ƶ�ַ: 0x10
 * ��λֵ: 0x0000
 */
#define USART_CR2_ADD       ((uint16)0x000F)    /* ��ַ,Add[3:0] Address of the USART node */
#define USART_CR2_LBDL      ((uint16)0x0020)    /* LIN�Ͽ���ⳤ��,LIN Break Detection Length,0:10λ,1:11λ */
#define USART_CR2_LBDIE     ((uint16)0x0040)    /* LIN�Ͽ�����ж�ʹ��,LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL      ((uint16)0x0100)    /* ���һ��ʱ������,Last Bit Clock pulse,ͬ��ģʽ,UART4��UART5������ */
#define USART_CR2_CPHA      ((uint16)0x0200)    /* ʱ����λ,Clock Phase,ͬ��ģʽ,UART4��UART5������ */
#define USART_CR2_CPOL      ((uint16)0x0400)    /* ʱ�Ӽ���,Clock Polarity,ͬ��ģʽ,UART4��UART5������ */
#define USART_CR2_CLKEN     ((uint16)0x0800)    /* ʱ��ʹ��,Clock Enable,UART4��UART5������ */
#define USART_CR2_STOP      ((uint16)0x3000)    /* ֹͣλλ��,STOP[1:0] bits (STOP bits) */
#define USART_CR2_LINEN     ((uint16)0x4000)    /* LINģʽʹ��,LIN mode enable */
/*
 * ���ƼĴ���3 USART_CR3
 * ƫ�Ƶ�ַ: 0x14
 * ��λֵ: 0x0000
 */
#define USART_CR3_EIE       ((uint16)0x0001)    /* �����ж�ʹ��, Error Interrupt Enable,�໺����ͨ��ģʽ */
#define USART_CR3_IREN      ((uint16)0x0002)    /* ����ģʽʹ��, IrDA mode Enable */
#define USART_CR3_IRLP      ((uint16)0x0004)    /* ����͹���, IrDA Low-Power */
#define USART_CR3_HDSEL     ((uint16)0x0008)    /* ��˫��ѡ��, Half-Duplex Selection */
#define USART_CR3_NACK      ((uint16)0x0010)    /* ���ܿ�NACKģʽ, Smartcard NACK enable */
#define USART_CR3_SCEN      ((uint16)0x0020)    /* ���ܿ�ģʽ, Smartcard mode enable */
#define USART_CR3_DMAR      ((uint16)0x0040)    /* DMA����ʹ��, DMA Enable Receiver */
#define USART_CR3_DMAT      ((uint16)0x0080)    /* DMA����ʹ��, DMA Enable Transmitter */
#define USART_CR3_RTSE      ((uint16)0x0100)    /* RTSʹ��, RTS Enable, UART4��UART5������ */
#define USART_CR3_CTSE      ((uint16)0x0200)    /* CTSʹ��, CTS Enable */
#define USART_CR3_CTSIE     ((uint16)0x0400)    /* CTS�ж�ʹ��, CTS Interrupt Enable */
/*
 * ����ʱ���Ԥ��Ƶ�Ĵ��� USART_GTPR
 * ƫ�Ƶ�ַ: 0x18
 * ��λֵ: 0x0000
 */
#define USART_GTPR_PSC      ((uint16)0x00FF)    /* Ԥ��Ƶ��, PSC[7:0] bits (Prescaler value) */
#define USART_GTPR_GT       ((uint16)0xFF00)    /* ����ʱ��, Guard time value */

/************************************************/
/* ���ڵ�ͬ��ģʽʱ������,�Կ��ƼĴ���2(USART_CR2)������
 * ���ڴ���UART4��UART5������,һ����㼴��.
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
 * usart_init_clock - ��ʼ������ͬ��ģʽʱ��
 *
 * ��������ͬ��ģʽʱ,��������
 *
 * @USARTx: ���ڼĴ���ָ�����
 * @conf: ͬ��ʱ������
 */
void usart_init_clock(usart_regs_t *USARTx, uint16 conf);

/************************************************/
/* ���ڵ�ͨ������
 */
typedef struct {
    uint32 baud_rate;               /* ������ */
    uint16 word_length;             /* �ֳ�,8|9λ */
    uint16 stop_bits;               /* ֹͣλ�� */
    uint16 parity;                  /* ��żУ�� */
    uint16 mode;                    /* �շ�ģʽ */
    uint16 hardwareFlowControl;     /* Ӳ�������� */
} usart_conf_t;

/* �ֳ� */
#define USART_WordLength_8  ((uint16)0x0000)    /* 8λ�ֳ� */
#define USART_WordLength_9  ((uint16)0x1000)    /* 9 */
#define is_usart_wordlength(p) (((p) == USART_WordLength_8) || ((p) == USART_WordLength_9))
/* ֹͣλ */
#define USART_StopBits_1   ((uint16)0x0000)     /* 1��ֹͣλ */
#define USART_StopBits_0_5 ((uint16)0x1000)     /* 0.5�� */
#define USART_StopBits_2   ((uint16)0x2000)     /* 2 */
#define USART_StopBits_1_5 ((uint16)0x3000)     /* 1.5 */
#define is_usart_stop_bits(p) (((p) == USART_StopBits_1) || \
                               ((p) == USART_StopBits_0_5) || \
                               ((p) == USART_StopBits_2) || \
                               ((p) == USART_StopBits_1_5))
/* ��żУ�� */
#define USART_Parity_No     ((uint16)0x0000)
#define USART_Parity_Even   ((uint16)0x0400)
#define USART_Parity_Odd    ((uint16)0x0600) 
#define is_usart_parity(p) (((p) == USART_Parity_No) || \
                            ((p) == USART_Parity_Even) || \
                            ((p) == USART_Parity_Odd))
/*  �շ�ģʽ */
#define USART_Mode_Rx       ((uint16)0x0004)
#define USART_Mode_Tx       ((uint16)0x0008)
#define USART_Mode_Both     ((uint32)0x000C)
#define is_usart_mode(p) ((0 != p) && (0 == ((p) & (~USART_Mode_Both))))
/* Ӳ�������� */
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
* usart_init - ��ʼ������
*
* @USARTx: Ŀ�괮��
* @conf: ������
*/
void usart_init(usart_regs_t *USARTx, const usart_conf_t *conf);

/*
 * usart_enable - ʹ�ܴ���
 *
 * @USARTx: Ŀ�괮��
 * @enable: �Ƿ�ʹ��
 */
void usart_switch(usart_regs_t *USARTx, uint16 enable);

/************************************************/
/* ���ڵ��ж�����
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
 * usart_it_config - ���ô����ж�
 *
 * @USARTx: Ŀ�괮�ڵ�ַ����
 * @it: �����ж���
 * @enable: �Ƿ�ʹ��
 */
void usart_it_config(usart_regs_t *USARTx, uint16 it, uint16 enable);

/************************************************/
/* ���ڵ�״̬
 */
#define is_usart_flag(FLAG) (((FLAG) == USART_SR_PE) || ((FLAG) == USART_SR_TXE) || \
                             ((FLAG) == USART_SR_TC) || ((FLAG) == USART_SR_RXNE) || \
                             ((FLAG) == USART_SR_IDLE) || ((FLAG) == USART_SR_LBD) || \
                             ((FLAG) == USART_SR_CTS) || ((FLAG) == USART_SR_ORE) || \
                             ((FLAG) == USART_SR_NE) || ((FLAG) == USART_SR_FE))

/*
 * usart_get_flag_status - ��ȡ���ڱ��λ״̬
 *
 * @USARTx: Ŀ�괮��
 * @flag: ��ʶ
 */
uint16 usart_get_flag_status(usart_regs_t* USARTx, uint16 flag);

#endif
