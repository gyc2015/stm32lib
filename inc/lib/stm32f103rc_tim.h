/*
 * STM32F103有8个计时器,其中Tim1和Tim8为Advanced Timer,
 * Tim2~Tim5为General Timer, Tim6和Tim7为Basic Timer.
 *
 * TIMER主要是由三部分组成:
 * 1. 时基单元
 * 2. 输入捕获
 * 3. 输出比较
 * 有两种模式控制功能:从模式控制和主模式控制
 *
 * 影子寄存器(Shadow Register),指物理上存在两个寄存器。一个是程序员可以访问的寄存器,称为预装载
 * 寄存器(Preload Register);一个是程序员不可见的,但是在操作中真正起作用的寄存器称为shadow register
 * 
 * 计时器的时钟源有4种:
 * 1. 内部时钟(CK_INT),定时器的时钟不是直接来自APB1或APB2,而是来自于输入为APB1或APB2的一个倍频器
 * 2. 外部输入引脚(external input pin)
 * 3. 外部输入触发ETR
 * 4. 内部触发(ITRx):一般用来配置一个时钟作为另一个时钟的预分频器
 *
 * 每个捕获比较通道都是建立在一个捕获比较寄存器(及一个引子寄存器),
 * 一个捕获输入组(包含一个数字滤波器、Multiplexing和一个分频器)
 * 一个输出组(含比较器和输出控制器)
 *
 * 更新事件(Update Event, UEV)在由如下的事件之一触发:
 * 1. 计数器溢出
 * 2. 设置UG位
 * 3. 由从模式控制器产生的更新
 * 在使能了更新事件并产生UEV后,影子寄存器(Shadow Register,那些具有缓存的寄存器)被装载新值
 * 屏蔽了UEV后,影子寄存器保留原值(ARR,PSC,CCRx),但是当UG置位或者接收到从模式控制器的复位
 * 信号后,将更新计数器和分频器.
 */


#ifndef STM32F103RC_TIM_H
#define STM32F103RC_TIM_H

#include <stm32f103rc.h>

typedef struct {
    volatile uint16 CR1;    /* 控制寄存器1 */
    uint16  RESERVED0;
    volatile uint16 CR2;    /* 控制寄存器2 */
    uint16  RESERVED1;
    volatile uint16 SMCR;   /* 从模式控制 */
    uint16  RESERVED2;
    volatile uint16 DIER;   /* DMA/中断使能 */
    uint16  RESERVED3;
    volatile uint16 SR;     /* 状态寄存器 */
    uint16  RESERVED4;
    volatile uint16 EGR;    /* 时间生成寄存器 */
    uint16  RESERVED5;
    volatile uint16 CCMR1;  /* 捕获/比较模式寄存器1 */
    uint16  RESERVED6;
    volatile uint16 CCMR2;  /* 捕获/比较模式寄存器2 */
    uint16  RESERVED7;
    volatile uint16 CCER;   /* 捕获/比较使能寄存器 */
    uint16  RESERVED8;
    volatile uint16 CNT;    /* 计数器 */
    uint16  RESERVED9;
    volatile uint16 PSC;    /* 分频器 */
    uint16  RESERVED10;
    volatile uint16 ARR;    /* 自动重装寄存器 */
    uint16  RESERVED11;
    volatile uint16 RCR;    /* 重复计数寄存器 */
    uint16  RESERVED12;
    volatile uint16 CCR1;   /* 捕获/比较寄存器1 */
    uint16  RESERVED13;
    volatile uint16 CCR2;   /* 捕获/比较寄存器2 */
    uint16  RESERVED14;
    volatile uint16 CCR3;   /* 捕获/比较寄存器3 */
    uint16  RESERVED15;
    volatile uint16 CCR4;   /* 捕获/比较寄存器4 */
    uint16  RESERVED16;
    volatile uint16 BDTR;   /* 刹车和死区寄存器 */
    uint16  RESERVED17;
    volatile uint16 DCR;    /* DMA控制寄存器 */
    uint16  RESERVED18;
    volatile uint16 DMAR;   /* 连续模式的DMA地址 */
    uint16  RESERVED19;
} tim_regs_t;

/* TIM寄存器地址映射 */
#define TIM1_BASE   (APB2_BASE + 0x2C00)
#define TIM2_BASE   (APB1_BASE + 0x0000)
#define TIM3_BASE   (APB1_BASE + 0x0400)
#define TIM4_BASE   (APB1_BASE + 0x0800)
#define TIM5_BASE   (APB1_BASE + 0x0C00)
#define TIM6_BASE   (APB1_BASE + 0x1000)
#define TIM7_BASE   (APB1_BASE + 0x1400)
#define TIM8_BASE   (APB2_BASE + 0x3400)
/* TIM寄存器指针访问 */
#define TIM1  ((tim_regs_t *) TIM1_BASE)
#define TIM2  ((tim_regs_t *) TIM2_BASE)
#define TIM3  ((tim_regs_t *) TIM3_BASE)
#define TIM4  ((tim_regs_t *) TIM4_BASE)
#define TIM5  ((tim_regs_t *) TIM5_BASE)
#define TIM6  ((tim_regs_t *) TIM6_BASE)
#define TIM7  ((tim_regs_t *) TIM7_BASE)
#define TIM8  ((tim_regs_t *) TIM8_BASE)
#define is_advanced_timer(p) ((p) == TIM1 || (p) == TIM8)
#define is_general_timer(p) ((p) == TIM2 || (p) == TIM3 || (p) == TIM4 || (p) == TIM5 )
#define is_basic_timer(p) ((p) == TIM6 || (p) == TIM7)
#define is_timer(p) (is_advanced_timer(p) || is_general_timer(p) || is_basic_timer(p))

/*
 * 控制寄存器1 TIM_CR1
 * 偏移地址: 0x00
 * 复位值: 0x0000
 */
#define TIM_CR1_CEN         ((uint16)0x0001)    /* 使能计数器,外部时钟、门控模式和编码器模式只能在设置了该位后工作,触发模式可以由硬件处理 */
#define TIM_CR1_UDIS        ((uint16)0x0002)    /* 禁止更新,不产生UEV事件 */
#define TIM_CR1_URS         ((uint16)0x0004)    /* 更新请求源,只有计数器溢出产生更新中断和DMA请求 */
#define TIM_CR1_OPM         ((uint16)0x0008)    /* 单脉冲模式 */
#define TIM_CR1_DIR         ((uint16)0x0010)    /* 计数方向,1:向下计数,0:向上计数 */
#define TIM_CR1_CMS         ((uint16)0x0060)    /* 中央对齐模式,CMS[1:0],在计数器打开时(CEN=1),不能有边沿对齐模式切换为中央对齐模式 */
#define TIM_CR1_ARPE        ((uint16)0x0080)    /* TIMx_ARR寄存器是否拥有缓存 */
#define TIM_CR1_CKD         ((uint16)0x0300)    /* 时钟分频因子,CKD[1:0] */
/*
 * 控制寄存器2 TIM_CR2
 * 偏移地址: 0x04
 * 复位值: 0x0000
 */
#define TIM_CR2_CCPC        ((uint16)0x0001)    /* 捕获比较预装载控制,只对有互补输出的通道有效,CCxE,CCxNE和CCxM是预装载的,只有在COM时间(Commutation event)发生时才更新 */
#define TIM_CR2_CCUS        ((uint16)0x0004)    /* 捕获比较控制更新选择,当CCPC=1时,COMG置位或者TRGI上升沿都可以触发更新,否则只有COMG触发 */
#define TIM_CR2_CCDS        ((uint16)0x0008)    /* 捕获比较DMA选择,0:当CCx时间发生时请求DMA,1:当更新事件发生时 */

#define TIM_CR2_MMS         ((uint16)0x0070)    /* 选择在主模式下送到定时器的同步信息(trigger out,TRGO).MMS[2:0] */
#define TIM_CR2_MMS_Reset   ((uint16)0x0000)    /* TIMx_EGR寄存器的UG位被用作TRGO */
#define TIM_CR2_MMS_Enable  ((uint16)0x0010)    /* 计数器使能信号CNT_EN用作TRGO */
#define TIM_CR2_MMS_Update  ((uint16)0x0020)    /* 更新事件,此时主计时器可以作为从记事起的分频器 */
#define TIM_CR2_MMS_Pulse   ((uint16)0x0030)    /* When the CC1IF flag is to be set (even if it was already high), as soon as a capture or a compare match occurred. */
#define TIM_CR2_MMS_OC1REF  ((uint16)0x0040)    /* OC1REF */
#define TIM_CR2_MMS_OC2REF  ((uint16)0x0050)    /* OC2REF */
#define TIM_CR2_MMS_OC3REF  ((uint16)0x0060)    /* OC3REF */
#define TIM_CR2_MMS_OC4REF  ((uint16)0x0070)    /* OC4REF */

#define TIM_CR2_TI1S        ((uint16)0x0080)    /* TI1选择,0:TIMx_CH1引脚连到TI1输入,1:TIMx_CH1,TIMx_CH2和TIMx_CH3引脚经异或后连到TI1 */
#define TIM_CR2_OIS1        ((uint16)0x0100)    /* OC1输出, 0:当MOE=0时,OC1=0, 当MOE=0时,OC1=1 */
#define TIM_CR2_OIS1N       ((uint16)0x0200)    /* OC1N输出,0:当MOE=0时,OC1N=0,当MOE=0时,OC1N=1 */
#define TIM_CR2_OIS2        ((uint16)0x0400)    /* OC2输出 */
#define TIM_CR2_OIS2N       ((uint16)0x0800)    /* OC2N输出 */
#define TIM_CR2_OIS3        ((uint16)0x1000)    /* OC3输出 */
#define TIM_CR2_OIS3N       ((uint16)0x2000)    /* OC3N输出 */
#define TIM_CR2_OIS4        ((uint16)0x4000)    /* OC4输出 */
/*
 * 从模式控制寄存器 TIM_SMCR
 * 偏移地址: 0x08
 * 复位值: 0x0000
 */
#define TIM_SMCR_SMS            ((uint16)0x0007)    /* 从模式选择位,SMS[2:0] */
#define TIM_SMCR_SMS_Internal   ((uint16)0x0000)    /* 关闭从模式,分频器直接由内部时钟驱动*/
#define TIM_SMCR_SMS_Encoder1   ((uint16)0x0001)    /* 编码器模式1,根据TI1FP1的电平在TI2FP2边沿上/下计数 */
#define TIM_SMCR_SMS_Encoder2   ((uint16)0x0002)    /* 编码器模式2,根据TI2FP2的电平在TI1FP1边沿上/下计数 */
#define TIM_SMCR_SMS_Encoder3   ((uint16)0x0003)    /* 编码器模式3,根据其它输入在TI1FP1和TI2FP2的边沿上/下计数 */
#define TIM_SMCR_SMS_Reset      ((uint16)0x0004)    /* 复位模式,触发输入信号(TRGI)的上升沿更新寄存器 */
#define TIM_SMCR_SMS_Gate       ((uint16)0x0005)    /* 门控模式,TRGI高电平时使能计数器时钟,TRGI低电平时屏蔽计数器时钟,计数器的开始和结束都是可控的 */
#define TIM_SMCR_SMS_Trigger    ((uint16)0x0006)    /* 触发模式,TRGI高电平时使能计数器时钟,只有计数器的开始是可控的 */
#define TIM_SMCR_SMS_External   ((uint16)0x0007)    /* TRGI的上升沿驱动计数器 */

#define TIM_SMCR_TS             ((uint16)0x0070)    /* 触发信号选择,TS[2:0] */
#define TIM_SMCR_TS_ITR0        ((uint16)0x0000)    /* 内部触发信号0 */
#define TIM_SMCR_TS_ITR1        ((uint16)0x0010)    /*            1 */
#define TIM_SMCR_TS_ITR2        ((uint16)0x0020)    /*            2 */
#define TIM_SMCR_TS_ITR3        ((uint16)0x0030)    /*            3 */
#define TIM_SMCR_TS_TI1F_EN     ((uint16)0x0040)    /* TI1边沿检测器Edge Detector, TI1F_ED */
#define TIM_SMCR_TS_TI1FP1      ((uint16)0x0050)    /* 滤波后的TI1输入1,TI1FP1 */
#define TIM_SMCR_TS_TI2FP2      ((uint16)0x0060)    /* 滤波后的TI2输入2,TI2FP2 */
#define TIM_SMCR_TS_ETRF        ((uint16)0x0070)    /* 外部触发输入 */

#define TIM_SMCR_MSM            ((uint16)0x0080)    /* 主从模式,TRGI上的事件被延迟了,以允许当前定时器TRGO与其从定时器同步,适用于多个定时器同步于一个时间的场合 */

#define TIM_SMCR_ETF            ((uint16)0x0F00)    /* 外部触发信号滤波器ETF[3:0] */
#define TIM_Ext_Trg_Filter_No   ((uint16)0x0000)    /* 外部触发信号无滤波器f_dts */
#define TIM_Ext_Trg_Filter_1_2  ((uint16)0x0100)    /* f_sam=f_ckint,  N=2 */
#define TIM_Ext_Trg_Filter_1_4  ((uint16)0x0200)    /* f_sam=f_ckint,  N=4 */
#define TIM_Ext_Trg_Filter_2_6  ((uint16)0x0400)    /* f_sam=f_dts/2,  N=6 */
#define TIM_Ext_Trg_Filter_2_8  ((uint16)0x0500)    /* f_sam=f_dts/2,  N=8 */
#define TIM_Ext_Trg_Filter_4_6  ((uint16)0x0600)    /* f_sam=f_dts/4,  N=6 */
#define TIM_Ext_Trg_Filter_4_8  ((uint16)0x0700)    /* f_sam=f_dts/4,  N=8 */
#define TIM_Ext_Trg_Filter_8_6  ((uint16)0x0800)    /* f_sam=f_dts/8,  N=6 */
#define TIM_Ext_Trg_Filter_8_8  ((uint16)0x0900)    /* f_sam=f_dts/8,  N=8 */
#define TIM_Ext_Trg_Filter_16_5 ((uint16)0x0A00)    /* f_sam=f_dts/16, N=5 */
#define TIM_Ext_Trg_Filter_16_6 ((uint16)0x0B00)    /* f_sam=f_dts/16, N=6 */
#define TIM_Ext_Trg_Filter_16_8 ((uint16)0x0C00)    /* f_sam=f_dts/16, N=8 */
#define TIM_Ext_Trg_Filter_32_5 ((uint16)0x0D00)    /* f_sam=f_dts/32, N=5 */
#define TIM_Ext_Trg_Filter_32_6 ((uint16)0x0E00)    /* f_sam=f_dts/32, N=6 */
#define TIM_Ext_Trg_Filter_32_8 ((uint16)0x0F00)    /* f_sam=f_dts/32, N=8 */

#define TIM_SMCR_ETPS           ((uint16)0x3000)    /* 外部触发信号分频器ETPS[1:0],ETRP信号频率必须是TIMxCLK频率的1/4 */
#define TIM_Ext_Trg_Prsc_No     ((uint16)0x0000)    /* 不需要预分频 */
#define TIM_Ext_Trg_Prsc_2      ((uint16)0x1000)    /* 2分频 */
#define TIM_Ext_Trg_Prsc_4      ((uint16)0x2000)    /* 4 */
#define TIM_Ext_Trg_Prsc_8      ((uint16)0x3000)    /* 8 */

#define TIM_SMCR_ECE            ((uint16)0x4000)    /* 启用外部时钟模式2, 计数器由ETRF信号上的任一边沿驱动 */
#define TIM_SMCR_ETP            ((uint16)0x8000)    /* 外部时钟极性,ETR被反相,低电平或下降沿有效 */
/*
 * DMA/中断使能寄存器 TIM_DIER
 * 偏移地址: 0x0C
 * 复位值: 0x0000
 */
#define TIM_DIER_UIE    ((uint16)0x0001)    /* 更新中断使能,   Update interrupt enable */
#define TIM_DIER_CC1IE  ((uint16)0x0002)    /* 捕获/比较1中断, Capture/Compare 1 interrupt enable */
#define TIM_DIER_CC2IE  ((uint16)0x0004)    /* 捕获/比较2中断, Capture/Compare 2 interrupt enable */
#define TIM_DIER_CC3IE  ((uint16)0x0008)    /* 捕获/比较3中断, Capture/Compare 3 interrupt enable */
#define TIM_DIER_CC4IE  ((uint16)0x0010)    /* 捕获/比较4中断, Capture/Compare 4 interrupt enable */
#define TIM_DIER_COMIE  ((uint16)0x0020)    /* COM中断, COM interrupt enable */
#define TIM_DIER_TIE    ((uint16)0x0040)    /* 触发中断, Trigger interrupt enable */
#define TIM_DIER_BIE    ((uint16)0x0080)    /* 刹车中断, Break interrupt enable */
#define TIM_DIER_UDE    ((uint16)0x0100)    /* 更新DMA请求, Update DMA request enable */
#define TIM_DIER_CC1DE  ((uint16)0x0200)    /* 捕获/比较1, Capture/Compare 1 DMA request enable */
#define TIM_DIER_CC2DE  ((uint16)0x0400)    /* 捕获/比较2, Capture/Compare 2 DMA request enable */
#define TIM_DIER_CC3DE  ((uint16)0x0800)    /* 捕获/比较3, Capture/Compare 3 DMA request enable */
#define TIM_DIER_CC4DE  ((uint16)0x1000)    /* 捕获/比较4, Capture/Compare 4 DMA request enable */
#define TIM_DIER_COMDE  ((uint16)0x2000)    /* COM DMA request enable */
#define TIM_DIER_TDE    ((uint16)0x4000)    /* Trigger DMA request enable */
/*
 * 状态寄存器 TIM_SR
 * 偏移地址: 0x10
 * 复位值: 0x0000
 */
#define TIM_SR_UIF      ((uint16)0x0001)    /*!< Update interrupt Flag */
#define TIM_SR_CC1IF    ((uint16)0x0002)    /*!< Capture/Compare 1 interrupt Flag */
#define TIM_SR_CC2IF    ((uint16)0x0004)    /*!< Capture/Compare 2 interrupt Flag */
#define TIM_SR_CC3IF    ((uint16)0x0008)    /*!< Capture/Compare 3 interrupt Flag */
#define TIM_SR_CC4IF    ((uint16)0x0010)    /*!< Capture/Compare 4 interrupt Flag */
#define TIM_SR_COMIF    ((uint16)0x0020)    /*!< COM interrupt Flag */
#define TIM_SR_TIF      ((uint16)0x0040)    /*!< Trigger interrupt Flag */
#define TIM_SR_BIF      ((uint16)0x0080)    /*!< Break interrupt Flag */
#define TIM_SR_CC1OF    ((uint16)0x0200)    /*!< Capture/Compare 1 Overcapture Flag */
#define TIM_SR_CC2OF    ((uint16)0x0400)    /*!< Capture/Compare 2 Overcapture Flag */
#define TIM_SR_CC3OF    ((uint16)0x0800)    /*!< Capture/Compare 3 Overcapture Flag */
#define TIM_SR_CC4OF    ((uint16)0x1000)    /*!< Capture/Compare 4 Overcapture Flag */
/*
 * 事件产生寄存器 TIM_EGR
 * 偏移地址: 0x14
 * 复位值: 0x0000
 */
#define TIM_EGR_UG      ((uint16)0x01)      /*!< Update Generation */
#define TIM_EGR_CC1G    ((uint16)0x02)      /*!< Capture/Compare 1 Generation */
#define TIM_EGR_CC2G    ((uint16)0x04)      /*!< Capture/Compare 2 Generation */
#define TIM_EGR_CC3G    ((uint16)0x08)      /*!< Capture/Compare 3 Generation */
#define TIM_EGR_CC4G    ((uint16)0x10)      /*!< Capture/Compare 4 Generation */
#define TIM_EGR_COMG    ((uint16)0x20)      /*!< Capture/Compare Control Update Generation */
#define TIM_EGR_TG      ((uint16)0x40)      /*!< Trigger Generation */
#define TIM_EGR_BG      ((uint16)0x80)      /*!< Break Generation */
/*
 * 捕获/比较模式寄存器1 TIM_CCMR1
 * 偏移地址: 0x18
 * 复位值: 0x0000
 */
#define TIM_CCMR1_CC1S      ((uint16)0x0003)    /* 输出比较/输入捕获选择位 CC1S[1:0] */
#define TIM_CCMR1_OC1FE     ((uint16)0x0004)    /* 输出比较1快速模式,只对PWM1和PWM2模式有效 */
#define TIM_CCMR1_OC1PE     ((uint16)0x0008)    /* 开启输出比较1预装载功能,只有在更新事件到来时,TIMx_CCR1影子寄存器才装载新值 */
#define TIM_CCMR1_OC1M      ((uint16)0x0070)    /* 输出比较1模式,OC1M[2:0] */
#define TIM_CCMR1_OC1CE     ((uint16)0x0080)    /* 一旦检测到ETRF输入高电平时,清除OC1REF=0 */

#define TIM_CCMR1_CC2S      ((uint16)0x0300)    /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_OC2FE     ((uint16)0x0400)    /*!< Output Compare 2 Fast enable */
#define TIM_CCMR1_OC2PE     ((uint16)0x0800)    /*!< Output Compare 2 Preload enable */
#define TIM_CCMR1_OC2M      ((uint16)0x7000)    /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2CE     ((uint16)0x8000)    /*!< Output Compare 2 Clear Enable */

#define TIM_CCMR1_IC1PSC    ((uint16)0x000C)    /* 预分频器,IC1PSC[1:0] */
#define TIM_CCMR1_IC1F      ((uint16)0x00F0)    /* 滤波器,IC1F[3:0] */
#define TIM_CCMR1_IC2PSC    ((uint16)0x0C00)    /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2F      ((uint16)0xF000)    /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
/*
 * 捕获/比较模式寄存器2 TIM_CCMR2
 * 偏移地址: 0x1C
 * 复位值: 0x0000
 */
#define TIM_CCMR2_CC3S      ((uint16)0x0003)    /* 输出比较/输入捕获选择位 CC1S[1:0] */
#define TIM_CCMR2_OC3FE     ((uint16)0x0004)    /* 输出比较1快速模式,只对PWM1和PWM2模式有效 */
#define TIM_CCMR2_OC3PE     ((uint16)0x0008)    /* 开启输出比较1预装载功能,只有在更新事件到来时,TIMx_CCR1影子寄存器才装载新值 */
#define TIM_CCMR2_OC3M      ((uint16)0x0070)    /* 输出比较1模式,OC1M[2:0] */
#define TIM_CCMR2_OC3CE     ((uint16)0x0080)    /* 一旦检测到ETRF输入高电平时,清除OC1REF=0 */

#define TIM_CCMR2_CC4S      ((uint16)0x0300)    /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR2_OC4FE     ((uint16)0x0400)    /*!< Output Compare 2 Fast enable */
#define TIM_CCMR2_OC4PE     ((uint16)0x0800)    /*!< Output Compare 2 Preload enable */
#define TIM_CCMR2_OC4M      ((uint16)0x7000)    /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR2_OC4CE     ((uint16)0x8000)    /*!< Output Compare 2 Clear Enable */

#define TIM_CCMR2_IC3PSC    ((uint16)0x000C)    /* 预分频器,IC1PSC[1:0] */
#define TIM_CCMR2_IC3F      ((uint16)0x00F0)    /* 滤波器,IC1F[3:0] */
#define TIM_CCMR2_IC4PSC    ((uint16)0x0C00)    /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR2_IC4F      ((uint16)0xF000)    /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
/*
 * 捕获/比较使能寄存器 TIM_CCER
 * 偏移地址: 0x20
 * 复位值: 0x0000
 */
#define TIM_CCER_CC1E       ((uint16)0x0001)    /* 开启通道1 */
#define TIM_CCER_CC1P       ((uint16)0x0002)    /* 输出:0-OC1高电平,1-OC1低电平;输入:0-上升沿,用作外部触发信号(external trigger)时不反相,1-下降沿,反相 */
#define TIM_CCER_CC1NE      ((uint16)0x0004)    /* 开启通道1的互补输出通道 */
#define TIM_CCER_CC1NP      ((uint16)0x0008)    /* 0-OC1高电平,1-OC1低电平 */
#define TIM_CCER_CC2E       ((uint16)0x0010)    /*!< Capture/Compare 2 output enable */
#define TIM_CCER_CC2P       ((uint16)0x0020)    /*!< Capture/Compare 2 output Polarity */
#define TIM_CCER_CC2NE      ((uint16)0x0040)    /*!< Capture/Compare 2 Complementary output enable */
#define TIM_CCER_CC2NP      ((uint16)0x0080)    /*!< Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E       ((uint16)0x0100)    /*!< Capture/Compare 3 output enable */
#define TIM_CCER_CC3P       ((uint16)0x0200)    /*!< Capture/Compare 3 output Polarity */
#define TIM_CCER_CC3NE      ((uint16)0x0400)    /*!< Capture/Compare 3 Complementary output enable */
#define TIM_CCER_CC3NP      ((uint16)0x0800)    /*!< Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E       ((uint16)0x1000)    /*!< Capture/Compare 4 output enable */
#define TIM_CCER_CC4P       ((uint16)0x2000)    /*!< Capture/Compare 4 output Polarity */
#define TIM_CCER_CC4NP      ((uint16)0x8000)    /*!< Capture/Compare 4 Complementary output Polarity */
/*
 * 刹车和死区寄存器 TIM_BDTR
 * 偏移地址: 0x44
 * 复位值: 0x0000
 */
#define TIM_BDTR_DTG    ((uint16)0x00FF)    /* 死区发生器设置 DTG[0:7] */

#define TIM_BDTR_LOCK   ((uint16)0x0300)    /* 锁定配置 LOCK[1:0] */
#define TIM_Lock_off    ((uint16)0x0000)    /* 关闭寄存器锁定 */
#define TIM_Lock_Lv1    ((uint16)0x0100)    /* 锁定级别1,BDTR的DTG、CR2的OISx和OISxN位、BDTR的BKE/BKP/AOE位 */
#define TIM_Lock_Lv2    ((uint16)0x0200)    /* 锁定级别2,包括级别1的各位和CC极性位、OSSR/OSSI位 */
#define TIM_Lock_Lv3    ((uint16)0x0300)    /* 锁定级别3,包括级别2的各位和CC控制位 */

#define TIM_BDTR_OSSI   ((uint16)0x0400)    /* 空闲模式下"关闭状态"的选择 */
#define TIM_BDTR_OSSR   ((uint16)0x0800)    /* 运行状态下"关闭状态"的选择 */
#define TIM_BDTR_BKE    ((uint16)0x1000)    /* 使能刹车功能*/
#define TIM_BDTR_BKP    ((uint16)0x2000)    /* 刹车输入极性,1:高电平有效,0:低电平有效 */
#define TIM_BDTR_AOE    ((uint16)0x4000)    /* 自动输出使能,0:MOE只能软件置1,1:MOE还可以在下一个更新事件自动置1 */
#define TIM_BDTR_MOE    ((uint16)0x8000)    /* 主动输出使能,0:OC和OCN输出禁止,1:开启OC和OCN */
/*
 * DMA控制寄存器 TIM_DCR
 * 偏移地址: 0x48
 * 复位值: 0x0000
 */
#define TIM_DCR_DBA     ((uint16)0x001F)    /*!< DMA基地址 DBA[4:0] */
#define TIM_DCR_DBL     ((uint16)0x1F00)    /*!< DMA连续传送长度 DBL[4:0] */

/************************************************************************************/
/* 计数器时基单元
 */
typedef struct {
    uint16 ar_value;        /* 自动重载数值Auto-Reload Value,0x0000-0xFFFF */
    uint16 prsc;            /* 分频器, 0x0000-0xFFFF */
    uint16 counter_mode;    /* 计数方式 */
    uint16 ckd;             /* 时钟分频因子 */
    uint8  repeat_times;    /* 重复计数次数,只对TIM1和TIM8有效 0x00-0xFF*/
} tim_timebase_t;

/* 计数模式 */
#define TIM_Counter_EdgeDir_Up      ((uint16)0x0000)    /* 边沿对齐,向下计数 */
#define TIM_Counter_EdgeDir_Down    ((uint16)0x0010)    /* 边沿对齐,向上计数 */
#define TIM_Counter_CenterIF_Down   ((uint16)0x0020)    /* 峰值对齐,只在向下计数时设置比较中断标志 */
#define TIM_Counter_CenterIF_Up     ((uint16)0x0040)    /* 峰值对齐,    向上                     */
#define TIM_Counter_CenterIF_Both   ((uint16)0x0060)    /* 峰值对齐,  在两个方向上                */
#define is_tim_counter_mode(p) (((p) == TIM_Counter_EdgeDir_Up) ||  \
                                ((p) == TIM_Counter_EdgeDir_Down) || \
                                ((p) == TIM_Counter_CenterIF_Down) || \
                                ((p) == TIM_Counter_CenterIF_Up) || \
                                ((p) == TIM_Counter_CenterIF_Both))

/*
 * 时钟分频因子
 * 
 * 用于配置t_dts,t_dts与死区时间,死区发生器,数字滤波器的采样时钟有关
 */
#define TIM_CKD_1       ((uint16)0x0000)    /* t_dts =     t_ckint */
#define TIM_CKD_2       ((uint16)0x0100)    /* t_dts = 2 * t_ckint */
#define TIM_CKD_4       ((uint16)0x0200)    /* t_dts = 4 * t_ckint */

/*
 * tim_init_timebase - 初始化时基单元
 *
 * @TIMx: 目标计时器
 * @conf: 配置内容
 */
void tim_init_timebase(tim_regs_t *TIMx, const tim_timebase_t *conf);



/************************************************************************************/


#define TIM_CCMode_Out      ((uint16)0x0000)    /* 输出比较 */
#define TIM_CCMode_In_Self  ((uint16)0x0001)    /* ICx映射为TIx输入 */
#define TIM_CCMode_In_Mate  ((uint16)0x0002)    /* ICx映射为TIy输入,x和y为同组的连个通道 */
#define TIM_CCMode_In_TRC   ((uint16)0x0003)    /* ICx映射为TRC输入 */

#define TIM_OCMode_Frozen    ((uint16)0x0000)    /* 冻结模式,TIMx_CCR1与TIMx_CNT间的比较对OC1REF不起作用 */
#define TIM_OCMode_Match_Act ((uint16)0x0010)    /* TIMx_CCR1与TIMx_CNT匹配时,置通道1为有效电平 */
#define TIM_OCMode_Match_InA ((uint16)0x0020)    /*                                   无效电平 */
#define TIM_OCMode_Toggle    ((uint16)0x0030)    /* TIMx_CCR1=TIMx_CNT时OC1REF翻转 */
#define TIM_OCMode_Force_InA ((uint16)0x0040)    /* OC1REF强制低电平 */
#define TIM_OCMode_Force_Act ((uint16)0x0050)    /* OC1REF强制高电平 */
#define TIM_OCMode_PWM1      ((uint16)0x0060)    /* 向上计数时,CNT < CCR1高电平,否则低电平;向下计数时,CNT > CCR1低电平,否则高电平 */
#define TIM_OCMode_PWM2      ((uint16)0x0070)    /* 与PWM1反向 */

#define TIM_IC_Filter_No   ((uint16)0x00)    /* 输入捕获无滤波,f_dts */
#define TIM_IC_Filter_1_2  ((uint16)0x01)    /* f_sam=f_ckint,  N=2 */
#define TIM_IC_Filter_1_4  ((uint16)0x02)    /* f_sam=f_ckint,  N=4 */
#define TIM_IC_Filter_1_8  ((uint16)0x03)    /* f_sam=f_ckint,  N=8 */
#define TIM_IC_Filter_2_6  ((uint16)0x04)    /* f_sam=f_dts/2,  N=6 */
#define TIM_IC_Filter_2_8  ((uint16)0x05)    /* f_sam=f_dts/2,  N=8 */
#define TIM_IC_Filter_4_6  ((uint16)0x06)    /* f_sam=f_dts/4,  N=6 */
#define TIM_IC_Filter_4_8  ((uint16)0x07)    /* f_sam=f_dts/4,  N=8 */
#define TIM_IC_Filter_8_6  ((uint16)0x08)    /* f_sam=f_dts/8,  N=6 */
#define TIM_IC_Filter_8_8  ((uint16)0x09)    /* f_sam=f_dts/8,  N=8 */
#define TIM_IC_Filter_16_5 ((uint16)0x0A)    /* f_sam=f_dts/16, N=5 */
#define TIM_IC_Filter_16_6 ((uint16)0x0B)    /* f_sam=f_dts/16, N=6 */
#define TIM_IC_Filter_16_8 ((uint16)0x0C)    /* f_sam=f_dts/16, N=8 */
#define TIM_IC_Filter_32_5 ((uint16)0x0D)    /* f_sam=f_dts/32, N=5 */
#define TIM_IC_Filter_32_6 ((uint16)0x0E)    /* f_sam=f_dts/32, N=6 */
#define TIM_IC_Filter_32_8 ((uint16)0x0F)    /* f_sam=f_dts/32, N=8 */

#define TIM_IC_Prsc_No     ((uint16)0x0000)    /* 不需要预分频 */
#define TIM_IC_Prsc_2      ((uint16)0x1000)    /* 2 */
#define TIM_IC_Prsc_4      ((uint16)0x2000)    /* 4 */
#define TIM_IC_Prsc_8      ((uint16)0x3000)    /* 8 */


#endif