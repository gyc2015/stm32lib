/*
 * STM32F103��8����ʱ��,����Tim1��Tim8ΪAdvanced Timer,
 * Tim2~Tim5ΪGeneral Timer, Tim6��Tim7ΪBasic Timer.
 *
 * TIMER��Ҫ�������������:
 * 1. ʱ����Ԫ
 * 2. ���벶��
 * 3. ����Ƚ�
 * ������ģʽ���ƹ���:��ģʽ���ƺ���ģʽ����
 *
 * Ӱ�ӼĴ���(Shadow Register),ָ�����ϴ��������Ĵ�����һ���ǳ���Ա���Է��ʵļĴ���,��ΪԤװ��
 * �Ĵ���(Preload Register);һ���ǳ���Ա���ɼ���,�����ڲ��������������õļĴ�����Ϊshadow register
 * 
 * ��ʱ����ʱ��Դ��4��:
 * 1. �ڲ�ʱ��(CK_INT),��ʱ����ʱ�Ӳ���ֱ������APB1��APB2,��������������ΪAPB1��APB2��һ����Ƶ��
 * 2. �ⲿ��������(external input pin)
 * 3. �ⲿ���봥��ETR
 * 4. �ڲ�����(ITRx):һ����������һ��ʱ����Ϊ��һ��ʱ�ӵ�Ԥ��Ƶ��
 *
 * ÿ������Ƚ�ͨ�����ǽ�����һ������ȽϼĴ���(��һ�����ӼĴ���),
 * һ������������(����һ�������˲�����Multiplexing��һ����Ƶ��)
 * һ�������(���Ƚ��������������)
 *
 * �����¼�(Update Event, UEV)�������µ��¼�֮һ����:
 * 1. ���������
 * 2. ����UGλ
 * 3. �ɴ�ģʽ�����������ĸ���
 * ��ʹ���˸����¼�������UEV��,Ӱ�ӼĴ���(Shadow Register,��Щ���л���ļĴ���)��װ����ֵ
 * ������UEV��,Ӱ�ӼĴ�������ԭֵ(ARR,PSC,CCRx),���ǵ�UG��λ���߽��յ���ģʽ�������ĸ�λ
 * �źź�,�����¼������ͷ�Ƶ��.
 */


#ifndef STM32F103RC_TIM_H
#define STM32F103RC_TIM_H

#include <stm32f103rc.h>

typedef struct {
    volatile uint16 CR1;    /* ���ƼĴ���1 */
    uint16  RESERVED0;
    volatile uint16 CR2;    /* ���ƼĴ���2 */
    uint16  RESERVED1;
    volatile uint16 SMCR;   /* ��ģʽ���� */
    uint16  RESERVED2;
    volatile uint16 DIER;   /* DMA/�ж�ʹ�� */
    uint16  RESERVED3;
    volatile uint16 SR;     /* ״̬�Ĵ��� */
    uint16  RESERVED4;
    volatile uint16 EGR;    /* ʱ�����ɼĴ��� */
    uint16  RESERVED5;
    volatile uint16 CCMR1;  /* ����/�Ƚ�ģʽ�Ĵ���1 */
    uint16  RESERVED6;
    volatile uint16 CCMR2;  /* ����/�Ƚ�ģʽ�Ĵ���2 */
    uint16  RESERVED7;
    volatile uint16 CCER;   /* ����/�Ƚ�ʹ�ܼĴ��� */
    uint16  RESERVED8;
    volatile uint16 CNT;    /* ������ */
    uint16  RESERVED9;
    volatile uint16 PSC;    /* ��Ƶ�� */
    uint16  RESERVED10;
    volatile uint16 ARR;    /* �Զ���װ�Ĵ��� */
    uint16  RESERVED11;
    volatile uint16 RCR;    /* �ظ������Ĵ��� */
    uint16  RESERVED12;
    volatile uint16 CCR1;   /* ����/�ȽϼĴ���1 */
    uint16  RESERVED13;
    volatile uint16 CCR2;   /* ����/�ȽϼĴ���2 */
    uint16  RESERVED14;
    volatile uint16 CCR3;   /* ����/�ȽϼĴ���3 */
    uint16  RESERVED15;
    volatile uint16 CCR4;   /* ����/�ȽϼĴ���4 */
    uint16  RESERVED16;
    volatile uint16 BDTR;   /* ɲ���������Ĵ��� */
    uint16  RESERVED17;
    volatile uint16 DCR;    /* DMA���ƼĴ��� */
    uint16  RESERVED18;
    volatile uint16 DMAR;   /* ����ģʽ��DMA��ַ */
    uint16  RESERVED19;
} tim_regs_t;

/* TIM�Ĵ�����ַӳ�� */
#define TIM1_BASE   (APB2_BASE + 0x2C00)
#define TIM2_BASE   (APB1_BASE + 0x0000)
#define TIM3_BASE   (APB1_BASE + 0x0400)
#define TIM4_BASE   (APB1_BASE + 0x0800)
#define TIM5_BASE   (APB1_BASE + 0x0C00)
#define TIM6_BASE   (APB1_BASE + 0x1000)
#define TIM7_BASE   (APB1_BASE + 0x1400)
#define TIM8_BASE   (APB2_BASE + 0x3400)
/* TIM�Ĵ���ָ����� */
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
 * ���ƼĴ���1 TIM_CR1
 * ƫ�Ƶ�ַ: 0x00
 * ��λֵ: 0x0000
 */
#define TIM_CR1_CEN         ((uint16)0x0001)    /* ʹ�ܼ�����,�ⲿʱ�ӡ��ſ�ģʽ�ͱ�����ģʽֻ���������˸�λ����,����ģʽ������Ӳ������ */
#define TIM_CR1_UDIS        ((uint16)0x0002)    /* ��ֹ����,������UEV�¼� */
#define TIM_CR1_URS         ((uint16)0x0004)    /* ��������Դ,ֻ�м�����������������жϺ�DMA���� */
#define TIM_CR1_OPM         ((uint16)0x0008)    /* ������ģʽ */
#define TIM_CR1_DIR         ((uint16)0x0010)    /* ��������,1:���¼���,0:���ϼ��� */
#define TIM_CR1_CMS         ((uint16)0x0060)    /* �������ģʽ,CMS[1:0],�ڼ�������ʱ(CEN=1),�����б��ض���ģʽ�л�Ϊ�������ģʽ */
#define TIM_CR1_ARPE        ((uint16)0x0080)    /* TIMx_ARR�Ĵ����Ƿ�ӵ�л��� */
#define TIM_CR1_CKD         ((uint16)0x0300)    /* ʱ�ӷ�Ƶ����,CKD[1:0] */
/*
 * ���ƼĴ���2 TIM_CR2
 * ƫ�Ƶ�ַ: 0x04
 * ��λֵ: 0x0000
 */
#define TIM_CR2_CCPC        ((uint16)0x0001)    /* ����Ƚ�Ԥװ�ؿ���,ֻ���л��������ͨ����Ч,CCxE,CCxNE��CCxM��Ԥװ�ص�,ֻ����COMʱ��(Commutation event)����ʱ�Ÿ��� */
#define TIM_CR2_CCUS        ((uint16)0x0004)    /* ����ȽϿ��Ƹ���ѡ��,��CCPC=1ʱ,COMG��λ����TRGI�����ض����Դ�������,����ֻ��COMG���� */
#define TIM_CR2_CCDS        ((uint16)0x0008)    /* ����Ƚ�DMAѡ��,0:��CCxʱ�䷢��ʱ����DMA,1:�������¼�����ʱ */

#define TIM_CR2_MMS         ((uint16)0x0070)    /* ѡ������ģʽ���͵���ʱ����ͬ����Ϣ(trigger out,TRGO).MMS[2:0] */
#define TIM_CR2_MMS_Reset   ((uint16)0x0000)    /* TIMx_EGR�Ĵ�����UGλ������TRGO */
#define TIM_CR2_MMS_Enable  ((uint16)0x0010)    /* ������ʹ���ź�CNT_EN����TRGO */
#define TIM_CR2_MMS_Update  ((uint16)0x0020)    /* �����¼�,��ʱ����ʱ��������Ϊ�Ӽ�����ķ�Ƶ�� */
#define TIM_CR2_MMS_Pulse   ((uint16)0x0030)    /* When the CC1IF flag is to be set (even if it was already high), as soon as a capture or a compare match occurred. */
#define TIM_CR2_MMS_OC1REF  ((uint16)0x0040)    /* OC1REF */
#define TIM_CR2_MMS_OC2REF  ((uint16)0x0050)    /* OC2REF */
#define TIM_CR2_MMS_OC3REF  ((uint16)0x0060)    /* OC3REF */
#define TIM_CR2_MMS_OC4REF  ((uint16)0x0070)    /* OC4REF */

#define TIM_CR2_TI1S        ((uint16)0x0080)    /* TI1ѡ��,0:TIMx_CH1��������TI1����,1:TIMx_CH1,TIMx_CH2��TIMx_CH3���ž���������TI1 */
#define TIM_CR2_OIS1        ((uint16)0x0100)    /* OC1���, 0:��MOE=0ʱ,OC1=0, ��MOE=0ʱ,OC1=1 */
#define TIM_CR2_OIS1N       ((uint16)0x0200)    /* OC1N���,0:��MOE=0ʱ,OC1N=0,��MOE=0ʱ,OC1N=1 */
#define TIM_CR2_OIS2        ((uint16)0x0400)    /* OC2��� */
#define TIM_CR2_OIS2N       ((uint16)0x0800)    /* OC2N��� */
#define TIM_CR2_OIS3        ((uint16)0x1000)    /* OC3��� */
#define TIM_CR2_OIS3N       ((uint16)0x2000)    /* OC3N��� */
#define TIM_CR2_OIS4        ((uint16)0x4000)    /* OC4��� */
/*
 * ��ģʽ���ƼĴ��� TIM_SMCR
 * ƫ�Ƶ�ַ: 0x08
 * ��λֵ: 0x0000
 */
#define TIM_SMCR_SMS            ((uint16)0x0007)    /* ��ģʽѡ��λ,SMS[2:0] */
#define TIM_SMCR_SMS_Internal   ((uint16)0x0000)    /* �رմ�ģʽ,��Ƶ��ֱ�����ڲ�ʱ������*/
#define TIM_SMCR_SMS_Encoder1   ((uint16)0x0001)    /* ������ģʽ1,����TI1FP1�ĵ�ƽ��TI2FP2������/�¼��� */
#define TIM_SMCR_SMS_Encoder2   ((uint16)0x0002)    /* ������ģʽ2,����TI2FP2�ĵ�ƽ��TI1FP1������/�¼��� */
#define TIM_SMCR_SMS_Encoder3   ((uint16)0x0003)    /* ������ģʽ3,��������������TI1FP1��TI2FP2�ı�����/�¼��� */
#define TIM_SMCR_SMS_Reset      ((uint16)0x0004)    /* ��λģʽ,���������ź�(TRGI)�������ظ��¼Ĵ��� */
#define TIM_SMCR_SMS_Gate       ((uint16)0x0005)    /* �ſ�ģʽ,TRGI�ߵ�ƽʱʹ�ܼ�����ʱ��,TRGI�͵�ƽʱ���μ�����ʱ��,�������Ŀ�ʼ�ͽ������ǿɿص� */
#define TIM_SMCR_SMS_Trigger    ((uint16)0x0006)    /* ����ģʽ,TRGI�ߵ�ƽʱʹ�ܼ�����ʱ��,ֻ�м������Ŀ�ʼ�ǿɿص� */
#define TIM_SMCR_SMS_External   ((uint16)0x0007)    /* TRGI������������������ */

#define TIM_SMCR_TS             ((uint16)0x0070)    /* �����ź�ѡ��,TS[2:0] */
#define TIM_SMCR_TS_ITR0        ((uint16)0x0000)    /* �ڲ������ź�0 */
#define TIM_SMCR_TS_ITR1        ((uint16)0x0010)    /*            1 */
#define TIM_SMCR_TS_ITR2        ((uint16)0x0020)    /*            2 */
#define TIM_SMCR_TS_ITR3        ((uint16)0x0030)    /*            3 */
#define TIM_SMCR_TS_TI1F_EN     ((uint16)0x0040)    /* TI1���ؼ����Edge Detector, TI1F_ED */
#define TIM_SMCR_TS_TI1FP1      ((uint16)0x0050)    /* �˲����TI1����1,TI1FP1 */
#define TIM_SMCR_TS_TI2FP2      ((uint16)0x0060)    /* �˲����TI2����2,TI2FP2 */
#define TIM_SMCR_TS_ETRF        ((uint16)0x0070)    /* �ⲿ�������� */

#define TIM_SMCR_MSM            ((uint16)0x0080)    /* ����ģʽ,TRGI�ϵ��¼����ӳ���,������ǰ��ʱ��TRGO����Ӷ�ʱ��ͬ��,�����ڶ����ʱ��ͬ����һ��ʱ��ĳ��� */

#define TIM_SMCR_ETF            ((uint16)0x0F00)    /* �ⲿ�����ź��˲���ETF[3:0] */
#define TIM_Ext_Trg_Filter_No   ((uint16)0x0000)    /* �ⲿ�����ź����˲���f_dts */
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

#define TIM_SMCR_ETPS           ((uint16)0x3000)    /* �ⲿ�����źŷ�Ƶ��ETPS[1:0],ETRP�ź�Ƶ�ʱ�����TIMxCLKƵ�ʵ�1/4 */
#define TIM_Ext_Trg_Prsc_No     ((uint16)0x0000)    /* ����ҪԤ��Ƶ */
#define TIM_Ext_Trg_Prsc_2      ((uint16)0x1000)    /* 2��Ƶ */
#define TIM_Ext_Trg_Prsc_4      ((uint16)0x2000)    /* 4 */
#define TIM_Ext_Trg_Prsc_8      ((uint16)0x3000)    /* 8 */

#define TIM_SMCR_ECE            ((uint16)0x4000)    /* �����ⲿʱ��ģʽ2, ��������ETRF�ź��ϵ���һ�������� */
#define TIM_SMCR_ETP            ((uint16)0x8000)    /* �ⲿʱ�Ӽ���,ETR������,�͵�ƽ���½�����Ч */
/*
 * DMA/�ж�ʹ�ܼĴ��� TIM_DIER
 * ƫ�Ƶ�ַ: 0x0C
 * ��λֵ: 0x0000
 */
#define TIM_DIER_UIE    ((uint16)0x0001)    /* �����ж�ʹ��,   Update interrupt enable */
#define TIM_DIER_CC1IE  ((uint16)0x0002)    /* ����/�Ƚ�1�ж�, Capture/Compare 1 interrupt enable */
#define TIM_DIER_CC2IE  ((uint16)0x0004)    /* ����/�Ƚ�2�ж�, Capture/Compare 2 interrupt enable */
#define TIM_DIER_CC3IE  ((uint16)0x0008)    /* ����/�Ƚ�3�ж�, Capture/Compare 3 interrupt enable */
#define TIM_DIER_CC4IE  ((uint16)0x0010)    /* ����/�Ƚ�4�ж�, Capture/Compare 4 interrupt enable */
#define TIM_DIER_COMIE  ((uint16)0x0020)    /* COM�ж�, COM interrupt enable */
#define TIM_DIER_TIE    ((uint16)0x0040)    /* �����ж�, Trigger interrupt enable */
#define TIM_DIER_BIE    ((uint16)0x0080)    /* ɲ���ж�, Break interrupt enable */
#define TIM_DIER_UDE    ((uint16)0x0100)    /* ����DMA����, Update DMA request enable */
#define TIM_DIER_CC1DE  ((uint16)0x0200)    /* ����/�Ƚ�1, Capture/Compare 1 DMA request enable */
#define TIM_DIER_CC2DE  ((uint16)0x0400)    /* ����/�Ƚ�2, Capture/Compare 2 DMA request enable */
#define TIM_DIER_CC3DE  ((uint16)0x0800)    /* ����/�Ƚ�3, Capture/Compare 3 DMA request enable */
#define TIM_DIER_CC4DE  ((uint16)0x1000)    /* ����/�Ƚ�4, Capture/Compare 4 DMA request enable */
#define TIM_DIER_COMDE  ((uint16)0x2000)    /* COM DMA request enable */
#define TIM_DIER_TDE    ((uint16)0x4000)    /* Trigger DMA request enable */
/*
 * ״̬�Ĵ��� TIM_SR
 * ƫ�Ƶ�ַ: 0x10
 * ��λֵ: 0x0000
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
 * �¼������Ĵ��� TIM_EGR
 * ƫ�Ƶ�ַ: 0x14
 * ��λֵ: 0x0000
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
 * ����/�Ƚ�ģʽ�Ĵ���1 TIM_CCMR1
 * ƫ�Ƶ�ַ: 0x18
 * ��λֵ: 0x0000
 */
#define TIM_CCMR1_CC1S      ((uint16)0x0003)    /* ����Ƚ�/���벶��ѡ��λ CC1S[1:0] */
#define TIM_CCMR1_OC1FE     ((uint16)0x0004)    /* ����Ƚ�1����ģʽ,ֻ��PWM1��PWM2ģʽ��Ч */
#define TIM_CCMR1_OC1PE     ((uint16)0x0008)    /* ��������Ƚ�1Ԥװ�ع���,ֻ���ڸ����¼�����ʱ,TIMx_CCR1Ӱ�ӼĴ�����װ����ֵ */
#define TIM_CCMR1_OC1M      ((uint16)0x0070)    /* ����Ƚ�1ģʽ,OC1M[2:0] */
#define TIM_CCMR1_OC1CE     ((uint16)0x0080)    /* һ����⵽ETRF����ߵ�ƽʱ,���OC1REF=0 */

#define TIM_CCMR1_CC2S      ((uint16)0x0300)    /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_OC2FE     ((uint16)0x0400)    /*!< Output Compare 2 Fast enable */
#define TIM_CCMR1_OC2PE     ((uint16)0x0800)    /*!< Output Compare 2 Preload enable */
#define TIM_CCMR1_OC2M      ((uint16)0x7000)    /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2CE     ((uint16)0x8000)    /*!< Output Compare 2 Clear Enable */

#define TIM_CCMR1_IC1PSC    ((uint16)0x000C)    /* Ԥ��Ƶ��,IC1PSC[1:0] */
#define TIM_CCMR1_IC1F      ((uint16)0x00F0)    /* �˲���,IC1F[3:0] */
#define TIM_CCMR1_IC2PSC    ((uint16)0x0C00)    /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2F      ((uint16)0xF000)    /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
/*
 * ����/�Ƚ�ģʽ�Ĵ���2 TIM_CCMR2
 * ƫ�Ƶ�ַ: 0x1C
 * ��λֵ: 0x0000
 */
#define TIM_CCMR2_CC3S      ((uint16)0x0003)    /* ����Ƚ�/���벶��ѡ��λ CC1S[1:0] */
#define TIM_CCMR2_OC3FE     ((uint16)0x0004)    /* ����Ƚ�1����ģʽ,ֻ��PWM1��PWM2ģʽ��Ч */
#define TIM_CCMR2_OC3PE     ((uint16)0x0008)    /* ��������Ƚ�1Ԥװ�ع���,ֻ���ڸ����¼�����ʱ,TIMx_CCR1Ӱ�ӼĴ�����װ����ֵ */
#define TIM_CCMR2_OC3M      ((uint16)0x0070)    /* ����Ƚ�1ģʽ,OC1M[2:0] */
#define TIM_CCMR2_OC3CE     ((uint16)0x0080)    /* һ����⵽ETRF����ߵ�ƽʱ,���OC1REF=0 */

#define TIM_CCMR2_CC4S      ((uint16)0x0300)    /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR2_OC4FE     ((uint16)0x0400)    /*!< Output Compare 2 Fast enable */
#define TIM_CCMR2_OC4PE     ((uint16)0x0800)    /*!< Output Compare 2 Preload enable */
#define TIM_CCMR2_OC4M      ((uint16)0x7000)    /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR2_OC4CE     ((uint16)0x8000)    /*!< Output Compare 2 Clear Enable */

#define TIM_CCMR2_IC3PSC    ((uint16)0x000C)    /* Ԥ��Ƶ��,IC1PSC[1:0] */
#define TIM_CCMR2_IC3F      ((uint16)0x00F0)    /* �˲���,IC1F[3:0] */
#define TIM_CCMR2_IC4PSC    ((uint16)0x0C00)    /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR2_IC4F      ((uint16)0xF000)    /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
/*
 * ����/�Ƚ�ʹ�ܼĴ��� TIM_CCER
 * ƫ�Ƶ�ַ: 0x20
 * ��λֵ: 0x0000
 */
#define TIM_CCER_CC1E       ((uint16)0x0001)    /* ����ͨ��1 */
#define TIM_CCER_CC1P       ((uint16)0x0002)    /* ���:0-OC1�ߵ�ƽ,1-OC1�͵�ƽ;����:0-������,�����ⲿ�����ź�(external trigger)ʱ������,1-�½���,���� */
#define TIM_CCER_CC1NE      ((uint16)0x0004)    /* ����ͨ��1�Ļ������ͨ�� */
#define TIM_CCER_CC1NP      ((uint16)0x0008)    /* 0-OC1�ߵ�ƽ,1-OC1�͵�ƽ */
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
 * ɲ���������Ĵ��� TIM_BDTR
 * ƫ�Ƶ�ַ: 0x44
 * ��λֵ: 0x0000
 */
#define TIM_BDTR_DTG    ((uint16)0x00FF)    /* �������������� DTG[0:7] */

#define TIM_BDTR_LOCK   ((uint16)0x0300)    /* �������� LOCK[1:0] */
#define TIM_Lock_off    ((uint16)0x0000)    /* �رռĴ������� */
#define TIM_Lock_Lv1    ((uint16)0x0100)    /* ��������1,BDTR��DTG��CR2��OISx��OISxNλ��BDTR��BKE/BKP/AOEλ */
#define TIM_Lock_Lv2    ((uint16)0x0200)    /* ��������2,��������1�ĸ�λ��CC����λ��OSSR/OSSIλ */
#define TIM_Lock_Lv3    ((uint16)0x0300)    /* ��������3,��������2�ĸ�λ��CC����λ */

#define TIM_BDTR_OSSI   ((uint16)0x0400)    /* ����ģʽ��"�ر�״̬"��ѡ�� */
#define TIM_BDTR_OSSR   ((uint16)0x0800)    /* ����״̬��"�ر�״̬"��ѡ�� */
#define TIM_BDTR_BKE    ((uint16)0x1000)    /* ʹ��ɲ������*/
#define TIM_BDTR_BKP    ((uint16)0x2000)    /* ɲ�����뼫��,1:�ߵ�ƽ��Ч,0:�͵�ƽ��Ч */
#define TIM_BDTR_AOE    ((uint16)0x4000)    /* �Զ����ʹ��,0:MOEֻ�������1,1:MOE����������һ�������¼��Զ���1 */
#define TIM_BDTR_MOE    ((uint16)0x8000)    /* �������ʹ��,0:OC��OCN�����ֹ,1:����OC��OCN */
/*
 * DMA���ƼĴ��� TIM_DCR
 * ƫ�Ƶ�ַ: 0x48
 * ��λֵ: 0x0000
 */
#define TIM_DCR_DBA     ((uint16)0x001F)    /*!< DMA����ַ DBA[4:0] */
#define TIM_DCR_DBL     ((uint16)0x1F00)    /*!< DMA�������ͳ��� DBL[4:0] */

/************************************************************************************/
/* ������ʱ����Ԫ
 */
typedef struct {
    uint16 ar_value;        /* �Զ�������ֵAuto-Reload Value,0x0000-0xFFFF */
    uint16 prsc;            /* ��Ƶ��, 0x0000-0xFFFF */
    uint16 counter_mode;    /* ������ʽ */
    uint16 ckd;             /* ʱ�ӷ�Ƶ���� */
    uint8  repeat_times;    /* �ظ���������,ֻ��TIM1��TIM8��Ч 0x00-0xFF*/
} tim_timebase_t;

/* ����ģʽ */
#define TIM_Counter_EdgeDir_Up      ((uint16)0x0000)    /* ���ض���,���¼��� */
#define TIM_Counter_EdgeDir_Down    ((uint16)0x0010)    /* ���ض���,���ϼ��� */
#define TIM_Counter_CenterIF_Down   ((uint16)0x0020)    /* ��ֵ����,ֻ�����¼���ʱ���ñȽ��жϱ�־ */
#define TIM_Counter_CenterIF_Up     ((uint16)0x0040)    /* ��ֵ����,    ����                     */
#define TIM_Counter_CenterIF_Both   ((uint16)0x0060)    /* ��ֵ����,  ������������                */
#define is_tim_counter_mode(p) (((p) == TIM_Counter_EdgeDir_Up) ||  \
                                ((p) == TIM_Counter_EdgeDir_Down) || \
                                ((p) == TIM_Counter_CenterIF_Down) || \
                                ((p) == TIM_Counter_CenterIF_Up) || \
                                ((p) == TIM_Counter_CenterIF_Both))

/*
 * ʱ�ӷ�Ƶ����
 * 
 * ��������t_dts,t_dts������ʱ��,����������,�����˲����Ĳ���ʱ���й�
 */
#define TIM_CKD_1       ((uint16)0x0000)    /* t_dts =     t_ckint */
#define TIM_CKD_2       ((uint16)0x0100)    /* t_dts = 2 * t_ckint */
#define TIM_CKD_4       ((uint16)0x0200)    /* t_dts = 4 * t_ckint */

/*
 * tim_init_timebase - ��ʼ��ʱ����Ԫ
 *
 * @TIMx: Ŀ���ʱ��
 * @conf: ��������
 */
void tim_init_timebase(tim_regs_t *TIMx, const tim_timebase_t *conf);



/************************************************************************************/


#define TIM_CCMode_Out      ((uint16)0x0000)    /* ����Ƚ� */
#define TIM_CCMode_In_Self  ((uint16)0x0001)    /* ICxӳ��ΪTIx���� */
#define TIM_CCMode_In_Mate  ((uint16)0x0002)    /* ICxӳ��ΪTIy����,x��yΪͬ�������ͨ�� */
#define TIM_CCMode_In_TRC   ((uint16)0x0003)    /* ICxӳ��ΪTRC���� */

#define TIM_OCMode_Frozen    ((uint16)0x0000)    /* ����ģʽ,TIMx_CCR1��TIMx_CNT��ıȽ϶�OC1REF�������� */
#define TIM_OCMode_Match_Act ((uint16)0x0010)    /* TIMx_CCR1��TIMx_CNTƥ��ʱ,��ͨ��1Ϊ��Ч��ƽ */
#define TIM_OCMode_Match_InA ((uint16)0x0020)    /*                                   ��Ч��ƽ */
#define TIM_OCMode_Toggle    ((uint16)0x0030)    /* TIMx_CCR1=TIMx_CNTʱOC1REF��ת */
#define TIM_OCMode_Force_InA ((uint16)0x0040)    /* OC1REFǿ�Ƶ͵�ƽ */
#define TIM_OCMode_Force_Act ((uint16)0x0050)    /* OC1REFǿ�Ƹߵ�ƽ */
#define TIM_OCMode_PWM1      ((uint16)0x0060)    /* ���ϼ���ʱ,CNT < CCR1�ߵ�ƽ,����͵�ƽ;���¼���ʱ,CNT > CCR1�͵�ƽ,����ߵ�ƽ */
#define TIM_OCMode_PWM2      ((uint16)0x0070)    /* ��PWM1���� */

#define TIM_IC_Filter_No   ((uint16)0x00)    /* ���벶�����˲�,f_dts */
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

#define TIM_IC_Prsc_No     ((uint16)0x0000)    /* ����ҪԤ��Ƶ */
#define TIM_IC_Prsc_2      ((uint16)0x1000)    /* 2 */
#define TIM_IC_Prsc_4      ((uint16)0x2000)    /* 4 */
#define TIM_IC_Prsc_8      ((uint16)0x3000)    /* 8 */


#endif