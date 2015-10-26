#include <stm32f103rc_tim.h>


/*
 * tim_init_timebase - ��ʼ��ʱ����Ԫ
 *
 * @TIMx: Ŀ���ʱ��
 * @conf: ��������
 */
void tim_init_timebase(tim_regs_t *TIMx, const tim_timebase_t *conf) {
    assert(is_timer(TIMx));
    assert(is_tim_counter_mode(conf->counter_mode));
    // ����ģʽ��ʱ�ӷ�Ƶ����
    uint16 reg = TIMx->CR1;
    if (!is_basic_timer(TIMx)) {
        reg &= ~(TIM_CR1_CKD | TIM_Counter_Mode_Mask);
        reg |= (conf->ckd | conf->counter_mode);
    }
    TIMx->CR1 = reg;
    // ������ֵ
    TIMx->ARR = conf->ar_value;
    // ��Ƶ��
    TIMx->PSC = conf->prsc;
    // �ظ���������
    if (is_advanced_timer(TIMx))
        TIMx->RCR = conf->repeat_times;
    // ����Update�¼�,���¼Ĵ���
    TIMx->EGR = TIM_EGR_UG;
}
/*
 * tim_switch - ��ʱ������
 *
 * @TIMx: Ŀ���ʱ��
 * @enable: �Ƿ�ʹ��
 */
void tim_switch(tim_regs_t *TIMx, uint16 enable) {
    assert(is_timer(TIMx));

    if (enable)
        TIMx->CR1 |= TIM_CR1_CEN;
    else
        TIMx->CR1 &= ~TIM_CR1_CEN;
}

/*
 * tim_conf_preload - ����Ԥװ��
 *
 * @TIMx: Ŀ���ʱ��
 * @enable: �Ƿ�ʹ��
 */
void tim_conf_preload(tim_regs_t *TIMx, uint16 enable) {
    assert(is_timer(TIMx));
    
    if (enable)
        TIMx->CR1 |= TIM_CR1_ARPE;
    else
        TIMx->CR1 &= ~TIM_CR1_ARPE;
}
/************************************************************************************/
/* ��ʱ�����ж�����
 */

/*
 * tim_it_config - ���ü�ʱ���ж�
 *
 * @TIMx: Ŀ���ʱ����ַ����
 * @it: �����ж���
 * @enable: �Ƿ�ʹ��
 */
void tim_it_config(tim_regs_t *TIMx, uint16 it, uint16 enable) {
    assert(is_timer(TIMx));
    assert(is_tim_it(it));
    
    if (enable)
        TIMx->DIER |= it;
    else
        TIMx->DIER &= ~it;
}

/************************************************************************************/
/* ���ģʽ����
 */

 /*
 * tim_init_oc1 - �������ͨ��1
 *
 * @TIMx: Ŀ���ʱ��
 * @conf: ���ò���
 */
void tim_init_oc(tim_regs_t *TIMx, tim_occonf_t *conf) {
    assert(is_advanced_timer(TIMx) || is_general_timer(TIMx));
    assert(is_tim_channel(conf->channel));
    assert(is_tim_ocmode(conf->ocmode));
    uint8 tmp = 0;
    /* �ر�ͨ��1 */
    tmp = TIM_CCER_CC1E << ((conf->channel - 1) * 4);
    TIMx->CCER &= ~tmp;
    /* ��λ���ͨ������ */
    tmp = 0;
    tmp |= conf->ocmode;
    tmp |= (conf->ocpe) ? TIM_CCMR_OCPE : 0;
    if (conf->channel == TIM_Channel_1 || conf->channel == TIM_Channel_2)
        TIMx->CCMR1 |= (conf->channel == TIM_Channel_1) ? tmp : (tmp << 8);
    else
        TIMx->CCMR2 |= (conf->channel == TIM_Channel_3) ? tmp : (tmp << 8);
    /* ���ͨ���ο����� */
    switch (conf->channel) {
    case TIM_Channel_1:
        TIMx->CCR1 = conf->ref;
        break;
    case TIM_Channel_2:
        TIMx->CCR2 = conf->ref;
        break;
    case TIM_Channel_3:
        TIMx->CCR3 = conf->ref;
        break;
    case TIM_Channel_4:
        TIMx->CCR4 = conf->ref;
        break;
    }
    /* �߼���ʱ��������idle���� */
    if (is_advanced_timer(TIMx)) {
        tmp = 0;
        tmp |= (conf->ocidle | conf->ocnidle);
        TIMx->CR2 |= tmp << ((conf->channel - 1) * 2);
    }
    /* ���ʹ�� */
    tmp = 0;
    tmp |= conf->ocp;
    tmp |= (conf->oce) ? TIM_CCER_CC1E : 0;
    if (is_advanced_timer(TIMx)) {
        tmp |= conf->ocnp;
        tmp |= (conf->ocne) ? TIM_CCER_CC1NE : 0;
    }
    TIMx->CCER |= tmp << ((conf->channel - 1) * 4);
}
