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

