#include <stm32f103rc_tim.h>


/*
 * tim_init_timebase - 初始化时基单元
 *
 * @TIMx: 目标计时器
 * @conf: 配置内容
 */
void tim_init_timebase(tim_regs_t *TIMx, const tim_timebase_t *conf) {
    assert(is_timer(TIMx));
    assert(is_tim_counter_mode(conf->counter_mode));
    // 计数模式和时钟分频因子
    uint16 reg = TIMx->CR1;
    if (!is_basic_timer(TIMx)) {
        reg &= ~(TIM_CR1_CKD | TIM_Counter_Mode_Mask);
        reg |= (conf->ckd | conf->counter_mode);
    }
    TIMx->CR1 = reg;
    // 重载数值
    TIMx->ARR = conf->ar_value;
    // 分频器
    TIMx->PSC = conf->prsc;
    // 重复计数次数
    if (is_advanced_timer(TIMx))
        TIMx->RCR = conf->repeat_times;
    // 产生Update事件,更新寄存器
    TIMx->EGR = TIM_EGR_UG;
}
/*
 * tim_switch - 计时器开关
 *
 * @TIMx: 目标计时器
 * @enable: 是否使能
 */
void tim_switch(tim_regs_t *TIMx, uint16 enable) {
    assert(is_timer(TIMx));

    if (enable)
        TIMx->CR1 |= TIM_CR1_CEN;
    else
        TIMx->CR1 &= ~TIM_CR1_CEN;
}

/************************************************************************************/
/* 计时器的中断配置
 */

/*
 * tim_it_config - 设置计时器中断
 *
 * @TIMx: 目标计时器地址访问
 * @it: 配置中断项
 * @enable: 是否使能
 */
void tim_it_config(tim_regs_t *TIMx, uint16 it, uint16 enable) {
    assert(is_timer(TIMx));
    assert(is_tim_it(it));
    
    if (enable)
        TIMx->DIER |= it;
    else
        TIMx->DIER &= ~it;
}

