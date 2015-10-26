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

/*
 * tim_conf_preload - 配置预装载
 *
 * @TIMx: 目标计时器
 * @enable: 是否使能
 */
void tim_conf_preload(tim_regs_t *TIMx, uint16 enable) {
    assert(is_timer(TIMx));
    
    if (enable)
        TIMx->CR1 |= TIM_CR1_ARPE;
    else
        TIMx->CR1 &= ~TIM_CR1_ARPE;
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

/************************************************************************************/
/* 输出模式配置
 */

 /*
 * tim_init_oc1 - 配置输出通道1
 *
 * @TIMx: 目标计时器
 * @conf: 配置参数
 */
void tim_init_oc(tim_regs_t *TIMx, tim_occonf_t *conf) {
    assert(is_advanced_timer(TIMx) || is_general_timer(TIMx));
    assert(is_tim_channel(conf->channel));
    assert(is_tim_ocmode(conf->ocmode));
    uint8 tmp = 0;
    /* 关闭通道1 */
    tmp = TIM_CCER_CC1E << ((conf->channel - 1) * 4);
    TIMx->CCER &= ~tmp;
    /* 复位输出通道参数 */
    tmp = 0;
    tmp |= conf->ocmode;
    tmp |= (conf->ocpe) ? TIM_CCMR_OCPE : 0;
    if (conf->channel == TIM_Channel_1 || conf->channel == TIM_Channel_2)
        TIMx->CCMR1 |= (conf->channel == TIM_Channel_1) ? tmp : (tmp << 8);
    else
        TIMx->CCMR2 |= (conf->channel == TIM_Channel_3) ? tmp : (tmp << 8);
    /* 输出通道参考计数 */
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
    /* 高级计时器的死区idle设置 */
    if (is_advanced_timer(TIMx)) {
        tmp = 0;
        tmp |= (conf->ocidle | conf->ocnidle);
        TIMx->CR2 |= tmp << ((conf->channel - 1) * 2);
    }
    /* 输出使能 */
    tmp = 0;
    tmp |= conf->ocp;
    tmp |= (conf->oce) ? TIM_CCER_CC1E : 0;
    if (is_advanced_timer(TIMx)) {
        tmp |= conf->ocnp;
        tmp |= (conf->ocne) ? TIM_CCER_CC1NE : 0;
    }
    TIMx->CCER |= tmp << ((conf->channel - 1) * 4);
}
