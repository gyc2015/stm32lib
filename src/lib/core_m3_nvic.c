#include <core_m3_scb.h>
#include <core_m3_nvic.h>

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
void nvic_config_prigroup(uint32 conf) {
    assert(is_nvic_prigroup(conf));
    SCB->AIRCR = SCB_AIRCR_Vectkey | conf;
}

/*
 * nvic_init - 初始化中断配置
 *
 * @conf: 配置项
 */
void nvic_init(const nvic_conf_t *conf) {
    assert(conf->pre_prior < 0x10);
    assert(conf->sub_prior < 0x10);

    if (!conf->enale) {
        NVIC->ICER[conf->IRQn >> 0x05] = 0x01 << (conf->IRQn & 0x1F);
        return;
    }

    uint32 pre = ((SCB->AIRCR & 0x0700) >> 0x08) + 0x01;
    uint32 prio = conf->pre_prior << pre;
    prio |= conf->sub_prior << 0x04;
    NVIC->IP[conf->IRQn] = prio;
    NVIC->ISER[conf->IRQn >> 0x05] = 0x01 << (conf->IRQn & 0x1F);
}

