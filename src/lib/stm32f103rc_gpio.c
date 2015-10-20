#include <stm32f103rc_gpio.h>

/*
 * gpio_init - 初始化通用I/O端口
 *
 * 在配置寄存器中为每个端口引脚提供了连续的4bit配置项:CNFx[1:0]MODx[1:0]
 * 其中CNFx对应着conf->mode,MODx对应着conf->speed
 *
 * @GPIOx: 端口组号
 * @conf: 端口配置
 */
void gpio_init(gpio_regs_t *GPIOx, const gpio_init_t *conf) {
    uint32 mode, reg, offset, mask;
    uint16 pin;
    assert(is_gpio_pariph(GPIOx));
    assert(is_gpio_mode(conf->mode));
    // 输出引脚需要设置端口速率
    mode = conf->mode & 0x0F;
    if (0x10 & conf->mode) {
        assert(is_gpio_speed(conf->speed));
        mode |= conf->speed;
    }
    // 低8位的引脚对应CRL寄存器
    if (0x00FF & conf->pin) {
        reg = GPIOx->CRL;
        for (int i = 0; i < 8; i++) {
            pin = (conf->pin & (0x01 << i));
            if (0 == pin)
                continue;
            // 计算偏移量并清空后写入新数据
            offset = i << 2;
            mask = 0x0F << offset;
            reg &= ~mask;
            reg |= mode << offset;
            // 针对上拉或者下拉输入引脚设置ODR位为1或者0
            if (conf->mode == GPIO_Mode_In_PU)
                GPIOx->BSRR = 0x01 << i;
            else if (conf->mode == GPIO_Mode_In_PD)
                GPIOx->BRR = 0x01 << i;
        }
        GPIOx->CRL = reg;
    }
    // 高8位引脚
    if (0xFF00 & conf->pin) {
        reg = GPIOx->CRH;
        for (int i = 0; i < 8; i++) {
            pin = (conf->pin & (0x01 << (i + 8)));
            if (0 == pin)
                continue;
            // 计算偏移量并清空后写入新数据
            offset = i << 2;
            mask = 0x0F << offset;
            reg &= ~mask;
            reg |= mode << offset;
            // 针对上拉或者下拉输入引脚设置ODR位为1或者0
            if (conf->mode == GPIO_Mode_In_PU)
                GPIOx->BSRR = 0x01 << (i + 8);
            else if (conf->mode == GPIO_Mode_In_PD)
                GPIOx->BRR = 0x01 << (i + 8);
        }
        GPIOx->CRH = reg;
    }
}
