#include <stm32f103rc_gpio.h>

/*
 * gpio_init - ��ʼ��ͨ��I/O�˿�
 *
 * �����üĴ�����Ϊÿ���˿������ṩ��������4bit������:CNFx[1:0]MODx[1:0]
 * ����CNFx��Ӧ��conf->mode,MODx��Ӧ��conf->speed
 *
 * @GPIOx: �˿����
 * @conf: �˿�����
 */
void gpio_init(gpio_regs_t *GPIOx, const gpio_init_t *conf) {
    uint32 mode, reg, offset, mask;
    uint16 pin;
    assert(is_gpio_pariph(GPIOx));
    assert(is_gpio_mode(conf->mode));
    // ���������Ҫ���ö˿�����
    mode = conf->mode & 0x0F;
    if (0x10 & conf->mode) {
        assert(is_gpio_speed(conf->speed));
        mode |= conf->speed;
    }
    // ��8λ�����Ŷ�ӦCRL�Ĵ���
    if (0x00FF & conf->pin) {
        reg = GPIOx->CRL;
        for (int i = 0; i < 8; i++) {
            pin = (conf->pin & (0x01 << i));
            if (0 == pin)
                continue;
            // ����ƫ��������պ�д��������
            offset = i << 2;
            mask = 0x0F << offset;
            reg &= ~mask;
            reg |= mode << offset;
            // ���������������������������ODRλΪ1����0
            if (conf->mode == GPIO_Mode_In_PU)
                GPIOx->BSRR = 0x01 << i;
            else if (conf->mode == GPIO_Mode_In_PD)
                GPIOx->BRR = 0x01 << i;
        }
        GPIOx->CRL = reg;
    }
    // ��8λ����
    if (0xFF00 & conf->pin) {
        reg = GPIOx->CRH;
        for (int i = 0; i < 8; i++) {
            pin = (conf->pin & (0x01 << (i + 8)));
            if (0 == pin)
                continue;
            // ����ƫ��������պ�д��������
            offset = i << 2;
            mask = 0x0F << offset;
            reg &= ~mask;
            reg |= mode << offset;
            // ���������������������������ODRλΪ1����0
            if (conf->mode == GPIO_Mode_In_PU)
                GPIOx->BSRR = 0x01 << (i + 8);
            else if (conf->mode == GPIO_Mode_In_PD)
                GPIOx->BRR = 0x01 << (i + 8);
        }
        GPIOx->CRH = reg;
    }
}
