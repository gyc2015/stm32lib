#include <stm32f103rc_usart.h>
#include <stm32f103rc_rcc.h>

/*
 * usart_init_clock - 初始化串口同步模式时钟
 *
 * 不工作在同步模式时,建议清零
 *
 * @USARTx: 串口寄存器指针访问
 * @conf: 同步时钟配置
 */
void usart_init_clock(usart_regs_t *USARTx, uint16 conf) {
    uint16 reg = USARTx->CR2;
    assert((0 == conf) || (0 == (conf & (~USART_Clock_Mask))));
    reg &= ~USART_Clock_Mask;
    reg |= conf;
    USARTx->CR2 = reg;
}

/*
 * usart_init - 初始化串口
 *
 * @USARTx: 目标串口
 * @conf: 配置项
 */
void usart_init(usart_regs_t *USARTx, const usart_conf_t *conf) {
    assert(is_usart_wordlength(conf->word_length));
    assert(is_usart_stop_bits(conf->stop_bits));
    assert(is_usart_parity(conf->parity));
    assert(is_usart_mode(conf->mode));
    assert(is_usart_hardware_flow_control(conf->hardwareFlowControl));
    // 只有串口1,2,3才有硬件控制流功能
    if (conf->hardwareFlowControl != USART_HardwareFlowControl_None)
        assert(USARTx != UART4 && USARTx != UART5);
    // 控制寄存器2
    uint16 reg = USARTx->CR2;
    reg &= ~USART_CR2_STOP;
    reg |= conf->stop_bits;
    USARTx->CR2 = reg;
    // 控制寄存器1
    reg = USARTx->CR1;
    reg &= ~(USART_CR1_M | USART_CR1_PS | USART_CR1_PCE | USART_CR1_RE | USART_CR1_TE);
    reg |= conf->word_length | conf->parity | conf->mode;
    USARTx->CR1 = reg;
    // 控制寄存器3
    reg = USARTx->CR3;
    reg &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
    reg |= conf->hardwareFlowControl;
    USARTx->CR3 = reg;
    // 波特率
    rcc_clocks_t clocks;
    rcc_get_clocks_freq(&clocks);
    // 串口1由APB2驱动,其余串口由APB1驱动
    uint32 fapb = (USART1 == USARTx) ? clocks.PCLK2 : clocks.PCLK1;
    uint32 idiv = ((25 * fapb) / (4 * conf->baud_rate));
    reg = (idiv / 100) << 4;
    uint32 fdiv = idiv - (100 * (reg >> 4));
    reg |= ((((fdiv * 16) + 50) / 100)) & 0x0F;
    USARTx->BRR = reg;
}

/*
* usart_enable - 使能串口
*
* @USARTx: 目标串口
* @enable: 是否使能
*/
void usart_switch(usart_regs_t *USARTx, uint16 enable) {
    assert(is_usart(USARTx));

    if (enable)
        USARTx->CR1 |= USART_CR1_UE;
    else
        USARTx->CR1 &= ~USART_CR1_UE;
}


#define IT_Mask ((uint16)0x001F)  /*!< USART Interrupt Mask */
/*
* usart_it_config - 设置串口中断
*
* @USARTx: 目标串口地址访问
* @it: 配置终端项
* @enable: 是否使能
*/
void usart_it_config(usart_regs_t *USARTx, uint16 it, uint16 enable) {
    assert(is_usart_periph(USARTx));
    assert(is_usart_conf_it(it));
    if (USART_IT_CTS == it)
        assert(USARTx != UART4 && USARTx != UART5);
    
    uint16 reg = it >> 0x05;
    uint16 itpos = it & IT_Mask;
    uint16 itmask = 0x01 << itpos;
    uint32 addr = (uint32)USARTx;
    if (reg == 0x01)
        addr += 0x0C;
    else if (reg == 0x02)
        addr += 0x10;
    else
        addr += 0x14;
    
    if (enable)
        *(volatile uint32*)addr |= itmask;
    else
        *(volatile uint32*)addr &= ~itmask;
}

/*
 * usart_get_flag_status - 获取串口标记位状态
 *
 * @USARTx: 目标串口
 * @flag: 标识
 */
uint16 usart_get_flag_status(usart_regs_t* USARTx, uint16 flag) {
    uint16 re = 0;

    assert(is_usart(USARTx));
    assert(is_usart_flag(flag));
    if (flag == USART_SR_CTS)
        assert(USARTx != UART4 && USARTx != UART5);

    re = (USARTx->SR & flag) ? 1 : 0;
    return re;
}



