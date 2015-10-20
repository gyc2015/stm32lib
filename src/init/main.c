#include <stm32f103rc_gpio.h>
#include <stm32f103rc_rcc.h>
#include <stm32f103rc_usart.h>
#include <queue.h>

char Uint2C(uint8 a) {
    switch (a) {
    case 0: return '0';
    case 1: return '1';
    case 2: return '2';
    case 3: return '3';
    case 4: return '4';
    case 5: return '5';
    case 6: return '6';
    case 7: return '7';
    case 8: return '8';
    case 9: return '9';
    case 10: return 'A';
    case 11: return 'B';
    case 12: return 'C';
    case 13: return 'D';
    case 14: return 'E';
    case 15: return 'F';
    default: return 0;
    }
}

Queue_T u5RxQ;
Queue_T u5TxQ;

void Init_UART5(unsigned int boud) {
    rcc_switch_apb2_periph_clock(APB2_GPIOD | APB2_GPIOC | APB2_AFIO, 1);
    rcc_switch_apb1_periph_clock(APB1_UART5, 1);

    // 端口复用配置
    gpio_init_t gpio;
    // USART5_TX PC.12
    gpio.pin = GPIO_Pin_12;
    gpio.mode = GPIO_Mode_AF_PP;
    gpio.speed = GPIO_Speed_50MHz;
    gpio_init(GPIOC, &gpio);
    // UART5_RX	 PD.2
    gpio.pin = GPIO_Pin_2;
    gpio.speed = GPIO_Speed_50MHz;
    gpio.mode = GPIO_Mode_In_Floating;
    gpio_init(GPIOD, &gpio);

    // USART 时钟配置, 不使用同步模式,故清零
    usart_init_clock(UART5, USART_CR2_CPHA);
    // USART 初始化配置
    usart_conf_t conf;
    conf.baud_rate = boud;
    conf.word_length = USART_WordLength_8;
    conf.stop_bits = USART_StopBits_1;
    conf.parity = USART_Parity_No;
    conf.mode = USART_Mode_Rx | USART_Mode_Tx;
    conf.hardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init(UART5, &conf);

    usart_it_config(UART5, USART_IT_RXNE, 1);
    usart_switch(UART5, 1);
	
	init_queue(&u5RxQ);
	init_queue(&u5TxQ);
}

static void U5_Poll_Rx(void) {
    uint8 tmp;
    while (!is_queue_empty(&u5RxQ)) {
        dequeue(&u5RxQ, &tmp);
        enqueue(&u5TxQ, tmp);
    }
}

static void U5_Poll_Tx(void) {
    uint8 tmp;
    while (!is_queue_empty(&u5TxQ)) {
        dequeue(&u5TxQ, &tmp);
        UART5->DR = tmp;
        while (!(USART_SR_TXE & UART5->SR)) {};
    }
}

void Poll_UART5(void) {
    U5_Poll_Rx();
    U5_Poll_Tx();
}

void UART5_Send_String(char *s) {
    while ('\0' != s[0]) {
        UART5->DR = s[0];
        s++;
        while (!(USART_SR_TXE & UART5->SR)) {};
    }
}

void UART5_Send_Hex(uint8 data) {
    char s[5] = "0x00";
    uint8 h = (uint8)(data / 16);
    uint8 l = data - 16 * h;
    s[2] = Uint2C(h);
    s[3] = Uint2C(l);
    UART5_Send_String(s);
}

void UART5_Send_HexArray(uint8 *array, int len) {
    for (int i = 0; i < len; i++) {
        UART5_Send_Hex(array[i]);
        UART5_Send_String(" ");
    }
    UART5_Send_String("\r\n");
}



int main(void) {
    Init_UART5(9600);
    UART5_Send_String("hehehe");
}
