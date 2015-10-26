#include <stm32f103rc_gpio.h>
#include <stm32f103rc_rcc.h>
#include <stm32f103rc_usart.h>
#include <stm32f103rc_tim.h>
#include <core_m3_nvic.h>
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
    usart_init_clock(UART5, 0);
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

void init_timer(void) {
    rcc_switch_apb1_periph_clock(APB1_TIM6, 1);

    tim_timebase_t conf;
    conf.ar_value = 900 - 1;
    conf.prsc = 80 - 1;
    tim_init_timebase(TIM6, &conf);

    tim_it_config(TIM6, TIM_IT_Update, 1);
    tim_switch(TIM6, 1);
}

/* 系统频率(MHz) 周期(ms) */
#define FREQUENCE_SYS 72
#define CYCLE_TIME_SYS (1.0 / 72000)
/* 分频系数 */
#define PRESCALER 80
/* 计数总数 */
#define PERIOD 9000
/* 计数频率(kHz)周期(ms) */
#define FREQUENCE_COUNT 900
#define CYCLE_TIME_COUNT (1.0 / 900)


void init_timer2(void) {
    gpio_init_t gpio;
    tim_timebase_t tconf;

    rcc_switch_apb2_periph_clock(APB2_GPIOA | APB2_AFIO, 1);
    gpio.pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.mode = GPIO_Mode_AF_PP;
    gpio.speed = GPIO_Speed_50MHz;
    gpio_init(GPIOA, &gpio);

    rcc_switch_apb1_periph_clock(APB1_TIM2, 1);
    tconf.ar_value = PERIOD - 1;
    tconf.prsc = PRESCALER - 1;
    tconf.ckd = TIM_CKD_1;
    tconf.counter_mode = TIM_Counter_EdgeDir_Up;
    tim_init_timebase(TIM2, &tconf);

    tim_occonf_t occonf;
    occonf.channel = 1;
    occonf.oce = 1;
    occonf.ocmode = TIM_OCMode_PWM1;
    occonf.ocp = TIM_OCPolarity_High;
    occonf.ref = 1260;
    occonf.ocpe = 1;
    tim_init_oc(TIM2, &occonf);

    occonf.channel = 2;
    occonf.oce = 1;
    occonf.ocmode = TIM_OCMode_PWM1;
    occonf.ocp = TIM_OCPolarity_High;
    occonf.ref = 1260;
    occonf.ocpe = 1;
    tim_init_oc(TIM2, &occonf);

    tim_conf_preload(TIM2, 1);
    tim_switch(TIM2, 1);
}

void Init_Motor(void) {
    gpio_init_t gpio;
    rcc_switch_apb2_periph_clock(APB2_GPIOA, 1);

    gpio.pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    gpio.mode = GPIO_Mode_Out_PP;
    gpio.speed = GPIO_Speed_50MHz;
    gpio_init(GPIOA, &gpio);
}

#define ENABLE_MOTOR (GPIOA->BSRR |= 0x0002)
#define DISABLE_MOTOR (GPIOA->BRR |= 0x0002)

#define FORWARD	(GPIOA->BSRR |= 0x0001)
#define BACKWARD (GPIOA->BRR |= 0x0001)

void Init_LED(void) {
    gpio_init_t gpio;
    rcc_switch_apb2_periph_clock(APB2_GPIOB, 1);

    gpio.pin = GPIO_Pin_12 | GPIO_Pin_13;
    gpio.mode = GPIO_Mode_Out_PP;
    gpio.speed = GPIO_Speed_50MHz;
    gpio_init(GPIOB, &gpio);
}

#define Light_LED2 		(GPIOB->BRR |= 0x1000)
#define Light_LED3 		(GPIOB->BRR |= 0x2000)
#define Light_LED_Both 	(GPIOB->BRR |= 0x3000)

#define Dark_LED2 		(GPIOB->BSRR |= 0x1000)
#define Dark_LED3		(GPIOB->BSRR |= 0x2000)
#define Dark_LED_Both	(GPIOB->BSRR |= 0x3000)

void config_nvic(void) {
    nvic_config_prigroup(NVIC_PriGroup_2);
    // TIM6 中断配置
    nvic_conf_t conf;
    conf.IRQn = TIM6_IRQn;
    //conf.IRQn = TIM2_IRQn;
    conf.pre_prior = 1;
    conf.sub_prior = 2;
    conf.enale = 1;
    nvic_init(&conf);
}

int gJiffies;

int main(void) {
    Init_LED();
    Init_Motor();

    Init_UART5(9600);
    UART5_Send_String("hehehe");

    init_timer();
    init_timer2();
    config_nvic();
	
    Light_LED2;
    Light_LED3;
    gJiffies = 0;

    while (1) {
        if (0 <= gJiffies && gJiffies < 100) {
            //FORWARD;
            //ENABLE_MOTOR;
            Light_LED_Both;
        } else if (100 <= gJiffies && gJiffies < 200) {
            Dark_LED_Both;
            //DISABLE_MOTOR;
            //BACKWARD;
        }  else if (gJiffies >= 200)
            gJiffies = 0;
    }
}
