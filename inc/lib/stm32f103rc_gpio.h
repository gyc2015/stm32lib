/*
 * 每个IO端口都有两个控制寄存器CRL和CRH，两个数据寄存器IDR和ODR
 * 一个32位的Set/Reset寄存器BSRR和一个16位的reset寄存器BRR,
 * 它们保证了对IO寄存器的原子操作
 */

/*
 * 在系统复位时或者刚完成复位时,IO端口的复用功能是关闭的,IO端口被设置为
 * Input Floating模式。
 */

/*
 * 可以把一些复用功能重映射到其他一些引脚上。这可以通过软件配置相应的寄存器
 * 来完成(参考AFIO寄存器描述)。这时，复用功能就不再映射到它们的原始引脚上了。
 */
#ifndef STM32F103RC_GPIO_H
#define STM32F103RC_GPIO_H

#include <stm32f103rc.h>

typedef struct {
    volatile uint32 CRL;
    volatile uint32 CRH;
    volatile uint32 IDR;
    volatile uint32 ODR;
    volatile uint32 BSRR;
    volatile uint32 BRR;
    volatile uint32 LCKR;
} gpio_regs_t;

/* GPIO寄存器地址映射 */
#define GPIOA_BASE  (APB2_BASE + 0x0800)
#define GPIOB_BASE  (APB2_BASE + 0x0C00)
#define GPIOC_BASE  (APB2_BASE + 0x1000)
#define GPIOD_BASE  (APB2_BASE + 0x1400)
#define GPIOE_BASE  (APB2_BASE + 0x1800)
#define GPIOF_BASE  (APB2_BASE + 0x1C00)
#define GPIOG_BASE  (APB2_BASE + 0x2000)
/* GPIO寄存器指针访问 */
#define GPIOA       ((gpio_regs_t *) GPIOA_BASE)
#define GPIOB       ((gpio_regs_t *) GPIOB_BASE)
#define GPIOC       ((gpio_regs_t *) GPIOC_BASE)
#define GPIOD       ((gpio_regs_t *) GPIOD_BASE)
#define GPIOE       ((gpio_regs_t *) GPIOE_BASE)
#define GPIOF       ((gpio_regs_t *) GPIOF_BASE)
#define GPIOG       ((gpio_regs_t *) GPIOG_BASE)
#define is_gpio_pariph(p)   (((p) == GPIOA) || \
                             ((p) == GPIOB) || \
                             ((p) == GPIOC) || \
                             ((p) == GPIOD) || \
                             ((p) == GPIOE) || \
                             ((p) == GPIOF) || \
                             ((p) == GPIOG))
/* GPIO引脚定义 */
#define GPIO_Pin_0      ((uint16)0x0001)  /*!< Pin 0 selected */
#define GPIO_Pin_1      ((uint16)0x0002)  /*!< Pin 1 selected */
#define GPIO_Pin_2      ((uint16)0x0004)  /*!< Pin 2 selected */
#define GPIO_Pin_3      ((uint16)0x0008)  /*!< Pin 3 selected */
#define GPIO_Pin_4      ((uint16)0x0010)  /*!< Pin 4 selected */
#define GPIO_Pin_5      ((uint16)0x0020)  /*!< Pin 5 selected */
#define GPIO_Pin_6      ((uint16)0x0040)  /*!< Pin 6 selected */
#define GPIO_Pin_7      ((uint16)0x0080)  /*!< Pin 7 selected */
#define GPIO_Pin_8      ((uint16)0x0100)  /*!< Pin 8 selected */
#define GPIO_Pin_9      ((uint16)0x0200)  /*!< Pin 9 selected */
#define GPIO_Pin_10     ((uint16)0x0400)  /*!< Pin 10 selected */
#define GPIO_Pin_11     ((uint16)0x0800)  /*!< Pin 11 selected */
#define GPIO_Pin_12     ((uint16)0x1000)  /*!< Pin 12 selected */
#define GPIO_Pin_13     ((uint16)0x2000)  /*!< Pin 13 selected */
#define GPIO_Pin_14     ((uint16)0x4000)  /*!< Pin 14 selected */
#define GPIO_Pin_15     ((uint16)0x8000)  /*!< Pin 15 selected */
#define GPIO_Pin_All    ((uint16)0xFFFF)  /*!< All pins selected */
/* GPIO引脚模式 */
#define GPIO_Mode_In_Analog     ((uint8)0x00)   /* 模拟输入 */
#define GPIO_Mode_In_Floating   ((uint8)0x04)   /* 浮空输入 */
#define GPIO_Mode_In_PD         ((uint8)0x28)   /* 下拉输入 */
#define GPIO_Mode_In_PU         ((uint8)0x48)   /* 上拉输入 */
#define GPIO_Mode_Out_OD        ((uint8)0x14)   /* 开漏输出 */
#define GPIO_Mode_Out_PP        ((uint8)0x10)   /* 推挽输出 */
#define GPIO_Mode_AF_OD         ((uint8)0x1C)   /* 开漏复用 */
#define GPIO_Mode_AF_PP         ((uint8)0x18)   /* 推挽复用 */
#define is_gpio_mode(p)     (((p) == GPIO_Mode_In_Analog)   || \
                             ((p) == GPIO_Mode_In_Floating) || \
                             ((p) == GPIO_Mode_In_PD)       || \
                             ((p) == GPIO_Mode_In_PU)       || \
                             ((p) == GPIO_Mode_Out_OD)      || \
                             ((p) == GPIO_Mode_Out_PP)      || \
                             ((p) == GPIO_Mode_AF_OD)       || \
                             ((p) == GPIO_Mode_AF_PP))
/* GPIO速率 */
#define GPIO_Speed_10MHz    ((uint8)0x01)
#define GPIO_Speed_2MHz     ((uint8)0x02)
#define GPIO_Speed_50MHz    ((uint8)0x03)
#define is_gpio_speed(p)    (((p) == GPIO_Speed_10MHz) || ((p) == GPIO_Speed_2MHz) || ((p) == GPIO_Speed_50MHz))

/* GPIO初始化数据结构 */
typedef struct {
    uint16 pin; 
    uint8 speed;
    uint8 mode;
} gpio_init_t;

/*
 * gpio_init - 初始化通用I/O端口
 *
 * @GPIOx: 端口组号
 * @conf: 端口配置
 */
void gpio_init(gpio_regs_t *GPIOx, const gpio_init_t *conf);


#endif
