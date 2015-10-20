/*
 * ÿ��IO�˿ڶ����������ƼĴ���CRL��CRH���������ݼĴ���IDR��ODR
 * һ��32λ��Set/Reset�Ĵ���BSRR��һ��16λ��reset�Ĵ���BRR,
 * ���Ǳ�֤�˶�IO�Ĵ�����ԭ�Ӳ���
 */

/*
 * ��ϵͳ��λʱ���߸���ɸ�λʱ,IO�˿ڵĸ��ù����ǹرյ�,IO�˿ڱ�����Ϊ
 * Input Floatingģʽ��
 */

/*
 * ���԰�һЩ���ù�����ӳ�䵽����һЩ�����ϡ������ͨ�����������Ӧ�ļĴ���
 * �����(�ο�AFIO�Ĵ�������)����ʱ�����ù��ܾͲ���ӳ�䵽���ǵ�ԭʼ�������ˡ�
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

/* GPIO�Ĵ�����ַӳ�� */
#define GPIOA_BASE  (APB2_BASE + 0x0800)
#define GPIOB_BASE  (APB2_BASE + 0x0C00)
#define GPIOC_BASE  (APB2_BASE + 0x1000)
#define GPIOD_BASE  (APB2_BASE + 0x1400)
#define GPIOE_BASE  (APB2_BASE + 0x1800)
#define GPIOF_BASE  (APB2_BASE + 0x1C00)
#define GPIOG_BASE  (APB2_BASE + 0x2000)
/* GPIO�Ĵ���ָ����� */
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
/* GPIO���Ŷ��� */
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
/* GPIO����ģʽ */
#define GPIO_Mode_In_Analog     ((uint8)0x00)   /* ģ������ */
#define GPIO_Mode_In_Floating   ((uint8)0x04)   /* �������� */
#define GPIO_Mode_In_PD         ((uint8)0x28)   /* �������� */
#define GPIO_Mode_In_PU         ((uint8)0x48)   /* �������� */
#define GPIO_Mode_Out_OD        ((uint8)0x14)   /* ��©��� */
#define GPIO_Mode_Out_PP        ((uint8)0x10)   /* ������� */
#define GPIO_Mode_AF_OD         ((uint8)0x1C)   /* ��©���� */
#define GPIO_Mode_AF_PP         ((uint8)0x18)   /* ���츴�� */
#define is_gpio_mode(p)     (((p) == GPIO_Mode_In_Analog)   || \
                             ((p) == GPIO_Mode_In_Floating) || \
                             ((p) == GPIO_Mode_In_PD)       || \
                             ((p) == GPIO_Mode_In_PU)       || \
                             ((p) == GPIO_Mode_Out_OD)      || \
                             ((p) == GPIO_Mode_Out_PP)      || \
                             ((p) == GPIO_Mode_AF_OD)       || \
                             ((p) == GPIO_Mode_AF_PP))
/* GPIO���� */
#define GPIO_Speed_10MHz    ((uint8)0x01)
#define GPIO_Speed_2MHz     ((uint8)0x02)
#define GPIO_Speed_50MHz    ((uint8)0x03)
#define is_gpio_speed(p)    (((p) == GPIO_Speed_10MHz) || ((p) == GPIO_Speed_2MHz) || ((p) == GPIO_Speed_50MHz))

/* GPIO��ʼ�����ݽṹ */
typedef struct {
    uint16 pin; 
    uint8 speed;
    uint8 mode;
} gpio_init_t;

/*
 * gpio_init - ��ʼ��ͨ��I/O�˿�
 *
 * @GPIOx: �˿����
 * @conf: �˿�����
 */
void gpio_init(gpio_regs_t *GPIOx, const gpio_init_t *conf);


#endif
