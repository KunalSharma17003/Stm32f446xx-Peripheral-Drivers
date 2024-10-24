/*
 * stm32f446xx.h
 *
 *  Created on: Sep 21, 2024
 *      Author: Kunal Sharma
 * Description: This is device header file and contains the macros for peripherals , clock etc.
 */

#ifndef STM32F446XX_H_
#define STM32F446XX_H_
#include <stddef.h> //for null
#include <stdint.h>

/*************************************************************************************************
 *Processor specific details
 *ARM Cortex m4 processor NVIC ISERx register address
**************************************************************************************************/
#define NVIC_ISER0            ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1            ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2            ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3            ((volatile uint32_t*)0xE000E10C)

/*
 *ARM Cortex m4 processor NVIC ISERx register address
*/
#define NVIC_ICER0            ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1            ((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2            ((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3            ((volatile uint32_t*)0xE000E18C)

/*
 *ARM Cortex m4 processor Priority register address
*/
#define NVIC_PR_BASEADDR      ((volatile uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED 4

/*FLASH AND SRAM ARE THE EMBEDDED MEMORIES*/
#define FLASH_BASEADDR        0x08000000U      /*!<Expalin this macro in the code here.>*/
#define SRAM1_BASEADDR        0x20000000U
#define SRAM2_BASEADDR        0x2001C000U
#define ROM                   0x1FFF0000U
#define SRAM                  SRAM1_BASEADDR

/*
 * AHB and APBx Bus Peripheral base address
 */
#define PERIPH_BASEADDR       0x40000000U
#define APB1PERIPH_BASEADDR   PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR   0x40010000U
#define AHB1PERIPH_BASEADDR   0x40020000U
#define AHB2PERIPH_BASEADDR   0x50000000U

/*
 * Base Addresses of the peripherals CONNECTED TO AHB1 BUS that will be used in this course
 */
#define GPIOA_BASEADDR       (AHB1PERIPH_BASEADDR + 0X0000U)
#define GPIOB_BASEADDR       (AHB1PERIPH_BASEADDR + 0X0400U)
#define GPIOC_BASEADDR       (AHB1PERIPH_BASEADDR + 0X0800U)
#define GPIOD_BASEADDR       (AHB1PERIPH_BASEADDR + 0X0C00U)
#define GPIOE_BASEADDR       (AHB1PERIPH_BASEADDR + 0X1000U)
#define GPIOF_BASEADDR       (AHB1PERIPH_BASEADDR + 0X1400U)
#define GPIOG_BASEADDR       (AHB1PERIPH_BASEADDR + 0X1800U)
#define GPIOH_BASEADDR       (AHB1PERIPH_BASEADDR + 0X1C00U)
#define RCC_BASEADDR         (AHB1PERIPH_BASEADDR + 0X3800U)

/*
 * Base Addresses of the peripherals CONNECTED TO APB1 BUS that will be used in this course
 */

#define TIM2_BASEADDR        (APB1PERIPH_BASEADDR + 0X0000U)
#define TIM3_BASEADDR        (APB1PERIPH_BASEADDR + 0X0400U)
#define TIM4_BASEADDR        (APB1PERIPH_BASEADDR + 0X0800U)
#define TIM5_BASEADDR        (APB1PERIPH_BASEADDR + 0X0C00U)
#define TIM6_BASEADDR        (APB1PERIPH_BASEADDR + 0X1000U)
#define TIM7_BASEADDR        (APB1PERIPH_BASEADDR + 0X1400U)
#define TIM12_BASEADDR       (APB1PERIPH_BASEADDR + 0X1800U)
#define TIM13_BASEADDR       (APB1PERIPH_BASEADDR + 0X1C00U)
#define TIM14_BASEADDR       (APB1PERIPH_BASEADDR + 0X2000U)
#define SPI2_BASEADDR        (APB1PERIPH_BASEADDR + 0X3800U)
#define SPI3_BASEADDR        (APB1PERIPH_BASEADDR + 0X3C00U)
#define USART2_BASEADDR      (APB1PERIPH_BASEADDR + 0X4400U)
#define USART3_BASEADDR      (APB1PERIPH_BASEADDR + 0X4800U)
#define UART4_BASEADDR       (APB1PERIPH_BASEADDR + 0X4C00U)
#define UART5_BASEADDR       (APB1PERIPH_BASEADDR + 0X5000U)
#define I2C1_BASEADDR        (APB1PERIPH_BASEADDR + 0X5400U)
#define I2C2_BASEADDR        (APB1PERIPH_BASEADDR + 0X5800U)
#define I2C3_BASEADDR        (APB1PERIPH_BASEADDR + 0X5C00U)


/*
 * Base Addresses of the peripherals CONNECTED TO APB2 BUS that will be used in this course
 */


#define EXTI_BASEADDR        (APB2PERIPH_BASEADDR + 0X3C00U)
#define USART1_BASEADDR      (APB2PERIPH_BASEADDR + 0X1000U)
#define USART6_BASEADDR      (APB2PERIPH_BASEADDR + 0X1400U)
#define SPI1_BASEADDR        (APB2PERIPH_BASEADDR + 0X3000U)
#define SPI4_BASEADDR        (APB2PERIPH_BASEADDR + 0X3400U)
#define USART1_BASEADDR      (APB2PERIPH_BASEADDR + 0X1000U)
#define SYSCFG_BASEADDR      (APB2PERIPH_BASEADDR + 0X3800U)


/********************************Peripheral Register definitions structures***************/
/*
 * Note: Registers of the peripherals are specific to the MCU
 * e.g: Number of registers of spi peripherals of stm32f4x family of mcu may be different (more or less)
 * Compared to number of registers of spi peripherals of stm32Lx or STM32F0x family of mcu's
 * For this we have to check the device reference mannual
 */
typedef struct
{
  volatile	uint32_t MODER;                   /*!<Give a short description,      Address offset: 0x00>*/
  volatile	uint32_t OTYPER;                  /*!<TODO,                          Address offset: 0x04>*/
  volatile	uint32_t OSPEEDER;
  volatile	uint32_t PUPDR;
  volatile	uint32_t IDR;
  volatile	uint32_t ODR;
  volatile	uint32_t BSRR;
  volatile	uint32_t LCKR;
  volatile	uint32_t AFRL;
  volatile	uint32_t AFRH;
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	         uint32_t RESSERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	         uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	         uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	         uint32_t RESERVED4[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
             uint32_t RESERVED5[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;

}RCC_RegDef_t;



typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

typedef struct
{
	volatile uint32_t  MEMRMP;
	volatile uint32_t  PMC;
	volatile uint32_t  EXTICR[4];
			 uint32_t  RESERVED0[2];
	volatile uint32_t  CMPCR;
	         uint32_t  RESERVED1[2];
	volatile uint32_t  CFGR;
}SYSCFG_RegDef_t;


typedef struct
{
volatile uint32_t CR1;
volatile uint32_t CR2;
volatile uint32_t SR;
volatile uint32_t DR;
volatile uint32_t CRCPR;
volatile uint32_t RXCRCR;
volatile uint32_t TXCRCR;
volatile uint32_t I2SCFGR;
volatile uint32_t I2SPR;

}SPI_RegDef_t;


typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t OAR1;
  volatile uint32_t OAR2;
  volatile uint32_t DR;
  volatile uint32_t SR1;
  volatile uint32_t SR2;
  volatile uint32_t CCR;
  volatile uint32_t TRISE;
  volatile uint32_t FLTR;
}I2C_RegDef_t;

typedef struct
{
  volatile uint32_t SR1;
  volatile uint32_t DR;
  volatile uint32_t BRR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t CR3;
  volatile uint32_t GTPR;
}USART_RegDef_t;





/*
 * Peripheral Definitions(peripheral base addresses typecasted to xxx_regdef_t)
 */

#define GPIOA    ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB    ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC    ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD    ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE    ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF    ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG    ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH    ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC      ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI     ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG   ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1     ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2     ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3     ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4     ((SPI_RegDef_t*)SPI4_BASEADDR)


#define I2C1     ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2     ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3     ((I2C_RegDef_t*)I2C3_BASEADDR)


#define USART1   ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2   ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3   ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4    ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5    ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6   ((USART_RegDef_t*)USART6_BASEADDR)

/*
 * CLOCK ENABLE/DISABLE MACROS
 */
/*FOR GPIO'S*/
#define GPIOA_PCLK_EN()   (RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()   (RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()   (RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()   (RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()   (RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()   (RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()   (RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()   (RCC->AHB1ENR |=(1<<7))

/*FOR I2C'S*/
#define I2C1_PCLK_EN()     (RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN()     (RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN()     (RCC->APB1ENR |=(1<<23))



/*FOR SPI'S*/
#define SPI1_PCLK_EN()     (RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN()     (RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN()     (RCC->APB1ENR |=(1<<15))
#define SPI4_PCLK_EN()     (RCC->APB2ENR |=(1<<13))

/*FOR USART*/
#define USART1_PCLK_EN()    (RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN()    (RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN()    (RCC->APB1ENR |=(1<<18))
#define UART4_PCLK_EN()    (RCC->APB1ENR |=(1<<19))
#define UART5_PCLK_EN()    (RCC->APB1ENR |=(1<<20))
#define USART6_PCLK_EN()    (RCC->APB2ENR |=(1<<5))


/*FOR SYSCFG*/
#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |=(1<<14))



/*DISABLING PERIPHERLAS*/
/*GPIO'S*/

#define GPIOA_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<7))


/*I2C'S*/
#define I2C1_PCLK_DI()     (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()     (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()     (RCC->APB1ENR &= ~(1<<23))


/*SPI'S*/
#define SPI1_PCLK_DI()     (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()     (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()     (RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()     (RCC->APB2ENR &= ~(1<<13))


/*USART*/
#define USART1_PCLK_DI()    (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()    (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()    (RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()     (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()     (RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()    (RCC->APB2ENR &= ~(1<<5))

/*SYSCFG*/
#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &= ~(1<<14))


/*
 * Macros used to reset the peripherals
*/
#define GPIOA_REG_RESET()   do{(RCC->AHB1RSTR |=(1<<0)); (RCC->AHB1RSTR &=~(1<<0));}while(0)
#define GPIOB_REG_RESET()   do{(RCC->AHB1RSTR |=(1<<1)); (RCC->AHB1RSTR &=~(1<<1));}while(0)
#define GPIOC_REG_RESET()   do{(RCC->AHB1RSTR |=(1<<2)); (RCC->AHB1RSTR &=~(1<<2));}while(0)
#define GPIOD_REG_RESET()   do{(RCC->AHB1RSTR |=(1<<3)); (RCC->AHB1RSTR &=~(1<<3));}while(0)
#define GPIOE_REG_RESET()   do{(RCC->AHB1RSTR |=(1<<4)); (RCC->AHB1RSTR &=~(1<<4));}while(0)
#define GPIOF_REG_RESET()   do{(RCC->AHB1RSTR |=(1<<5)); (RCC->AHB1RSTR &=~(1<<5));}while(0)
#define GPIOG_REG_RESET()   do{(RCC->AHB1RSTR |=(1<<6)); (RCC->AHB1RSTR &=~(1<<6));}while(0)
#define GPIOH_REG_RESET()   do{(RCC->AHB1RSTR |=(1<<7)); (RCC->AHB1RSTR &=~(1<<7));}while(0)

/*
 * Returns the port base address code
*/
#define GPIO_BASEADDR_TO_CODE(x)  ((x== GPIOA) ? 0 :\
								   (x== GPIOB) ? 1 :\
								   (x== GPIOC) ? 2 :\
								   (x== GPIOD) ? 3 :\
								   (x== GPIOE) ? 4 :\
								   (x== GPIOF) ? 5 :\
								   (x== GPIOG) ? 6 :\
								   (x== GPIOH) ? 7 :0)

/*
 * IRQ (Interrupt Request) Numbers of STM32F446XX Micro-controller
 * Note: Update these macros with valid values according to your mcu(may vary on different micro-controllers)
 */
#define IRQ_NO_EXTI0               6
#define IRQ_NO_EXTI1               7
#define IRQ_NO_EXTI2               8
#define IRQ_NO_EXTI3               9
#define IRQ_NO_EXTI4               10
#define IRQ_NO_EXTI9_5             23
#define IRQ_NO_EXTI15_10           40

#define IRQ_NO_SPI1                35
#define IRQ_NO_SPI2                36
#define IRQ_NO_SPI3                51
#define IRQ_NO_SPI4                84

#define IRQ_NO_I2C1_EV             31
#define IRQ_NO_I2C1_ER             32

#define IRQ_NO_I2C2_EV             33
#define IRQ_NO_I2C2_ER             34

#define IRQ_NO_I2C3_EV             72
#define IRQ_NO_I2C3_ER             73





#define NVIC_IRQ_PRI0              0
#define NVIC_IRQ_PRI1              1
#define NVIC_IRQ_PRI2              2
#define NVIC_IRQ_PRI3              3
#define NVIC_IRQ_PRI4              4
#define NVIC_IRQ_PRI5              5
#define NVIC_IRQ_PRI6              6
#define NVIC_IRQ_PRI7              7
#define NVIC_IRQ_PRI8              8
#define NVIC_IRQ_PRI9              9
#define NVIC_IRQ_PRI10             10
#define NVIC_IRQ_PRI11             11
#define NVIC_IRQ_PRI12             12
#define NVIC_IRQ_PRI13             13
#define NVIC_IRQ_PRI14             14
#define NVIC_IRQ_PRI15             15

/*********************************************************
 *    Some general Macros that will be used by the source files and the api functions
 *
**********************************************************/
#define ENABLE           1
#define DISABLE          0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET
#define FLAG_SET         SET
#define FLAG_RESET       RESET



/***********************************************
 * Bit position definitions of spi peripherals
 ***********************************************/
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_LSB_FIRST   7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RX_ONLY     10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRC_NEXT    12
#define SPI_CR1_CRC_EN      13
#define SPI_CR1_BIDI_OE     14
#define SPI_CR1_BIDI_MODE   15


/*
 * For CR2 Register
 * */
#define SPI_CR2_RXD_MAEN    0
#define SPI_CR2_TXD_MAEN    1
#define SPI_CR2_SS_OE       2
#define SPI_CR2_FRF         4
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7

/*
 * For SPI_SR register
 * */

#define SPI_SR_RXNE        0
#define SPI_SR_TXE         1
#define SPI_SR_CHSIDE      2
#define SPI_SR_UDR         3
#define SPI_SR_CRC_ERR     4
#define SPI_SR_MODF        5
#define SPI_SR_OVR         6
#define SPI_SR_BSY         7
#define SPI_SR_FRE         8



/*
 * BIT POSITION DEFINITIONS I2C_CR1
*/
#define I2C_CR1_PE         0
#define I2C_CR1_SMBUS      1
#define I2C_CR1_RESERVED0  2
#define I2C_CR1_SMBTYPE    3
#define I2C_CR1_ENARP      4
#define I2C_CR1_ENPEC      5
#define I2C_CR1_ENGC       6
#define I2C_CR1_NOSTRETCH  7
#define I2C_CR1_START      8
#define I2C_CR1_STOP       9
#define I2C_CR1_ACK        10
#define I2C_CR1_POS        11
#define I2C_CR1_PEC        12
#define I2C_CR1_ALERT      13
#define I2C_CR1_RESERVED1  14
#define I2C_CR1_SWRST      15

/*
 * BIT POSITION DEFINITIONS I2C_CR2
*/

#define I2C_CR2_FREQ        0
#define I2C_CR2_ITERREN     8
#define I2C_CR2_ITEVTEN     9
#define I2C_CR2_ITBUFEN     10
#define I2C_CR2_DMAEN       11
#define I2C_CR2_LAST        12


/*
 * BIT POSITION DEFINITIONS I2C_OAR1
*/
#define I2C_OAR1_ADD0       0
#define I2C_OAR1_ADD71       1
#define I2C_OAR1_ADD98       8
#define I2C_OAR1_ADD_MODE   15


/*
 * Bit positions for i2c oar2
*/
#define I2C_OAR2_ENDUAL   0
#define I2C_OAR2_ADD2     1


/*
 * Bit position for i2c_sr
 * */
#define I2C_SR1_SB        0
#define I2C_SR1_ADDR      1
#define I2C_SR1_BTF       2
#define I2C_SR1_ADD10     3
#define I2C_SR1_STOPF     4
#define I2C_SR1_RXNE      6
#define I2C_SR1_TXE       7
#define I2C_SR1_BERR      8
#define I2C_SR1_ARLO      9
#define I2C_SR1_AF        10
#define I2C_SR1_OVR       11
#define I2C_SR1_PECERR    12
#define I2C_SR1_TIMEOUT   14
#define I2C_SR1_SMBALERT  15



/*
 * Bit position for i2c_sr2
*/

#define I2C_SR2_MSL        0
#define I2C_SR2_BUSY       1
#define I2C_SR2_TRA        2
#define I2C_SR2_GENCALL    4
#define I2C_SR2_SMBDEFAULT 5
#define I2C_SR2_SMBHOST    6
#define I2C_SR2_DUALF      7
#define I2C_SR2_PEC        8


/*
 * Bit position for i2c_ccr
 * */

#define I2C_CCR_CCR        0
#define I2C_CCR_DUTY       14
#define I2C_CCR_F_S        15


/*
 * Bit position for USART register CR
 */

#define USART_SR_PE        0
#define USART_SR_FE        1
#define USART_SR_NF        2
#define USART_SR_ORE       3
#define USART_SR_IDLE      4
#define USART_SR_RXNE      5
#define USART_SR_TC        6
#define USART_SR_TXE       7
#define USART_SR_LBD       8
#define USART_SR_CTS       9

/*
 * Bit position for USART register DR
 */

#define USART_DR            0

/*
 * Bit position for USART register DIV Fraction
 */
#define USART_BRR_DIV_FRAC  0
#define USART_BRR_DIV_MANT  1

/*
 * Bit position for USART register CR1
 */
#define USART_CR1_SBK        0
#define USART_CR1_RWU        1
#define USART_CR1_RE         2
#define USART_CR1_TE         3
#define USART_CR1_IDLEIE     4
#define USART_CR1_RXNEIE     5
#define USART_CR1_TCIE       6
#define USART_CR1_TXEIE      7
#define USART_CR1_PEIE       8
#define USART_CR1_PS         9
#define USART_CR1_PCE        10
#define USART_CR1_WAKE       11
#define USART_CR1_M          12
#define USART_CR1_UE         13
#define USART_CR1_OVER8      15

/*
 * Bit position for USART register CR2
 */
#define USART_CR2_ADD          0
#define USART_CR2_RESERVED0    4
#define USART_CR2_LBDL         5
#define USART_CR2_LBDIE        6
#define USART_CR2_RESERVED1    7
#define USART_CR2_LBCL         8
#define USART_CR2_CPHA         9
#define USART_CR2_CPOL         10
#define USART_CR2_CLKEN        11
#define USART_CR2_STOP         12
#define USART_CR2_LINEN        14

/*
 * Bit position for USART register CR3
 */
#define USART_CR3_EIE         0
#define USART_CR3_IREN        1
#define USART_CR3_IRLP        2
#define USART_CR3_HDSEL       3
#define USART_CR3_NACK        4
#define USART_CR3_SCEN        5
#define USART_CR3_DMAR        6
#define USART_CR3_DMAT        7
#define USART_CR3_RTSE        8
#define USART_CR3_CTSE        9
#define USART_CR3_CTSIE       10
#define USART_CR3_ONEBIT      11


/*
 * Bit position for USART register GTPR
 */
#define USART_GTPR_PSC         0
#define USART_GTPR_GT          8









#include"stm32f446xx_gpio_driver.h"
#include"stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"

#endif /* STM32F446XX_H_ */
