/*
 * Definitions.h
 *
 *  Created on: Jun 23, 2021
 *      Author: Žan Hertiš
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include "stdint.h"

#define GPIOC_BASE_ADDRESS	 0x48000800
#define GPIOA_BASE_ADDRESS 	 0x48000000
#define GPIOB_BASE_ADDRESS	 0x48000400

typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
	//volatile uint32_t AFR[2];
	volatile uint32_t BRR;
	volatile uint32_t ASCR;

}GPIO_Typedef;

#define	GPIOA			((GPIO_Typedef*)GPIOA_BASE_ADDRESS)
#define GPIOB			((GPIO_Typedef*)GPIOB_BASE_ADDRESS)
#define GPIOC			((GPIO_Typedef*)GPIOC_BASE_ADDRESS)


#define RCC_BASE_ADDRESS 		0x40021000
#define RCC_AHB2ENR_OFFSET		0x4C
#define	RCC_AHB2ENR_ADDRESS		(RCC_BASE_ADDRESS + RCC_AHB2ENR_OFFSET)
#define	RCC_AHB2ENR_POINTER		((uint32_t*)RCC_AHB2ENR_ADDRESS)

typedef struct {
	volatile uint32_t CR;
	volatile uint32_t ICSCR;
	volatile uint32_t CFGR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t PLLSAI1CFGR;
	volatile uint32_t PLLSAI2CFGR;
	volatile uint32_t CIER;
	volatile uint32_t CIFR;
	volatile uint32_t CICR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t APB1RSTR1;
	volatile uint32_t APB1RSTR2;
	volatile uint32_t APB2RSTR;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t APB1ENR1;
	volatile uint32_t APB1ENR2;
	volatile uint32_t APB2ENR;
	volatile uint32_t AHB1SMENR;
	volatile uint32_t AHB2SMENR;
	volatile uint32_t AHB3SMENR;
	volatile uint32_t APB1SMENR1;
	volatile uint32_t APB1SMENR2;
	volatile uint32_t APB2SMENR;
	volatile uint32_t CCIPR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	//volatile uint32_t CRRCR;
	//volatile uint32_t CCIPR2;

}RCC_Typedef;

#define RCC 	((RCC_Typedef*)RCC_BASE_ADDRESS)


#define TIM2_BASE_ADDRESS 			0x40000000
#define TIM5_BASE_ADDRESS			0x40000C00
#define TIM3_BASE_ADDRESS			0x40000400

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t Reserved1;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	volatile uint32_t Reserved2;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
	volatile uint32_t OR1;
	volatile uint32_t OR2;
}TIM_Typedef;

#define TIM2 		((TIM_Typedef*)TIM2_BASE_ADDRESS)
#define TIM5 		((TIM_Typedef*)TIM5_BASE_ADDRESS)
#define TIM3 		((TIM_Typedef*)TIM3_BASE_ADDRESS)


#define	NVIC_ISER_0_ADDRESS				0xE000E100
#define	NVIC_ISER_0_POINTER				((uint32_t*)NVIC_ISER_0_ADDRESS)


typedef struct {
	volatile uint32_t ISER[8];
	//volatile uint8_t Reserved1[96];
	volatile uint32_t Reserved1[24];
	volatile uint32_t ICER[8];
	volatile uint32_t Reserved2[24];
	volatile uint32_t ISPR[8];
	volatile uint32_t Reserved3[24];
	volatile uint32_t ICPR[8];
	volatile uint32_t Reserved4[24];
	volatile uint32_t IABR[8];
	//volatile uint32_t IPR[60]
	volatile uint32_t Reserved5[56];
	volatile uint8_t IPR[240];
	volatile uint32_t Reserved6[644];
	volatile uint32_t STIR;
} NVIC_TypeDef;

#define NVIC_BASE_ADDRESS				0xE000E100
#define	NVIC							((NVIC_TypeDef*)NVIC_BASE_ADDRESS)



#define USART2_BASE_ADDRESS 		0x40004400

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t BRR;
	volatile uint32_t GTPR;
	volatile uint32_t RTOR;
	volatile uint32_t RQR;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint32_t RDR;
	volatile uint32_t TDR;

}USART2_Typedef;

#define USART2		((USART2_Typedef*)USART2_BASE_ADDRESS)

#define SPI1_BASE_ADDRESS			0x40013000

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;

}SPI1_Typdef;

#define SPI1 		((SPI1_Typdef*)SPI1_BASE_ADDRESS)


#endif /* DEFINITIONS_H_ */
