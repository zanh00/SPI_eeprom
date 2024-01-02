/*
 * Drivers.h
 *
 *  Created on: 13. April 2023
 *      Author: Žan Hertiš
 */

#ifndef DRIVERS_H_
#define DRIVERS_H_

#include "Definitions.h"

extern void gpio_Init();
extern void spi1_Init();
extern void SPI_transmit_receive(uint8_t buffer_tx[], uint8_t buffer_rx[]);
extern void SPI_page_write (uint8_t address24[], uint8_t data_write[]);
extern void SPI_read (uint8_t address24[], uint8_t buffer_rx[], uint8_t numBytesToRead);
extern void delay();
extern void delay2();


#endif /* DRIVERS_H_ */
