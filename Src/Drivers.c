/*
 * Drivers.c
 *
 *  Created on: 13. April 2023
 *      Author: Žan Hertiš
 */
#include "Drivers.h"

void gpio_Init()
{
    //reference manual -> Registri: 303

    //RCC->AHB2ENR |= 0x1<<0; //enable clock za GPIOA
    *((uint32_t*)(0x40021000+0x4c)) |= 1<<0;

    // SPI SCLK
    GPIOA->MODER &= ~(0x3 << 10); // izbris PA5
    GPIOA->MODER |= 0x2 << 10; // AF  mode za PA5
    GPIOA->AFRL |= 0x5 << 20; // AF 5 select za PA5

    // SPI1 MISO
    GPIOA->MODER &= ~(0x3 << 12); // izbris PA6
    GPIOA->MODER |= 0x2 << 12; // AF  mode za PA6
    GPIOA->AFRL |= 0x5 << 24; // AF 5 select za PA6

    // SPI MOSI
    GPIOA->MODER &= ~(0x3 << 14); // izbris PA7
    GPIOA->MODER |= 0x2 << 14; // AF  mode za PA7
    GPIOA->AFRL |= 0x5 << 28; // AF 5 select za PA7

    // Chip Select PA8
    GPIOA->MODER &= ~(0x3 << 16); //izbris PA8
    GPIOA->MODER |= 0x1 << 16; // general purpose output PA8
    GPIOA->ODR |= 1 << 8; //PA8 state: High

}
void spi1_Init()
{
    //reference manual -> opis konfiguracije: 1459; Registri: 1476 

    //RCC->APB2ENR |= 0x1<<12; //enable clock za SPI
    *((uint32_t*)(0x40021060)) |= 1<<12;

    SPI1->CR1 &= ~(0x1 << 15); // Set 2 line unidirectional; BIDIOMODE = 0
    SPI1->CR1 &= ~(0x1 << 13); // disable CRC; CRCEN = 0
    SPI1->CR1 &= ~(0x1 << 10); // Set full duplex; RXONLY = 0
    SPI1->CR1 &= ~(0x1 << 9); // Software slave management disabled; SSM = 0
    SPI1->CR1 &= ~(0x1 << 7); // set MSB first; LSBFIRST = 0
    SPI1->CR1 |= 0x1 << 3; // baud rate = f/4; BR = 001
    SPI1->CR1 |= 0x1 << 2; // set master configuration; MSTR = 1
    SPI1->CR1 &= ~(0x1 << 1); // CPOL = 0
    SPI1->CR1 &= ~(0x1 << 0); // CPHA = 0

    SPI1->CR2 |= 0x1 << 12; //RXNE se postavi če je v buffere >= 8 bitov; FRXTH = 1; če želiš >= 16 bitov, nastavi na 0
    SPI1->CR2 |= 0x7 << 8; // set data size = 8 bitov; DS = 0111
    SPI1->CR2 &= ~((0x1 << 7) | (0x1 << 6) | (0x1 << 5)); // mask TXE, RXNE, error int. (brez interuptov)
    SPI1->CR2 &= ~(0x1 << 4); // Motorola mode; FRF = 0
    SPI1->CR2 &= ~(0x1 << 3); // no NSS pulse; NSSP = 0
    SPI1->CR2 |= 0x1 << 2; // SS output enable; SSOE = 1
    SPI1->CR2 &= ~((0x1 << 1) | (0x1 << 0)); // TX, RX buffer DMA disable

    //SPI1->CR1 &= ~(0x1 << 6); // SPI disable
    //SPI1->CR1 |= 0x1 << 6; // SPI enable

}

void SPI_transmit_receive(uint8_t buffer_tx[], uint8_t buffer_rx[])
{
    uint16_t SR_data;
    uint16_t dummy;

    SPI1->CR1 |= 0x1 << 6; // SPI enable
    GPIOA->ODR &= ~(1 << 8); // CS = 0

    for (int i = 0; i < 10; i++)
    {
       
        while (!((SPI1->SR) & (1 << 1)));
        SPI1->DR = buffer_tx[i];

        while (!((SPI1->SR) & (1 << 1)));
        while ((SPI1->SR) & (1 << 7));

        buffer_rx[i] = SPI1->DR;
        SR_data = SPI1->SR;
        
    }

    GPIOA->ODR |= 1 << 8; // CS = 1
    SPI1->CR1 &= ~(0x1 << 6); // SPI disable
    dummy = SPI1->DR;
    
}

void SPI_page_write (uint8_t address24[], uint8_t data_write[])
{
    uint16_t dummy_read;
    uint8_t write_enable = 0x06;
    uint8_t page_write = 0x02;
    //uint8_t statusRegInst[] = {0x05, 0x00};

    SPI1->CR1 |= 0x1 << 6; // SPI enable

    

    ////////// WRITE ENABLE ///////////
    GPIOA->ODR &= ~(1 << 8);            // CS = 0
    while (!((SPI1->SR) & (1 << 1)));
    *(uint8_t*)&SPI1->DR = write_enable;            // omogoči vpis
    while (!((SPI1->SR) & (1 << 1)));
    while ((SPI1->SR) & (1 << 7));
    dummy_read = SPI1->DR;
    dummy_read = SPI1->SR;
    GPIOA->ODR |= 1 << 8;               // CS = 1
    //dummy_read = SPI1->DR;

    delay(); 

    ///////// WRITE DATA ///////////
    GPIOA->ODR &= ~(1 << 8);            // CS = 0

    //// page write instruction ////
    while (!((SPI1->SR) & (1 << 1)));
    *(uint8_t*)&SPI1->DR = page_write;              // page write instruction
    while (!((SPI1->SR) & (1 << 1)));
    while ((SPI1->SR) & (1 << 7));
    dummy_read = SPI1->DR;
    dummy_read = SPI1->SR;

    ////// address /////////////
    for (int i = 0; i < 3; i++)
    {
        while (!((SPI1->SR) & (1 << 1)));
        *(uint8_t*)&SPI1->DR = address24[i];            // memory address
        while (!((SPI1->SR) & (1 << 1)));
        while ((SPI1->SR) & (1 << 7));
        dummy_read = SPI1->DR;
        dummy_read = SPI1->SR;
    }

    /////// write /////////////
    for (int i = 0; i < sizeof(data_write)/sizeof(uint8_t); i++)
    {
        while (!((SPI1->SR) & (1 << 1)));
        *(uint8_t*)&SPI1->DR = data_write[i];            // data to write
        while (!((SPI1->SR) & (1 << 1)));
        while ((SPI1->SR) & (1 << 7));
        dummy_read = SPI1->DR;
        dummy_read = SPI1->SR;
    }

    GPIOA->ODR |= 1 << 8;                   // CS = 1 

    delay();
    SPI1->CR1 &= ~(0x1 << 6);               // SPI disable
    dummy_read = SPI1->DR;
}

void SPI_read (uint8_t address24[], uint8_t buffer_rx[], uint8_t numBytesToRead)
{
    uint8_t dummy_write = 0x00;
    uint16_t dummy_read;
    uint8_t read_instruct = 0x03;

    SPI1->CR1 |= 0x1 << 6;              // SPI enable
    GPIOA->ODR &= ~(1 << 8);            // CS = 0

    while (!((SPI1->SR) & (1 << 1)));
    *(uint8_t*)&SPI1->DR = read_instruct;              // read instruction
    while (!((SPI1->SR) & (1 << 1)));
    while ((SPI1->SR) & (1 << 7));
    dummy_read = SPI1->DR;
    dummy_read = SPI1->SR;

    for (int i = 0; i < 3; i++)
    {
        while (!((SPI1->SR) & (1 << 1)));
        *(uint8_t*)&SPI1->DR = address24[i];            // memory address
        while (!((SPI1->SR) & (1 << 1)));
        while ((SPI1->SR) & (1 << 7));
        dummy_read = SPI1->DR;
        dummy_read = SPI1->SR;
    }

    /////// read /////////////
    for (int i = 0; i < numBytesToRead; i++)
    {
       while ((SPI1->SR) & (1 << 7));
       *(uint8_t*)&SPI1->DR = dummy_write;
       while (!((SPI1->SR) & (1 << 0)));
       buffer_rx[i] = SPI1->DR;
    }

    GPIOA->ODR |= 1 << 8;                   // CS = 1
    SPI1->CR1 &= ~(0x1 << 6);               // SPI disable
    dummy_read = SPI1->DR;

}

void delay()
{
    for (uint16_t i = 0; i < 3000; i++)
    {

    }
}

void delay2()
{
    for (uint32_t i = 0; i < 1000000; i++)
    {

    }
}
