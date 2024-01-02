/**

 */

#include <stdint.h>
#include "Definitions.h"
#include "Drivers.h"

int main(void)
{
  gpio_Init();
  spi1_Init();

  uint8_t data[] = {0x9f,0,0,0,0,0,0,0,0,0};
  uint8_t write_data[] = {0xbc, 0xcb, 0x15, 0x08};
  uint8_t mem_addr[] = {0x10, 0x80, 0x00};
  uint8_t rx_data[10];

  

  while(1)
  {
//    SPI_transmit_receive(data, rx_data);
    SPI_page_write(mem_addr, write_data);
    SPI_read(mem_addr, rx_data, 8);
    while(1);
  }
}
