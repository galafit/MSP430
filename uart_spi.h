#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include "utypes.h"

void uart_init();
bool uart_read(uchar* chp);
void uart_transmit(uchar *data, int data_size);
void uart_flush();


void spi_init();
uchar spi_exchange(uchar tx_data);
void spi_transmit(uchar* data, int data_size);
void spi_read(uchar* read_buffer, int data_size);
bool spi_transfer_finished();
void spi_flush();


#endif //UART_H



