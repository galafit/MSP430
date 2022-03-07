#ifndef ADS1292_H
#define ADS1292_H
#include <stdbool.h>
#include "bynary.h"
#include "utypes.h"


void ads_init();
uchar ads_read_reg(uchar address);
void ads_write_regs(uchar address, uchar* data, uchar data_size);
void ads_start_recording();
uchar ads_number_of_signals();
void ads_stop_recording();
bool ads_data_received();
uchar* ads_get_data();
void ads_DRDY_interrupt_callback(void (*func)(void));


#endif //ADS1292_H
