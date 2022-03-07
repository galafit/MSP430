#include "ads1292.h"
#include "adc.h"
#include <stdbool.h>
#include "utypes.h"
#include "uart_spi.h"
#include "leds.h"

#define START_MARKER 0xAA
#define STOP_MARKER 0x55

/**======================== Формат данных ======================

START_MARKER|START_MARKER|счетчик фреймов(2bytes)|данные . . .|STOP_MARKER

Данные имеют следующий вид:
n_0 samples from ads_channel_0 (if this ads channel enabled)
n_1 samples from ads_channel_1 (if this ads channel enabled)
...
n_8 samples from ads_channel_8 (if this ads channel enabled)
2 bytes from accelerometer_x channel
2 bytes from accelerometer_y channel
2 bytes from accelerometer_Z channel
2 bytes with BatteryVoltage info (if BatteryVoltageMeasure  enabled)
1 byte(for 2 channels) or 2 bytes(for 8 channels) with lead-off detection info (if lead-off detection enabled)

Каждый sample данных занимает 3 байта.
n_i = ads_channel_i_sampleRate * durationOfDataRecord
durationOfDataRecord = sps/10 (sps частота оцифровки)
последовательность байт Little Endian
 =========================================================**/

#define ADS_BATCH_SIZE (10*6)  //The ADS's share in the total batch
#define ADS_HALF_BATCH_SIZE ADS_BATCH_SIZE/2
#define BATCH_HEADER_SIZE 4

//Total size of the whole batch (10 samples for two channels+accelerometer,
// battery and a stop byte)
#define MAX_BATCH_SIZE (BATCH_HEADER_SIZE + ADS_BATCH_SIZE + 9)

static int batch_size;
static uchar* ads_signal_dividers;

/*******  double buffer for all signals: ADS, ADC and helper info ******/
static uchar data_buffer_0[MAX_BATCH_SIZE];
static uchar data_buffer_1[MAX_BATCH_SIZE];
static uchar* fill_buffer = data_buffer_0; // ссылка на буфер для заполнения
static uchar* display_buffer = data_buffer_1;  //ссылка на заполненный буфер готовый для обработки
/***********************************************************************/

//TODO сделать упаковщик универсальным и для 2 и для 8 каналов

//Counters for frames of data (batches)
static unsigned int batch_counter = 0;
static int sample_pointer = 0;

/*void set_batch_size(){
    batch_size = ((10*3)/fr_divider[0]) + ((10*3)/fr_divider[1]) + 8 + 1;
}*/

static void set_batch_size(){
    batch_size = BATCH_HEADER_SIZE + 10*3 + 10*3 + 8 + 1;
}

static void make_batch(){
    uchar* acc_data = adc_get_data();
    //Adding acc data to the batch  По 2 байта на каждую из осей x, y ,z
    //Adding acc data to the batch
    fill_buffer[batch_size - 9] = acc_data[6];
    fill_buffer[batch_size - 8] = acc_data[7];
    fill_buffer[batch_size - 7] = acc_data[4];
    fill_buffer[batch_size - 6] = acc_data[5];
    fill_buffer[batch_size - 5] = acc_data[2];
    fill_buffer[batch_size - 4] = acc_data[3];
    //Adding battery info
    fill_buffer[batch_size - 3] = acc_data[0];
    fill_buffer[batch_size - 2] = acc_data[1];
    //Stop marker
    fill_buffer[batch_size - 1] = STOP_MARKER;
    //Writing header info
    fill_buffer[0] = START_MARKER;
    fill_buffer[1] = START_MARKER;
    //Assigning  batch a number
    fill_buffer[2] = (uchar)batch_counter;
    fill_buffer[3] = (uchar)(batch_counter >> 8);
    //Increasing the batch no int (two bytes)
    batch_counter++;
    // swap double buffers
    uchar *tmp = display_buffer;
    display_buffer = fill_buffer;
    fill_buffer = tmp;
    //send data to uart
    uart_transmit(display_buffer, batch_size);
}


static void process_ads_samples(uchar* ads_sample){
    signed char counter;
    uchar* ads_buffer = fill_buffer + BATCH_HEADER_SIZE;
    //Reading 1st channel (3 bytes). меняем порядок байт ?
    for(counter = 2; counter >= 0; counter--){
        ads_buffer[counter + sample_pointer] = *ads_sample++;
    }
    //Reading 2nd channel (3 bytes)
    for(counter = 2; counter >= 0; counter--){
        ads_buffer[counter + sample_pointer + ADS_HALF_BATCH_SIZE] = *ads_sample++;
    }
    sample_pointer += 3; //Pointing at the place to write the next byte
    //If all the ADS data is written, move on
    if(sample_pointer >= ADS_HALF_BATCH_SIZE){
        sample_pointer = 0;  
        make_batch(); 
    }    
}

void databatch_start(uchar* ads_dividers) {
    batch_counter = 0;//Setting the next batch number to zero
    set_batch_size();
    ads_signal_dividers = ads_dividers;
}


void databatch_process() {
    if(ads_data_received()) {
        process_ads_samples(ads_get_data());
    }
}

