#include <stdbool.h>
#include "utypes.h"
#include "uart_spi.h"
#include "ads1292.h"
#include "databatch.h"
#include "leds.h"

#define FRAME_START  0xAA
#define FRAME_STOP 0x55

/**=========================== COMMAND FORMAT===============================
command that need confirm:
FRAME_START|COMMAND_START|frame size(bytes)|COMMAND_MARKER|...|COMMAND_NEED_CONFIRM|FRAME_STOP
Комманды высокой надежности, которые требуют подтверждения, сначала посылаются
назад и выполняются только после того как придет подтверждение что команда принята правильно

command that do not need confirm:
FRAME_START|COMMAND_START|frame size(bytes)|COMMAND_MARKER|...|FRAME_STOP|FRAME_STOP
Обычные команды, не требующие подтверждения, выполняются сразу
**************************************/

#define COMMAND_START 0x5A
#define COMMAND_NEED_CONFIRM 0xCC

/****** COMMANDS MARKERS *************/
// Processor registers addresses are 16bit (2 bytes) LITTLE ENDIAN
#define PROCESSOR_REGISTER_WRITE       0xA1
// FRAME_START|COMMAND_START|0X09|PROCESSOR_REGISTER_WRITE|reg_address_bottom|reg_address_top|reg_value|COMMAND_NEED_CONFIRM|FRAME_STOP

#define PROCESSOR_REGISTER_SET_BITS    0xA2
// FRAME_START|COMMAND_START|0X09|PROCESSOR_REGISTER_SET_BITS|reg_address_bottom|reg_address_top|reg_set_bits|COMMAND_NEED_CONFIRM|FRAME_STOP

#define PROCESSOR_REGISTER_CLEAR_BITS  0xA3
// FRAME_START|COMMAND_START|0X09|PROCESSOR_REGISTER_CLEAR_BITS|reg_address_bottom|reg_address_top|reg_clear_bits|COMMAND_NEED_CONFIRM|FRAME_STOP

#define PROCESSOR_REGISTER_READ        0xA4
// FRAME_START|COMMAND_START|0X08|PROCESSOR_REGISTER_READ|reg_address_bottom|reg_address_top|FRAME_STOP|FRAME_STOP

// ADS registers addresses are 8bit (1 byte)
#define ADS_REGISTER_WRITE             0xA6
// FRAME_START|COMMAND_START|0X08|ADS_REGISTER_WRITE|reg_address|reg_value|COMMAND_NEED_CONFIRM|FRAME_STOP

#define ADS_REGISTER_READ              0xA7
// FRAME_START|COMMAND_START|0X07|PROCESSOR_REGISTER_READ|reg_address|FRAME_STOP|FRAME_STOP

#define ADS_START_RECORDING            0xA8
// FRAME_START|COMMAND_START|0X08|ADS_START_RECORDING|divider_1|divider_2|COMMAND_NEED_CONFIRM|FRAME_STOP (двухканалка)
// FRAME_START|COMMAND_START|0X0E|ADS_START_RECORDING|divider_1|...|divider_8|COMMAND_NEED_CONFIRM|FRAME_STOP (восьмиканалка)

// one byte commands
#define ADS_STOP_RECORDING             0xA9
#define HELLO_REQUEST                  0xAB
#define HARDWARE_REQUEST               0xAC
#define PING                           0xAD
#define COMMAND_CONFIRMED              0xAE
// FRAME_START|COMMAND_START|0X06|COMMAND_MARKER|COMMAND_NEED_CONFIRM|FRAME_STOP
// FRAME_START|COMMAND_START|0X06|COMMAND_MARKER|FRAME_STOP|FRAME_STOP

/**=========================== MESSAGES FORMAT===============================
FRAME_START|MESSAGE_START|frame size(bytes)|MESSAGE_MARKER|...|FRAME_STOP
*************************************/
#define MESSAGE_START 0xA5

/****** MESSAGES MARKERS ***********/
#define MESSAGE_HELLO_MARKER 0xA0
// FRAME_START|MESSAGE_START|0X05|MESSAGE_HELLO_MARKER|FRAME_STOP

#define MESSAGE_HARDWARE_MARKER 0xA4
// FRAME_START|MESSAGE_START|0X06|MESSAGE_HARDWARE_MARKER|0x02|FRAME_STOP  (двухканалка)
// FRAME_START|MESSAGE_START|0X06|MESSAGE_HARDWARE_MARKER|0x08|FRAME_STOP (восьмиканалка)
/**===========================================================================*/
#define MSG_HELLO_SIZE 0X05
static uchar message_hello[] = {FRAME_START, MESSAGE_START, MSG_HELLO_SIZE, MESSAGE_HELLO_MARKER, FRAME_STOP};
#define MSG_HARDWARE_SIZE 0X06
static uchar message_hardware[] = {FRAME_START, MESSAGE_START, MSG_HARDWARE_SIZE, MESSAGE_HARDWARE_MARKER, 0x02, FRAME_STOP};

#define ADS_MAX_NUMBER_OF_SIGNALS 8
#define MAX_COMMAND_LENGTH 16
static uchar buffer0[MAX_COMMAND_LENGTH];
static uchar buffer1[MAX_COMMAND_LENGTH];
static uchar* fill_buffer = buffer0; // ссылка на буфер для заполнения
static uchar* command_buffer = buffer1;

static uchar fill_buffer_index;
static uchar command_length;
static bool command_buffered;
static uchar ads_dividers[ADS_MAX_NUMBER_OF_SIGNALS];

#define REGISTER_ADDRESS(byte_bottom, byte_top) ((unsigned char*)byte_bottom + (byte_top << 8))

// TODO PING
static void do_command(uchar *command) {
    uchar command_marker = command[3];
    /************** PROCESSOR REGISTERS *******************/
    // Processor register address is 2 bytes. Must be send in little endian order
    if (command_marker == PROCESSOR_REGISTER_WRITE) {
        uchar *address = REGISTER_ADDRESS(command[4], command[5]);
        *address = command[6];
    } else if (command_marker == PROCESSOR_REGISTER_SET_BITS) {
        uchar *address = REGISTER_ADDRESS(command[4], command[5]);
        *address |= command[6];
    } else if (command_marker == PROCESSOR_REGISTER_CLEAR_BITS) {
        uchar *address = REGISTER_ADDRESS(command[4], command[5]);
        *address &= ~command[6];
    } else if (command_marker == PROCESSOR_REGISTER_READ) {
        uchar *address = REGISTER_ADDRESS(command[4], command[5]);
        uart_flush(); // ждем завершения отправки по uart
        uart_transmit(address, 1);
    }
        /************** ADS REGISTERS *******************/
        // Ads register address is 1 byte.
    else if (command_marker == ADS_REGISTER_WRITE) {
        ads_write_regs(command[4], &command[5], 1);
    } else if (command_marker == ADS_REGISTER_READ) {
        uchar ch = ads_read_reg(command[4]);
        uart_flush(); // ждем завершения отправки по uart
        uart_transmit(&ch, 1);
    }
        /************** MACRO COMMANDS *******************/
    else if (command_marker == ADS_START_RECORDING) {
        uchar number_of_signals = ads_number_of_signals();
        for (int i = 0; i < number_of_signals; ++i) {
            ads_dividers[i] = command[4 + i];
        }
        databatch_start(ads_dividers);
        ads_start_recording();
    } else if (command_marker == ADS_STOP_RECORDING) {
        ads_stop_recording();
    } else if (command_marker == HELLO_REQUEST) {
        uart_flush(); // ждем завершения отправки по uart
        uart_transmit(message_hello, MSG_HELLO_SIZE);
    } else if (command_marker == HARDWARE_REQUEST) {
        // предпоследний байт содержит информацию о числе каналов ADS (2 или 8)
        message_hardware[MSG_HARDWARE_SIZE - 2] = ads_number_of_signals();
        uart_flush(); // ждем завершения отправки по uart
        uart_transmit(message_hardware, MSG_HARDWARE_SIZE);
    } else if (command_marker == COMMAND_CONFIRMED) {
        if (command_buffered) {
            command_buffered = false;
            do_command(command_buffer);
        }
    }
}

void commands_process() {
    uchar ch;
    while(uart_read(&ch)) { // читаем символы из uart
        if (fill_buffer_index == 0 && ch == FRAME_START) {
            fill_buffer[fill_buffer_index++] = ch;
        } else if (fill_buffer_index == 1 && ch == COMMAND_START) {
            fill_buffer[fill_buffer_index++] = ch;
        } else if (fill_buffer_index == 2 && ch < MAX_COMMAND_LENGTH) {
            fill_buffer[fill_buffer_index++] = ch;
            command_length = ch;
        } else if (fill_buffer_index > 2 && fill_buffer_index < (command_length - 1)) {
            fill_buffer[fill_buffer_index++] = ch;
        } else if ((fill_buffer_index == (command_length - 1)) && (ch == FRAME_STOP)) {
            fill_buffer[fill_buffer_index] = ch;
            // проверяем предпоследний байт
            if (fill_buffer[fill_buffer_index - 1] == FRAME_STOP) { // команда не требует подтверждения
                do_command(fill_buffer);
                fill_buffer_index = 0;
            } else if (fill_buffer[fill_buffer_index - 1] == COMMAND_NEED_CONFIRM) { // комманда требует подтверждения
                //swap double buffers
                uchar *tmp = command_buffer;
                command_buffer = fill_buffer;
                fill_buffer = tmp;
                uart_flush(); // ждем завершения отправки по uart
                // отправляем комманду назад на проверку
                uart_transmit(command_buffer, command_length);
                //выставляем флаг
                command_buffered = true;
                fill_buffer_index = 0;
            } else {
                fill_buffer_index = 0; //invalid command
            }
        } else {
          /********send broken command back for debug purpose**********/
            uchar *tmp = command_buffer;
            command_buffer = fill_buffer;
            fill_buffer = tmp;
            uart_flush(); // ждем завершения отправки по uart
            if(fill_buffer_index == 0) {
              uart_transmit(&ch, 1); // send back received char
              LED1_ON();
            } else {
              // send back received "broken command"
              uart_transmit(command_buffer, (fill_buffer_index+1)); 
            }
             LED3_ON();
            /********************************************************/
            
             fill_buffer_index = 0; //invalid command
        }
    }
}



