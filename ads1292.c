#include "msp430f2274.h"
#include <stdbool.h>
#include "bynary.h"
#include "utypes.h"
#include "leds.h"
#include "uart_spi.h"
#include "ads1292.h"
#include "interrupts.h"

/**
 * ADS выставляет флаг(бит) DRDY (data ready) когда данные готовы.
 * Если разрешены прерывания по этому флагу то произойдет прерывание.
 *
 * ADS прерывания привязаны к порту [P1]
 * [P1_INTERRUPT_FLAG_REGISTER] (P1IFG) порт где выставляется флаг прерываний DRDY
 * [P1_INTERRUPT_ENABLE_REGISTER] (P1IE) порт для разрешения/запрещения прерываний по DRDY
 *
 * Согласно datasheet все флаги PxIFG должны сбрасываться программно (?)
 * Поэтому в обработчике прерываний флаг DRDY нужно обязательно очищать вручную!
 */
#define DRDY_BIT  BIT2
#define ADS_DRDY_FLAG_CHECK()  (P1IFG & DRDY_BIT)
#define ADS_DRDY_FLAG_CLEAR()  (P1IFG &= ~DRDY_BIT)
#define ADS_DRDY_INTERRUPT_ENABLE()  (P1IE |= DRDY_BIT)
#define ADS_DRDY_INTERRUPT_DISABLE()  (P1IE &= ~DRDY_BIT)


#define NULL 0
#define ADS_NUMBER_OF_CHANNELS 2
#define ADS_SAMPLE_SIZE 9 // одно измерение: 3 байта служебные + 3 байта канал 1 + 3 байта канал 2

// указатель на функцию без параметров например "void func()" определяется следующим образом: void (*func)(void)
// и дальше этому указателю можно присваивать адрес любой  соответсвующей функции и вызывать ее просто как func();
/** указатель на внешнюю функцию которая будет вызываться из прерывания DRDY (ads данные готовы) */
static void (*DRDY_interrupt_callback)(void);

/*******  double buffer for ads samples ******/
static uchar sample_buffer_0[ADS_SAMPLE_SIZE];
static uchar sample_buffer_1[ADS_SAMPLE_SIZE];
static uchar* fill_buffer = sample_buffer_0; // ссылка на буфер для заполнения
static uchar* display_buffer = sample_buffer_1;  //ссылка на заполненный буфер готовый для обработки
/**********************************************************/

static bool data_ready;
static bool data_receiving;
static bool data_received;

#define DELAY_32()   __delay_cycles(32)
#define DELAY_64()   __delay_cycles(64)
#define DELAY_320()   __delay_cycles(320)
#define DELAY_450000()   __delay_cycles(450000)

/******** ADS ONE BYTE COMMANDS (from data sheet) *********/
typedef enum {
    ADS_WAKEUP = B00000010, //Any following command must be sent after 4 tCLK cycles.
    ADS_STANDBY = B00000100, //It takes 9 fMOD cycles to execute
    ADS_RESET = B00000110,
    ADS_START = B00001000, //Start or restart (synchronize) conversions
    ADS_STOP = B00001010, //Stop conversion
    ADS_OFFSETCAL = B00011010, //Channel offset calibration
    ADS_ENABLE_CONTINUOUS_MODE = B00010000, //(RDATAC) Read Data Continuous
    ADS_DISABLE_CONTINUOUS_MODE = B00010001 //(SDATAC) Stop Read Data Continuously
} ADS_COMMAND;


//ADS initial register values for the proper startup (for testing purposes)
static uchar test_reg_values[] = {0x02, //reg 0x01  Set sampling ratio to 500 SPS
                                  0xA3,  //reg 0x02 Set internal reference PDB_REFBUF = 1, test enable
                                  0x10, //reg 0x03
                                  0x05,  //reg 0x04 Set Channel 1 to test
                                  0x10, //reg 0x05 Route Channel 2 to input and set amplification to 1
                                  0x00, //reg 0x06 Turn on Drl. //0x20 (Sasha) ?
                                  0x00, //reg 0x07
                                  0x40, //reg 0x08 clock divider Fclc/16 2048mHz external clock
                                  0x02, //reg 0x09 Set mandatory bit. RLD REF INT doesn't work without it.
                                  0x03}; //reg 0x0A Set RLDREF_INT


static void ads_write_command1(ADS_COMMAND command) {
    spi_exchange(command);
    DELAY_32();
}


//initial ADS startup for testing purposes
static void ads_test_config() {
    ads_write_command1(ADS_DISABLE_CONTINUOUS_MODE);   //Disable Read Data Continuous mode
    ads_write_command1(ADS_STOP);  //Stop recording, the ADS will be "auto" deselected
    //writing test startup data
    ads_write_regs(0x01, test_reg_values, sizeof(test_reg_values));
}

void ads_init() {
    //Configuring ports
    //4.4=CS, 4.5=RESET, 4.6=START, 1.2=DRDY
    P4DIR |= (BIT4 + BIT5 + BIT6);
    P4SEL &= ~(BIT4 + BIT5 + BIT6);
    //Ads in reset state and stopped
    P4OUT &= ~(BIT5 + BIT6); //  may be it is not needed. Read datasheet?
    //Ads is deselected as slave for spi
    P4OUT |= BIT4;
    //DRDY pin is input, sensitive to high-to-low transition
    P1REN &= ~DRDY_BIT;
    P1IES |= DRDY_BIT;  //When interrupt is on, it will happen on high-to-low edge transition
    P1OUT &= ~DRDY_BIT;
    P1DIR &= ~DRDY_BIT;
    P1SEL &= ~DRDY_BIT;
    //Setting up SMCLK output pin and feeding it as the ADS clocking signal
    P1REN &= ~BIT4;
    P1DIR |= BIT4;
    P1SEL |= BIT4;                                         //feeding SMCLK out
    //Starting clocking via TimerB at 2Mhz
    //TBCCR0 = 0x04;
    //TBCTL |= MC_3;
    //Startup,

    //Releasing ADS, wait for it to start and then reset
   
    P4OUT |= BIT5;
    DELAY_450000(); // стас говорит что в этой процедуре не проходят прерывания
    P4OUT &= ~BIT5;  // ads reset
    DELAY_64();
    P4OUT |= BIT5; // ads releasing
    DELAY_320();

    P4OUT &= ~BIT4; //Selecting ADS as SPI slave for microcontroller
    //ads_test_config();
}

/**
 * Запись подряд нескольких регистров
 * @param addres - starting register address
 * @param data указатель на массив данных
 * @param data_size размер данных
 */
void ads_write_regs(uchar address, uchar* data, uchar data_size) {
   
    //The Register Write command is a two-byte opcode followed by the input of the register data.
    //First opcode byte: 010r rrrr, where r rrrr is the starting register address.
    //Second opcode byte: 000n nnnn, where n nnnn is the (number of registers to write – 1)
    uchar opcode_first_byte = address | B01000000;
    uchar opcode_second_byte = data_size - 1; // (number of registers to write – 1)
    // отправляем команду записи в регистр
    spi_exchange(opcode_first_byte);
    spi_exchange(opcode_second_byte);
    for (uchar i = 0; i < data_size; i++) {
        spi_exchange(data[i]);  // write data
    }
    DELAY_32();
}

/**
 * запись одного регистра
 * @param address адресс регистра
 * @param data данные для записи в регистр
 */
/*void ads_write_1_reg(uchar address, uchar data) {
   
    //The Register Write command is a two-byte opcode followed by the input of the register data.
    //First opcode byte: 010r rrrr, where r rrrr is the starting register address.
    //Second opcode byte: 000n nnnn, where n nnnn is the (number of registers to write – 1)
    uchar opcode_first_byte = address | B01000000;
    uchar opcode_second_byte = 0x00; // (number of registers to write – 1) = 0
    // отправляем команду записи в регистр
    spi_exchange(opcode_first_byte);
    spi_exchange(opcode_second_byte);
    spi_exchange(data); //Writing one byte
    DELAY_32();
}*/

/**
 * Чтение одного реристра. Возращает прочитанное значение
 */
uchar ads_read_reg(uchar address) {
    
    //The Register Read command is a two-byte opcode followed by the output of the register data.
    //First opcode byte: 001r rrrr, where r rrrr is the starting register address.
    //Second opcode byte: 000n nnnn, where n nnnn is the number of registers to read – 1.
    uchar opcode_first_byte = address | B00100000;
    uchar opcode_second_byte = 0x00; // (number of registers to read – 1) = 0
    // отправляем команду чтения из регистра
    spi_exchange(opcode_first_byte);
    spi_exchange(opcode_second_byte);
    // отправляем 0 чтобы прочитать данные
    uchar ads_data = spi_exchange(0x00); //Reading one byte
    return ads_data;
}

void ads_stop_recording() {
    //Данные по SPI получаются в асинхронном режиме основанном на прерываниях
    // А все комманды отправляются в синхронном ждущем режиме без прерываний.
    // Эти два режима нельзя смешивать!
    // Поэтому перед тем как отправлять команды ждем окончания асинхронного приема данных по SPI
    spi_flush();
    ads_write_command1(ADS_DISABLE_CONTINUOUS_MODE); // stop continuous recording
    ads_write_command1(ADS_STOP); //ads stop
}

void ads_start_recording() {
    ADS_DRDY_INTERRUPT_DISABLE(); //disable interrupt on DRDY чтобы прерывания не нарушали процесс старта
    // очищаем флаги
    ADS_DRDY_FLAG_CLEAR(); //Clearing interrput flag DRDY
    data_ready = false;
    data_receiving = false;
    data_received = false;
    // Ждем окончания асинхронного приема данных по SPI (на всякий случай)
    spi_flush();
    ads_write_command1(ADS_ENABLE_CONTINUOUS_MODE); // enable continuous recording
    ads_write_command1(ADS_START); //start recording
    ADS_DRDY_INTERRUPT_ENABLE(); //Enabling the interrupt on DRDY
}

// метод передает указатель на конкретную функцию которая будет вызываться в DRDY прерывании (данные готовы)
void ads_DRDY_interrupt_callback(void (*func)(void)) {
    DRDY_interrupt_callback = func;
}

uchar ads_number_of_signals() {
    return ADS_NUMBER_OF_CHANNELS;
}

bool ads_data_received() {
    if (data_ready) {
        /****** Обработчик прерывания *****/
        // запускаем чтение данных из ADS по SPI
        spi_read(fill_buffer, ADS_SAMPLE_SIZE);
        // вызвываем callback функцию если ее адрес не нулевой
        if (*DRDY_interrupt_callback != NULL) {
            DRDY_interrupt_callback();
        }
        data_ready = false;
        data_receiving = true;
        /*************************************/
    }
    if (data_receiving && spi_transfer_finished() ) {
        // swap double buffers
        uchar* tmp = display_buffer;
        display_buffer = fill_buffer;
        fill_buffer = tmp;
        data_receiving = false;
        data_received = true;
    }
    return data_received;
}

/**
 * Перед тем как получить данные убедиться что они готовы. Метод ads_data_ready()!
 *
 * Возвращает ссылку на массив из 3 * ADS_NUMBER_OF_CHANNELS  байт:
 * 3 байта от первого канала
 * 3 байта от второго канала
 * ...
 * Порядок байт?
 */
uchar* ads_get_data() {
    //Dropping the first 3 bytes from ADS (там служебная информация)
    data_received = false;
    return display_buffer + 3;
}

/**
 * Перед тем как получить значение лофф статуса
 * убедиться что данные от ADS считаны. Метод ads_data_ready()
 */
// скопировано у Саши. Разобраться что за 3 служебных байта выдает ADS
uchar ads_get_loff_status() {
    uchar result = ((display_buffer[0] << 1) & 0x0E) | ((display_buffer[1] >> 7) & 0x01);
    return result;
}

#pragma vector=PORT1_VECTOR

__interrupt void PORT1_ISR(void) {
    if (ADS_DRDY_FLAG_CHECK()) { //if interrput from DRDY
        data_ready = true; // выставляем флаг
        ADS_DRDY_FLAG_CLEAR();
    }
    interrupt_flag = true;
    __low_power_mode_off_on_exit();
}









