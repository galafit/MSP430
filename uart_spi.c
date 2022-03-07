#include "msp430f2274.h"
#include <stdbool.h>
#include "utypes.h"
#include "leds.h"
#include "interrupts.h"

/**
 * Обмен информацией через UART происходит в дуплексном режиме,
 * т.е. передача данных может происходить одновременно с приемом.
 * Для этого в интерфейсе UART есть два сигнала:
 *   TX – выход  для отправки данных (transmitter)
 *   RX – вход для приема данных (receiver)
 * **********************************************************************
 * UART имеет 2 буффера - один для получения и один для отправки данных:
 *
 * UCAxRXBUF Receive buffer register
 * UCAxTXBUF Transmit buffer register
 * **********************************************************************
 * Когда данные поступают в RXBUF или отправляются из TXBUF выставляется соотвествующий флаг в
 * регистре флагов прерываний [INTERRUPT_FLAG_REGISTER_2] (IFG2)
 *
 * Флаг(бит) UCA0RXIFG выставляется когда символ поступил во входной буффер RXBUF
 * Флаг сбрасывается когда символ из RXBUF прочитан
 *
 * Флаг(бит) UCA0TXIFG выставляется когда символ из буфера передачи TXBUF
 * перебрасываются в сдвиговый регистр и TXBUF становится свободен для записи след символа
 * Флаг сбрасывается когда в TXBUF помещается символ
 * Tак же этот флаг выставляется after a PUC or when UCSWRST = 1
 *
 * PS Флаги UCA0RXIFG и UCA0TXIFG устанавливет и сбрасывает система.
 * Ручками это делать не нужно и не правильно!
 * **********************************************************************
 * При установке флагов прерываний срабатывает соответсвующий вектор прерываний.
 * UART имеет 2 вектора прерываний:
 *
 * RX-interrupt срабатывает когда выставлен флаг UCA0RXIFG
 * (данные поступилит во входной буффер RXBUF)
 *
 * TX-interrupt срабатывает когда выставлен флаг UCA0TXIFG
 * (буфер на отправку TXBUF свободен для записи)
 *
 * PS UART прерывания работают по уровню а не по фронту! То есть прерывание будет вызываться
 * все время пока флаг установлен.
 *
 * **********************************************************************
 * Чтобы прерывания сработали они должны быть разрешены.
 * За разрешение прерываний отвечает регистр [INTERRUPT_ENABLE_REGISTER_2](IE2)
 *
 * флаг(бит) UCAxRXIE разрешает/запрещает прерывания при получении символа (когда выставлен флаг UCA0RXIFG)
 * флаг(бит) UCA0TXIE разрешает/запрещает прерывания при отправке символа (когда выставлен флаг UCA0TXIFG)
 *
 * PS При старте TXBUF свободен, соответсвенно флаг UCA0TXIFG выставлен и
 * как только мы установим флаг разрешения прерывания UCA0TXIE мы сразу же в это прерывание
 * влетим!
 *************************************************************************
 *
 * SPI все тоже самое только чтение-отправление идут всегда одновременно и для того чтобы
 * прочитать символ нужно какой-то символ отправить. Если отправлять нечего то для чтения
 * оправляют нулевой байт (0x00)
 *
 *  USCI_A0 используется UART
 *  USCI_B0 используется SPI
 *
 *  USCI_A0 и USCI_B0 имеют общий вектор прерывания на получение и отправку данных
 *  vector=USCIAB0RX_VECTOR и  vector=USCIAB0TX_VECTOR
 *  Поэтому они обьеденены в один блок.
 *  При срабатывании этих прерываний нужно проверять соответсвующий флаг прерываний
 *  чтобы понять что именно вызвало прерывание - SPI  или UART
 */
/*==========================================================*/
/**
 * RX - RECEIVE
 * TX - TRANSMIT
 */
/**======================== UART BLOCK==================================*/
#define UART_RX_BUFFER  UCA0RXBUF //uart receive buffer
#define UART_TX_BUFFER  UCA0TXBUF //uart transmit buffer

// флаг выставляется когда символ приходит в буфер приемник (RXBUF)
#define UART_RX_FLAG_CHECK()  (IFG2 & UCA0RXIFG)

// флаг выставляется
// 1) когда символ из буфера передачи (TXBUF) перебрасываются в сдвиговый регистр и TXBUF становится свободным для записи
// 2) after a PUC or when UCSWRST = 1

#define UART_TX_FLAG_CHECK()   (IFG2 & UCA0TXIFG)
#define UART_RX_INTERRUPT_ENABLE()  (IE2 |= UCA0RXIE)
#define UART_TX_INTERRUPT_ENABLE()  (IE2 |= UCA0TXIE)
#define UART_TX_INTERRUPT_DISABLE()  (IE2 &= ~UCA0TXIE)

/*------------ UART receive circular fifo buffer ------------*/
#define UART_RX_FIFO_BUFFER_SIZE 32
static uchar uart_rx_fifo_buffer[UART_RX_FIFO_BUFFER_SIZE];
static volatile uint uart_rx_buffer_head;
static volatile uint uart_rx_buffer_tail;
/*__________________________________________________*/

static uchar* uart_tx_data;
static volatile int uart_tx_data_size;

// TODO сделать enum с несколькими скоростями который передавать как параметр в init
void uart_init() {
    // DEFAULT: parity disabled - LSB - 8bit data - one stop bit - UART mode - Asynchoronous mode (page 434)

    // As said in 15.3.1 of holy user guide,
    // before initialization of USCI we must reset it.
    UCA0CTL1 |= UCSWRST;  //stopping uart
    UCA0CTL1 |= UCSSEL0;  //uart clock source - ACLK

    //setting the baud rate
    UCA0BR1 = 0x00;
    UCA0BR0 = 0x02;
    // UCA0MCTL, регистр управления модуляцией модуля USCI_A0.
    UCA0MCTL = 0x37; // Магическое число. Примерно вычисляется по даташиту и экспериментально уточняется.
    // В даташите slas504g.pdf (MSP430F22x2_MSP430F22x4) стр. 69 и дальше указано, что для битов
    // с 1 по 7 порта 3, при установке P3SEL, значение P3DIR может быть любым.
    P3SEL |= (BIT4 + BIT5);

    UCA0CTL1 &= ~UCSWRST; //releasing uart  *Initialize USCI state machine*

    /*---------------- Enabling the interrupts ------------------*/
    UART_RX_INTERRUPT_ENABLE(); // включить прерывания на передачу
}

/**
 * Блокирующая операция отправки данных. Для тестовых целей
 * Не делать одновременно блокирующее и неблокирующее отправление!!!
 * результат не предсказуем
 */
//void uart_blocking_transmit(uchar ch) {
//    // Wait for TX buffer to be ready for new data
//    while (! UART_TX_FLAG);
//    // Push data to TX buffer
//    UART_TX_BUFFER = ch;
//}

/**
* Не блокирующая  отправка  напрямую из переданного массива.
*  Переданный массив нельзя изменять пока все данные не будут отправлены.
* Также в это время нельзя добавлять другие данные на отправку
* Для работы по принципу двойной буфферизации
* (когда есть два массива одинаковой длины -
* один для отправку а второй в это время заполнять
*/
void uart_transmit(uchar* data, int data_size) {
    UART_TX_INTERRUPT_DISABLE(); // выключить прерывания на прием
    uart_tx_data = data;
    uart_tx_data_size = data_size;
    UART_TX_INTERRUPT_ENABLE(); // включить прерывания на прием
}

/**
 *  Waits for the transmission of outgoing uart data to complete
 */
void uart_flush() {
    while(uart_tx_data_size > 0);  //SLEEP_WITH_ENABLED_INTERRUPTS();
}

/**
 * @return true если ассинхронная передача по UART завершены
 */
//bool uart_transmit_finished() {
//    if(uart_tx_data_size <= 0) {
//        return true;
//    }
//    return false;
//}

/**
 * Берет элемент из входящего fifo buffer где накапливаются поступающие данные
 * и записывает его в переменную по указанному адресу.
 * return true(1) if element was read successfully
 * and false(0) if uart fifo buffer is empty and element can not be read
 */
bool uart_read(uchar* chp) {
    // если буфер пустой
    if (uart_rx_buffer_head == uart_rx_buffer_tail) {
        return false;
    }
    *chp = uart_rx_fifo_buffer[uart_rx_buffer_tail];

    uint next_tail = (uint) (uart_rx_buffer_tail + 1);
    if (next_tail >= UART_RX_FIFO_BUFFER_SIZE) {
        next_tail = 0;
    }
    uart_rx_buffer_tail = next_tail;
    return true;
}

/**======================== SPI BLOCK==================================*/
#define NULL 0x00

#define SPI_RX_BUFFER  UCB0RXBUF //spi receive buffer
#define SPI_TX_BUFFER  UCB0TXBUF //spi transmit buffer

// флаг выставляется когда символ приходит в буфер приемник (SPI RXBUF)
#define SPI_RX_FLAG_CHECK()  (IFG2 & UCB0RXIFG)

// флаг выставляется
//1) когда символ из буфера передачи (TXBUF) перебрасываются в сдвиговый регистр 
//   и TXBUF становится свободным для записи
// 2) after a PUC or when UCSWRST = 1
#define SPI_TX_FLAG_CHECK()  (IFG2 & UCB0TXIFG)

//The UCBUSY flag in UCBxSTAT is set while the USCI (SPI) is receiving or sending
#define SPI_BUSY_FLAG_CHECK() (UCB0STAT & UCBUSY)

#define SPI_RX_INTERRUPT_ENABLE()  (IE2 |= UCB0RXIE)
#define SPI_TX_INTERRUPT_ENABLE()  (IE2 |= UCB0TXIE)
#define SPI_RX_INTERRUPT_DISABLE()  (IE2 &= ~UCB0RXIE)
#define SPI_TX_INTERRUPT_DISABLE()  (IE2 &= ~UCB0TXIE)

/*---- ссылка на буфер куда будут сохраняться поступающие данные-----*/
static volatile uchar* spi_rx_data;
static volatile int spi_rx_data_size;

/*---- ссылка на буфер из которого будут отправляться данные-----*/
static volatile uchar* spi_tx_data;
static volatile int spi_tx_data_size;

static volatile bool read_available; //true поступающие данные сохраняются в буфер spi_rx_data
static volatile bool transmit_available; //true данные отправляются из буффера spi_tx_data, false вместо данных отправляется NULL

void spi_init() {
    UCB0CTL1 |= UCSWRST;                          //Stopping SPI

    //UCB0CTL0 |= (UCMST + UCMSB + UCSYNC);  _!!!_ //SPI 3-wire, master mode, 8 bits per byte, MSB first
    // UCMST почему то дает предупреждение поэтому он заменен на свое значение  0x0008
    UCB0CTL0 |= (0x0008 + UCMSB + UCSYNC);        //SPI 3-wire, master mode, 8 bits per byte, MSB first

    UCB0CTL1 |= UCSSEL_1;                         //Clock source ACLK
    //setting SPI frequency to 16/2=8Mhz
    UCB0BR1 = 0x00;
    UCB0BR0 |= 0x08;  //SPI speed = 2Mhz; SPI frequency to 16/8 = 2Mhz

    UCB0STAT = 0x00;   //Resetting all SPI statistics flags and disable the transmitter feed into receiver

    //configuring ports
    // В даташите slas504g.pdf (MSP430F22x2_MSP430F22x4) стр. 69 и дальше указано, что для битов
    // с 1 по 7 порта 3, при установке P3SEL, значение P3DIR может быть любым.
    P3SEL |= (BIT1 + BIT2 + BIT3);  //Connecting pins to USCIB0 module

    UCB0CTL1 &= ~UCSWRST;                 //Releasing SPI
}

/**
 * Блокирующая операция отправки и получения! Transmit and receive one byte by the SPI
 * Отправляет 1 байт, ждет получения 1 байта и возвращает его.
 * Не применять одновременно с неблокирующи  SPI обменом
 * использующим прерывания! Результат не предсказуем
 * PS этот метод в ардуино библиотеках обычно называют spi_transfer
 */
uchar spi_exchange(uchar tx_data) {
    SPI_RX_INTERRUPT_DISABLE(); // Выключаем прерывание на прием по SPI
    SPI_TX_INTERRUPT_DISABLE(); // Выключаем прерывание на получение по SPI
    while (!SPI_TX_FLAG_CHECK()); // Wait for TXBUF ready
    SPI_TX_BUFFER = tx_data;         // Send data
    while (SPI_BUSY_FLAG_CHECK());   // Wait for TX and RX to complete
    uchar rx_data = SPI_RX_BUFFER; // Read received data
    return rx_data;
}

/**
* Не блокирующая  отправка.
* Отправка будет осуществлятся напрямую из переданного массива.
* Переданный массив нельзя изменять пока все данные не будут отправлены!
* Также в это время нельзя добавлять другие данные на отправку
*/
void spi_transmit(uchar* data, int data_size) {
    SPI_RX_INTERRUPT_DISABLE(); // Выключаем прерывание на прием по SPI
    SPI_TX_INTERRUPT_DISABLE(); // Выключаем прерывание на получение по SPI
    spi_tx_data = data;
    spi_tx_data_size = data_size;
    spi_rx_data_size = data_size;
    transmit_available = true;
    read_available = false;
    SPI_RX_INTERRUPT_ENABLE();  // Enable Receive  interrupt
    SPI_TX_INTERRUPT_ENABLE(); // Enable Transmit  interrupt
}

/**
 * Неблокирующее получение заданного числа элементов по SPI.
 * Чтобы их получить будут отправлено соответсвуюющее количество нулей
 * Для чтения больших массивов данных
 */
void spi_read(uchar* read_buffer, int data_size) {
    SPI_RX_INTERRUPT_DISABLE(); // Выключаем прерывание на прием по SPI
    SPI_TX_INTERRUPT_DISABLE(); // Выключаем прерывание на получение по SPI
    spi_rx_data = read_buffer;
    spi_rx_data_size = data_size;
    spi_tx_data_size = data_size;
    transmit_available = false;
    read_available = true;
    SPI_RX_INTERRUPT_ENABLE();  // Enable Receive  interrupt
    SPI_TX_INTERRUPT_ENABLE(); // Enable Transmit  interrupt
}

/**
 *  Ждет пока  ассинхронная передача и чтение по SPY будут завершены
 */
void spi_flush() {
    while(spi_rx_data_size > 0);// SLEEP_WITH_ENABLED_INTERRUPTS();
}

/**
 * @return true если ассинхронная передача и чтение по SPY завершены
 */
bool spi_transfer_finished() {
  if(spi_rx_data_size <= 0) {
    return true;
  }
  return false;
}

/**======================== UART/SPI TX and RX INTERRUPTS==================================*/
#pragma vector = USCIAB0RX_VECTOR
__interrupt void RX_ISR(void) {
    // UART
    if (UART_RX_FLAG_CHECK()) {
        // Прочитать символ из буфера-приемника
        uchar ch = UART_RX_BUFFER;
        // Проверить что uart fifo buffer не полон
        uint next_head = (uint) (uart_rx_buffer_head + 1);
        if (next_head >= UART_RX_FIFO_BUFFER_SIZE) {
            next_head = 0;
        }
        if (next_head != uart_rx_buffer_tail) { // буфер неполон
            // Положить пришедший символ в фифо буффер
            uart_rx_fifo_buffer[uart_rx_buffer_head] = ch;
            uart_rx_buffer_head = next_head;
        }
    }
    // SPI
    if (SPI_RX_FLAG_CHECK()) {
      // Прочитать символ из буфера-приемника 
        uchar ch = SPI_RX_BUFFER;
        if (spi_rx_data_size <= 0) { // ожидаемое количество данных уже принято
            // Выключаем прерывание на прием по SPI
            SPI_RX_INTERRUPT_DISABLE();
        } else {
            if(read_available) {
                *spi_rx_data++ = ch; // положить символ в буффер для получения данных
            }   
            spi_rx_data_size--;
        }
    }
    interrupt_flag = true;
    __low_power_mode_off_on_exit();
}

int count = 0;
#pragma vector = USCIAB0TX_VECTOR
__interrupt void TX_ISR(void) {
    // UART
    if (UART_TX_FLAG_CHECK()) {
        if (uart_tx_data_size <= 0) { // Исходящий буфер пуст
            // Выключаем прерывание на передачу USCI
            UART_TX_INTERRUPT_DISABLE();
        } else {
            UART_TX_BUFFER = *uart_tx_data++;
            uart_tx_data_size--;
        }
    }
    // SPI
    if (SPI_TX_FLAG_CHECK()) {
        if (spi_tx_data_size <= 0) { // нечего передавать
            // Выключаем прерывание на передачу SPI
            SPI_TX_INTERRUPT_DISABLE();
        } else {
            if(transmit_available) {
                SPI_TX_BUFFER = *spi_tx_data++; // отправляем данные
            } else {
                SPI_TX_BUFFER = NULL; // отправляем ноль чтобы прочитать данные
            }
            spi_tx_data_size--;
        }
    }
      interrupt_flag = true;
    __low_power_mode_off_on_exit();
}

