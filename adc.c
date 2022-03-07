#include "msp430f2274.h"
#include "interrupts.h"

#define  ADS_NUMBER_OF_CHANNELS 4
static unsigned int adc_data[ADS_NUMBER_OF_CHANNELS];
// двойная буфферизация
static unsigned int adc_accumulator_0[ADS_NUMBER_OF_CHANNELS];
static unsigned int adc_accumulator_1[ADS_NUMBER_OF_CHANNELS];
static unsigned char adc_accumulator_index = 0;
static unsigned int* adc_accumulator_fill = adc_accumulator_0;


void adc_init(){
  //P2.0=accX, P2.1=accY, P2.2=accZ, P2.3=battery, P3.0=accSleep
  P2SEL |= (BIT0 + BIT1 + BIT2 + BIT3);                  //P2.0-P2.2 are used by the ADC
  P2DIR &= ~(BIT0 + BIT1 + BIT2 + BIT3);                 //acc XYZ pins are inputs
  P2OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);
  P3DIR |= BIT0;                        //Use this pin as output to put the acc into a low power mode
  P3SEL &= ~BIT0;                       //This pin is an ordinary GPIO pin
  P3OUT &= ~BIT0;                        //Disable acc sleep mode
  //Configuring ADC
  ADC10CTL0=ADC10CTL1 = 0;                               //Ensuring that the ADC is off
  ADC10CTL0 |= (SREF_1 + ADC10SHT_3);                    //ADC reference = 2.5V to VSS, sample and hold = 64clk
  ADC10CTL0 |= (ADC10SR + REFBURST + REF2_5V);           //Reference buffer for 50ksps max, 2.5v, buffer only on during conversion 
  ADC10CTL0 |= (REFON + ADC10IE + MSC);                  //Reference on, interrupts on, converting the whole sequence via a single trigger
  ADC10CTL1 |= (INCH_3 + ADC10DIV_1 + ADC10SSEL_1 + CONSEQ_1); //Choose A3-A0 as inputs, clock=ACLK/3, sequence of channels single conversion
  ADC10AE0 |= (BIT0 + BIT1 + BIT2 + BIT3);                     //Arming pins for ADC
  //DMA settings
  ADC10DTC1 = 0x04;                                      //We will transfer 4 conversions at a time
  ADC10CTL0 |= ADC10ON;                                  //Start ADC
}

/* --------------------- Конвертация по 4м каналам -------------------- */

void adc_convert_begin(){
    // disable conversion to allow change
    // Все делают но нам скорей всего не надо потому как мы тут в ADC ничего не настраиваем
    //DC10CTL0 &= ~ENC;
    while(ADC10CTL1 & BUSY);              // Wait if ADC10 busy
    ADC10SA = (unsigned int)(adc_data);   //Start address of the acc data buffer
    ADC10CTL0 |= ENC + ADC10SC;           //Start conversion
}
/* -------------------------------------------------------------------------- */

unsigned char* adc_get_data(){
    unsigned char* data = (unsigned char*) adc_accumulator_fill;
    // переключаемся на второй буффер и обнуляем его значения
    if(adc_accumulator_index == 0) {
        adc_accumulator_index = 1;
        adc_accumulator_fill = adc_accumulator_1;

    } else {
        adc_accumulator_index = 0;
        adc_accumulator_fill = adc_accumulator_0;
    }
    for (int i = 0; i < ADS_NUMBER_OF_CHANNELS; ++i) {
        adc_accumulator_fill[i] = 0;
    }
    return data;
}

#pragma vector=ADC10_VECTOR
__interrupt void adc10_isr(void){
 //uart_send_bytes(sizeof(adc_data), (unsigned char*)adc_data);
 //Approximating Acc data
    for (int i = 0; i < ADS_NUMBER_OF_CHANNELS; ++i) {
        adc_accumulator_fill[i] += adc_data[i];
    }
    interrupt_flag = true;
    __low_power_mode_off_on_exit();
}
