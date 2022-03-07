#include "msp430f2274.h"
#include "core_inits.h"
#include "uart_spi.h"
#include "leds.h"
#include "commands.h"
#include "ads1292.h"
#include "adc.h"
#include "databatch.h"
#include "interrupts.h"

volatile bool interrupt_flag;

int main(void){
  stop_watchdog();
  io_init();
  LEDS_INIT();
  clock_init();
  uart_init();
  spi_init();
  ads_init();
  adc_init();
  // передаем в ADS ссылку на функцию из ADC10 которую ads будет вызывать в прерывании DRDY при поступлении данных
    ads_DRDY_interrupt_callback(adc_convert_begin);
   // __bis_SR_register(GIE); // enable global interrupts
    INTERRUPTS_ENABLE();
  while(1){
      while (interrupt_flag) {
          interrupt_flag = false;
          commands_process();
          databatch_process();
      }
      // need to read the interrupt flag again without allowing any new interrupts:
      INTERRUPTS_DISABLE();
      if (interrupt_flag == false) {
          SLEEP_WITH_ENABLED_INTERRUPTS(); // an interrupt will cause a wake up and run the while loop
      }
      INTERRUPTS_ENABLE();
  }
}
