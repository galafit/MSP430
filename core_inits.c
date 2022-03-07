#include "msp430f2274.h"

void stop_watchdog(){
// Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
}

//initialization of free I / O pins
//Free pins:
//P1.1 , P1.3 , P1.5 , P1.6 
//P2.5 
//P4.1 , P4.2 , P4.3 
void io_init(){
  //After reset, all pins are input 
  //Turning on the pull-up resistors
  P1REN |= (BIT1 + BIT3); // + BIT5 + BIT6);
  P2REN |=  BIT5;
  P4REN |= (BIT1 + BIT2 + BIT3);
}

//initializing the Xtal oscillator and making it the souce for all peripheral clocks
//all clocks are sourced from the xtal, full peed, no division
void clock_init(){
  BCSCTL3 &= ~XCAP_3;           //Our HF xtal doesn't need internal capacitors
  BCSCTL1 = CALBC1_16MHZ; 
  BCSCTL1 |= XTS;               //Our xtal is a high frequency one
  BCSCTL1 &= ~DIVA_3;           //ACLK full speed
  DCOCTL = CALDCO_16MHZ;
  BCSCTL2 |= DIVS_3;            //SMCLK 1/8=2Mhz
  BCSCTL3 |= LFXT1S1;           // 3 – 16MHz crystal or resonator
  while(BCSCTL3&LFXT1OF){       //Giving xtal time to stabilize
    IFG1 = 0;
  };
  BCSCTL2 |= (SELM_3 + SELS);   //sourcing MCLK&SMCLK from the xtal
}




