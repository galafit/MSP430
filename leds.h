#ifndef LEDS_H
#define LEDS_H

#include "msp430f2274.h"

#define LEDS_INIT() P1DIR |= (BIT5 + BIT6 + BIT7); P1OUT &= ~(BIT5 + BIT6 + BIT7)

#define LED1_ON()  (P1OUT |= BIT5)
#define LED2_ON()  (P1OUT |= BIT6)
#define LED3_ON()  (P1OUT |= BIT7)
#define LED1_OFF()  (P1OUT &= ~BIT5)
#define LED2_OFF()  (P1OUT &= ~BIT5)
#define LED3_OFF()  (P1OUT &= ~BIT7)
#define LED1_SWITCH()  (P1OUT ^= BIT5)
#define LED2_SWITCH()  (P1OUT ^= BIT6)
#define LED3_SWITCH()  (P1OUT ^= BIT7)


#endif //LEDS_H
