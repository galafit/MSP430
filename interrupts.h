#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <stdbool.h>

#define INTERRUPTS_ENABLE()  __enable_interrupt() // Enable Global Interrupts by GIE = 1
#define INTERRUPTS_DISABLE() __disable_interrupt() // Disable Global Interrupts by GIE = 0
#define SLEEP_WITH_ENABLED_INTERRUPTS()   __bis_SR_register(LPM0_bits + GIE) // Going to LPM0 interrapts enabled

// ВСЕ прерывания должны выставлять этот флаг!
extern volatile bool interrupt_flag;

#endif //INTERRUPT_H
