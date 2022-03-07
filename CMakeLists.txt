cmake_minimum_required(VERSION 3.14)
project(MSP430 C)

set(CMAKE_C_STANDARD 11)

include_directories(.)

add_executable(MSP430
        msp430f2274.h
        intrinsics.h
        in430.h
        iomacros.h
        bynary.h
        ringbuffer.h
        main.c
        core_inits.c
        core_inits.h
        leds.h
        commands.c
        commands.h
        uart_spi.c
        uart_spi.h
        ads1292.c
        ads1292.h
        adc.c
        adc.h
        databatch.h
        databatch.c
        utypes.h
        interrupts.h)
