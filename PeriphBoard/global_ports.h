#ifndef GLOBAL_PORTS_HHHHHHH8848484
#define GLOBAL_PORTS_HHHHHHH8848484

#include <asf.h>

// Define a bunch of global variables that multiple parts of a program may need

    // Ports
extern Port* port;
extern PortGroup* bankA, *bankB;

    // ADC and DAC
extern Adc* adc;
extern Dac* dac;

    // Timers
    // timerX refers to TcX while timerX_Y refers to TcX Y-bit counter
extern Tc* timer2_set, *timer4_set, *timer6_set, *timer7_set;
    // Be careful with TcCount instances since they are part of a union in Tc
extern TcCount8* timer2_8;
extern TcCount8* timer4_8;
extern TcCount16* timer2_16;
extern TcCount8* timer6_8;
extern TcCount8* timer7_8;
extern TcCount16* timer7_16;

    // Assign valid memory addresses to each pointer.
    //  Always call this before dereferencing any of the global pointers.
void configure_global_ports(void);

#endif