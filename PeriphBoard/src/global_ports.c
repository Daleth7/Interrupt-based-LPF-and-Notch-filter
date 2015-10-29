#include "global_ports.h"

    // Ports
Port* port;
PortGroup* bankA, *bankB;

    // ADC and DAC
Adc* adc;
Dac* dac;

    // Timers
    // timerX refers to TcX while timerX_Y refers to TcX Y-bit counter
Tc* timer2_set, *timer4_set, *timer7_set;
    // Be careful with TcCount instances since they are part of a union in Tc
TcCount8* timer2_8;
TcCount8* timer4_8;
TcCount16* timer2_16;
TcCount8* timer7_8;
TcCount16* timer7_16;

    // Assign valid memory addresses to each pointer.
    //  Always call this before dereferencing any of the global pointers.
void configure_global_ports(void){
    port = (Port*)(PORT);
    bankA = (PortGroup*)(&port->Group[0]);
    bankB = (PortGroup*)(&port->Group[1]);

    adc = (Adc*)(ADC);
    dac = (Dac*)(DAC);

    timer2_set = (Tc*)(TC2);
    timer2_8 = (TcCount8*)(&timer2_set->COUNT8);

    timer4_set = (Tc*)(TC4);
    timer4_8 = (TcCount8*)(&timer4_set->COUNT8);
    timer2_16 = (TcCount16*)(&timer2_set->COUNT16);

    timer7_set = (Tc*)(TC7);
    timer7_8 = (TcCount8*)(&timer7_set->COUNT8);
    timer7_16 = (TcCount16*)(&timer7_set->COUNT16);
}