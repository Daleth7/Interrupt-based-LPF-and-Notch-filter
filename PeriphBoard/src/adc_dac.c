#include "adc_dac.h"

#include "global_ports.h"

#ifndef NO_ADC__

static Adc* adc_ptr;
static PortGroup* bankA_ptr;

void map_to_adc_odd(UINT8 port_pin){
        // Choose function B, which is analog input
        //  Shift by 4 to enter into the odd register
    bankA_ptr->PMUX[(port_pin - 1)/2].reg |= 0x1 << 4;
        // Connect pin to multiplexer to allow analog input
    bankA_ptr->PINCFG[port_pin].reg |= 0x1;
}

void map_to_adc_even(UINT8 port_pin){
        // Choose function B, which is analog input
    bankA_ptr->PMUX[port_pin/2].reg |= 0x1;
        // Connect pin to multiplexer to allow analog input
    bankA_ptr->PINCFG[port_pin].reg |= 0x1;
}

void enable_adc_clk(void){
    PM->APBCMASK.reg |= 1 << 16;             // PM_APBCMASK enable is in the 16 position
    
    uint32_t temp = 0x17;                 // ID for ADC 0x17 (see table 14-2)
    temp |= 0<<8;                             // Selection Generic clock generator 0
    GCLK->CLKCTRL.reg = temp;                 // Setup in the CLKCTRL register
    GCLK->CLKCTRL.reg |= 0x1u << 14;         // enable it.
}

void configure_adc_default(UINT8 ain){
    configure_adc(
        0x2,    // Select a V_DD_AN/2 (1.65) reference
        0x8,    // Now collect 256 samples at a time.
                //  Theoretical result has 20-bit precision.
                //  ADC will automatically right shift
                //  6 times, so result has 16-bit precision.
            // Total sampling time length = (SAMPLEN+1)*(Clk_ADC/2)
        0x1,    // Set sampling time to 1 adc clock cycle?
        0x2,    // Relative to main clock, have adc clock
                //  run 8 times slower
        0x1,    // For averaging more than 2 samples,
                //  change RESSEL (0x1 for 16-bit)
        0xF,    // Since reference is 1/2,
                //  set gain to 1/2 to keep largest
                // input voltage range (expected input
                //  will be 0 - 3.3V)
        0x18,   // Not using the negative for differential, so ground it.
        ain     // Map the adc to analog pin ain
        );
}

void configure_adc(
    UINT8 ref, UINT8 samp_rate, UINT8 samp_time, UINT8 presc,
    UINT8 ressel, UINT8 gain, UINT8 neg_ain, UINT8 ain
){
    configure_global_ports();
    enable_adc_clk();
    adc_ptr = adc;
    bankA_ptr = bankA;

    disable_adc();

    /*
        BIN = (0xFFF)(V_in * GAIN / V_ref)
    */
    adc_ptr->REFCTRL.reg |= ref;        // Select a reference
    adc_ptr->AVGCTRL.reg |= samp_rate;  // Now collect a number
                                        //  of samples at a time.
        // Total sampling time length = (SAMPLEN+1)*(Clk_ADC/2)
    adc_ptr->SAMPCTRL.reg = samp_time;    // Set sampling time
    adc_ptr->CTRLB.reg |= presc << 8u;    // Set clock prescaler
    adc_ptr->CTRLB.reg |= ressel << 4;     // Select resolution
    adc_ptr->INPUTCTRL.reg |= gain << 24u;   // Select Gain
    adc_ptr->INPUTCTRL.reg |=
        neg_ain << 8    // Map the negative terminal
        | ain           // Map the positive terminal
        ;
    
    enable_adc();
}

void enable_adc(void){
    adc_ptr->CTRLA.reg |= 0x2;  //ADC block is enabled   
}

void disable_adc(void){
    adc_ptr->CTRLA.reg &= ~0x2;  //ADC block is disabled   
}

unsigned int read_adc(void){

    // start the conversion
    adc_ptr->SWTRIG.reg |= 1 << 1u;
    while(!adc_ptr->INTFLAG.bit.RESRDY);    //wait for conversion to be available

        // Use a sample and hold method
    unsigned int toreturn = adc_ptr->RESULT.reg;

    UINT32 count = 0;
    for(count = 0; count < 350; ++count){
        adc_ptr->SWTRIG.reg |= 1 << 1u;
        while(!adc_ptr->INTFLAG.bit.RESRDY);    //wait for conversion to be available
    }
    
    return toreturn; // Extract stored value
    
}

#endif

#ifndef NO_DAC__

static Dac* dac_ptr;

void map_to_dac_odd(UINT8 port_pin){
	bankA_ptr->PINCFG[port_pin].reg |= 0x1; // Enable multiplexing
        // Set to function B, which include DAC
        //  Shift by 4 for PUXO
	bankA_ptr->PMUX[(port_pin-1)/2].reg |= 0x1 << 4u;
}

void map_to_dac_even(UINT8 port_pin){
	bankA_ptr->PINCFG[port_pin].reg |= 0x1; // Enable multiplexing
	bankA_ptr->PMUX[port_pin/2].reg |= 0x1; // Set to function B, which include DAC
}

void enable_dac_clk(void){
    PM->APBCMASK.reg |= 1 << 18;             // PM_APBCMASK enable is in the 16 position
    
    uint32_t temp = 0x1A;                 // ID for ADC 0x17 (see table 14-2)
    temp |= 0<<8;                             // Selection Generic clock generator 0
    GCLK->CLKCTRL.reg = temp;                 // Setup in the CLKCTRL register
    GCLK->CLKCTRL.reg |= 0x1u << 14;         // enable it.
}

void configure_dac_default(void){
    configure_dac(0x1);

	dac_ptr->CTRLB.reg |= 0x1;  // Enable DAC output to Vout

	while (dac_ptr->STATUS.reg & DAC_STATUS_SYNCBUSY);
}

void configure_dac(UINT8 ref){
    configure_global_ports();
    enable_dac_clk();
    dac_ptr = dac;

        // With each register change, wait for synchronization

	while (dac_ptr->STATUS.reg & DAC_STATUS_SYNCBUSY);

    dac_ptr->CTRLB.reg |= 0x1 << 6; // Set reference to be V_dd_ana

	while (dac_ptr->STATUS.reg & DAC_STATUS_SYNCBUSY);

    enable_dac();

	dac_ptr->CTRLB.reg |= 0x1;  // Enable DAC output to Vout

	while (dac_ptr->STATUS.reg & DAC_STATUS_SYNCBUSY);
}

void enable_dac(void){
    dac_ptr->CTRLA.reg |= 0x2; // Enable DAC

	while (dac_ptr->STATUS.reg & DAC_STATUS_SYNCBUSY);  // Synchronize clock
}

void disable_dac(void){
    dac_ptr->CTRLA.reg &= ~0x2; // Disable DAC
}

#endif