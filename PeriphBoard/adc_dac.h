#ifndef ADCDAC_COMB232INED_HEADER__
#define ADCDAC_COMB232INED_HEADER__

// Uncomment the below macros to remove either
//  the adc or dac from the translation unit
//#define NO_ADC__
//#define NO_DAC__

#if !defined(NO_ADC__) || !defined(NO_DAC__)
    #include <asf.h>
    #include "extended_types.h"
#endif

#ifndef NO_ADC__
    void map_to_adc_odd(UINT8 port_pin);
    void map_to_adc_even(UINT8 port_pin);
    void enable_adc_clk(void);
        // Use default settings for a specific analog pin
        //  in a single-ended application.
    void configure_adc_default(UINT8 ain);
        // Set up the adc with specific behaviours.
        //  ref         - Reference voltage (BIN = 0xFFFFF*(Vin/Vref)*Gain)
        //  samp_rate   - Sampling rate per conversion
        //  samp_time   - Sampling time ((samp_time+1)*(ADC_Clk/2))
        //  presc       - ADC clock prescaler (ADC_Clk = Gen_Clk/presc)
        //  ressel      - Resolution
        //  gain        - Gain (BIN = 0xFFFFF*(Vin/Vref)*Gain)
        //  neg_ain     - Analog pin to map to the differential's negative terminal
        //  ain         - Analog pin to map to the differential's positive terminal
        //  See Chap. 28 of the SAMD20 datasheet for exact values to use.
    void configure_adc(
        UINT8 ref, UINT8 samp_rate, UINT8 samp_time, UINT8 presc,
        UINT8 ressel, UINT8 gain, UINT8 neg_ain, UINT8 ain
    );
    void enable_adc(void);
    void disable_adc(void);
    unsigned int read_adc(void);
#endif

#ifndef NO_DAC__
    void map_to_dac_odd(UINT8 port_pin);
    void map_to_dac_even(UINT8 port_pin);
    void enable_dac_clk(void);
        // Use the default reference
    void configure_dac_default(void);
        // Set up the dac with specific behaviours.
        //  ref         - Reference voltage (Vout = BIN*Vref/0xFFFFF)
    void configure_dac(UINT8 ref);
    void enable_dac(void);
    void disable_dac(void);
    void write_to_dac(UINT16 val);
#endif

#endif