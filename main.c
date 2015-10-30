#include <asf.h>

#include "PeriphBoard/system_clock.h"
#include "PeriphBoard/global_ports.h"
#include "PeriphBoard/ssd.h"
#include "PeriphBoard/adc_dac.h"
#include "PeriphBoard/utilities.h"

    // Switch between using 16-bit and 12-bit resolution for the ADC
#define RESOLUTION 12

void enable_adc_tc_clocks(void);
void enable_adc_timer(void);
void disable_adc_timer(void);
    // To prevent premature interrupts, the timer will
    //  remain disabled after configuration.
void configure_adc_interrupt(void);

void adc_handler(void);

void enable_display_tc_clocks(void);
void enable_display_timer(void);
void disable_display_timer(void);
    // To prevent premature interrupts, the timer will
    //  remain disabled after configuration.
void configure_display_interrupt(void);

void display_handler(void);

static TcCount16* disp_timer;
static TcCount8* adc_timer;

#define POT_SRC     13      // Pin POT_SRC supplies voltage to voltage divider circuit
#define ADC_PIN     11      // Use pin 11 for analog input from voltage divider
#define AIN_PIN     0x13    // Use 0x13 as the port map to the analog pin
#define DAC_PIN     2       // Use pin 2 to output waveform

#define DISPLAY_DIGIT_SIZE_MAX 4
static UINT8 display_number[DISPLAY_DIGIT_SIZE_MAX] = {1, 1, 1, 1};

int main (void)
{
    Simple_Clk_Init();
    delay_init();
    configure_global_ports();
    configure_ssd_ports();

#if RESOLUTION == 16
    // 16-bit resolution
    configure_adc(
        0x2,    // Select a V_DD_AN/2 (1.65) reference
        0x8,    // Now collect 256 samples at a time.
                //  Theoretical result has 20-bit precision.
                //  ADC will automatically right shift
                //  4 times, so result has 16-bit precision.
            // Total sampling time length = (SAMPLEN+1)*(Clk_ADC/2)
        0x1,    // Set sampling time to 1 adc clock cycle?
        0x2,    // Relative to main clock, have adc clock run 8 times slower
        0x1,    // For averaging more than 2 samples, change RESSEL (0x1 for 16-bit)
        0xF,    // Since reference is 1/2, set gain to 1/2 to keep largest
                // input voltage range (expected input will be 0 - 3.3V)
        0x18,   // Not using the negative for differential, so ground it.
        AIN_PIN // Map the adc to analog pin AIN_PIN
        );
#elif RESOLUTION == 12
    // 12-bit resolution
    configure_adc(
        0x2,    // Select a V_DD_AN/2 (1.65) reference
        0x0,    // Now collect 1 sample at a time.
            // Total sampling time length = (SAMPLEN+1)*(Clk_ADC/2)
        0x1,    // Set sampling time to 1 adc clock cycle?
        0x0,    // Relative to main clock, have adc clock run 4 times slower
        0x1,    // For averaging more than 2 samples, change RESSEL (0x1 for 16-bit)
        0xF,    // Since reference is 1/2, set gain to 1/2 to keep largest
                // input voltage range (expected input will be 0 - 3.3V)
        0x18,   // Not using the negative for differential, so ground it.
        AIN_PIN // Map the adc to analog pin AIN_PIN
        );
#endif
    map_to_adc_odd(ADC_PIN);

    map_to_dac_even(DAC_PIN);
    configure_dac_default();

    configure_adc_interrupt();
    enable_adc_timer();

    configure_display_interrupt();
    enable_display_timer();

    return 0;
}

///////////////////////////////////////////////////////////////////////////////////
//////////////////     Start ADC interrupt implementation     /////////////////////
///////////////////////////////////////////////////////////////////////////////////

void enable_adc_tc_clocks(void){
    PM->APBCMASK.reg |= (1 << 14u);  // TC6 is in the 14th position (see pg 129)
    
    uint32_t temp=0x16;   // ID for TC7 is 0x16  (see table 14-2)
    temp |= 0<<8;         //  Selection Generic clock generator 0
    GCLK->CLKCTRL.reg=temp;   //  Setup in the CLKCTRL register
    GCLK->CLKCTRL.reg |= 0x1u << 14;    // enable it.
}

void enable_adc_timer(void){
    while(adc_timer->STATUS.reg & (1 << 7u));    // Synchronize before proceeding
    adc_timer->CTRLA.reg |= 1 << 1u;    // Re-enable the timer
}

void disable_adc_timer(void){
    adc_timer->CTRLA.reg &= ~(1 << 1u);    // Disable the timer
    while(adc_timer->STATUS.reg & (1 << 7u));    // Synchronize before proceeding
}

void configure_adc_interrupt(void){
    /*
        As a rule, pointers to TcCountN structures will follow
        the convention: timer#S_#B
            - #S refers to TC#S, or a TC set
            - #B refers to the bit-size of the counter (*TC#S)->COUNT#B
    */
    adc_timer = timer6_8; // Use the one of the count structures within the union

    enable_adc_tc_clocks();
    disable_adc_timer();

        // Set up timer 7 settings
        //  Sampling frequency = f_s = freq_tc_clk/Prescale_simple_clk/(Period+1)/Prescale_adc_clk
        //      Let freq_tc_clk = 8Mhz/8
        //      Let Prescale_simple_clk = 1
        //      Let Prescale_adc_clk = 4
        //      Let Period = 249
        //  --> f_s = (8000/8)/1/250/4 kHz = 1 kHz
    adc_timer->CTRLA.reg |=
          (0x1 << 12u)  // Set presynchronizer to prescaled clock
        | (0x3 << 8u)   // Prescale clock by 8
        | (0x1 << 2u)   // Start in 8-bit mode
        | (0x2 << 5u)   // Select the Normal PWM waveform generator
        ;

    adc_timer->PER.reg = 249;
    adc_timer->CC[0].reg = 1;

        // Set up timer 6 interrupt
    NVIC->ISER[0] |= 1 << 19u;
    adc_timer->INTENSET.reg |= 1;
    adc_timer->INTFLAG.reg |= 0x1;
}

void adc_handler(void){
        // Create static storage space
    static UINT32 adc_raw = 0, adc_volt = 0;
    static float x = 0, y = 0, y_prev = 0, x_prev = 0;

#define SAMP_FREQ 1000
#define PI 3.14
#define BW 100
    static const float omega = BW*2*PI/SAMP_FREQ;

    if(adc_timer->INTFLAG.reg & 0x1){
            // Read and convert raw pot value
        adc_raw = read_adc();
            // Output to dac
        x = adc_raw;
        y = (1-omega)*y_prev + omega*x_prev;
        write_to_dac(mapf(
            y,
            0,
#if RESOLUTION == 16
                0xFFFF,
#elif RESOLUTION == 12
                4095,
#endif
            0, 1023
            ));
        y_prev = y;
        x_prev = x;

            // Update display
        adc_volt = map32(adc_raw, 0, 0xFFFF, 0, 3300);
        display_number[0] = adc_volt%10;
        display_number[1] = (adc_volt%100)/10;
        display_number[2] = (adc_volt%1000)/100;
        display_number[3] = (adc_volt%10000)/1000;

        adc_timer->INTFLAG.reg |= 0x1;
    }
}

void TC6_Handler(void){
    adc_handler();
}

///////////////////////////////////////////////////////////////////////////////////
//////////////////     End ADC interrupt implementation     ///////////////////////
///////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
////////////////     Start Display interrupt implementation     ///////////////////
///////////////////////////////////////////////////////////////////////////////////

void enable_display_tc_clocks(void){
    PM->APBCMASK.reg |= (1 << 15u);  // PM_APBCMASK is in the 15 position
    
    uint32_t temp=0x16;   // ID for TC7 is 0x16  (see table 14-2)
    temp |= 0<<8;         //  Selection Generic clock generator 0
    GCLK->CLKCTRL.reg=temp;   //  Setup in the CLKCTRL register
    GCLK->CLKCTRL.reg |= 0x1u << 14;    // enable it.
}

void enable_display_timer(void){
    while(disp_timer->STATUS.reg & (1 << 7u));    // Synchronize before proceeding
    disp_timer->CTRLA.reg |= 1 << 1u;    // Re-enable the timer
}

void disable_display_timer(void){
    disp_timer->CTRLA.reg &= ~(1 << 1u);    // Disable the timer
    while(disp_timer->STATUS.reg & (1 << 7u));    // Synchronize before proceeding
}

    // In a future implementation, possibly write an event-driven system
    // Possibly rewrite for control over brightness
void configure_display_interrupt(void){
    disp_timer = timer7_16; // Use the one of the count structures within the union

    enable_display_tc_clocks();
    disable_display_timer();

        // Set up timer 7 settings
    disp_timer->CTRLA.reg |=
          (0x1 << 12u)  // Set presynchronizer to prescaled clock
        | (0x5 << 8u)   // Prescale clock by 32
        | (0x0 << 2u)   // Start in 16-bit mode
        | (0x1 << 5u)   // Select the Match Frequncy waveform generator
                        //  Allow control over refresh speed and brightness
        ;
    disp_timer->CC[0].reg /*= disp_timer->CC[1].reg*/ = 0x50;

        // Set up timer 7 interrupt
    NVIC->ISER[0] |= 1 << 20u;
    disp_timer->INTENSET.reg |= 1;
    disp_timer->INTFLAG.reg |= 0x1;
}

void display_handler(void){
    static UINT8 dig = 0;
    if(disp_timer->INTFLAG.reg & 0x1){
        display_dig(0, display_number[DISPLAY_DIGIT_SIZE_MAX-1-dig], dig, FALSE__, FALSE__);
        dig = (dig == DISPLAY_DIGIT_SIZE_MAX-1) ? 0 : (dig+1);
    }
}

void TC7_Handler(void){
    display_handler();
        disp_timer->INTFLAG.reg |= 0x1;
}

///////////////////////////////////////////////////////////////////////////////////
/////////////////     End Display interrupt implementation     ////////////////////
///////////////////////////////////////////////////////////////////////////////////
