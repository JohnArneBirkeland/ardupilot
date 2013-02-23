// -------------------------------------------------------------
// ArduPPM (PPM Encoder) V2.4.0b Beta
// -------------------------------------------------------------
// Improved servo to ppm for ArduPilot MEGA v1.x (ATmega328p),
// PhoneDrone and APM2.x (ATmega32u2)

// By: John Arne Birkeland - 2012
// APM v1.x adaptation and "difficult" receiver testing by Olivier ADLER
// -------------------------------------------------------------


// -------------------------------------------------------------
// ARDUPPM OPERATIONAL DESCRIPTION
// -------------------------------------------------------------

// APM 2.x LED STATUS:
// -------------------
// RX - OFF                = No input signals detected
// RX - SLOW TOGGLE        = Input signals OK
// RX - FAST TOGGLE        = Invalid input signals detected
// RX - VERY FAST TOGGLE   = Input signals lost (>1sec) and recovered during operation
// RX - ON                 = Input signals lost and channel fail-safe activated

// TX - OFF                = PPM output disabled
// TX - SLOW TOGGLE        = PPM output enabled
// TX - FAST TOGGLE        = PPM pass-trough mode

// SERVO INPUT (PWM) MODE:
// -----------------------
// - PPM output will not be enabled unless a input signal has been detected
// - If inputs are lost during operaton (lose servo wire or receiver malfunction)
//   - The last known value of the lost input channel is keept for ~1 second
//       - If the lost input channel is not restored within ~1 second, it will be set to the default fail-safe value
//        - If a lost channel signal is restored, normal channel operation is resumed.

// PPM PASS-THROUGH MODE (signal pin 2&3 shorted):
// -----------------------------------------------
// - PPM output will not be enabled unless a input signal has been detected
// - If signals on input channel 1 has been detected
//   - Any input level changes will be passed directly to the PPM output (PPM pass-trough)
//   - If no input level changes are detected withing 250ms
//     - PPM output is enabled and default fail-safe values for all eight channels transmitted
//     - If input level changes are detected again, PPM fail-safe output is terminated and normal PPM pass-through operation is resumed

// Changelog:

// 01-08-2011
// V2.2.3 - Changed back to BLOCKING interrupts.
//          Assembly PPM compare interrupt can be switch back to non-blocking, but not recommended.
// V2.2.3 - Implemented 0.5us cut filter to remove servo input capture jitter.

// 04-08-2011
// V2.2.4 - Implemented PPM passtrough funtion.
//          Shorting channel 2&3 enabled ppm passtrough on channel 1.

// 04-08-2011
// V2.2.5 - Implemented simple average filter to smooth servo input capture jitter.
//          Takes fewer clocks to execute and has better performance then cut filter.

// 05-08-2011
// V2.2.51 - Minor bug fixes.

// 06-08-2011
// V2.2.6 - PPM passtrough failsafe implemented.
//          The PPM generator will be activated and output failsafe values while ppm passtrough signal is missing.

// 01-09-2011
// V2.2.61 - Temporary MUX pin always high patch for APM beta board

// 22-09-2011
// V2.2.62 - ATmegaXXU2 USB connection status pin (PC2) for APM UART MUX selection (removed temporary high patch)
//         - Removed assembly optimized PPM generator (not usable for production release)

// 23-09-2011
// V2.2.63 - Average filter disabled

// 24-09-2011
// V2.2.64 - Added distincts Power on / Failsafe PPM values
//         - Changed CH5 (mode selection) PPM Power on and Failsafe values to 1555 (Flight mode 4)
//         - Added brownout detection : Failsafe values are copied after a brownout reset instead of power on values

// 25-09-2011
// V2.2.65 - Implemented PPM output delay until input signal is detected (PWM and PPM pass-trough mode)
//         - Changed brownout detection and FailSafe handling to work with XXU2 chips
//         - Minor variable and define naming changes to enhance readability

// 15-03-2012
// V2.2.66  - Added APM2 (ATmega32U2) support for using TX and RX status leds to indicate PWM and PPM traffic 
//            - <RX>: <OFF> = no pwm input detected, <TOGGLE> = speed of toggle indicate how many channel are active, <ON> = input lost (failsafe)
//            - <TX>: <OFF> = ppm output not started, <FAST TOGGLE> = normal PWM->PPM output or PPM passtrough failsafe, <SLOW TOGGLE> = PPM passtrough

// 03-06-2012
// V2.2.67 - Implemented detection and failsafe (throttle = 900us) for missing throttle signal. 

// 04-06-2012
// V2.2.68 - Fixed possible logic flaw in throttle failsafe reset if  _JITTER_FILTER_ is enabled

// 02-11-2012
// V2.2.69 - Added PPM output positive polarity - mainly for standalone PPM encoder board use

// 03-11-2012
// V2.3.0 - Implemented single channel fail-safe detection for all inputs

// 16-11-2012
// V2.3.1 - Improved watchdog timer reset, so that only valid input signals will prevent the watchdog timer from triggering

// 22-11-2012
// V2.3.11 - Very experimental test forcing throttle fail-safe (RTL) on single channel loss. !DO NOT RELEASE TO PUBLIC!
//         - Test for active input channels during init

// 23-11-2012 !VERY EXPERIMENTAL! 
// V2.3.11 - Nested interrupt PWM output interrupt (PPM generator) for greatly improved jitter performance
//         - Active input channel test during init removed
//         - Implemented dynamic testing of active input channels

// 23-12-2012
// V2.3.12 - New improved fail-safe detection and handeling for single or multible signal loss and receiver malfuntion
//         - Improved LED status for APM 2.x
//         - Improved jitter performance (PPM output using nested interrupts)

// 30-11-2012
// V2.3.13pre - Internal dev only pre-release
//            - Improved active channel detection requering 100 valid pulses before channel is marked active
//            - Removed forced throttle fail-safe after channel loss
//            - Changed fail-safe values to 800us, this will be used by APM code to detect channel loss and decide fail safe actions

// 16-12-2012
// V2.3.13 - Official release
//         - Fail-safe vales changed back to normal fail-safe values. Use until APM code knows how to handle lost channel signal (800us)

// V2.4.0b - Completely new interrupt system to minimize input pin capture latency
//         - PWM input pin changes queued for processing in PPM interrupt
//         - Pin changes, channel loss detection (fail-safe) and LED status processed in the PPM generator interrupt
//         - ppmUpdate() - Faster input pin change processing using unrolled loops and absolute memory addressing for accessing data arrays

// -------------------------------------------------------------

#ifndef _PPM_ENCODER_H_
#define _PPM_ENCODER_H_

#include <avr/io.h>

// -------------------------------------------------------------

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>


// -------------------------------------------------------------
// SERVO INPUT FILTERS AND PARAMETERS
// -------------------------------------------------------------
// Using both filters is not recommended and may reduce servo input resolution
//#define _AVERAGE_FILTER_            // Average filter to smooth servo input capture jitter
//#define _JITTER_FILTER_ 2           // Cut filter to remove servo input capture jitter (1 unit = 1us)
// -------------------------------------------------------------

#ifndef F_CPU
#define F_CPU             16000000UL
#endif

#ifndef true
#define true                1
#endif

#ifndef false
#define false               0
#endif

#ifndef bool
#define bool                _Bool
#endif

// Version stamp for firmware hex file ( decode hex file using <avr-objdump -s file.hex> and look for "ArduPPM" string )
const char ver[15] = "ArduPPMv2.4.0b"; 

// -------------------------------------------------------------
// INPUT MODE
// -------------------------------------------------------------

#define JUMPER_SELECT_MODE    0    // Default - PPM passtrough mode selected if channel 2&3 shorted. Normal servo input (pwm) if not shorted.
#define SERVO_PWM_MODE        1    // Normal 8 channel servo (pwm) input
#define PPM_PASSTROUGH_MODE   2    // PPM signal passtrough on channel 1
#define JETI_MODE             3    // Reserved
#define SPEKTRUM_MODE         4    // Reserved for Spektrum satelitte on channel 1

// Servo input mode (jumper (default), pwm, ppm, jeti or spektrum)
volatile uint8_t servo_input_mode = JUMPER_SELECT_MODE;

// -------------------------------------------------------------
// FAILSAFE MODE
// -------------------------------------------------------------
//#define _APM_FAILSAFE_   // Used to spesify APM 800us channel loss fail safe values, remove to use normal fail safe values (stand alone encoder board)

// -------------------------------------------------------------
// DEBUG
// -------------------------------------------------------------
//#define _INPUT_PROFILER_    // Simple interrupt profiler to messure timing issues (!!always turn off for release builds!!)

// -------------------------------------------------------------
// SERVO LIMIT VALUES
// -------------------------------------------------------------

// Number of Timer1 ticks in one microsecond
#define ONE_US                F_CPU / 8 / 1000 / 1000

// 400us PPM pre pulse
#define PPM_PRE_PULSE         ONE_US * 400

// Servo minimum position
#define PPM_SERVO_MIN         ONE_US * 900 - PPM_PRE_PULSE

// Servo center position
#define PPM_SERVO_CENTER      ONE_US * 1500 - PPM_PRE_PULSE

// Servo maximum position
#define PPM_SERVO_MAX         ONE_US * 2100 - PPM_PRE_PULSE

// Throttle default at power on
#define PPM_THROTTLE_DEFAULT  ONE_US * 1100 - PPM_PRE_PULSE

// Throttle during failsafe
#define PPM_THROTTLE_FAILSAFE ONE_US * 900 - PPM_PRE_PULSE

// Channel loss failsafe
#define PPM_CHANNEL_LOSS ONE_US * 800 - PPM_PRE_PULSE

// CH5 power on values (mode selection channel)
#define PPM_CH5_MODE_4        ONE_US * 1555 - PPM_PRE_PULSE

#ifdef _INPUT_PROFILER_
volatile uint16_t profiler_accumulator;
volatile uint16_t profiler_counter;
volatile uint16_t profiler_time = ONE_US * 1100 - PPM_PRE_PULSE;
#endif

// -------------------------------------------------------------
// PPM OUTPUT SETTINGS
// -------------------------------------------------------------
//#define _POSITIVE_PPM_FRAME_    // Switch to positive pulse PPM
// (the actual timing is encoded in the length of the low between two pulses)

// Number of servo input channels
#define SERVO_CHANNELS        8

// PPM period 18.5ms - 26.5ms (54hz - 37Hz) 
#define PPM_PERIOD            ONE_US * ( 22500 - ( 8 * 1500 ) )

// Size of ppm[..] data array ( servo channels * 2 + 2)
#define PPM_ARRAY_MAX         18

#ifdef _APM_FAILSAFE_
// -------------------------------------------------------------
// APM FAILSAFE VALUES
// -------------------------------------------------------------
const uint16_t failsafe_ppm[ PPM_ARRAY_MAX ] =                               
{
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 1
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 2
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 5
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 6
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 7
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 8
    PPM_PRE_PULSE,
    PPM_PERIOD
};
#else
// -------------------------------------------------------------
// SERVO FAILSAFE VALUES
// -------------------------------------------------------------
const uint16_t failsafe_ppm[ PPM_ARRAY_MAX ] =                               
{
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 1
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 2
    PPM_PRE_PULSE,
    PPM_THROTTLE_FAILSAFE,    // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CH5_MODE_4,           // Channel 5
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 6
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 7
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_PERIOD
};
#endif

// -------------------------------------------------------------
// Data array for storing ppm (8 channels) pulse widths.
// -------------------------------------------------------------
volatile uint16_t ppm[ PPM_ARRAY_MAX ] =                                
{
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 1 
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 2
    PPM_PRE_PULSE,
    PPM_THROTTLE_DEFAULT,     // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CH5_MODE_4,           // Channel 5
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 6
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 7
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_PERIOD
};

// -------------------------------------------------------------
// Input pins change queue 
// Simple ringbuffer hard coded to 64 entries for speed
// Worst case scenario with HS receiver and (7ms, ~140hz) updates during 10ms PPM period pulse is 24 puls edges
// -------------------------------------------------------------
struct
{
    uint16_t time;
    uint8_t  pins;
    bool     used;
} volatile pins_queue[ 64 ];

// !!!Ringbuffer push and pop index's are declared static inside interrupts to optimize execution speed
//volatile uint8_t pins_queue_push = 0; // Pointer to next free data entry in pins_queue
//volatile uint8_t pins_queue_pop = 0; // Pointer to latest data entry in pins_queue

/*
void pins_queue_flush( void )
{
    for( uint8_t i = 0; i < 64; i++ )
    {
        pins_queue[ i ].time = 0;
        pins_queue[ i ].pins = 0;
        pins_queue[ i ].used = false;
    }
}
*/

// -------------------------------------------------------------
// Data arraw for storing ppm timeout (missing channel detection)
// -------------------------------------------------------------
#define SERVO_INPUT_CONNECTED 100
#define SERVO_INPUT_TIMEOUT_VALUE 40 // ~    1sec before triggering missing channel detection
#define SERVO_INPUT_ERROR 25   // Number of invalid input signals allowed before triggering alarm

volatile uint8_t servo_input_counter[ SERVO_CHANNELS ] = { 0, 0, 0, 0, 0, 0, 0, 0 };
volatile uint8_t servo_input_timeout[ SERVO_CHANNELS ] = { 0, 0, 0, 0, 0, 0, 0, 0 };
volatile uint8_t signalErrors = 0;    
volatile uint8_t servo_led_delay = 0;
volatile uint8_t servo_led_reset_delay = 24;

// AVR parameters for PhoneDrone and APM2 boards using ATmega32u2
#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)

#define SERVO_DDR               DDRB
#define SERVO_PORT              PORTB
#define SERVO_INPUT             PINB
#define SERVO_INT_VECTOR        PCINT0_vect
#define SERVO_INT_MASK          PCMSK0
#define SERVO_INT_CLEAR_FLAG    PCIF0
#define SERVO_INT_ENABLE        PCIE0
#define SERVO_TIMER_CNT         TCNT1

#define PPM_DDR                 DDRC
#define PPM_PORT                PORTC
#define PPM_OUTPUT_PIN          PC6
#define PPM_INT_VECTOR          TIMER1_COMPA_vect
#define PPM_COMPARE             OCR1A
#define PPM_COMPARE_FLAG        COM1A0
#define PPM_COMPARE_ENABLE      OCIE1A
#define PPM_COMPARE_FORCE_MATCH FOC1A

#define USB_MUX_DDR             DDRC
#define USB_MUX_PORT            PORTC
#define USB_MUX_PIN             PC2

#define USB_CON_DDR             DDRD    
#define USB_CON_PORT            PORTD
#define USB_CON_PIN             PD0

#define LED_RX_DDR              DDRD
#define LED_RX_PORT             PORTD
#define LED_RX_INPUT            PIND
#define LED_RX_PIN              PD4

#define LED_TX_DDR              DDRD
#define LED_TX_PORT             PORTD
#define LED_TX_INPUT            PIND
#define LED_TX_PIN              PD5

// true if we have received a USB device connect event
static bool usb_connected;

// USB connected event
void EVENT_USB_Device_Connect(void)
{
    // Toggle USB pin high if USB is connected
    USB_MUX_PORT |= (1 << USB_MUX_PIN);

    usb_connected = true;

    // this unsets the pin connected to PA1 on the 2560
    // when the bit is clear, USB is connected
    //PORTD &= ~1;
    USB_CON_PORT &= ~(1 << USB_CON_PIN);
}

// USB disconnect event
void EVENT_USB_Device_Disconnect(void)
{
    // toggle USB pin low if USB is disconnected
    USB_MUX_PORT &= ~(1 << USB_MUX_PIN);

    usb_connected = false;

    // this sets the pin connected to PA1 on the 2560
    // when the bit is clear, USB is connected
    //PORTD |= 1;
    USB_CON_PORT |= (1 << USB_CON_PIN);
}

// AVR parameters for ArduPilot MEGA v1.4 PPM Encoder (ATmega328P)
#elif defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)

#define SERVO_DDR             DDRD
#define SERVO_PORT            PORTD
#define SERVO_INPUT           PIND
#define SERVO_INT_VECTOR      PCINT2_vect
#define SERVO_INT_MASK        PCMSK2
#define SERVO_INT_CLEAR_FLAG  PCIF2
#define SERVO_INT_ENABLE      PCIE2
#define SERVO_TIMER_CNT       TCNT1
    
#define PPM_DDR               DDRB
#define PPM_PORT              PORTB
#define PPM_OUTPUT_PIN        PB2
#define PPM_INT_VECTOR        TIMER1_COMPB_vect
#define PPM_COMPARE           OCR1B
#define PPM_COMPARE_FLAG      COM1B0
#define PPM_COMPARE_ENABLE    OCIE1B
#define PPM_COMPARE_FORCE_MATCH    FOC1B

#define LED_RX_DDR              DDRB
#define LED_RX_PORT             PORTB
#define LED_RX_INPUT            PINB
#define LED_RX_PIN              PB0

#define LED_TX_DDR              DDRB
#define LED_TX_PORT             PORTB
#define LED_TX_INPUT            PINB
#define LED_TX_PIN              PB3

#else
#error NO SUPPORTED DEVICE FOUND! (ATmega16u2 / ATmega32u2 / ATmega328p)
#endif
    
// LED CONTROL
#define LED_RX_OFF              LED_RX_PORT  |=  ( 1 << LED_RX_PIN)
#define LED_RX_ON               LED_RX_PORT  &= ~( 1 << LED_RX_PIN)
#define LED_RX_TOGGLE           LED_RX_INPUT |=  ( 1 << LED_RX_PIN)

#define LED_TX_OFF              LED_TX_PORT  |=  ( 1 << LED_TX_PIN)
#define LED_TX_ON               LED_TX_PORT  &= ~( 1 << LED_TX_PIN)
#define LED_TX_TOGGLE           LED_TX_INPUT |=  ( 1 << LED_TX_PIN)
    
// Used to indicate PPM input signals
volatile bool ppm_input_found = false;

// Used to indicate if PPM generator is active
volatile bool ppm_generator_active = false;

// Used to indicate a brownout restart
volatile bool brownout_reset = false;


// ------------------------------------------------------------------------------
// PPM GENERATOR START - TOGGLE ON COMPARE INTERRUPT ENABLE
// ------------------------------------------------------------------------------
void ppm_start( void )
{
        // Prevent reenabling an already active PPM generator
        if( ppm_generator_active ) return;
        
        // Store interrupt status and register flags
        volatile uint8_t SREG_tmp = SREG;

        // Stop interrupts
        cli();

        // Make sure initial output state is low
        PPM_PORT &= ~(1 << PPM_OUTPUT_PIN);
        
        // Wait for output pin to settle
        //_delay_us( 1 );

        // Set initial compare toggle to maximum (32ms) to give other parts of the system time to start
        SERVO_TIMER_CNT = 0;
        PPM_COMPARE = 0xFFFF;

        // Set toggle on compare output
        TCCR1A = (1 << PPM_COMPARE_FLAG);

        // Set TIMER1 8x prescaler
        TCCR1B = ( 1 << CS11 );
        
        #if defined (_POSITIVE_PPM_FRAME_)
        // Force output compare to reverse polarity
        TCCR1C |= (1 << PPM_COMPARE_FORCE_MATCH);
        #endif

        // Enable output compare interrupt
        TIMSK1 |= (1 << PPM_COMPARE_ENABLE);

        // Indicate that PPM generator is active
        ppm_generator_active = true;

        // Turn on TX led if PPM generator is active
        //LED_TX_ON;

        // Restore interrupt status and register flags
        SREG = SREG_tmp;
}

// ------------------------------------------------------------------------------
// PPM GENERATOR STOP - TOGGLE ON COMPARE INTERRUPT DISABLE
// ------------------------------------------------------------------------------
void ppm_stop( void )
{
        // Store interrupt status and register flags
        volatile uint8_t SREG_tmp = SREG;

        // Stop interrupts
        cli();

        // Disable output compare interrupt
        TIMSK1 &= ~(1 << PPM_COMPARE_ENABLE);

        // Reset TIMER1 registers
        TCCR1A = 0;
        TCCR1B = 0;

        // Indicate that PPM generator is not active
        ppm_generator_active = false;

        // Turn off TX led if PPM generator is off
        //LED_TX_OFF;
        
        // Restore interrupt status and register flags
        SREG = SREG_tmp;
}

// ------------------------------------------------------------------------------
// Watchdog Interrupt (interrupt only mode, no reset)
// ------------------------------------------------------------------------------
ISR( WDT_vect ) // If watchdog is triggered then enable missing signal flag and copy power on or failsafe positions
{
    if( brownout_reset )
    {
        //memcpy( (uint16_t *)ppm, (uint16_t *)failsafe_ppm, PPM_ARRAY_MAX * 2 ); // Copy failsafe values to ppm[..]
        ppm_start(); // Start PPM generator
        brownout_reset = false;
    }

    // If we are in PPM passtrough mode and a input signal has previously been detected
    if( servo_input_mode == PPM_PASSTROUGH_MODE && ppm_input_found )
    {
        //memcpy( (uint16_t *)ppm, (uint16_t *)failsafe_ppm, PPM_ARRAY_MAX * 2 ); // Copy failsafe values to ppm[..]
        ppm_start(); // Start PPM generator
        ppm_input_found = false;

        // Set very fast RX toggle mode to indicate PPM-passtrough with fail-safe values
        servo_led_reset_delay = 6;
        signalErrors = 0;
    }

    // Use failsafe values if PPM generator has been turned on
    if( ppm_generator_active )
    {
        for( uint8_t i = 0; i < PPM_ARRAY_MAX; i++ )
        {
            ppm[ i ] = failsafe_ppm[ i ];
        }
    }
}

// ------------------------------------------------------------------------------
// SERVO/PPM INPUT - PIN CHANGE INTERRUPT
// ------------------------------------------------------------------------------
ISR( SERVO_INT_VECTOR )
{
    // ------------------------------------------------------------------------------    
    // !!! THIS INTERUPT IS TIME SENSITIVE !!!
    // - DO NOT ADD UNNECESSARY CODE UNDER ANY CIRCUMSTANCE, USE THE PPM OUTPUT INTERRUPT INSTEAD
    // ------------------------------------------------------------------------------    
    static uint8_t pins_queue_push = 0; // Pointer to next free data entry in pins_queue (static inside interupt more efficient then global volatile)
    
    #ifdef _INPUT_PROFILER_
    uint16_t _time = SERVO_TIMER_CNT;
    #endif

    // Read current servo pulse change time
    uint16_t pins_time = SERVO_TIMER_CNT;

    // Store current servo input pins
    uint8_t pins = SERVO_INPUT;
    
    // ------------------------------------------------------------------------------
    // PPM passtrough mode ( signal passtrough from channel 1 to ppm output pin)
    // ------------------------------------------------------------------------------
    if( servo_input_mode == PPM_PASSTROUGH_MODE )
    {
        // Has watchdog timer failsafe started PPM generator? If so we need to stop it.
        if( ppm_generator_active ) ppm_stop();
        
        // PPM (channel 1) input pin is high
        if( pins & 1 )
        {
            PPM_PORT |= (1 << PPM_OUTPUT_PIN); // Set PPM output pin high
        }
        // PPM (channel 1) input pin is low
        else 
        {
            PPM_PORT &= ~(1 << PPM_OUTPUT_PIN); // Set PPM output pin low
        }
        
        
        
        // Set servo input missing flag false to indicate that we have received servo input signals
        ppm_input_found = true;

        // Toggle TX LED at PPM passtrough
        if( !servo_led_delay-- ) 
        {
            servo_led_delay = 96; // Fast toggle to indicate PPM pass-trough
            LED_TX_TOGGLE; // Toggle TX led
        }
        
        // Reset Watchdog Timer
        wdt_reset(); 
        
        // Leave interrupt
        return;
    }
        
    // ------------------------------------------------------------------------------
    // SERVO PWM MODE
    // ------------------------------------------------------------------------------

    // Push current time and input pins to free entry in queue
    if( pins_queue[ pins_queue_push ].used )
    {
        // The pins queue is full, this is bad....
        signalErrors++;
    }
    else
    {
        // Push time and pins to entry and mark active (order of struct parameter access is important for generated asm code efficiency)
        pins_queue[ pins_queue_push ].used = true;
        pins_queue[ pins_queue_push ].time = pins_time;
        pins_queue[ pins_queue_push ].pins = pins;
        pins_queue_push++; // Select next push entry
        pins_queue_push &= 63; // Wrap to zero as needed (hardcoded to 64 entries)
    }
    
    // Start PPM generator if not already running
    if( !ppm_generator_active ) ppm_start();

    #ifdef _INPUT_PROFILER_
    profiler_accumulator += SERVO_TIMER_CNT - _time;
    profiler_counter++;

    if( profiler_counter >= (512*10) )
    {
        profiler_time = ONE_US * (( profiler_accumulator / (ONE_US * 512 )) + 1000 ) - PPM_PRE_PULSE;
        profiler_accumulator = 0;
        profiler_counter = 0;
    }
    //ppm[ 15 ] = profiler_time; // Updated in ppmUpdate() instead
    #endif
}

// ------------------------------------------------------------------------------
// PIN CHANGE CHECK
// ------------------------------------------------------------------------------
static inline void ppmUpdate( uint16_t pins_time, uint8_t pins )
{
    #define SERVO_PIN1 1
    #define SERVO_PIN2 2
    #define SERVO_PIN3 4
    #define SERVO_PIN4 8
    #define SERVO_PIN5 16
    #define SERVO_PIN6 32
    #define SERVO_PIN7 64
    #define SERVO_PIN8 128

    #define SERVO_PPM1 1
    #define SERVO_PPM2 3
    #define SERVO_PPM3 5
    #define SERVO_PPM4 7
    #define SERVO_PPM5 9
    #define SERVO_PPM6 11
    #define SERVO_PPM7 13
    #define SERVO_PPM8 15

    // Servo pulse start timing
    static uint16_t pins_start[ SERVO_CHANNELS ] = { 0, 0, 0, 0, 0, 0, 0, 0 };
 
    // Servo input pin storage 
    static uint8_t pins_old = 0;

    // Calculate servo input pin change mask
    uint8_t pins_change = pins ^ pins_old;

    // Store current servo input pins for next check
    pins_old = pins;

    // Unrolled pin check loop for speed
    // ------------------------------------------------------------------------------
    //CHANNEL1:
    if( pins_change & SERVO_PIN1 )
    {
        if( !(pins & SERVO_PIN1) )
        {     // Low pin
            uint16_t servo_width = pins_time - pins_start[ 0 ] - PPM_PRE_PULSE;
            if( servo_width < PPM_SERVO_MIN || servo_width > PPM_SERVO_MAX ) { signalErrors++; goto CHANNEL2; } // Valid servo signal?
            #ifdef _JITTER_FILTER_ // Cut filter to remove input jitter
            int16_t ppm_tmp = ppm[ SERVO_PPM1 ] - servo_width;
            if( ppm_tmp > _JITTER_FILTER_ || ppm_tmp < -_JITTER_FILTER_ ) ppm[ SERVO_PPM1 ] = servo_width; // Update ppm with valid signal
            #else
            ppm[ SERVO_PPM1 ]  = servo_width; // Update ppm with valid signal
            #endif
            servo_input_counter[ 0 ]++;
        }
        else pins_start[ 0 ] = pins_time; // High pin
    }
    // ------------------------------------------------------------------------------
    CHANNEL2:
    if( pins_change & SERVO_PIN2 )
    {
        if( !(pins & SERVO_PIN2) )
        {     // Low pin
            uint16_t servo_width = pins_time - pins_start[ 1 ] - PPM_PRE_PULSE;
            if( servo_width < PPM_SERVO_MIN || servo_width > PPM_SERVO_MAX ) { signalErrors++; goto CHANNEL3; } // Valid servo signal?
            #ifdef _JITTER_FILTER_ // Cut filter to remove input jitter
            int16_t ppm_tmp = ppm[ SERVO_PPM2 ] - servo_width;
            if( ppm_tmp > _JITTER_FILTER_ || ppm_tmp < -_JITTER_FILTER_ ) ppm[ SERVO_PPM2 ] = servo_width; // Update ppm with valid signal
            #else
            ppm[ SERVO_PPM2 ]  = servo_width; // Update ppm with valid signal
            #endif
            servo_input_counter[ 1 ]++;
        }
        else pins_start[ 1 ] = pins_time; // High pin
    }
    // ------------------------------------------------------------------------------
    CHANNEL3:
    if( pins_change & SERVO_PIN3 )
    {
        if( !(pins & SERVO_PIN3) )
        {     // Low pin
            uint16_t servo_width = pins_time - pins_start[ 2 ] - PPM_PRE_PULSE;
            if( servo_width < PPM_SERVO_MIN || servo_width > PPM_SERVO_MAX ) { signalErrors++; goto CHANNEL4; } // Valid servo signal?
            #ifdef _JITTER_FILTER_ // Cut filter to remove input jitter
            int16_t ppm_tmp = ppm[ SERVO_PPM3 ] - servo_width;
            if( ppm_tmp > _JITTER_FILTER_ || ppm_tmp < -_JITTER_FILTER_ ) ppm[ SERVO_PPM3 ] = servo_width; // Update ppm with valid signal
            #else
            ppm[ SERVO_PPM3 ]  = servo_width; // Update ppm with valid signal
            #endif
            servo_input_counter[ 2 ]++;
        }
        else pins_start[ 2 ] = pins_time; // High pin
    }
    // ------------------------------------------------------------------------------
    CHANNEL4:
    if( pins_change & SERVO_PIN4 )
    {
        if( !(pins & SERVO_PIN4) )
        {     // Low pin
            uint16_t servo_width = pins_time - pins_start[ 3 ] - PPM_PRE_PULSE;
            if( servo_width < PPM_SERVO_MIN || servo_width > PPM_SERVO_MAX ) { signalErrors++; goto CHANNEL5; } // Valid servo signal?
            #ifdef _JITTER_FILTER_ // Cut filter to remove input jitter
            int16_t ppm_tmp = ppm[ SERVO_PPM4 ] - servo_width;
            if( ppm_tmp > _JITTER_FILTER_ || ppm_tmp < -_JITTER_FILTER_ ) ppm[ SERVO_PPM4 ] = servo_width; // Update ppm with valid signal
            #else
            ppm[ SERVO_PPM4 ]  = servo_width; // Update ppm with valid signal
            #endif
            servo_input_counter[ 3 ]++;
        }
        else pins_start[ 3 ] = pins_time; // High pin
    }
    // ------------------------------------------------------------------------------
    CHANNEL5:
    if( pins_change & SERVO_PIN5 )
    {
        if( !(pins & SERVO_PIN5) )
        {     // Low pin
            uint16_t servo_width = pins_time - pins_start[ 4 ] - PPM_PRE_PULSE;
            if( servo_width < PPM_SERVO_MIN || servo_width > PPM_SERVO_MAX ) { signalErrors++; goto CHANNEL6; } // Valid servo signal?
            #ifdef _JITTER_FILTER_ // Cut filter to remove input jitter
            int16_t ppm_tmp = ppm[ SERVO_PPM5 ] - servo_width;
            if( ppm_tmp > _JITTER_FILTER_ || ppm_tmp < -_JITTER_FILTER_ ) ppm[ SERVO_PPM5 ] = servo_width; // Update ppm with valid signal
            #else
            ppm[ SERVO_PPM5 ]  = servo_width; // Update ppm with valid signal
            #endif
            servo_input_counter[ 4 ]++;
        }
        else pins_start[ 4 ] = pins_time; // High pin
    }
    // ------------------------------------------------------------------------------
    CHANNEL6:
    if( pins_change & SERVO_PIN6 )
    {
        if( !(pins & SERVO_PIN6) )
        {     // Low pin
            uint16_t servo_width = pins_time - pins_start[ 5 ] - PPM_PRE_PULSE;
            if( servo_width < PPM_SERVO_MIN || servo_width > PPM_SERVO_MAX ) { signalErrors++; goto CHANNEL7; } // Valid servo signal?
            #ifdef _JITTER_FILTER_ // Cut filter to remove input jitter
            int16_t ppm_tmp = ppm[ SERVO_PPM6 ] - servo_width;
            if( ppm_tmp > _JITTER_FILTER_ || ppm_tmp < -_JITTER_FILTER_ ) ppm[ SERVO_PPM6 ] = servo_width; // Update ppm with valid signal
            #else
            ppm[ SERVO_PPM6 ]  = servo_width; // Update ppm with valid signal
            #endif
            servo_input_counter[ 5 ]++;
        }
        else pins_start[ 5 ] = pins_time; // High pin
    }
    // ------------------------------------------------------------------------------
    CHANNEL7:
    if( pins_change & SERVO_PIN7 )
    {
        if( !(pins & SERVO_PIN7) )
        {     // Low pin
            uint16_t servo_width = pins_time - pins_start[ 6 ] - PPM_PRE_PULSE;
            if( servo_width < PPM_SERVO_MIN || servo_width > PPM_SERVO_MAX ) { signalErrors++; goto CHANNEL8; } // Valid servo signal?
            #ifdef _JITTER_FILTER_ // Cut filter to remove input jitter
            int16_t ppm_tmp = ppm[ SERVO_PPM7 ] - servo_width;
            if( ppm_tmp > _JITTER_FILTER_ || ppm_tmp < -_JITTER_FILTER_ ) ppm[ SERVO_PPM7 ] = servo_width; // Update ppm with valid signal
            #else
            ppm[ SERVO_PPM7 ]  = servo_width; // Update ppm with valid signal
            #endif
            servo_input_counter[ 6 ]++;
        }
        else pins_start[ 6 ] = pins_time; // High pin
    }
    // ------------------------------------------------------------------------------
    CHANNEL8:
    if( pins_change & SERVO_PIN8 )
    {
        if( !(pins & SERVO_PIN8) )
        {     // Low pin
            uint16_t servo_width = pins_time - pins_start[ 7 ] - PPM_PRE_PULSE;
            if( servo_width < PPM_SERVO_MIN || servo_width > PPM_SERVO_MAX ) { signalErrors++; goto CHECK_PINS_DONE; } // Valid servo signal?
            #ifdef _JITTER_FILTER_ // Cut filter to remove input jitter
            int16_t ppm_tmp = ppm[ SERVO_PPM8 ] - servo_width;
            if( ppm_tmp > _JITTER_FILTER_ || ppm_tmp < -_JITTER_FILTER_ ) ppm[ SERVO_PPM8 ] = servo_width; // Update ppm with valid signal
            #else
            ppm[ SERVO_PPM8 ]  = servo_width; // Update ppm with valid signal
            #endif
            servo_input_counter[ 7 ]++;
        }
        else pins_start[ 7 ] = pins_time; // High pin
    }

    // Processing done, cleanup and exit
    // ------------------------------------------------------------------------------
    CHECK_PINS_DONE:
    
    #ifdef _INPUT_PROFILER_
    // Quick and dirty method to show input timing as the channel 8 value
    cli();
    ppm[ 15 ] = profiler_time;
    sei();
    #endif

    return;
}

// ------------------------------------------------------------------------------
// PPM OUTPUT - TIMER1 COMPARE INTERRUPT
// ------------------------------------------------------------------------------
ISR( PPM_INT_VECTOR, ISR_NOBLOCK )  
{
    // ------------------------------------------------------------------------------
    // !! NESTED INTERRUPT !!
    // - ACCESSING ANY GLOBAL VARIABLES >8BIT MUST BE DONE USING CLI/SEI
    // - REMEMBER TO USE VOLATILE ON GLOBAL VARIABLES
    // ------------------------------------------------------------------------------   
    static uint8_t ppm_out_channel = PPM_ARRAY_MAX - 1; // Current position in in ppm[..]
    static uint8_t ppm_led_delay; // Used to control the toggle speed of the PPM status led (TX led)
    static uint8_t pins_queue_pop = 0; // Pointer to oldest data entry in pins_queue
    
    // Process all pins_queue entries and updater ppm[..] with results
    while( pins_queue[ pins_queue_pop ].used )
    {
        ppmUpdate( pins_queue[ pins_queue_pop ].time, pins_queue[ pins_queue_pop ].pins ); // Process pin changes and update ppm[..]  as needed ( ringbuffer access is made interrupt safe by checking .used and does not require cli/sei )
        pins_queue[ pins_queue_pop ].used = false; // Release entry
        pins_queue_pop++; // Select next pop entry
        pins_queue_pop &= 63; // // Wrap to zero as needed (hardcoded to 64 entries)
    }    
    
    // Use latest input value 
    // Timer compare register changes must be interrupt safe using cli/sei
    cli();
    PPM_COMPARE += ppm[ ppm_out_channel ];
    sei();
    
    if( ++ppm_out_channel >= PPM_ARRAY_MAX ) 
    {
        // ------------------------------------------------------------------------------
        // We are now in the PPM delay pulse period (~10ms),
        // giving us plenty of time to perform any failsafe or LED status updates
        // !! Execution time must never exceed 10ms !!
        // ------------------------------------------------------------------------------
        
        ppm_out_channel = 0;

        // Check for fail-safe condition if servo input is connected
        for( uint8_t i = 0; i < SERVO_CHANNELS; i++ )
        {
            // Servo channel has received new input (servo_input_counter[..] incremented)
            if( servo_input_counter[ i ] > SERVO_INPUT_CONNECTED ) 
            {
                servo_input_counter[ i ] = SERVO_INPUT_CONNECTED;
                servo_input_timeout[ i ] = 0; // Reset Fail-safe timeout
                wdt_reset(); // Reset Watchdog Timer
            }
            // Servo channel is confirmed with active connection (100 valid inputs),
            // but has not received new input since last iteration
            else if( servo_input_counter[ i ] == SERVO_INPUT_CONNECTED ) 
            {
                if( ++servo_input_timeout[ i ] >= SERVO_INPUT_TIMEOUT_VALUE ) // Fail-safe?
                {
                    servo_input_timeout[ i ] = SERVO_INPUT_TIMEOUT_VALUE;

                    uint8_t ppm_index = ( i << 1 ) + 1;
                    ppm[ ppm_index ] = failsafe_ppm[ ppm_index ]; // Use failsafe value

                    // Set very fast RX toggle mode to indicate fail-safe
                    servo_led_delay = 50; // Set delay ~1sec to prevent toggle during fail-safe (RX LED always on)
                    servo_led_reset_delay = 1; // Very fast toggle
                    signalErrors = 0;

                    // Turn on RX LED to indicate a fail-safe condition
                    LED_RX_ON;
                }
            }
        }        
        
        // Toggle RX LED when receiving servo control signals
        if( !servo_led_delay-- )
        {
            // Status LED mode
            if( signalErrors >= SERVO_INPUT_ERROR ) 
            {
                signalErrors = 0;
                servo_led_reset_delay = 6; // Fast RX toggle to indicate input signal errors
            }

            servo_led_delay = servo_led_reset_delay;
            LED_RX_TOGGLE; // Toggle RX led
        }

        // Toggle TX LED to inducate PPM status
        if( !ppm_led_delay-- )
        {
            ppm_led_delay = 24; // Toggle every 24th PPM pulse train (slow toggle)
            LED_TX_TOGGLE;
        }
    }
}
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
// PPM READ - INTERRUPT SAFE PPM SERVO CHANNEL READ
// ------------------------------------------------------------------------------
uint16_t ppm_read_channel( uint8_t channel )
{
    // Limit channel to valid value
    uint8_t _channel = channel;
    if( _channel == 0 ) _channel = 1;
    if( _channel > SERVO_CHANNELS ) _channel = SERVO_CHANNELS;

    // Calculate ppm[..] position
    uint8_t ppm_index = ( _channel << 1 ) + 1;
    
    // Read ppm[..] in a non blocking interrupt safe manner
    uint16_t ppm_tmp = ppm[ ppm_index ];
    while( ppm_tmp != ppm[ ppm_index ] ) ppm_tmp = ppm[ ppm_index ];

    // Return as normal servo value
    return ppm_tmp + PPM_PRE_PULSE;    
}
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
// PPM ENCODER INIT
// ------------------------------------------------------------------------------
void ppm_encoder_init( void )
{
    // ------------------------------------------------------------------------------    
    // Reset Source checkings
    // ------------------------------------------------------------------------------
    if( MCUSR & 1 )    // Power-on Reset
    {
        // custom code here
    }
    else if( MCUSR & 2 )    // External Reset
    {
       // custom code here
    }
    else if( MCUSR & 4 )    // Brown-Out Reset
    {
       brownout_reset = true;
    }
    else    // Watchdog Reset
    {
       // custom code here
    }
    MCUSR = 0; // Clear MCU Status register

    // ATmegaXXU2 only init code
    // ------------------------------------------------------------------------------    
    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
        // APM USB connection status and UART MUX selector pin
        // Start with inital USB status set as disconnected
        // ------------------------------------------------------------------------------
        USB_MUX_DDR |= (1 << USB_MUX_PIN);  // Set USB/UART mux control pin as output
        USB_MUX_PORT &= ~(1 << USB_MUX_PIN); // Select UART
        
        // On 32U2 set PD0 to be an output, and clear the bit. This tells
        // the 2560 that USB is connected. The USB connection event fires
        // later to set the right value
        USB_CON_DDR |= (1 << USB_CON_PIN); // USB status pin as output
        USB_CON_PORT |= (1 << USB_CON_PIN); // USB disconnected
    #endif

    // USE JUMPER TO CHECK FOR PWM OR PPM PASSTROUGH MODE (channel 2&3 shorted)
    // ------------------------------------------------------------------------------
    if( servo_input_mode == JUMPER_SELECT_MODE )
    {
        // channel 3 status counter
        uint8_t channel3_status = 0;
        // Set channel 3 to input
        SERVO_DDR &= ~(1 << 2);
        // Enable channel 3 pullup
        SERVO_PORT |= (1 << 2);
        // Set channel 2 to output
        SERVO_DDR |= (1 << 1);
        // Set channel 2 output low
        SERVO_PORT &= ~(1 << 1);
        // Wait a bit
        _delay_us( 10 );
        // Increment channel3_status if channel 3 is set low by channel 2
        if( ( SERVO_INPUT & (1 << 2) ) == 0 ) channel3_status++;
        // Set channel 2 output high
        SERVO_PORT |= (1 << 1);
        // Wait a bit
        _delay_us( 10 );
        // Increment channel3_status if channel 3 is set high by channel 2
        if( ( SERVO_INPUT & (1 << 2) ) != 0 ) channel3_status++;
        // Set channel 2 output low
        SERVO_PORT &= ~(1 << 1);
        // Wait a bit
        _delay_us( 10 );
        // Increment channel3_status if channel 3 is set low by channel 2
        if( ( SERVO_INPUT & (1 << 2) ) == 0 ) channel3_status++;
        // Set servo input mode based on channel3_status
        if( channel3_status == 3 ) servo_input_mode = PPM_PASSTROUGH_MODE;
        else servo_input_mode = SERVO_PWM_MODE;
    }

    // STATUS LEDS
    // ------------------------------------------------------------------------------
    LED_RX_DDR |= ( 1 << LED_RX_PIN ); // RX LED OUTPUT
    LED_TX_DDR |= ( 1 << LED_TX_PIN ); // TX LED OUTPUT
    LED_RX_PORT |= ( 1 << LED_RX_PIN ); // RX LED OFF
    LED_TX_PORT |= ( 1 << LED_TX_PIN ); // TX LED OFF
    
    // SERVO/PPM INPUT PINS
    // ------------------------------------------------------------------------------
    // Set all servo input pins to inputs
    SERVO_DDR = 0;

    // Activate pullups on all input pins
    SERVO_PORT |= 0xFF;

    // PPM OUTPUT
    // ------------------------------------------------------------------------------
    // PPM generator (PWM output timer/counter) is started either by pin change interrupt or by watchdog interrupt
    // Set PPM pin to output
    PPM_DDR |= (1 << PPM_OUTPUT_PIN);

     // PPM PASS-THROUGH MODE
    if( servo_input_mode == PPM_PASSTROUGH_MODE )
    {
        // Set servo input interrupt pin mask to servo input channel 1
        SERVO_INT_MASK = 0x01;
    }
    
    // SERVO PWM INPUT MODE
    // ------------------------------------------------------------------------------
    if( servo_input_mode == SERVO_PWM_MODE )
    {
        // Interrupt on all input pins
        SERVO_INT_MASK = 0xFF;
    }
    
    // Enable servo input interrupt
    PCICR |= (1 << SERVO_INT_ENABLE);
    
    // ------------------------------------------------------------------------------
    // Enable watchdog interrupt mode
    // ------------------------------------------------------------------------------
    // Disable watchdog
    wdt_disable();
     // Reset watchdog timer
    wdt_reset();
     // Start timed watchdog setup sequence
    WDTCSR |= (1<<WDCE) | (1<<WDE );
    // Set 250 ms watchdog timeout and enable interrupt
    WDTCSR = (1<<WDIE) | (1<<WDP2);
    
    //_delay_ms( 100 );
    //ppm_start();
}
// ------------------------------------------------------------------------------

#endif // _PPM_ENCODER_H_

