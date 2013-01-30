// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// ArduPPM Version v1.0.0 Beta
// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// ARDUCOPTER 2 : PPM ENCODER for AT Mega 328p and APM v1.4 Boards
// By:John Arne Birkeland - 2011
//
//  By: Olivier ADLER - 2011 - APM v1.4 adaptation and testing
//
// Compiled with Atmel AVR Studio 4.0 / AVR GCC
// ------------------------------------------------------------------------------------------------------------------------------------------------------------

// 		Changelog		//
//
// Code based on John Arne PPM v1 encoder. Mux / Led / Failsafe control from Olivier ADLER.
// Adaptation to APM v1.4 / ATMEGA 328p by Olivier ADLER, with great code base, help  and advices from John Arne.
//
//	0.9.0 -> 0.9.4 : experimental versions. Not publicly available. Jitter problems, good reliability.
// 	
//	New PPM code base V2 from John Arne designed for 32u2 AVRs
//
//	0.9.5 : first reliable and jitter free version based on new John PPM V2 code and Olivier interrupt nesting idea.
//	0.9.6 : enhanced jitter free version with non bloking servo interrupt and ultra fast ppm generator interrupt(John's ideas)
//	0.9.7 : mux (passthrough mode) switchover reliability enhancements and error reporting improvements.
//	0.9.75 : implemented ppm_encoder.h library with support for both atmega328p and atmega32u2 chips
//	0.9.76 : timers 0 and 2 replaced by a delayed loop for simplicity. Timer 0 and 2 are now free for use.
//	              reworked error detection with settable time window, errors threshold and Led delay
//	0.9.77 : Implemented ppm_encoder.h into latest version.
//	0.9.78 : Implemented optimzed assembly compare interrupt
//	0.9.79 : Removed Non Blocking attribute for servo input interrupt
//	0.9.80 : Removed non blocking for compare interrupt, added optionnal jitter filter and optionnal non blocking attribute for assembly version of compare interrupt
//	0.9.81 : Added PPM PASSTROUGH Mode and LED Codes function to report special modes
//	0.9.82 : LED codes function simplification
//	0.9.83 : Implemented PPM passtrough failsafe
//	0.9.84 : Corrected pin and port names in Encoder-PPM.c according to #define for Mega32U2 compatibility
//	0.9.85 : Added brownout reset detection flag
//	0.9.86 : Added a #define to disable Radio Passthrough mode (hardware failsafe for Arduplane)
//	0.9.87 : #define correction for radio passthrough (was screwed up).
//  0.9.88 : LED fail-safe indication is on whenever throttle is low
//  0.9.89 : LED fail-safe change can be reverted with a define
//  0.9.90 : Small improvements in library
// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// PREPROCESSOR DIRECTIVES
// ------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "../Libraries/PPM_Encoder.h"

#define PASSTHROUGH_MODE_ENABLED	// Comment this line to remove CH8 radio passthrough mode support (hardware failsafe for Arduplane)

// ------------------------------------------------------------------------------------------------------------------------------------------------------------
// PPM ENCODER INIT AND AUXILIARY TASKS
// ------------------------------------------------------------------------------------------------------------------------------------------------------------
int main(void)
{
	ppm_encoder_init(); // Enable PPM encoder

	DDRB |= ( 1 << PB1 ); // Set MUX pin (PB1) to output
	PORTB |= ( 1 << PB1 );	// Set PB1 output HIGH to disable Radio passthrough (mux)

	sei();			// Enable Global interrupt flag
	
	// ------------------------------------------------------------------------------
	// Disable radio passthrough (mux chip A/B control)
	// ------------------------------------------------------------------------------
    
    while( 1 )
    {
		#ifdef PASSTHROUGH_MODE_ENABLED
		// ------------------------------------------------------------------------------
		// Radio passthrough control (mux chip A/B control)
		// ------------------------------------------------------------------------------
        #define PASSTHROUGH_CHANNEL		8 	// Channel for passthrough mode selection
        #define PASSTHROUGH_CHANNEL_OFF_US		ONE_US * 1600 - PPM_PRE_PULSE	// Passthrough off threshold
        #define PASSTHROUGH_CHANNEL_ON_US		ONE_US * 1800 - PPM_PRE_PULSE	// Passthrough on threshold
        #define LOOP_TIMER_10MS	10			// Loop timer ticks for 10 ms duration
        
        uint16_t mux_ppm = ppm_read_channel( PASSTHROUGH_CHANNEL - 1 ); // Safely read passthrough channel ppm position

        if ( mux_ppm < ( PASSTHROUGH_CHANNEL_OFF_US ) )	// Check ppm value and update validation counter
        {
            PPM_PORT |= ( 1 << PB1 );	// Set PIN B1 (Mux) to disable Radio passthrough

        }
        else if ( mux_ppm > ( PASSTHROUGH_CHANNEL_ON_US ) )
        {
            PPM_PORT &= ~( 1 << PB1 );	// Reset PIN B1 (Mux) to enable Radio passthrough
        }
		
		#endif
    };
    
} // main function end


