/** SerialMidi class 
 * Used for traditional USART MIDI communication at 31250 BAUD 
 * not suitable MIDI-USB.   This MIDI implementations 
 * follows the MIDI specification strictly and employs running_status 
 * reducing data transmissions considerably.     
 * 
 * ported from C avr to C++ on ARM mbed platform 
 *
 *  Example:
 *  @code
 * include "mbed.h"
 * include "serial-midi.h"
 *   
 *  SerialMidi serialMidiGlob(
 *		&midi_note_on_handler,
 *		&realtime_handler,
 *		&midi_note_off_handler,
 *		&midi_control_change_handler,
 *		&midi_pitchwheel_handler
 * ); 
 *
 *
 * void midi_note_on_handler(uint8_t note, uint8_t velocity) {
 *	printf("midi_note_on_handler(%2X, %2X)\n", note, velocity);
 *	return; 
 *}
 *
 *int main(void) {
 *    while (true) {
 *		serialMidiGlob.ReceiveParser();
 *	} 
 * } 
 * @endcode
 *
 *  Created by Jan-Willem Smaal on 21/12/14.
 *  ported to C++ on 20 Oct 2020 
 *  APACHE 2.0 license - Copyright (c) 2014 Jan-Willem Smaal. 
 */
#ifndef _SERIAL_USART_MIDI
#define _SERIAL_USART_MIDI

// MBED OS Includes
#include <cstdint>
#include <stdint.h>
#include "MK64F12.h"
#include "mbed.h"

// Hardware specific defines 
// TODO: adjust for your board! 
#define USART_TX PTC17
#define USART_RX PTC16 




/*-----------------------------------------------------------------------*/
// Defines

/* 440 Hz for the A4 note */
#define BASE_A4_NOTE 440
#define MIDI_BAUD_RATE 31250

/* MIDI channel/mode masks */
#define CHANNEL_VOICE_MASK      0x80    //  Bit 7 == 1
#define CHANNEL_MODE_MASK       0xB0
#define SYSTEM_EXCLUSIVE_MASK   0xF0
#define SYSTEM_REALTIME_MASK    0xF8
#define SYSTEM_COMMON_MASK      0xF0
#define MIDI_DATA               0x7F    //  Bit 7 == 0


/* System exclusive */
#define SYSTEM_EXCLUSIVE_START  0xF0
#define SYSTEM_TUNE_REQUEST     0xF6
#define SYSTEM_EXCLUSIVE_END    0xF7


/* MIDI channel commands */
#define C_NOTE_ON               0x90    // 2 bytes
#define C_NOTE_OFF              0x80    // 2 bytes
#define C_POLYPHONIC_AFTERTOUCH 0xA0    // 2 bytes
#define C_PITCH_WHEEL           0xE0    // 2 bytes
#define C_CONTROL_CHANGE        0xB0    // 2 bytes
#define C_PROGRAM_CHANGE        0xC0    // 1 byte
#define C_CHANNEL_AFTERTOUCH    0xD0    // 1 byte

// This one is not implemented i.e. ignored. 
#define ACTIVE_SENSE            0xFE    

/* System Real Time commands */
#define RT_TIMING_CLOCK         0xF8
#define RT_START                0xFA
#define RT_CONTINUE             0xFB
#define RT_STOP                 0xFC
#define RT_ACTIVE_SENSING       0xFE
#define RT_RESET                0xFF


/*-----------------------------------------------------------------------*/
//  Prototypes

class SerialMidi {
public: 
	/**  During the SerialMidiInit the delegate callback functions need to be assigned
	 */
	SerialMidi( 
		void (*note_on_handler_ptr)(	uint8_t note, uint8_t velocity),
		void (*realtime_handler_ptr)(	uint8_t msg),
		void (*note_off_handler_ptr)(	uint8_t note, uint8_t velocity),
		void (*control_change_handler_ptr)(uint8_t controller, uint8_t value),
		void (*midi_pitchwheel_ptr)(	uint8_t valueLSB, uint8_t valueMSB)
	);

	void ReceiveParser(void);
	//void SerialMidiReceiveParser2(void);

	char * Text(); // Text Representation of the Class status  

	// Channel mode messages
	void NoteON( 	uint8_t channel, uint8_t key, uint8_t velocity);
	void NoteOFF(	uint8_t channel, uint8_t key, uint8_t velocity);
	void ControlChange(uint8_t channel, uint8_t controller, uint8_t val);
	void PitchWheel(uint8_t channel, uint16_t val);
	void PitchWheel(uint8_t channel, int16_t val);	
	void ModWheel(	uint8_t channel, uint16_t val);
	void ModWheel(	uint8_t channel, uint8_t val); // Only MSB sent 8 bits
	void ModWheel(	uint8_t channel, int val) { 
		SerialMidi::ModWheel(channel,(uint8_t)val); 
	}
	void ChannelAfterTouch(uint8_t channel, uint8_t val);

	// System Common messages
	void TimingClock(void);
	void Start(void);
	void Continue(void);
	void Stop(void);
	void Active_Sensing(void);
	void Reset(void);

	enum midi_state_machine {
    	RESET,
    	RX_1_SYSEX_BYTE,
    	HANDLE_SYSEX,
    	DISPATCH,       // Stores MIDI running status
    	RX_DATA_BYTE,   // 1 byte MIDI messages
    	RX_1_DATA_BYTE, // 2 byte MIDI messages (e.g. pitch bend)
    	RX_2_DATA_BYTE
	};

	enum midi_control_change {
		CTL_MSB_BANK             = 0x00,  // Bank Selection
		CTL_MSB_MODWHEEL         = 0x01,  // Modulation
		CTL_MSB_BREATH           = 0x02,  // Breath
		CTL_MSB_FOOT             = 0x04,  // Foot
		CTL_MSB_PORTAMENTO_TIME  = 0x05,  // Portamento Time
		CTL_MSB_DATA_ENTRY       = 0x06,  // Data Entry
		CTL_MSB_MAIN_VOLUME      = 0x07,  // Main Volume
		CTL_MSB_BALANCE          = 0x08,  // Balance
		CTL_MSB_PAN              = 0x0A,  // Panpot
		CTL_MSB_EXPRESSION       = 0x0B,  // Expression
		CTL_MSB_EFFECT1          = 0x0C,  // Effect1
		CTL_MSB_EFFECT2          = 0x0D,  // Effect2
		CTL_MSB_GENERAL_PURPOSE1 = 0x10,  // General Purpose 1
		CTL_MSB_GENERAL_PURPOSE2 = 0x11,  // General Purpose 2
		CTL_MSB_GENERAL_PURPOSE3 = 0x12,  // General Purpose 3
		CTL_MSB_GENERAL_PURPOSE4 = 0x13,  // General Purpose 4
		CTL_LSB_BANK             = 0x20,  // Bank Selection
		CTL_LSB_MODWHEEL         = 0x21,  // Modulation
		CTL_LSB_BREATH           = 0x22,  // Breath
		CTL_LSB_FOOT             = 0x24,  // Foot
		CTL_LSB_PORTAMENTO_TIME  = 0x25,  // Portamento Time
		CTL_LSB_DATA_ENTRY       = 0x26,  // Data Entry
		CTL_LSB_MAIN_VOLUME      = 0x27,  // Main Volume
		CTL_LSB_BALANCE          = 0x28,  // Balance
		CTL_LSB_PAN              = 0x2A,  // Panpot
		CTL_LSB_EXPRESSION       = 0x2B,  // Expression
		CTL_LSB_EFFECT1          = 0x2C,  // Effect1
		CTL_LSB_EFFECT2          = 0x2D,  // Effect2
		CTL_LSB_GENERAL_PURPOSE1 = 0x30,  // General Purpose 1
		CTL_LSB_GENERAL_PURPOSE2 = 0x31,  // General Purpose 2
		CTL_LSB_GENERAL_PURPOSE3 = 0x32,  // General Purpose 3
		CTL_LSB_GENERAL_PURPOSE4 = 0x33,  // General Purpose 4
		CTL_SUSTAIN              = 0x40,  // Sustain Pedal
		CTL_PORTAMENTO           = 0x41,  // Portamento
		CTL_SOSTENUTO            = 0x42,  // Sostenuto
		CTL_SOFT_PEDAL           = 0x43,  // Soft Pedal
		CTL_LEGATO_FOOTSWITCH    = 0x44,  // Legato Foot Switch
		CTL_HOLD2                = 0x45,  // Hold2
		CTL_SC1_SOUND_VARIATION  = 0x46,  // SC1 Sound Variation
		CTL_SC2_TIMBRE           = 0x47,  // SC2 Timbre
		CTL_SC3_RELEASE_TIME     = 0x48,  // SC3 Release Time
		CTL_SC4_ATTACK_TIME      = 0x49,  // SC4 Attack Time
		CTL_SC5_BRIGHTNESS       = 0x4A,  // SC5 Brightness
		CTL_SC6                  = 0x4B,  // SC6
		CTL_SC7                  = 0x4C,  // SC7
		CTL_SC8                  = 0x4D,  // SC8
		CTL_SC9                  = 0x4E,  // SC9
		CTL_SC10                 = 0x4F,  // SC10
		CTL_GENERAL_PURPOSE5     = 0x50,  // General Purpose 5
		CTL_GENERAL_PURPOSE6     = 0x51,  // General Purpose 6
		CTL_GENERAL_PURPOSE7     = 0x52,  // General Purpose 7
		CTL_GENERAL_PURPOSE8     = 0x53,  // General Purpose 8
		CTL_PORTAMENTO_CONTROL   = 0x54,  // Portamento Control
		CTL_E1_REVERB_DEPTH      = 0x5B,  // E1 Reverb Depth
		CTL_E2_TREMOLO_DEPTH     = 0x5C,  // E2 Tremolo Depth
		CTL_E3_CHORUS_DEPTH      = 0x5D,  // E3 Chorus Depth
		CTL_E4_DETUNE_DEPTH      = 0x5E,  // E4 Detune Depth
		CTL_E5_PHASER_DEPTH      = 0x5F,  // E5 Phaser Depth
		CTL_DATA_INCREMENT       = 0x60,  // Data Increment
		CTL_DATA_DECREMENT       = 0x61,  // Data Decrement
		CTL_NRPN_LSB             = 0x62,  // Non-registered Parameter Number
		CTL_NRPN_MSB             = 0x63,  // Non-registered Parameter Number
		CTL_RPN_LSB              = 0x64,  // Registered Parameter Number
		CTL_RPN_MSB              = 0x65,  // Registered Parameter Number
		CTL_ALL_SOUNDS_OFF       = 0x78,  // All Sounds Off
		CTL_RESET_CONTROLLERS    = 0x79,  // Reset Controllers
		CTL_LOCAL_CONTROL_SWITCH = 0x7A,  // Local Control Switch
		CTL_ALL_NOTES_OFF        = 0x7B,  // All Notes Off
		CTL_OMNI_OFF             = 0x7C,  // Omni Off
		CTL_OMNI_ON  		     = 0x7D,  // Omni On
		CTL_MONO1                = 0x7E,  // Mono1
		CTL_MONO2                = 0x7F   // Mono2
};

	// Midi channel 1 == 0x00 
	// Cause for frequent confusion so better to use the defines.  
	enum channel {
			CH1   = 0,
			CH2   = 1,
			CH3   = 2,
			CH4   = 3,
			CH5   = 4,
			CH6   = 5,
			CH7   = 6,
			CH8   = 7,
			CH9   = 8,
			CH10  = 9,
			CH11  = 10,
			CH12  = 11,
			CH13  = 12,
			CH14  = 13,
			CH15  = 14,
			CH16  = 15,
	};

/* ------------------------------------------------------------- */
private: 
	void (*midi_note_on_delegate)(uint8_t note, uint8_t velocity);
	void (*realtime_handler_delegate)(uint8_t msg);
    void (*midi_note_off_delegate)(uint8_t note, uint8_t velocity);
	void (*midi_control_change_delegate)(uint8_t controller, uint8_t value);
	void (*midi_pitchwheel_delegate)(uint8_t valueLSB, uint8_t valueMSB); 
	
	BufferedSerial serial_port;

	/** Required to be able to process MIDI data.
	 *  while keeping running state. 
 	 */
	uint8_t global_running_status_tx;
	uint8_t global_running_status_rx;
	uint8_t global_3rd_byte_flag;
	uint8_t global_midi_c2;
	uint8_t global_midi_c3;
	uint8_t global_midi_state;
};

#endif /* _SERIAL_USART_MIDI */
