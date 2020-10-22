/** SerialMidi Class that implements USART  MIDI transmissions and parsing 
 * for parsing pointers to callbacks/delegates need to 
 * be provided at instanciation.  
 *
 *  Created by Jan-Willem Smaal on 21/12/2014.
 *  Ported to C++ on 21/10/2020 for ARM MBED platform   
 *  Copyright (c) 2014 Jan-Willem Smaal. All rights reserved.
 * 
 */
#include "serial-midi.h"
#include "mbed.h"
#include <cstdint>
#include <cstdio>

/**
 * Constructor 
 * Inits the serial USART with MIDI clock speed and 
 * registers delegates for the callbacks of the parser. 
 */
SerialMidi::SerialMidi ( 
	void (*note_on_handler_ptr)(uint8_t note, uint8_t velocity),
	void (*realtime_handler_ptr)(uint8_t msg),
	void (*note_off_handler_ptr)(uint8_t note, uint8_t velocity),
	void (*control_change_handler_ptr)(uint8_t controller, uint8_t value), 
	void (*midi_pitchwheel_ptr)(uint8_t valueLSB, uint8_t valueMSB)
) 
: serial_port(USART_TX, USART_RX, MIDI_BAUD_RATE) 	// Override default constructor
{
    // Assign delegate's
    midi_note_on_delegate 		= note_on_handler_ptr;
    realtime_handler_delegate 	= realtime_handler_ptr;
    midi_note_off_delegate 		= note_off_handler_ptr;
    midi_control_change_delegate = control_change_handler_ptr;
	midi_pitchwheel_delegate 	= midi_pitchwheel_ptr; 
    
    // init the serial-usart system done via Constructor just to be on the safe
	// side we set it explicitly below 
    serial_port.set_baud(MIDI_BAUD_RATE);
    serial_port.set_format(
        /* bits */ 		8,
        /* parity */ 	BufferedSerial::None,
        /* stop bit */ 	1
    );

	// State information that must be kept "static" as MIDI
	// information is processed byte by byte.  
	global_running_status_rx = 0; 
	global_3rd_byte_flag = 0;
	global_midi_c2 = 0;
	global_midi_c3 = 0;
	global_midi_state = RESET; 
}


void SerialMidi::NoteON(uint8_t channel, uint8_t key, uint8_t velocity)
{
	uint8_t buf[4]; 

    buf[0] = C_NOTE_ON | channel;
    buf[1] = key;
    buf[2] = velocity; 

	// Running status especially usefull for fast passages 
	if(global_running_status_tx == buf[0]) {
		serial_port.write(&buf[1], 2);
	} 
	else {
		serial_port.write(buf, 3);
		global_running_status_tx = buf[0];
	}
}


void SerialMidi::NoteOFF(uint8_t channel, uint8_t key, uint8_t velocity)
{
	uint8_t buf[4]; 

    buf[0] = C_NOTE_OFF | channel;
    buf[1] = key;
    buf[2] = velocity; 

	// Running status especially usefull for fast passages 
	if(global_running_status_tx == buf[0]) {
		serial_port.write(&buf[1], 2);
	} 
	else {
		serial_port.write(buf, 3);
		global_running_status_tx = buf[0];
	}
}


void SerialMidi::ControlChange(uint8_t channel, uint8_t controller, uint8_t val)
{
	uint8_t buf[4]; 
	
    buf[0] = C_CONTROL_CHANGE | channel;
    buf[1] = controller;
    buf[2] = val; 

	// Running status especially usefull for smooth control change
	if(global_running_status_tx == buf[0]) {
		serial_port.write(&buf[1], 2);
	} 
	else {
		serial_port.write(buf, 3);
		global_running_status_tx = buf[0];
	}
}


void SerialMidi::ChannelAfterTouch(uint8_t channel, uint8_t val)
{

	uint8_t buf[4]; 
	
    buf[0] = C_CHANNEL_AFTERTOUCH| channel;
    buf[1] = val;
 
	if(global_running_status_tx == buf[0]) {
		serial_port.write(&buf[1], 1);
	} 
	else {
		serial_port.write(buf, 2);
		global_running_status_tx = buf[0];
	}
}


/**
 * Modulation Wheel both LSB and MSB  MIDI sends LSB dirst.  
 * range: 0 --> 16383
 * only a 14 bit value 
 */
void SerialMidi::ModWheel(uint8_t channel, uint16_t val)
{	
    ControlChange(channel, CTL_LSB_MODWHEEL,  ~(CHANNEL_VOICE_MASK) & val);
    ControlChange(channel, CTL_MSB_MODWHEEL,  ~(CHANNEL_VOICE_MASK) & (val>>7));
}

/* 
 * If we only want to send a MSB with a 8 bit value this one will be used 
 */
void SerialMidi::ModWheel(uint8_t channel, uint8_t val)
{	
    ControlChange(channel, CTL_MSB_MODWHEEL,  ~(CHANNEL_VOICE_MASK) & val);
}


/**
 * PitchWheel is always with 14 bit value.
 * this one takes unsigned 14 bit input. 
 *       LOW   MIDDLE   HIGH
 * range: 0 --> 8192  --> 16383
 */
void SerialMidi::PitchWheel(uint8_t channel, uint16_t val)
{
	uint8_t buf[4]; 
	
    buf[0] = C_PITCH_WHEEL | channel;
    buf[1] = val & ~(CHANNEL_VOICE_MASK);
	buf[2] = (val>>7) & ~(CHANNEL_VOICE_MASK);

	// Running status especially usefull for smooth control change
	if(global_running_status_tx == buf[0]) {
		serial_port.write(&buf[1], 2);
	} 
	else {
		serial_port.write(buf, 3);
		global_running_status_tx = buf[0];
	}
}

/**
 * PitchWheel is always with 14 bit value.
 * this takes a signed argument val. 
 *       LOW   MIDDLE   HIGH
 * range: -8192   0     8192 
 */
void SerialMidi::PitchWheel(uint8_t channel, int16_t val)
{
	uint16_t pitch = uint16_t(val + 0x2000);	
	PitchWheel(channel, pitch);
}


inline void SerialMidi::TimingClock(void)
{
	uint8_t c = RT_TIMING_CLOCK;
	serial_port.write(&c,1);
}


inline void SerialMidi::Start(void)
{
	uint8_t c = RT_START;
	serial_port.write(&c,1);
}


inline void SerialMidi::Continue(void)
{
	uint8_t c = RT_CONTINUE;
	serial_port.write(&c,1);
}


inline void SerialMidi::Stop(void)
{
	uint8_t c = RT_STOP;
	serial_port.write(&c,1);
}


inline void SerialMidi::Active_Sensing(void)
{
	uint8_t c = RT_ACTIVE_SENSING;
	serial_port.write(&c,1);
}


inline void SerialMidi::Reset(void)
{
	uint8_t c = RT_RESET;
	serial_port.write(&c,1);
}



char * SerialMidi::Text() 
{
	static char buf[64];
	snprintf(buf, 64,
			"run_tx:%2X,run_rx:%2X,3rd_byte:%2X,state:%2X",  
			global_running_status_tx, 
			global_running_status_rx,
			global_3rd_byte_flag,
			global_midi_state
		);
	return buf; 
}


/**
 * MIDI Parser
 */
void SerialMidi::ReceiveParser(void)
{
	uint8_t c;

    // Read one byte from the circular FIFO input buffer
    // This buffer is filled by the ISR routine on receipt of
    // data on the port.
    if( serial_port.read(&c, 1) == 0) {
		return;
	}
	//printf("%2X ", c);
	// MIDI through (kind of with some processing delay)
	//serial_port.write(&c,1);
    
    // Check if bit7 = 1
    if ( c & CHANNEL_VOICE_MASK ) {
	    // if (! (c & SYSTEM_REALTIME_MASK)) {
		// is it a real-time message?  0xF8 up to 0xFF
        if (c >= 0xF8 ) {
			realtime_handler_delegate(c);
            return;
        }
        else {
            global_running_status_rx = c;
            global_3rd_byte_flag = 0;
            // Is this a tune request
            if(c == SYSTEM_TUNE_REQUEST) {
                global_midi_c2 = c; // Store in FIFO.
                // TODO: Process something.
                return;
            }
            // Do nothing
            // Ignore for now.
            return;
        }
    }
    else {  // Bit 7 == 0   (data)
	    if(global_3rd_byte_flag == 1) {
            global_3rd_byte_flag = 0;
            global_midi_c3 = c;

			// We don't care about the input channel (OMNI) for now.
            global_running_status_rx &= 0xF0;
            if(global_running_status_rx == C_NOTE_ON){
				if(global_midi_c3 == 0 ) {
					// Most MIDI implementation use velocity zero
					// as a note-off.  
					midi_note_off_delegate(global_midi_c2, global_midi_c3);
					return;
				}  
				else {
	            	midi_note_on_delegate(global_midi_c2, global_midi_c3);
					return;
				}
				return;
            }
            else if(global_running_status_rx == C_NOTE_OFF) {
                midi_note_off_delegate(global_midi_c2, global_midi_c3);
                return;
            }
			else if(global_running_status_rx == C_PITCH_WHEEL) {
				midi_pitchwheel_delegate(global_midi_c2, global_midi_c3);
				return; 
			}
			else if(global_running_status_rx == C_PROGRAM_CHANGE) {
				return; 
			}
			else if(global_running_status_rx ==  C_POLYPHONIC_AFTERTOUCH) {
				return; 
			}
			else if(global_running_status_rx ==  C_CHANNEL_AFTERTOUCH) {
				return; 
			}
            else if(global_running_status_rx == C_CONTROL_CHANGE) {
                midi_control_change_delegate(global_midi_c2, global_midi_c3);
                return;
            }
			//else {
			//	return; 
			//	}
        }
        else {
            if(global_running_status_rx == 0) {
                // Ignore data Byte if running status is  0
                return;
            }
            else {
                if (global_running_status_rx < 0xC0) { // All 2 byte commands
                    global_3rd_byte_flag = 1;
                    global_midi_c2 = c;
                    // At this stage we have only 1 byte out of 2.
                    return;
                }
                else if (global_running_status_rx < 0xE0) {    // All 1 byte commands
                    global_midi_c2 = c;
                    // TODO: !! Process callback/delegate for two bytes command.
                    return;
                }
                else if ( global_running_status_rx < 0xF0){
                    global_3rd_byte_flag = 1;
                    global_midi_c2 = c;
                }
				//!!
                else if ( global_running_status_rx >= 0xF0) {
                    if (global_running_status_rx == 0xF2) {
                        global_running_status_rx = 0;
                        global_3rd_byte_flag = 1;
                        global_midi_c2 = c;
                        return;
                    }
                    else if (global_running_status_rx >= 0xF0 ){
                        if(global_running_status_rx == 0xF3 ||
                           global_running_status_rx == 0xF3 ) {
                            global_running_status_rx = 0;
                            global_midi_c2 = c;
                            // TODO: !! Process callback for two bytes command.
                            return;
                        }
                        else {
                            // Ignore status
                            global_running_status_rx = 0;
                            return;
                        }
                    }
                }
            }  
        }  // global_3rd_byte_flag
    } // end of data bit 7 == 0

} // End of SerialMidiReceiveParser


// EOF
