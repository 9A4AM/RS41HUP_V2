//
// Created by SQ5RWU on 2016-12-24.
// Modified by VK5QI in 2018-ish
//

#ifndef RS41HUP_CONFIG_H
#define RS41HUP_CONFIG_H

#ifdef USE_EXTERNAL_CONFIG
#include "config_external.h"
#else


//************GLOBAL Settings*****************
// This is the main frequency.  To send alternating with a second frequency, enable the following line
#define TRANSMIT_FREQUENCY  434.7140f //Mhz middle frequency
//#define TRANSMIT_FREQUENCY_2ND  437.6000f //Mhz middle frequency

#define BAUD_RATE  100 // RTTY & MFSK Baud rate.
                       // NOTE: Currently supported MFSK baud rates with horus-gui are 50 and 100 baud,
                       // with the suggested MFSK baud rate being 100 baud.

// Modulation Types - Comment out a line below to enable/disable a modulation.
//#define RTTY_ENABLED 1
#define MFSK_4_ENABLED 1

//*************RTTY SETTINGS******************
#define CALLSIGN "URCALL" // put your RTTY callsign here, max. 15 characters
#define RTTY_DEVIATION 0x3	// RTTY shift = RTTY_DEVIATION x 270Hz
#define RTTY_7BIT   1 // if 0 --> 5 bits
#define RTTY_USE_2_STOP_BITS   1

//************MFSK Binary Settings************
// Binary Payload ID (0 though 255) - For your own flights, you will need to request a payload ID,
// and set this value to that. 
// Refer to the payload ID list here: https://github.com/projecthorus/horusdemodlib/blob/master/payload_id_list.txt
// Payload IDs can be reqested by either raising an Issue, or a Pull Request on https://github.com/projecthorus/horusdemodlib/
// VERSION 1 = An ID of 0 ('4FSKTEST') can be used for on-ground testing, but DO NOT use this for a flight!!!
// VERSION 2 = An ID of 256 ('4FSKTEST-V2') can be used for on-ground testing, but DO NOT use this for a flight!!!

//#define HORUS_V1
#define HORUS_V2

#ifdef HORUS_V1
  #define BINARY_PAYLOAD_ID 0 // Payload ID for use in Binary packets
#endif
// ...or... (dont activate both)
#ifdef HORUS_V2
  #define BINARY_PAYLOAD_ID 256 //  Payload ID for use in Binary packets
#endif

#if defined(HORUS_V1) && defined(HORUS_V2)
#error "Please select only one HORUS-Mode."
#endif


// TX Power
#define TX_POWER  5 // PWR 0...7 0- MIN ... 7 - MAX
// Power Levels measured at 434.650 MHz, using a Rigol DSA815, and a 10 kHz RBW
// Power measured by connecting a short (30cm) length of RG316 directly to the
// antenna/ground pads at the bottom of the RS41 PCB.

// 0 --> -1.9dBm
// 1 --> 1.3dBm
// 2 --> 3.6dBm
// 3 --> 7.0dBm
// 4 --> 11.0dBm
// 5 --> 13.1dBm
// 6 --> 15.0dBm
// 7 --> 16.3dBm

// *********** Power Saving ******************
//
// Power Consumption Notes (all @ 3V):
// - GPS Max Performance, Transmitting @ 13 dBm = ~150 mA
// - GPS Max Performance, Not Transmitting = 70-90mA
// - GPS in PowerSave Mode, Transmitting @ 13 dBm = ~120 mA
// - GPS in PowerSave Mode, not Transmitting = 30-50mA, depending on GPS state.
// If enabled, transmit incrementing tones in the 'idle' period between packets.
// This will only function if ONLY MFSK is enabled.
// Note, you need to COMMENT this line out to disable this, not just set it to 0
//#define CONTINUOUS_MODE 1

// If anyway GPS Fix gets lost a counter increments each TX-Intervall
// if counter reaches NOGPS_RESET_AFTER_TXCOUNT then an SystemRestart will be fired
// Timeout can be calculated :  TX_DELAY * NOGPS_RESET_AFTER_TXCOUNT
// After this, the TX Counter starts again at nbr 1
// This works at start of the Sonde, also during flight if something disturbs GPS reception (jamming over military areas)
// DISABLE:  comment out this line //
// Recomandation:  request reboot after 7 Minutes: = 420 Seconds * 1000 (ms) / TX_DELAY (ms) - round to integer please
#define NOGPS_RESET_AFTER_TXCOUNT 7

// Delay *between* transmitted packets (milliseconds)
// If you only have MFSK_4 enabled, and MFSK_CONTINUOUS (below) is disabled,
// Then the transmitter will turn off between transmissions. This saves about 50mA of power consumption.
// The maximum TX_DELAY is 65535*(1000/BAUD_RATE), so about 655.35 seconds for 100 baud
#define TX_DELAY  60000
// Try to sync the TX to start on full minute if GPSfix is available.
// Disable: insert "//" before
// The value behind is the offset after the minute. VALUE IS NOT IN USE AT THIS POINT
#define SYNC_TX_WITH_GPS 1

// If defined, transmit a short 20ms 'pip' between transmissions every X milliseconds.
// This number needs to be smaller than TX_DELAY
// Comment out the line to disable this.
//#define TX_PIP  5000

// Number of symbols to transmit the pip for. 
// At 100 baud, each symbol is 10ms
#define TX_PIP_SYMBOLS  5

// Enable uBlox PowerSave Mode
// Drops current consummption from the GPS somewhat.
// Positional accuracy may be slightly impacted. Suggest not using this for short flights.
// Flight-tested on 2020-12.
// if "1" then the sat-counter may be start with 100 oder 200 values. If 2xx reached, this needs half enery then in full mode
#define UBLOX_POWERSAVE 1

// *********** Morse Ident **********************
// If uncommented, send a morse code ident every X transmit cycles, to comply
// with amateur radio regulations, if operating under an amateur radio license.
// With continuous 4FSK transmissions, 100 transmit cycles is approx 5 minutes.
//#define MORSE_IDENT 10

// Morse message to send. 
#define MORSE_MESSAGE "DE Callsign... your text"

// Speed of morse transmission
#define MORSE_WPM 25


// *********** Deep Sleep Mode ****(not tested yet by whallmann) **************
// Deep Sleep Mode intended for long duration flights only!
//
// Notes:
// - Only have MFSK transmissions enabled.
// - Disable continuous mode.
// - Disable GPS powersave mode. 
//
// Power consumption in sleep mode = 32mA @ 3V
//
// In this mode, the GPS is turned into a sleep mode in between transmissions.
// During this sleep period, we sent one 'pip' every few seconds.
// At the end of the sleep period, the GPS is powered back up, and we await the GPS
// to obtain a fix before transmitting our position. While waiting for GPS lock, we
// send a 'double pip'.

// Go to sleep for X minutes between transmissions.
// In this time, the GPS will be completely powered down.
// If defined, sleep for this many minutes between transmissions.
//#define DEEP_SLEEP 2

// Send a short pip every X milliseconds to let people know the transmitter is running.
#define DEEP_SLEEP_PIPS 10000



//***********Other Settings ******************
// Switch sonde ON/OFF via Button
// If this is a flight you might prevent sonde from powered off by button
#define ALLOW_DISABLE_BY_BUTTON 1


#endif

#endif //RS41HUP_CONFIG_H
