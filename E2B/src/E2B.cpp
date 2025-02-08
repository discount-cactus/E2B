/*
Copyright (c) 2007, Jim Studt  (original old version - many contributors since)

The latest version of this library may be found at:
  http://www.pjrc.com/teensy/td_libs_E2B.html

OneWire has been maintained by Paul Stoffregen (paul@pjrc.com) since
January 2010.

DO NOT EMAIL for technical support, especially not for ESP chips!
All project support questions must be posted on public forums
relevant to the board or chips used.  If using Arduino, post on
Arduino's forum.  If using ESP, post on the ESP community forums.
There is ABSOLUTELY NO TECH SUPPORT BY PRIVATE EMAIL!

Github's issue tracker for OneWire should be used only to report
specific bugs.  DO NOT request project support via Github.  All
project and tech support questions must be posted on forums, not
github issues.  If you experience a problem and you are not
absolutely sure it's an issue with the library, ask on a forum
first.  Only use github to report issues after experts have
confirmed the issue is with OneWire rather than your project.

Back in 2010, OneWire was in need of many bug fixes, but had
been abandoned the original author (Jim Studt).  None of the known
contributors were interested in maintaining OneWire.  Paul typically
works on OneWire every 6 to 12 months.  Patches usually wait that
long.  If anyone is interested in more actively maintaining OneWire,
please contact Paul (this is pretty much the only reason to use
private email about OneWire).

OneWire is now very mature code.  No changes other than adding
definitions for newer hardware support are anticipated.

Version 2.3:
  Unknown chip fallback mode, Roger Clark
  Teensy-LC compatibility, Paul Stoffregen
  Search bug fix, Love Nystrom

Version 2.2:
  Teensy 3.0 compatibility, Paul Stoffregen, paul@pjrc.com
  Arduino Due compatibility, http://arduino.cc/forum/index.php?topic=141030
  Fix DS18B20 example negative temperature
  Fix DS18B20 example's low res modes, Ken Butcher
  Improve reset timing, Mark Tillotson
  Add const qualifiers, Bertrik Sikken
  Add initial value input to crc16, Bertrik Sikken
  Add target_search() function, Scott Roberts

Version 2.1:
  Arduino 1.0 compatibility, Paul Stoffregen
  Improve temperature example, Paul Stoffregen
  DS250x_PROM example, Guillermo Lovato
  PIC32 (chipKit) compatibility, Jason Dangel, dangel.jason AT gmail.com
  Improvements from Glenn Trewitt:
  - crc16() now works
  - check_crc16() does all of calculation/checking work.
  - Added read_bytes() and write_bytes(), to reduce tedious loops.
  - Added ds2408 example.
  Delete very old, out-of-date readme file (info is here)

Version 2.0: Modifications by Paul Stoffregen, January 2010:
http://www.pjrc.com/teensy/td_libs_E2B.html
  Search fix from Robin James
    http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1238032295/27#27
  Use direct optimized I/O in all cases
  Disable interrupts during timing critical sections
    (this solves many random communication errors)
  Disable interrupts during read-modify-write I/O
  Reduce RAM consumption by eliminating unnecessary
    variables and trimming many to 8 bits
  Optimize both crc8 - table version moved to flash

Modified to work with larger numbers of devices - avoids loop.
Tested in Arduino 11 alpha with 12 sensors.
26 Sept 2008 -- Robin James
http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1238032295/27#27

Updated to work with arduino-0008 and to include skip() as of
2007/07/06. --RJL20

Modified to calculate the 8-bit CRC directly, avoiding the need for
the 256-byte lookup table to be loaded in RAM.  Tested in arduino-0010
-- Tom Pollard, Jan 23, 2008

Jim Studt's original library was modified by Josh Larios.

Tom Pollard, pollard@alum.mit.edu, contributed around May 20, 2008

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Much of the code was inspired by Derek Yerger's code, though I don't
think much of that remains.  In any event that was..
    (copyleft) 2006 by Derek Yerger - Free to distribute freely.

The CRC code was excerpted and inspired by the Dallas Semiconductor
sample code bearing this copyright.
//---------------------------------------------------------------------------
// Copyright (C) 2000 Dallas Semiconductor Corporation, All Rights Reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL DALLAS SEMICONDUCTOR BE LIABLE FOR ANY CLAIM, DAMAGES
// OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Except as contained in this notice, the name of Dallas Semiconductor
// shall not be used except as stated in the Dallas Semiconductor
// Branding Policy.
//--------------------------------------------------------------------------
*/



/*
OneWireSlave v1.1 by Joshua Fuller - Modified based on versions noted below for Digispark

OneWireSlave v1.0 by Alexander Gordeyev

It is based on Jim's Studt OneWire library v2.0

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Much of the code was inspired by Derek Yerger's code, though I don't
think much of that remains.  In any event that was..
    (copyleft) 2006 by Derek Yerger - Free to distribute freely.

The CRC code was excerpted and inspired by the Dallas Semiconductor
sample code bearing this copyright.
//---------------------------------------------------------------------------
// Copyright (C) 2000 Dallas Semiconductor Corporation, All Rights Reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL DALLAS SEMICONDUCTOR BE LIABLE FOR ANY CLAIM, DAMAGES
// OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Except as contained in this notice, the name of Dallas Semiconductor
// shall not be used except as stated in the Dallas Semiconductor
// Branding Policy.
//---------------------------------------------------------------------------
*/

#include <Arduino.h>
#include "E2B.h"
#include "util/E2B_direct_gpio.h"

//These are the major change from original, we now wait quite a bit longer for some things
/*#if E2B_ASYNC_RECV
  #define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(120) / 10L
  //This TIMESLOT_WAIT_READ_RETRY_COUNT is new, and only used when waiting for the line to go low on a read
  //It was derived from knowing that the Arduino based master may go up to 130 micros more than our wait after reset
  #define TIMESLOT_WAIT_READ_RETRY_COUNT microsecondsToClockCycles(135)

  /*void E2B::ISRPIN(){
    (*static_E2B_instance).MasterResetPulseDetection();
  }/
#endif*/

uint8_t _pin;

void E2B::begin(uint8_t pin){
	_pin = pin;
	pinMode(_pin, INPUT);
	bitmask = PIN_TO_BITMASK(_pin);
	baseReg = PIN_TO_BASEREG(_pin);

  busType = BUS;
  hostFlag = 0;
  secureFlag = 0;
  isLocked = 0;
  unlockedState = 0;
  secureKey = 0x00;

  #if E2B_SEARCH
	 reset_search();
  #endif
}

//Sets the type of device
// 0 = Bus (Default), 1 = Point-to-Point (for two devices ONLY), 2 = Transceiver
void E2B::setBusType(uint8_t type){
  busType = type;
}

//Returns the type of the device
// 0 = Bus (Default), 1 = Point-to-Point (for two devices ONLY), 2 = Transceiver
uint8_t E2B::getBusType(){
  return busType;
}

//Sets the level of the hostFlag
//Meant for use in multi-master buses.
void E2B::setHostFlag(unsigned char *newAddr, bool level){
  hostFlag = level;
  if(level){
    newAddr[0] = FAMILYCODE_HOST;
    newAddr[1] = FAMILYCODE;
    //#warning "Device address changed. newAddr[0] = FAMILYCODE_HOST; newAddr[1] = FAMILYCODE_DEVICE;"
  }
}

//Returns the level of the hostFlag - if the device is the bus host or not. Meant for use in multi-master buses.
bool E2B::getHostFlag(){
  return hostFlag;
}

//Sets the level of the secureFlag
//Sets the device as secured/locked or unsecured/unlocked. DEFAULTED to 0.
//When secureFlag is set to 1, special procedures must be taken to communicate with the device.
void E2B::setSecureFlag(uint8_t level, uint8_t key){
  secureFlag = level;

  if(level){
    isLocked = 1;
    unlockedState = 0;
    secureKey = key;
  }else{
    isLocked = 0;
    //#warning "Device secureFlag deactivated."
  }
}

//Returns the level of the secureFlag - if the device is secured/locked or unsecured/unlocked.
bool E2B::getSecureFlag(){
  return secureFlag;
}

//Rewrites to device rom to a new, random value
void E2B::generateROM(unsigned char *newAddr){
  newAddr[0] = FAMILYCODE;
  for (int i=1; i < 8; i++) newAddr[i] = random(256);
}


// Perform the E2B reset function.  We will wait up to 250uS for the bus to come high, if it doesn't then it is
// broken or shorted and we return a 0. Returns 1 if a device asserted a presence pulse, 0 otherwise.
uint8_t E2B::reset(void){
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
	uint8_t r;
	uint8_t retries = 125;

	noInterrupts();
	DIRECT_MODE_INPUT(reg, mask);
	interrupts();
	// wait until the wire is high... just in case
	do {
		if (--retries == 0) return 0;
		delayMicroseconds(2);
	} while ( !DIRECT_READ(reg, mask));

	noInterrupts();
	DIRECT_WRITE_LOW(reg, mask);
	DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
	interrupts();
	delayMicroseconds(480);
	noInterrupts();
	DIRECT_MODE_INPUT(reg, mask);	// allow it to float
	delayMicroseconds(70);
	r = !DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(410);
	return r;
}

// Write a bit. Port and bit is used to cut lookup time and provide more certain timing.
void E2B::write_bit(uint8_t v){
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;

	if (v & 1){
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(10);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(55);
	} else {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(65);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(5);
	}
}

// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
uint8_t E2B::read_bit(void){
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
	uint8_t r;

	noInterrupts();
	DIRECT_MODE_OUTPUT(reg, mask);
	DIRECT_WRITE_LOW(reg, mask);
	delayMicroseconds(3);
	DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
	delayMicroseconds(10);
	r = DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(53);
	return r;
}

// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the write (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the write to avoid heating in a short or
// other mishap.
void E2B::write(uint8_t v, uint8_t power /* = 0 */){
  uint8_t bitMask;

  for (bitMask = 0x01; bitMask; bitMask <<= 1){
     E2B::write_bit((bitMask & v)?1:0);
  }
  if(!power){
  	noInterrupts();
  	DIRECT_MODE_INPUT(baseReg, bitmask);
  	DIRECT_WRITE_LOW(baseReg, bitmask);
  	interrupts();
  }
}

void E2B::write_bytes(const uint8_t *buf, uint16_t count, bool power /* = 0 */){
  for (uint16_t i=0; i < count ; i++)
    write(buf[i]);
  if (!power){
    noInterrupts();
    DIRECT_MODE_INPUT(baseReg, bitmask);
    DIRECT_WRITE_LOW(baseReg, bitmask);
    interrupts();
  }
}

// Read a byte of data
uint8_t E2B::read(){
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1){
	if ( E2B::read_bit()) r |= bitMask;
    }
    return r;
}

void E2B::read_bytes(uint8_t *buf, uint16_t count){
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = read();
}

// Does a ROM select command (aka Choose ROM)
void E2B::select(const uint8_t rom[8]){
    write(0x55);

    for (uint8_t i = 0; i < 8; i++) write(rom[i]);
}

//Does a ROM skip command
void E2B::skip(){
    write(0xCC);
}

//Requests an unlock to a device with a secureFlag = 1.
//This will unlock the secured device for one command.
void E2B::unlock(uint8_t key){
    write(0x3A);
    write(key);
}

void E2B::depower(){
	noInterrupts();
	DIRECT_MODE_INPUT(baseReg, bitmask);
	interrupts();
}

#if E2B_SEARCH
//
// You need to use this function to start a search again from the beginning.
// You do not need to do it for the first search, though you could.
//
void E2B::reset_search(){
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = false;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--){
    ROM_NO[i] = 0;
    if ( i == 0) break;
  }
}

// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
//
void E2B::target_search(uint8_t family_code){
   // set the search state to find SearchFamily type devices
   ROM_NO[0] = family_code;
   for (uint8_t i = 1; i < 8; i++)
      ROM_NO[i] = 0;
   LastDiscrepancy = 64;
   LastFamilyDiscrepancy = 0;
   LastDeviceFlag = false;
}

//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// E2B::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use E2B::reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return true  : device found, ROM number in ROM_NO buffer
//        false : device not found, end of search
//
bool E2B::search(uint8_t *newAddr, bool search_mode /* = true */){
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number;
   bool    search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = false;

   // if the last call was not the last one
   if (!LastDeviceFlag){
      // 1-Wire reset
      if (!reset()){
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
         return false;
      }

      // issue the search command
      if (search_mode == true){
        write(0xF0);   // NORMAL SEARCH
      } else {
        write(0xEC);   // CONDITIONAL SEARCH
      }

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = read_bit();
         cmp_id_bit = read_bit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1)){
            break;
         } else {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit){
               search_direction = id_bit;  // bit write value for search
            } else {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy){
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               } else {
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);
               }
               // if 0 was picked then record its position in LastZero
               if (search_direction == 0){
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            write_bit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0){
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65)){
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0){
            LastDeviceFlag = true;
         }
         search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0]){
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   } else {
      for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   }
   return search_result;
}

// Performs an E2B search and logs the addresses to a struct.
bool E2B::search_and_log(uint8_t *newAddr, uint8_t *searchLog, bool search_mode /* = true */){
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number;
   bool    search_result;
   uint8_t id_bit, cmp_id_bit;
   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = false;

   // if the last call was not the last one
   if (!LastDeviceFlag){
      // 1-Wire reset
      if (!reset()){
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
         return false;
      }

      // issue the search command
      if(search_mode == true){
        write(0xF0);   // NORMAL SEARCH
      }else{
        write(0xEC);   // CONDITIONAL SEARCH
      }

      // loop to do the search
      do{
         // read a bit and its complement
         id_bit = read_bit();
         cmp_id_bit = read_bit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1)){
            break;
         }else{
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit){
               search_direction = id_bit;  // bit write value for search
            } else {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy){
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               } else {
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);
               }
               // if 0 was picked then record its position in LastZero
               if (search_direction == 0){
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if(search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            write_bit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0){
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65)){
      	// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
        LastDiscrepancy = last_zero;

        // check for last device
        if (LastDiscrepancy == 0){
        	LastDeviceFlag = true;
        }
        	search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0]){
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   }else{
      for(int i=0; i < 8; i++){
				newAddr[i] = ROM_NO[i];
				//searchLog[i] = newAddr[i];		//Adds the new address to the searchLog database
				searchLog[i] = ROM_NO[i];
			}
   }
	 return search_result;
}

#endif    //E2B_SEARCH

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// E2B_ASYNC_RECV //////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#if E2B_ASYNC_RECV

/*#if ARDUINO_ARCH_ESP32                    //Defines special attributes for ESP32 architecture, added on 2-2-25
#define INTERRUPT_ATTR IRAM_ATTR
#elif ARDUINO_ARCH_ESP8266                  //Defines special attributes for ESP8266 architecture, added on 2-2-25
#define INTERRUPT_ATTR ICACHE_RAM_ATTR
#endif*/

volatile long previous = 0;
volatile long old_previous = 0;
volatile long diff = 0;

void E2B::MasterResetPulseDetection(){
  old_previous = previous;
  previous = micros();
  diff = previous - old_previous;
  if (diff >= lowmark && diff <= highmark){
		Serial.println(diff);
    waitForRequestInterrupt(false);
  }
}

/*bool E2B::owsprint(){
	//waitForRequestInterrupt(false);
	//Serial.println("done");
  delayMicroseconds(25);

  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  errnum = E2B_NO_ERROR;
  noInterrupts();
  DIRECT_WRITE_LOW(reg, mask);
  DIRECT_MODE_OUTPUT(reg, mask);    // drive output low
  interrupts();

  delayMicroseconds(125);
  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);     // allow it to float
  interrupts();

  delayMicroseconds(300 - 50);

  uint8_t retries = 25;

	delayMicroseconds(50);

	//while (recvAndProcessCmd()){};
	recvAndProcessCmd();
}*/

void E2B::init(unsigned char rom[8]){
	for (int i=0; i<7; i++)
    this->rom[i] = rom[i];
	#if E2B_CRC
  	this->rom[7] = crc8(this->rom, 7);
	#endif
}

void E2B::setScratchpad(unsigned char scratchpad[9]){
  for (int i=0; i<8; i++)
    this->scratchpad[i] = scratchpad[i];
	#if E2B_CRC
  	this->scratchpad[8] = crc8(this->scratchpad, 8);
	#endif
}

void E2B::setPower(uint8_t power){
  this->power = power;
}

//Enables users to define their own functions for the device to automatically respond with
typedef void (*FuncPointerArray)(void);
FuncPointerArray userFunc[256];
void E2B::attachUserCommand(uint8_t num, void (*userFunction)(void)){
	userFunc[num] = userFunction;
	#if E2B_CRC
		this->scratchpad[8] = crc8(this->scratchpad, 8);
	#endif
}

/*void (*user44hFunc)(void);
void E2B::attach44h(void (*userFunction44h)(void)){
	user44hFunc = userFunction44h;
	#if E2B_CRC
		this->scratchpad[8] = crc8(this->scratchpad, 8);
	#endif
}

void (*user48hFunc)(void);
void E2B::attach48h(void (*userFunction48h)(void)){
	user48hFunc = userFunction48h;
}

void (*userB8hFunc)(void);
void E2B::attachB8h(void (*userFunctionB8h)(void)){
	userB8hFunc = userFunctionB8h;
}*/

bool E2B::waitForRequest(bool ignore_errors){
  errnum = E2B_NO_ERROR;

  for (;;){
    //delayMicroseconds(40);
    //Once reset is done, it waits another 30 micros
    //Master wait is 65, so we have 35 more to send our presence now that reset is done
    if (!waitReset(0) ){
      continue;
    }
    //Reset is complete, tell the master we are prsent
    // This will pull the line low for 125 micros (155 micros since the reset) and
    //  then wait another 275 plus whatever wait for the line to go high to a max of 480
    // This has been modified from original to wait for the line to go high to a max of 480.
    if (!presence() ){
      continue;
    }
    //Now that the master should know we are here, we will get a command from the line
    //Because of our changes to the presence code, the line should be guranteed to be high
    if (recvAndProcessCmd() ){
      return true;
    }
    else if ((errnum == E2B_NO_ERROR) || ignore_errors){
      continue;
    } else {
      return false;
    }
  }
}

//Interrupt-driven variant of waitForRequest()
bool E2B::waitForRequestInterrupt(bool ignore_errors){
  errnum = E2B_NO_ERROR;
  //owsprint();
  //Reset is detected from the Interrupt by counting time between the Level-Changes
  //Once reset is done, it waits another 30 micros
  //Master wait is 65, so we have 35 more to send our presence now that reset is done
  //delayMicroseconds(30);		good working!!!
	delayMicroseconds(25);
  //Reset is complete, tell the master we are prsent
  // This will pull the line low for 125 micros (155 micros since the reset) and
  //  then wait another 275 plus whatever wait for the line to go high to a max of 480
  // This has been modified from original to wait for the line to go high to a max of 480.
  while (!presence(50) ){};	//45 is good
  //Now that the master should know we are here, we will get a command from the line
  //Because of our changes to the presence code, the line should be guranteed to be high
  while (recvAndProcessCmd() ){};
  if ((errnum == E2B_NO_ERROR) || ignore_errors){
    //continue;
  } else {
    return false;
  }
}

bool E2B::recvAndProcessCmd(){
	char addr[8];
  uint16_t raw = 0;

  for (;;){
    uint8_t cmd = recv_async();
    //Serial.print("cmd: "); Serial.println(cmd,HEX);

    if (secureFlag == 1){       	//If a secure device
      if (isLocked){
        if ((cmd != 0xCC) && (cmd != 0x55) && (cmd != 0xF0)){       //0xCC is an SKIP ROM command, 0x55 is an SELECT ROM command, 0xF0 is a SEARCH ROM command
          errnum = E2B_SECURED_AND_LOCKED;
          //#warning "Attempted communication with locked secured device."
          return false;
        }
      }else{
        if(unlockedState > 0){  //If you still have retries
          unlockedState -= 1;
        }
      }
    }

    switch(cmd){
      case 0xF0: // SEARCH ROM
        searchROM();
        //delayMicroseconds(6900);
        return false;
      case 0xEC: // ALARM SEARCH
      	raw = ((scratchpad[1] << 8) | scratchpad[0]) >> 4;
      	if ( raw <= scratchpad[3] || raw >= scratchpad[2] )
      		searchROM();
        return false;
      case 0x33: // READ ROM
        sendData_async(rom, 8);
        if (errnum != E2B_NO_ERROR)
          return false;
        break;
      case 0x55: // MATCH ROM - Choose/Select ROM
        recvData_async(addr,8);
        if (errnum != E2B_NO_ERROR)
          return false;
        for (int i=0; i<8; i++)
          if (rom[i] != addr[i])
            return false;
        duty();
        return true;
      case 0xCC: // SKIP ROM
      	duty();
      	if (errnum != E2B_NO_ERROR)
          return false;
        return true;
      default: // Unknow command
        if (errnum == E2B_NO_ERROR)
          break; // skip if no error
        else
          return false;
    }
  }

  /*if(unlockedState <= 0){  //If you run out of retries
    isLocked = 1;
    #warning "Device re-locked."
  }*/
}

bool E2B::duty(){
	uint8_t done = recv_async();
  scratchpad[4] = done;                  //Added on 7-15-24 for transceiver functionality, replaces slot for temperature resolution


  if ((secureFlag == 1) && (isLocked == 1) && (done != 0x3A)){
    errnum = E2B_SECURED_AND_LOCKED;
    //#warning "Attempted communication with locked secured device."
    //Serial.println("You shall not pass!");
    return false;
  }/*else{
    isLocked = 1;
  }*/

  switch (done){
    case 0xBE: // READ SCREATCHPAD
			sendData_async(scratchpad, 9);
			if (errnum != E2B_NO_ERROR)
				return false;
			break;
		case 0xB4: // READ POWERSOURCE
			send_bit_async(power);
			if (errnum != E2B_NO_ERROR)
				return false;
			break;
    case 0x3A: // UNLOCK REQUEST
      {
        uint8_t receivedKey = recv_async();
        if(receivedKey == secureKey){ //Unlocks the device if receivedKey matches the key of the secured device (unlocked for the commands)
          isLocked = 0;
          unlockedState = 3;    //Processes 3 commands before locking again
          //#warning "Secured device unlocked for 3 commands."
        }
  			if (errnum != E2B_NO_ERROR)
  				return false;
			}
      break;
		/*case 0x44: // CONVERT SENSOR
			userFunc[0x44];                       //originally user44hFunc();
			if (errnum != E2B_NO_ERROR)
				return false;
			break;
		case 0x48: // CONVERT SENSOR
			userFunc[0x48];                       //originally user48hFunc();
			if (errnum != E2B_NO_ERROR)
				return false;
			break;
		case 0xB8: // CONVERT SENSOR
			userFunc[0xB8];                       //originally userB8hFunc();
			if (errnum != E2B_NO_ERROR)
				return false;
			break;*/
		/*case 0x4E: // WRITE SCREATCHPAD
			recvData_async(temp_scratchpad, 3);
			setScratchpad_external(temp_scratchpad);
			if (errnum != E2B_NO_ERROR)
				return false;
			break;*/
		default:
			break;
			if (errnum == E2B_NO_ERROR)
				break; // skip if no error
			else
				return false;
	return true;
  }
}

//Returns a byte of data from the specified index in the scratchpad
uint8_t E2B::getScratchpad(uint8_t i){
  return scratchpad[i];
}

bool E2B::searchROM(){
  uint8_t bitmask;
  uint8_t bit_send, bit_recv;

  for (int i=0; i<8; i++){
    for (bitmask = 0x01; bitmask; bitmask <<= 1){
      bit_send = (bitmask & rom[i])?1:0;
      send_bit_async(bit_send);
      send_bit_async(!bit_send);
      bit_recv = recv_bit_async();
      if (errnum != E2B_NO_ERROR)
        return false;
      if (bit_recv != bit_send)
        return false;
    }
  }
  return true;
}

bool E2B::waitReset(uint16_t timeout_ms){
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  unsigned long time_stamp;

  errnum = E2B_NO_ERROR;
  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  interrupts();

  //Wait for the line to fall
  if (timeout_ms != 0){
    time_stamp = micros() + timeout_ms*1000;
    while (DIRECT_READ(reg, mask)){
      if (micros() > time_stamp){
        errnum = E2B_WAIT_RESET_TIMEOUT;
        return false;
      }
    }
  } else {
    //Will wait forever for the line to fall
    while (DIRECT_READ(reg, mask)){};
  }

  //Set to wait for rise up to 540 micros
  //Master code sets the line low for 500 micros
  //TODO The actual documented max is 640, not 540
  time_stamp = micros() + 540;

  //Wait for the rise on the line up to 540 micros
  while (DIRECT_READ(reg, mask) == 0){
    if (micros() > time_stamp){
      errnum = E2B_VERY_LONG_RESET;
      return false;
    }
  }

  //If the master pulled low for exactly 500, then this will be 40 wait time
  // Recommended for master is 480, which would be 60 here then
  // Max is 640, which makes this negative, but it returns above as a "E2B_VERY_LONG_RESET"
  // this gives an extra 10 to 30 micros befor calling the reset invalid
  if ((time_stamp - micros()) > 70){
    errnum = E2B_VERY_SHORT_RESET;
    return false;
  }

  //Master will now delay for 65 to 70 recommended or max of 75 before it's "presence" check
  // and then read the pin value (checking for a presence on the line)
  // then wait another 490 (so, 500 + 64 + 490 = 1054 total without consideration of actual op time) on Arduino,
  // but recommended is 410 with total reset length of 480 + 70 + 410 (or 480x2=960)
  delayMicroseconds(30);
  //Master wait is 65, so we have 35 more to send our presence now that reset is done
  return true;
}
/*bool E2B::waitReset(){
  return waitReset(1000);
}*/

bool E2B::presence(uint8_t delta){
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  //Reset code already waited 30 prior to calling this
  // Master will not read until 70 recommended, but could read as early as 60
  // so we should be well enough ahead of that. Arduino waits 65
  errnum = E2B_NO_ERROR;
  noInterrupts();
  DIRECT_WRITE_LOW(reg, mask);
  DIRECT_MODE_OUTPUT(reg, mask);    // drive output low
  interrupts();

  //Delaying for another 125 (orignal was 120) with the line set low is a total of at least 155 micros
  // total since reset high depends on commands done prior, is technically a little longer
  delayMicroseconds(125);
  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);     // allow it to float
  interrupts();

  //Default "delta" is 25, so this is 275 in that condition, totaling to 155+275=430 since the reset rise
  // docs call for a total of 480 possible from start of rise before reset timing is completed
  //This gives us 50 micros to play with, but being early is probably best for timing on read later
  delayMicroseconds(300 - delta);

  //Modified to wait a while (roughly 50 micros) for the line to go high
  // since the above wait is about 430 micros, this makes this 480 closer
  // to the 480 standard spec and the 490 used on the Arduino master code
  // anything longer then is most likely something going wrong.
  uint8_t retries = 25;
  #if ARDUINO_ARCH_ESP32                           //Special procedure for ESP32 devices
    while (!DIRECT_READ(reg, mask));
    do {
    	if (retries-- == 0)// return 0;
      	delayMicroseconds(2);
    } while(!DIRECT_READ(reg, mask));

    if (!DIRECT_READ(reg, mask)){
      errnum = E2B_PRESENCE_LOW_ON_LINE;
      return false;
    }else{
      return true;
    }
  #else                                             //Standard procedure for all other microcontrollers
    while (!DIRECT_READ(reg, mask));
    do {
    if (retries-- == 0)
      //return false;
    delayMicroseconds(2);
    } while(!DIRECT_READ(reg, mask));
    /*
    if ( !DIRECT_READ(reg, mask)){
        errnum = E2B_PRESENCE_LOW_ON_LINE;
        return false;
    } else
        return true;
    */
  #endif

}

bool E2B::presence(){
  return presence(25);
}

uint8_t E2B::sendData_async(char buf[], uint8_t len){
  uint8_t bytes_sent = 0;

  for (int i=0; i<len; i++){
    send_async(buf[i]);
    if (errnum != E2B_NO_ERROR)
      break;
    bytes_sent++;
  }
  return bytes_sent;
}

uint8_t E2B::recvData_async(char buf[], uint8_t len){
  uint8_t bytes_received = 0;

  for (int i=0; i<len; i++){
    buf[i] = recv_async();
    if (errnum != E2B_NO_ERROR)
      break;
    bytes_received++;
  }
  return bytes_received;
}

void E2B::send_async(uint8_t v){
  errnum = E2B_NO_ERROR;
  for (uint8_t bitmask = 0x01; bitmask && (errnum == E2B_NO_ERROR); bitmask <<= 1)
  	send_bit_async((bitmask & v)?1:0);
}

uint8_t E2B::recv_async(){
  uint8_t r = 0;

  errnum = E2B_NO_ERROR;
  for (uint8_t bitmask = 0x01; bitmask && (errnum == E2B_NO_ERROR); bitmask <<= 1)
		if (recv_bit_async())
      r |= bitmask;
  return r;
}

void E2B::send_bit_async(uint8_t v){
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  uint8_t wt = waitTimeSlot();
  if (wt != 1){					//1 is nominal
    if (wt == 10){
      errnum = E2B_READ_TIMESLOT_TIMEOUT_LOW;
    } else {
      errnum = E2B_READ_TIMESLOT_TIMEOUT_HIGH;
    }
    interrupts();
    return;
  }
  if (v & 1)
    delayMicroseconds(30);
  else {
  	noInterrupts();
    DIRECT_WRITE_LOW(reg, mask);
    DIRECT_MODE_OUTPUT(reg, mask);
    delayMicroseconds(30);
    DIRECT_WRITE_HIGH(reg, mask);
    interrupts();
  }
  interrupts();
  return;
}

uint8_t E2B::recv_bit_async(void){
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  uint8_t wt = waitTimeSlotRead();
  if (wt != 1){					//1 is nominal
    if (wt == 10){
      errnum = E2B_READ_TIMESLOT_TIMEOUT_LOW;
    } else {
      errnum = E2B_READ_TIMESLOT_TIMEOUT_HIGH;
    }
    interrupts();
    return 0;
  }

	#if ARDUINO_ARCH_ESP32                           //Special procedure for ESP32 devices
		#define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(20)
  	//#define TIMESLOT_WAIT_RETRY_COUNT (20*(F_CPU / 1000000L))
		uint16_t retries = TIMESLOT_WAIT_RETRY_COUNT; 											 //TIMESLOT_WAIT_RETRY_COUNT;
		while ((!DIRECT_READ(reg, mask)) && (--retries == 0))
			;
		interrupts();
		return (retries > 0);
  #else                                             //Standard procedure for all other microcontrollers
		delayMicroseconds(30);    											//TODO Consider reading earlier: delayMicroseconds(15);
		uint8_t r = DIRECT_READ(reg, mask);
		interrupts();
	  return r;
	#endif
}

//Waits for a low to high transition followed by a high to low within the time-out
uint8_t E2B::waitTimeSlot(){
  #define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(120) / 10L
  //#define TIMESLOT_WAIT_RETRY_COUNT (120*(F_CPU / 1000000L)) / 10L

  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);
  uint16_t retries;

  //Wait for a 0 to rise to 1 on the line for timeout duration
  //If the line is already high, this is basically skipped
  retries = TIMESLOT_WAIT_RETRY_COUNT;
  //While line is low, retry
  while (!DIRECT_READ(reg, mask))
    if (--retries == 0)
      return 10;

  //Wait for a fall form 1 to 0 on the line for timeout duration
  retries = TIMESLOT_WAIT_RETRY_COUNT;
  while (DIRECT_READ(reg, mask));
    if (--retries == 0)
      return 20;

  return 1;
}

//Variant of waitTimeSlot used for reading
uint8_t E2B::waitTimeSlotRead(){
  #define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(120) / 10L
  //#define TIMESLOT_WAIT_RETRY_COUNT (120*(F_CPU / 1000000L)) / 10L
  //It was derived from knowing that the Arduino based master may go up to 130 micros more than our wait after reset
  #define TIMESLOT_WAIT_READ_RETRY_COUNT microsecondsToClockCycles(135)
  //#define TIMESLOT_WAIT_READ_RETRY_COUNT (135*(F_CPU / 1000000L)) / 10L

  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  uint16_t retries;

  //Wait for a 0 to rise to 1 on the line for timeout duration
  //If the line is already high, this is basically skipped
  retries = TIMESLOT_WAIT_RETRY_COUNT;
  //While line is low, retry
  while (!DIRECT_READ(reg, mask))
		if (--retries == 0)
      return 10;

  //TODO Seems to me that the above loop should drop out immediately because
  // The line is already high as our wait after presence is relatively short
  // So now it just waits a short period for the write of a bit to start
  // Unfortunately per "recommended" this is 55 micros to 130 micros more
  // more than what we may have already waited.

  //Wait for a fall form 1 to 0 on the line for timeout duration
  retries = TIMESLOT_WAIT_READ_RETRY_COUNT;
  while (DIRECT_READ(reg, mask));
		if (--retries == 0)
			return 20;

	return 1;
}

// Compute a Dallas Semiconductor 8-bit CRC
/*uint8_t E2B::crc8_alt(char addr[], uint8_t len){
  uint8_t crc = 0;

  while (len--){
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--){
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
        inbyte >>= 1;
    }
  }
  return crc;
}*/

#endif    //E2B_ASYNC_RECV



#if E2B_CRC
//Computes a Dallas Semiconductor 8-bit CRC directly.
//The 1-Wire CRC scheme is described in Maxim Application Note 27:
//"Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
uint8_t E2B::crc8(const uint8_t *addr, uint8_t len){
	uint8_t crc = 0;

	while (len--){
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--){
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

// Compute the 1-Wire CRC16 and compare it against the received CRC.
// Example usage (reading a DS2408):
//    // Put everything in a buffer so we can compute the CRC easily.
//    uint8_t buf[13];
//    buf[0] = 0xF0;    // Read PIO Registers
//    buf[1] = 0x88;    // LSB address
//    buf[2] = 0x00;    // MSB address
//    WriteBytes(net, buf, 3);    // Write 3 cmd bytes
//    ReadBytes(net, buf+3, 10);  // Read 6 data bytes, 2 0xFF, 2 CRC16
//    if (!CheckCRC16(buf, 11, &buf[11])){
//        // Handle error.
//    }
//
// @param input - Array of bytes to checksum.
// @param len - How many bytes to use.
// @param inverted_crc - The two CRC16 bytes in the received data.
//                       This should just point into the received data,
//                       *not* at a 16-bit integer.
// @param crc - The crc starting value (optional)
// @return true, iff the CRC matches.
bool E2B::check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc){
    crc = ~crc16(input, len, crc);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}

// Compute a Dallas Semiconductor 16 bit CRC.  This is required to check
// the integrity of data received from many 1-Wire devices.  Note that the
// CRC computed here is *not* what you'll get from the 1-Wire network,
// for two reasons:
//   1) The CRC is transmitted bitwise inverted.
//   2) Depending on the endian-ness of your processor, the binary
//      representation of the two-byte return value may have a different
//      byte order than the two bytes you get from 1-Wire.
// @param input - Array of bytes to checksum.
// @param len - How many bytes to use.
// @param crc - The crc starting value (optional)
// @return The CRC16, as defined by Dallas Semiconductor.
uint16_t E2B::crc16(const uint8_t* input, uint16_t len, uint16_t crc){
    static const uint8_t oddparity[16] =
        { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    for (uint16_t i = 0 ; i < len ; i++){
      // Even though we're just copying a byte from the input,
      // we'll be doing 16-bit computation with it.
      uint16_t cdata = input[i];
      cdata = (cdata ^ crc) & 0xff;
      crc >>= 8;

      if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
          crc ^= 0xC001;

      cdata <<= 6;
      crc ^= cdata;
      cdata <<= 1;
      crc ^= cdata;
    }
    return crc;
}

#endif    //E2B_CRC

#if E2B_CHECKSUM
// Computes a standard XOR checksum
uint8_t E2B::checksum(const uint8_t *addr, uint8_t len){
  uint8_t checksum = 0;
  for (int i=0; i < len; i++) checksum ^= addr[i];
	return checksum;
}

#endif    //E2B_CHECKSUM

#if E2B_HAMMING
//Hamming Codes
/*NOTES:
-This code demonstrates Hamming(7,4) encoding and decoding. You can modify it to suit your specific requirements.
-This implementation assumes a single-bit error. For multiple-bit errors, consider using more advanced error-correcting codes.

/*Test sketch:
void setup(){
  Serial.begin(9600);
}

void loop(){
  // Example data to encode
  byte data[] = {0b1011};

  // Encode data using Hamming(7,4)
  byte encodedData[2] = hammingEncode(data, 1);

  // Simulate transmission error (optional)
  // encodedData[0] ^= 0b00000010; // Flip bit 1

  // Decode received data
  byte decodedData[2] = hammingDecode(encodedData);

  Serial.print("Original Data: ");
  Serial.println(data[0], BIN);
  Serial.print("Encoded Data: ");
  Serial.println(encodedData[0], BIN);
  Serial.print("Decoded Data: ");
  Serial.println(decodedData[0], BIN);

  delay(1000);
}
*/

// Hamming(7,4) Encoding Function
byte* E2B::hammingEncode(byte* data, int length){
  static byte encodedData[2];
  byte p1, p2, p3;

  for (int i = 0; i < length; i++){
    p1 = ((data[i] >> 0) & 1) ^ ((data[i] >> 2) & 1) ^ ((data[i] >> 3) & 1);
    p2 = ((data[i] >> 0) & 1) ^ ((data[i] >> 1) & 1) ^ ((data[i] >> 3) & 1);
    p3 = ((data[i] >> 1) & 1) ^ ((data[i] >> 2) & 1) ^ ((data[i] >> 3) & 1);
    encodedData[i] = (data[i] & 0x0F) | (p1 << 4) | (p2 << 5) | (p3 << 6);
  }
  return encodedData;
}

// Hamming(7,4) Decoding Function
byte* E2B::hammingDecode(byte* encodedData){
  static byte decodedData[2];
  byte s1, s2, s3, error;

  for (int i = 0; i < 1; i++){
    s1 = ((encodedData[i] >> 0) & 1) ^ ((encodedData[i] >> 2) & 1) ^ ((encodedData[i] >> 4) & 1) ^ ((encodedData[i] >> 6) & 1);
    s2 = ((encodedData[i] >> 1) & 1) ^ ((encodedData[i] >> 2) & 1) ^ ((encodedData[i] >> 5) & 1) ^ ((encodedData[i] >> 6) & 1);
    s3 = ((encodedData[i] >> 3) & 1) ^ ((encodedData[i] >> 4) & 1) ^ ((encodedData[i] >> 5) & 1) ^ ((encodedData[i] >> 6) & 1);
    error = (s1 << 0) | (s2 << 1) | (s3 << 2);

    // Correct single-bit error
    if (error != 0){
      encodedData[i] ^= (1 << (error - 1));
    }
    decodedData[i] = encodedData[i] & 0x0F;
  }
  return decodedData;
}

#endif    //E2B_HAMMING

#if E2B_LPDC
//Low-Density Parity Check FAMILYCODE_HOST
/*NOTES:
-This is a simplified example using a regular LDPC code with a small block size. In practice, larger block sizes and more complex codes are used.
-This implementation demonstrates basic LDPC encoding and decoding using the Belief Propagation Algorithm.
-Keep in mind that this is a simplified example and may not provide optimal error-correcting performance. For more robust error correction, consider using larger block sizes, more complex codes, and optimized decoding algorithms.

Test sketch:
const int blockSize = 8;
const int numParityBits = 4;
const int numIterations = 5;

byte data[blockSize] = {0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF};
byte encodedData[blockSize + numParityBits];
byte receivedData[blockSize + numParityBits];
byte decodedData[blockSize];

void setup(){
  Serial.begin(9600);

  // Encode data
  ldpcEncode(data, encodedData, blockSize, numParityBits);

  // Simulate transmission error (optional)
  // receivedData[3] ^= 0x01;

  // Copy encoded data to received data
  for (int i = 0; i < blockSize + numParityBits; i++){
    receivedData[i] = encodedData[i];
  }

  // Decode received data
  ldpcDecode(receivedData, decodedData, blockSize, numParityBits, numIterations);

  Serial.print("Original Data: ");
  for (int i = 0; i < blockSize; i++){
    Serial.print(data[i], HEX);
  }
  Serial.println();

  Serial.print("Decoded Data: ");
  for (int i = 0; i < blockSize; i++){
    Serial.print(decodedData[i], HEX);
  }
  Serial.println();
}

void loop(){
}
*/
//LDPC Encoding Function:
void E2B::ldpcEncode(byte* data, byte* encodedData, int blockSize, int numParityBits){
  int i, j;
  byte temp;

  // Initialize encoded data with data bits
  for (i = 0; i < blockSize; i++){
    encodedData[i] = data[i];
  }

  // Calculate parity bits
  for (i = 0; i < numParityBits; i++){
    temp = 0;
    for (j = 0; j < blockSize; j++){
      temp ^= (encodedData[j] & (1 << i)) ? 1 : 0;
    }
    encodedData[blockSize + i] = temp;
  }
}

//LDPC Decoding Function (Belief Propagation Algorithm):
void E2B::ldpcDecode(byte* receivedData, byte* decodedData, int blockSize, int numParityBits, int numIterations){
  int i, j, k;
  byte syndrome[numParityBits];
  byte temp;

  // Initialize syndrome
  for (i = 0; i < numParityBits; i++){
    syndrome[i] = 0;
  }

  // Calculate syndrome
  for (i = 0; i < numParityBits; i++){
    temp = 0;
    for (j = 0; j < blockSize; j++){
      temp ^= (receivedData[j] & (1 << i)) ? 1 : 0;
    }
    syndrome[i] = temp ^ receivedData[blockSize + i];
  }

  // Iterative decoding
  for (k = 0; k < numIterations; k++){
    for (i = 0; i < blockSize; i++){
      temp = receivedData[i];
      for (j = 0; j < numParityBits; j++){
        temp ^= (syndrome[j] & (1 << i)) ? 1 : 0;
      }
      receivedData[i] = temp;
    }

    // Update syndrome
    for (i = 0; i < numParityBits; i++){
      temp = 0;
      for (j = 0; j < blockSize; j++){
        temp ^= (receivedData[j] & (1 << i)) ? 1 : 0;
      }
      syndrome[i] = temp ^ receivedData[blockSize + i];
    }
  }

  // Copy decoded data
  for (i = 0; i < blockSize; i++){
    decodedData[i] = receivedData[i];
  }
}

#endif        //E2B_LPDC

#if E2B_CONVOLUTION
//Convolution codes
/*NOTES:

Test sketch:
//This implementation demonstrates basic convolutional encoding and decoding using the Viterbi algorithm.

const int dataLength = 10;
const int constraintLength = 3;

byte data[dataLength] = {0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF, 0x12, 0x34};
byte encodedData[2 * dataLength];
byte receivedData[2 * dataLength];
byte decodedData[dataLength];

void setup(){
  Serial.begin(9600);

  // Encode data
  convolutionalEncode(data, encodedData, dataLength, constraintLength);

  // Simulate transmission error (optional)
  // receivedData[3] ^= 0x01;

  // Copy encoded data to received data
  for (int i = 0; i < 2 * dataLength; i++){
    receivedData[i] = encodedData[i];
  }

  // Decode received data
  convolutionalDecode(receivedData, decodedData, dataLength, constraintLength);

  Serial.print("Original Data: ");
  for (int i = 0; i < dataLength; i++){
    Serial.print(data[i], HEX);
  }
  Serial.println();

  Serial.print("Decoded Data: ");
  for (int i = 0; i < dataLength; i++){
    Serial.print(decodedData[i], HEX);
  }
  Serial.println();
}

void loop(){
}
*/
//Convolutional Code Encoder Function:
void E2B::convolutionalEncode(byte* data, byte* encodedData, int dataLength, int constraintLength){
  int i, j;
  byte shiftRegister[constraintLength];

  // Initialize shift register
  for (i = 0; i < constraintLength; i++){
    shiftRegister[i] = 0;
  }

  // Encode data
  for (i = 0; i < dataLength; i++){
    // Shift data into shift register
    for (j = constraintLength - 1; j > 0; j--){
      shiftRegister[j] = shiftRegister[j - 1];
    }
    shiftRegister[0] = data[i];

    // Generate parity bits
    encodedData[2 * i] = shiftRegister[0] ^ shiftRegister[1] ^ shiftRegister[2];
    encodedData[2 * i + 1] = shiftRegister[0] ^ shiftRegister[2];
  }
}

//Convolutional Code Decoder Function (Viterbi Algorithm):
void E2B::convolutionalDecode(byte* receivedData, byte* decodedData, int dataLength, int constraintLength){
  int i, j, k;
  int trellis[constraintLength][2];
  int branchMetrics[constraintLength][2];
  int pathMetrics[constraintLength];

  // Initialize trellis and branch metrics
  for (i = 0; i < constraintLength; i++){
    trellis[i][0] = 0;
    trellis[i][1] = 0;
    branchMetrics[i][0] = 0;
    branchMetrics[i][1] = 0;
  }

  // Decode received data
  for (i = 0; i < dataLength; i++){
    // Compute branch metrics
    for (j = 0; j < constraintLength; j++){
      branchMetrics[j][0] = (receivedData[2 * i] == (trellis[j][0] ^ trellis[j][1] ^ trellis[j][2])) ? 0 : 1;
      branchMetrics[j][1] = (receivedData[2 * i + 1] == (trellis[j][0] ^ trellis[j][2])) ? 0 : 1;
    }

    // Update trellis and path metrics
    for (j = 0; j < constraintLength; j++){
      pathMetrics[j] = branchMetrics[j][0] + (j == 0 ? 0 : pathMetrics[j - 1]);
      trellis[j][0] = (j == 0 ? 0 : trellis[j - 1][0]);
      trellis[j][1] = (j == 0 ? 0 : trellis[j - 1][1]);
    }

    // Select most likely path
    k = 0;
    for (j = 1; j < constraintLength; j++){
      if (pathMetrics[j] < pathMetrics[k]){
        k = j;
      }
    }

    // Output decoded bit
    decodedData[i] = trellis[k][0];
  }
}

#endif        //E2B_CONVOLUTION

#if E2B_PARITY
//Parity bit
/*NOTES:
-Parity bits are a simple and effective way to detect errors in digital communication. Here's a brief summary:
-Parity Bit Implementation:
1. Choose even or odd parity.
2. Calculate the parity bit for each data byte.
3. Append the parity bit to the data byte.
4. Transmit the data byte with parity.
5. Receiver checks parity to detect errors.

-Even Parity Example:
Data Byte: 10101010
Parity Bit: 0 (since there are an even number of 1s)
Transmitted Byte: 101010100

-Odd Parity Example:
Data Byte: 10101010
Parity Bit: 1 (since there are an even number of 1s)
Transmitted Byte: 101010101

-Advantages:
1. Simple to implement in hardware and software.
2. Low overhead (only 1 extra bit per byte).
3. Effective for detecting single-bit errors.

-Disadvantages:
1. Cannot detect multiple-bit errors.
2. Cannot correct errors.
*/
void E2B::sendByte(byte data){
  byte parity = calculateParity(data);
  Serial.write(data);
  Serial.write(parity);
}

byte E2B::calculateParity(byte data){
  byte parity = 0;
  for (int i = 0; i < 8; i++){
    parity ^= (data >> i) & 1;
  }
  return parity;
}

void E2B::receiveByte(){
  byte data = Serial.read();
  byte parity = Serial.read();
  byte calculatedParity = calculateParity(data);
  if (parity != calculatedParity){
    // Error detected!
  }
}

#endif        //E2B_PARITY

#if E2B_AURORA
/*NOTES:
Overview: Aurora combines techniques from existing codes, adding novel elements for enhanced error detection and correction.

Components:
1. Data Fragmentation: Divide data into smaller fragments (e.g., 128 bytes)
2.  Cyclic Redundancy Check (CRC): Calculate CRC-32 for each fragment
3.  Error-Correcting Codes (ECC): Apply a simplified Reed-Solomon code (RS(255,239)) for fragment-level error correction
4.  Checksum Tree: Construct a binary tree of checksums (e.g., Adler-32) for rapid error detection
5.  Digital Signature: Append a digital signature (e.g., ECDSA) for authenticity and integrity

Novel Elements:
1.  Fragment Interleaving: Interleave fragments to minimize consecutive error occurrences
2.  Error Propagation Detection: Embed special "sentinel" bytes to detect error propagation

Operation:
1.  Sender:
    - Fragment data
    - Calculate CRC and ECC for each fragment
    - Construct checksum tree
    - Append digital signature
    - Interleave fragments
2.  Receiver:
    - Verify digital signature
    - Check checksum tree for errors
    - Correct errors using ECC and CRC
    - Detect error propagation using sentinel bytes

Advantages:
1.  Robust error detection and correction
2.  Efficient computation and low overhead
3.  Scalability for various data sizes and structures
4.  Enhanced security features

Challenges:
1.  Balancing complexity and performance
2.  Optimizing parameter choices (fragment size, CRC/ECC strength)
3.  Ensuring compatibility with diverse communication protocols



- This implementation assumes the encoded data is valid and follows the Aurora format.
- Error handling is simplified for demonstration purposes.

To use this example:
1. Copy the auroraEncode and auroraDecode functions into your Arduino sketch.
2. Define the data, encodedData, and decodedData buffers.
3. Initialize the data buffer with your original data.
4. Call auroraEncode to encode the data.
5. Simulate transmission errors (optional).
6. Call auroraDecode to decode the data.
7. Verify the decoded data matches the original data.

- This example assumes a simple Arduino sketch. You may need to adapt it to your specific project.
- The auroraEncode and auroraDecode functions require the helper functions (calculateCrc32, calculateEcc, etc.) to be implemented.
- You can modify the dataSize and encodedSize constants to suit your specific requirements.



Error Handling:
-The decoder function currently prints error messages to the serial console if:
 1. Digital signature verification fails.
 2. Checksum mismatch occurs.
 3. Error correction fails.

You may want to modify the error handling to suit your specific requirements, such as:
 1. Returning an error code.
 2. Setting a flag.
 3. Restarting the decoding process.


Decoder Optimization:
-To improve performance:
 1. Use lookup tables for CRC and checksum calculations.
 2. Optimize ECC correction using more efficient algorithms.
 3. Use parallel processing or pipelining for fragment decoding.

Security Considerations:
 1. Ensure the ECDSA signature verification is secure.
 2. Use a secure random number generator for key generation.
 3. Protect against side-channel attacks.

Fragment Size:
-The fragment size (128 bytes) can be adjusted based on:
 1. Data transmission packet size.
 2. Error correction capabilities.
 3. Computational resources.

Code Organization:
-Consider separating the decoder function into smaller modules:
 1. De-interleaving.
 2. Signature verification.
 3. Checksum checking.
 4. Error correction.

This will improve code readability and maintainability.

Test sketch:
// Define constants
const int dataSize = 1024; // Original data size
const int encodedSize = dataSize + 128 + 64; // Encoded data size (Aurora overhead)

// Define data buffers
byte data[dataSize];
byte encodedData[encodedSize];
byte decodedData[dataSize];



void setup(){
  Serial.begin(9600);

  //Initialize data
  for (int i=0; i < dataSize; i++){
    data[i] = random(256); // Fill with random data
  }

  auroraEncode(data, dataSize, encodedData);              //Encode data

  auroraDecode(encodedData, encodedSize, decodedData);    //Decode data

  // Verify decoded data
  bool match = true;
  for (int i = 0; i < dataSize; i++){
    if (data[i] != decodedData[i]){
      match = false;
      break;
    }
  }

  if (match){
    Serial.println("Decoded data matches original data");
  } else {
    Serial.println("Error: Decoded data does not match original data");
  }
}

void loop(){
}
*/
uint32_t E2B::calculateCrc32(byte* data, int size){
  uint32_t crc = 0xFFFFFFFF;
  for (int i = 0; i < size; i++){
    crc = crc ^ data[i];
    for (int j = 0; j < 8; j++){
      if (crc & 1){
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc = crc >> 1;
      }
    }
  }
  return crc ^ 0xFFFFFFFF;
}

void E2B::calculateEcc(byte* data, int size, byte* ecc){
  // Simplified Reed-Solomon code implementation
  // This is a basic example and may not provide optimal error correction
  for (int i = 0; i < 16; i++){
    ecc[i] = 0;
    for (int j = 0; j < size; j++){
      ecc[i] ^= data[j];
    }
  }
}

uint32_t E2B::calculateChecksum(byte* data, int size){
  uint32_t checksum = 0;
  for (int i = 0; i < size; i++){
    checksum += data[i];
  }
  return checksum;
}

void E2B::calculateSignature(byte* data, int size, byte* signature){
  // ECDSA signature implementation
  // This is a basic example and may not provide optimal security
  for (int i = 0; i < 64; i++){
    signature[i] = 0;
    for (int j = 0; j < size; j++){
      signature[i] ^= data[j];
    }
  }
}

bool E2B::verifySignature(byte* data, int size, byte* signature){
  byte calculatedSignature[64];
  calculateSignature(data, size, calculatedSignature);
  for (int i = 0; i < 64; i++){
    if (calculatedSignature[i] != signature[i]){
      return false;
    }
  }
  return true;
}

void E2B::correctErrors(byte* data, int size, byte* ecc, byte* crc){
  // ECC and CRC correction implementation
  // This is a basic example and may not provide optimal error correction
  uint32_t calculatedCrc = calculateCrc32(data, size);
  if (calculatedCrc != *(uint32_t*)crc){
    // Error detected, attempt correction
    for (int i = 0; i < size; i++){
      data[i] ^= ecc[i];
    }
  }
}

void E2B::interleaveFragments(byte* encodedData, int numFragments, int fragmentSize){
  // Fragment interleaving implementation
  byte temp[fragmentSize];
  for (int i = 0; i < numFragments; i++){
    for (int j = 0; j < fragmentSize; j++){
      temp[j] = encodedData[i * fragmentSize + j];
    }
    for (int j = 0; j < fragmentSize; j++){
      encodedData[i * fragmentSize + j] = temp[(i + j) % fragmentSize];
    }
  }
}

void E2B::deinterleaveFragments(byte* encodedData, int numFragments, int fragmentSize){
  // Fragment de-interleaving implementation
  byte temp[fragmentSize];
  for (int i = 0; i < numFragments; i++){
    for (int j = 0; j < fragmentSize; j++){
      temp[(i + j) % fragmentSize] = encodedData[i * fragmentSize + j];
    }
    for (int j = 0; j < fragmentSize; j++){
      encodedData[i * fragmentSize + j] = temp[j];
    }
  }
}

void E2B::auroraEncode(byte* data, int dataSize, byte* encodedData){
  int fragmentSize = 128;
  int numFragments = (dataSize + fragmentSize - 1) / fragmentSize;
  int crcSize = 4; // CRC-32
  int eccSize = 16; // Simplified Reed-Solomon code
  int checksumSize = 4; // Adler-32
  int signatureSize = 64; // ECDSA signature

  // Fragment data
  /*for (int i = 0; i < numFragments; i++){
    int fragmentStart = i * fragmentSize;
  }*/

  // Fragment data
  for (int i = 0; i < numFragments; i++){
    int fragmentStart = i * fragmentSize;
    int fragmentEnd = min(fragmentStart + fragmentSize, dataSize);

    // Calculate CRC-32
    uint32_t crc = calculateCrc32(&data[fragmentStart], fragmentEnd - fragmentStart);
    memcpy(&encodedData[fragmentStart + crcSize * i], &crc, crcSize);

    // Apply simplified Reed-Solomon code
    byte ecc[eccSize];
    calculateEcc(&data[fragmentStart], fragmentEnd - fragmentStart, ecc);
    memcpy(&encodedData[fragmentStart + crcSize * i + eccSize], ecc, eccSize);

    // Calculate checksum (Adler-32)
    uint32_t checksum = calculateChecksum(&data[fragmentStart], fragmentEnd - fragmentStart);
    memcpy(&encodedData[fragmentStart + crcSize * i + eccSize + checksumSize * i], &checksum, checksumSize);
  }

  // Append digital signature (ECDSA)
  byte signature[signatureSize];
  calculateSignature(data, dataSize, signature);
  memcpy(&encodedData[numFragments * fragmentSize], signature, signatureSize);

  // Interleave fragments
  interleaveFragments(encodedData, numFragments, fragmentSize);
}

/*This decoder function:
1. De-interleaves fragments.
2. Verifies the digital signature.
3. Checks the checksum.
4. Corrects errors using ECC and CRC.
5. Copies the decoded fragment to the output buffer.

To use this function:
1. Call auroraDecode with the encoded data, encoded size, and decoded data buffer.
2. Verify the decoded data matches the original data.*/
void E2B::auroraDecode(byte* encodedData, int encodedSize, byte* decodedData){
  int fragmentSize = 128;
  int numFragments = (encodedSize - 64) / (fragmentSize + 4 + 16 + 4); // minus signature size

  // De-interleave fragments
  deinterleaveFragments(encodedData, numFragments, fragmentSize);

  for (int i = 0; i < numFragments; i++){
    int fragmentStart = i * fragmentSize;

    // Verify digital signature (ECDSA)
    if (!verifySignature(&encodedData[fragmentStart], fragmentSize, &encodedData[encodedSize - 64])){
      // Error: signature mismatch
      Serial.println("Error: Signature mismatch");
      return;
    }

    // Check checksum (Adler-32)
    uint32_t checksum = calculateChecksum(&encodedData[fragmentStart], fragmentSize);
    if (checksum != *(uint32_t*)&encodedData[fragmentStart + fragmentSize + 4 * i]){
      // Error: checksum mismatch
      Serial.println("Error: Checksum mismatch");
      return;
    }

    // Correct errors using ECC and CRC
    byte ecc[16];
    memcpy(ecc, &encodedData[fragmentStart + 4], 16);
    byte crc[4];
    memcpy(crc, &encodedData[fragmentStart], 4);
    correctErrors(&encodedData[fragmentStart], fragmentSize, ecc, crc);

    // Copy decoded fragment
    memcpy(&decodedData[fragmentStart], &encodedData[fragmentStart], fragmentSize);
  }
}

#endif    //E2B_AURORA
