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
#if E2B_ASYNC_RECV
  #define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(120) / 10L
  //This TIMESLOT_WAIT_READ_RETRY_COUNT is new, and only used when waiting for the line to go low on a read
  //It was derived from knowing that the Arduino based master may go up to 130 micros more than our wait after reset
  #define TIMESLOT_WAIT_READ_RETRY_COUNT microsecondsToClockCycles(135)

  void E2B::ISRPIN() {
    (*static_OWS_instance).MasterResetPulseDetection();
  }
#endif

uint8_t _pin;

/*E2B::E2B(uint8_t pin) {
	_pin = pin;
	pinMode(_pin, INPUT);
	bitmask = PIN_TO_BITMASK(_pin);
	baseReg = PIN_TO_BASEREG(_pin);

}*/

volatile long previous = 0;
volatile long old_previous = 0;
volatile long diff = 0;

void E2B::begin(uint8_t pin){
	_pin = pin;
	pinMode(_pin, INPUT);
	bitmask = PIN_TO_BITMASK(_pin);
	baseReg = PIN_TO_BASEREG(_pin);

  #if E2B_SEARCH
	 reset_search();
  #endif
}


// Perform the E2B reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return a 0;
//
// Returns 1 if a device asserted a presence pulse, 0 otherwise.
//
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

//
// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
void E2B::write_bit(uint8_t v){
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;

	if (v & 1) {
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

//
// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
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

//
// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the write (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the write to avoid heating in a short or
// other mishap.
//
void E2B::write(uint8_t v, uint8_t power /* = 0 */) {
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	E2B::write_bit( (bitMask & v)?1:0);
    }
    if ( !power) {
	noInterrupts();
	DIRECT_MODE_INPUT(baseReg, bitmask);
	DIRECT_WRITE_LOW(baseReg, bitmask);
	interrupts();
    }
}

void E2B::write_bytes(const uint8_t *buf, uint16_t count, bool power /* = 0 */) {
  for (uint16_t i = 0 ; i < count ; i++)
    write(buf[i]);
  if (!power) {
    noInterrupts();
    DIRECT_MODE_INPUT(baseReg, bitmask);
    DIRECT_WRITE_LOW(baseReg, bitmask);
    interrupts();
  }
}

//
// Read a byte
//
uint8_t E2B::read() {
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	if ( E2B::read_bit()) r |= bitMask;
    }
    return r;
}

void E2B::read_bytes(uint8_t *buf, uint16_t count) {
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = read();
}

//
// Do a ROM select
//
void E2B::select(const uint8_t rom[8]){
    uint8_t i;

    write(0x55);           // Choose ROM

    for (i = 0; i < 8; i++) write(rom[i]);
}

//
// Do a ROM skip
//
void E2B::skip(){
    write(0xCC);           // Skip ROM
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
  for(int i = 7; ; i--) {
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
   if (!LastDeviceFlag) {
      // 1-Wire reset
      if (!reset()) {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
         return false;
      }

      // issue the search command
      if (search_mode == true) {
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
         if ((id_bit == 1) && (cmp_id_bit == 1)) {
            break;
         } else {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit) {
               search_direction = id_bit;  // bit write value for search
            } else {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy) {
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               } else {
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);
               }
               // if 0 was picked then record its position in LastZero
               if (search_direction == 0) {
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
            if (rom_byte_mask == 0) {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65)) {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0) {
            LastDeviceFlag = true;
         }
         search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0]) {
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   } else {
      for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   }
   return search_result;
  }

#endif    //E2B_SEARCH

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#if E2B_ASYNC_RECV

void E2B::MasterResetPulseDetection() {
  old_previous = previous;
  previous = micros();
  diff = previous - old_previous;
  if (diff >= lowmark && diff <= highmark) {
    waitForRequestInterrupt(false);
  }
}

bool E2B::owsprint() {
	//waitForRequestInterrupt(false);
	//Serial.println("done");
  delayMicroseconds(25);

  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  errnum = ONEWIRE_NO_ERROR;
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

	//while (recvAndProcessCmd()) {};
	recvAndProcessCmd();
}

void E2B::init(unsigned char rom[8]) {
	for (int i=0; i<7; i++)
    this->rom[i] = rom[i];
  this->rom[7] = crc8_alt(this->rom, 7);
}

void E2B::setScratchpad(unsigned char scratchpad[9]) {
  for (int i=0; i<8; i++)
    this->scratchpad[i] = scratchpad[i];
  this->scratchpad[8] = crc8_alt(this->scratchpad, 8);
}

void E2B::setPower(uint8_t power) {
  this->power = power;
}

void E2B::setResolution(uint8_t resolution) {
	switch (resolution) {
      case 12:
      	this->scratchpad[4] = TEMP_12_BIT;
        break;
      case 11:
      	this->scratchpad[4] = TEMP_11_BIT;
        break;
      case 10:
      	this->scratchpad[4] = TEMP_10_BIT;
        break;
      case 9:
      	this->scratchpad[4] = TEMP_9_BIT;
        break;
	}
	this->scratchpad[8] = crc8_alt(this->scratchpad, 8);
}

uint8_t E2B::getResolution() {
	switch (scratchpad[4]) {
      case TEMP_12_BIT:
        return 12;

      case TEMP_11_BIT:
        return 11;

      case TEMP_10_BIT:
        return 10;

      case TEMP_9_BIT:
        return 9;
	}
}

void (*user44hFunc)(void);

void E2B::attach44h(void (*userFunction44h)(void)) {
	user44hFunc = userFunction44h;
	this->scratchpad[8] = crc8_alt(this->scratchpad, 8);
}

void (*user48hFunc)(void);

void E2B::attach48h(void (*userFunction48h)(void)) {
	user48hFunc = userFunction48h;
}

void (*userB8hFunc)(void);

void E2B::attachB8h(void (*userFunctionB8h)(void)) {
	userB8hFunc = userFunctionB8h;
}

bool E2B::waitForRequest(bool ignore_errors) {
  errnum = ONEWIRE_NO_ERROR;

  for (;;) {
    //delayMicroseconds(40);
    //Once reset is done, it waits another 30 micros
    //Master wait is 65, so we have 35 more to send our presence now that reset is done
    if (!waitReset(0) ) {
      continue;
    }
    //Reset is complete, tell the master we are prsent
    // This will pull the line low for 125 micros (155 micros since the reset) and
    //  then wait another 275 plus whatever wait for the line to go high to a max of 480
    // This has been modified from original to wait for the line to go high to a max of 480.
    if (!presence() ) {
      continue;
    }
    //Now that the master should know we are here, we will get a command from the line
    //Because of our changes to the presence code, the line should be guranteed to be high
    if (recvAndProcessCmd() ) {
      return true;
    }
    else if ((errnum == ONEWIRE_NO_ERROR) || ignore_errors) {
      continue;
    }
    else {
      return false;
    }
  }
}

bool E2B::waitForRequestInterrupt(bool ignore_errors) {
  errnum = ONEWIRE_NO_ERROR;
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
  while (!presence(50) ) {};	//50	//45 arbeitet schon sehr gut
  //Now that the master should know we are here, we will get a command from the line
  //Because of our changes to the presence code, the line should be guranteed to be high
  while (recvAndProcessCmd() ) {};
  if ((errnum == ONEWIRE_NO_ERROR) || ignore_errors) {
    //continue;
  }
  else {
    return false;
  }
}

bool E2B::recvAndProcessCmd() {
	char addr[8];
  uint8_t oldSREG = 0;
  uint16_t raw = 0;

  for (;;) {
    uint8_t cmd = recv();
    //scratchpad[4] = cmd;                  //Added on 7-15-24 for transceiver functionality, replaces slot for temperature resolution, moved to duty();
    switch (cmd) {
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
        sendData(rom, 8);
        if (errnum != ONEWIRE_NO_ERROR)
          return false;
        break;
      case 0x55: // MATCH ROM - Choose/Select ROM
        recvData(addr, 8);
        if (errnum != ONEWIRE_NO_ERROR)
          return false;
        for (int i=0; i<8; i++)
          if (rom[i] != addr[i])
            return false;
        duty();
      case 0xCC: // SKIP ROM
      	duty();
      	if (errnum != ONEWIRE_NO_ERROR)
          return false;
        return true;
      default: // Unknow command
        if (errnum == ONEWIRE_NO_ERROR)
          break; // skip if no error
        else
          return false;
    }
  }
}

bool E2B::duty() {
	uint8_t done = recv();
  scratchpad[4] = done;                  //Added on 7-15-24 for transceiver functionality, replaces slot for temperature resolution
	switch (done) {
		case 0xBE: // READ SCREATCHPAD
			sendData(scratchpad, 9);
			if (errnum != ONEWIRE_NO_ERROR)
				return false;
			break;
		case 0xB4: // READ POWERSOURCE
			sendBit(power);
			if (errnum != ONEWIRE_NO_ERROR)
				return false;
			break;
		case 0x44: // CONVERT SENSOR
			user44hFunc();
			if (errnum != ONEWIRE_NO_ERROR)
				return false;
			break;
		case 0x48: // CONVERT SENSOR
			user48hFunc();
			if (errnum != ONEWIRE_NO_ERROR)
				return false;
			break;
		case 0xB8: // CONVERT SENSOR
			userB8hFunc();
			if (errnum != ONEWIRE_NO_ERROR)
				return false;
			break;
		case 0x4E: // WRITE SCREATCHPAD
			recvData(temp_scratchpad, 3);
			setScratchpad_external(temp_scratchpad);
			if (errnum != ONEWIRE_NO_ERROR)
				return false;
			break;
		default:
			break;
			if (errnum == ONEWIRE_NO_ERROR)
				break; // skip if no error
			else
				return false;
	return true;
	}
}

void E2B::setScratchpad_external(char temp_scratchpad[3]) {
  for (int i=2; i<5; i++)
    this->scratchpad[i] = temp_scratchpad[i-2];
  this->scratchpad[8] = crc8_alt(this->scratchpad, 8);
}

void E2B::setTemperature(unsigned char scratchpadtemperature[2]) {
	for (int i=0; i<2; i++)
    this->scratchpad[i] = scratchpadtemperature[i];
  this->scratchpad[8] = crc8_alt(this->scratchpad, 8);
}

bool E2B::searchROM() {
  uint8_t bitmask;
  uint8_t bit_send, bit_recv;

  for (int i=0; i<8; i++) {
    for (bitmask = 0x01; bitmask; bitmask <<= 1) {
      bit_send = (bitmask & rom[i])?1:0;
      sendBit(bit_send);
      sendBit(!bit_send);
      bit_recv = recvBit();
      if (errnum != ONEWIRE_NO_ERROR)
        return false;
      if (bit_recv != bit_send)
        return false;
    }
  }
  return true;
}

bool E2B::waitReset(uint16_t timeout_ms) {
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  unsigned long time_stamp;

  errnum = ONEWIRE_NO_ERROR;
  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  interrupts();

  //Wait for the line to fall
  if (timeout_ms != 0) {
    time_stamp = micros() + timeout_ms*1000;
    while (DIRECT_READ(reg, mask)) {
      if (micros() > time_stamp) {
        errnum = ONEWIRE_WAIT_RESET_TIMEOUT;
        return false;
      }
    }
  } else {
    //Will wait forever for the line to fall
    while (DIRECT_READ(reg, mask)) {};
  }

  //Set to wait for rise up to 540 micros
  //Master code sets the line low for 500 micros
  //TODO The actual documented max is 640, not 540
  time_stamp = micros() + 540;

  //Wait for the rise on the line up to 540 micros
  while (DIRECT_READ(reg, mask) == 0) {
    if (micros() > time_stamp) {
      errnum = ONEWIRE_VERY_LONG_RESET;
      return false;
    }
  }

  //If the master pulled low for exactly 500, then this will be 40 wait time
  // Recommended for master is 480, which would be 60 here then
  // Max is 640, which makes this negative, but it returns above as a "ONEWIRE_VERY_LONG_RESET"
  // this gives an extra 10 to 30 micros befor calling the reset invalid
  if ((time_stamp - micros()) > 70) {
    errnum = ONEWIRE_VERY_SHORT_RESET;
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
bool E2B::waitReset() {
  return waitReset(1000);
}

bool E2B::presence(uint8_t delta) {
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  //Reset code already waited 30 prior to calling this
  // Master will not read until 70 recommended, but could read as early as 60
  // so we should be well enough ahead of that. Arduino waits 65
  errnum = ONEWIRE_NO_ERROR;
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
  //delayMicroseconds(300 - delta);
  delayMicroseconds(300 - delta);

  //Modified to wait a while (roughly 50 micros) for the line to go high
  // since the above wait is about 430 micros, this makes this 480 closer
  // to the 480 standard spec and the 490 used on the Arduino master code
  // anything longer then is most likely something going wrong.
  uint8_t retries = 25;
  while (!DIRECT_READ(reg, mask));
  do {
	if ( retries-- == 0)
		//return false;
	delayMicroseconds(2);
  } while(!DIRECT_READ(reg, mask));
  /*
  if ( !DIRECT_READ(reg, mask)) {
      errnum = ONEWIRE_PRESENCE_LOW_ON_LINE;
      return false;
  } else
      return true;
  */
}
bool E2B::presence() {
  return presence(25);
}

uint8_t E2B::sendData(char buf[], uint8_t len) {
  uint8_t bytes_sended = 0;

  for (int i=0; i<len; i++) {
    send(buf[i]);
    if (errnum != ONEWIRE_NO_ERROR)
      break;
    bytes_sended++;
  }
  return bytes_sended;
}

uint8_t E2B::recvData(char buf[], uint8_t len) {
  uint8_t bytes_received = 0;

  for (int i=0; i<len; i++) {
    buf[i] = recv();
    if (errnum != ONEWIRE_NO_ERROR)
      break;
    bytes_received++;
  }
  return bytes_received;
}

void E2B::send(uint8_t v) {
  errnum = ONEWIRE_NO_ERROR;
  for (uint8_t bitmask = 0x01; bitmask && (errnum == ONEWIRE_NO_ERROR); bitmask <<= 1)
  	sendBit((bitmask & v)?1:0);
}

uint8_t E2B::recv() {
  uint8_t r = 0;

  errnum = ONEWIRE_NO_ERROR;
  for (uint8_t bitmask = 0x01; bitmask && (errnum == ONEWIRE_NO_ERROR); bitmask <<= 1)
    if (recvBit())
      r |= bitmask;
  return r;
}

void E2B::sendBit(uint8_t v) {
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  //waitTimeSlot waits for a low to high transition followed by a high to low within the time-out
  uint8_t wt = waitTimeSlot();
  if (wt != 1 ) { //1 is success, others are failure
    if (wt == 10) {
      errnum = ONEWIRE_READ_TIMESLOT_TIMEOUT_LOW;
    } else {
      errnum = ONEWIRE_READ_TIMESLOT_TIMEOUT_HIGH;
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

uint8_t E2B::recvBit(void) {
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);
  uint8_t r;

  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  //waitTimeSlotRead is a customized version of the original which was also
  // used by the "write" side of things.
  uint8_t wt = waitTimeSlotRead();
  if (wt != 1 ) { //1 is success, others are failure
    if (wt == 10) {
      errnum = ONEWIRE_READ_TIMESLOT_TIMEOUT_LOW;
    } else {
      errnum = ONEWIRE_READ_TIMESLOT_TIMEOUT_HIGH;
    }
    interrupts();
    return 0;
  }
  delayMicroseconds(30);
  //TODO Consider reading earlier: delayMicroseconds(15);
  r = DIRECT_READ(reg, mask);
  interrupts();
  return r;
}

uint8_t E2B::waitTimeSlot() {
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);
  uint16_t retries;

  //Wait for a 0 to rise to 1 on the line for timeout duration
  //If the line is already high, this is basically skipped
  retries = TIMESLOT_WAIT_RETRY_COUNT;
  //While line is low, retry
  while ( !DIRECT_READ(reg, mask))
    if (--retries == 0)
      return 10;

  //Wait for a fall form 1 to 0 on the line for timeout duration
  retries = TIMESLOT_WAIT_RETRY_COUNT;
  while ( DIRECT_READ(reg, mask));
    if (--retries == 0)
      return 20;

  return 1;
}

//This is a copy of what was orig just "waitTimeSlot"
// it is customized for the reading side of things
uint8_t E2B::waitTimeSlotRead() {
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  uint16_t retries;

  //Wait for a 0 to rise to 1 on the line for timeout duration
  //If the line is already high, this is basically skipped
  retries = TIMESLOT_WAIT_RETRY_COUNT;
  //While line is low, retry
  while ( !DIRECT_READ(reg, mask))
    if (--retries == 0)
      return 10;

  //TODO Seems to me that the above loop should drop out immediately because
  // The line is already high as our wait after presence is relatively short
  // So now it just waits a short period for the write of a bit to start
  // Unfortunately per "recommended" this is 55 micros to 130 micros more
  // more than what we may have already waited.

  //Wait for a fall form 1 to 0 on the line for timeout duration
  retries = TIMESLOT_WAIT_READ_RETRY_COUNT;
  while ( DIRECT_READ(reg, mask));
    if (--retries == 0)
      return 20;

  return 1;
}

//
// Compute a Dallas Semiconductor 8 bit CRC directly.
//
uint8_t E2B::crc8_alt(char addr[], uint8_t len) {
  uint8_t crc = 0;

  while (len--) {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
        inbyte >>= 1;
    }
  }
  return crc;
}

#endif    //E2B_ASYNC_RECV



#if E2B_CRC
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//
//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but a little smaller, than the lookup table.
//
uint8_t E2B::crc8(const uint8_t *addr, uint8_t len){
	uint8_t crc = 0;

	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
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
//    if (!CheckCRC16(buf, 11, &buf[11])) {
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

    for (uint16_t i = 0 ; i < len ; i++) {
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
