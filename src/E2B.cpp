/*
This library began as a branch from Paul Stoffregen's OneWire v2.3.8 library and
Markus Lange's OneWireSlave v1.1 library to implement application-specific data
transmission between my custom hardware both on-board and off-board and over
time began to take shape as a pretty different beast all together.

Credit is due to Paul Stoffregen, Tom Pollard, Josh Larios, Jim Studt,
Robin James, Guillermo Lovato, Jason Dangel, Glenn Trewitt, Ken Butcher, Mark
Tillotson, Bertrik Sikken, Scott Roberts, Roger Clark, Love Nystrom for their
contributions to development of the OneWire library as well as Markus Lange,
Alexander Gordeyev and Joshua Fuller for their contributions to development of
the OneWireSlave library.



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
//--------------------------------------------------------------------------
*/

#include <Arduino.h>
#include "E2B.h"
#include "util/E2B_direct_gpio.h"

#ifdef ARDUINO_ARCH_ESP32
#define CRIT_TIMING IRAM_ATTR
#else
#define CRIT_TIMING
#endif

//Initializes the E2B port
void E2B::begin(uint8_t pin){
	pinMode(pin, INPUT);
	bitmask = PIN_TO_BITMASK(pin);
	baseReg = PIN_TO_BASEREG(pin);
	_pin = pin;

  busType = BUS;
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
uint8_t CRIT_TIMING E2B::reset(void){
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
void CRIT_TIMING E2B::write_bit(uint8_t v){
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

// Reads a single bit. Port and bit is used to cut lookup time and provide more certain timing.
uint8_t CRIT_TIMING E2B::read_bit(void){
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

// Writes a byte of data.
/*The writing code uses the active drivers to raise the pin high, if you need power after the write (e.g. DS18S20 inparasite power mode)
then set 'power' to 1, otherwise the pin will go tri-state at the end of the write to avoid heating in a short or other mishap.*/
void E2B::write(uint8_t v, uint8_t power){
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

//Writes multiple bytes of data
void E2B::write_bytes(const uint8_t *buf, uint16_t count, bool power){
  for (uint16_t i=0; i < count ; i++)
    write(buf[i]);
  if (!power){
    noInterrupts();
    DIRECT_MODE_INPUT(baseReg, bitmask);
    DIRECT_WRITE_LOW(baseReg, bitmask);
    interrupts();
  }
}

//Reads a byte of data
uint8_t E2B::read(){
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1){
	if ( E2B::read_bit()) r |= bitMask;
    }
    return r;
}

//Reads multiple bytes of data
void E2B::read_bytes(uint8_t *buf, uint16_t count){
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = read();
}

//Performs a SELECT ROM (or CHOOSE ROM) command
void E2B::select(const uint8_t rom[8]){
    write(0x55);

    for (uint8_t i = 0; i < 8; i++) write(rom[i]);
}

//Performs a SKIP ROM command
void E2B::skip(){
    write(0xCC);
}

//Performs a READ SCRATCHPAD command
void E2B::read_scratchpad(){
    write(0xBE);
}

//Performs a WRITE SCRATCHPAD command
void E2B::write_scratchpad(){
    write(0x4E);
}

//Requests an unlock to a device with a secureFlag = 1.
//This will unlock the secured device for one command.
void E2B::unlock(uint8_t key){
    write(0x3A);
    write(key);
}

//Disables power to the E2B port
void E2B::depower(){
	noInterrupts();
	DIRECT_MODE_INPUT(baseReg, bitmask);
	interrupts();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// E2B_SEARCH ////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#if E2B_SEARCH
//Resets the search state to the beginning. Always needed before a search, but optional on the first search
void E2B::reset_search(){
  LastDiscrepancy = 0;
  LastDeviceFlag = false;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--){
    ROM_NO[i] = 0;
    if (i == 0) break;
  }
}

//Sets up the search to find the device type 'family_code' on the next call to search(*newAddr) if it is present.
void E2B::target_search(uint8_t family_code){
   // set the search state to find SearchFamily type devices
   ROM_NO[0] = family_code;
   for (uint8_t i=1; i < 8; i++)
      ROM_NO[i] = 0;
   LastDiscrepancy = 64;
   LastFamilyDiscrepancy = 0;
   LastDeviceFlag = false;
}

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
bool E2B::search(uint8_t *newAddr, bool search_mode){
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
      }else{
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
         }else{
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit){
               search_direction = id_bit;  // bit write value for search
            }else{
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy){
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               }else{
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
   }else{
      for (int i=0; i < 8; i++) newAddr[i] = ROM_NO[i];
   }
   return search_result;
}

// Performs an E2B search and logs the addresses to a struct for later use.
bool E2B::search_and_log(uint8_t *newAddr, uint8_t *searchLog, bool search_mode){
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
///////////////////////////////////////// E2B_ASYNC_RECV //////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#if E2B_ASYNC_RECV

//Computes the pull-down time of the bus to determine if a pulse/bit is detected
void E2B::MasterResetPulseDetection(){
  old_previous = previous;
  previous = micros();
  diff = previous - old_previous;
  if (diff >= lowmark && diff <= highmark){
    waitForRequestInterrupt(false);
  }
}

//Initializes the E2B port for asynchronous receiving of data
void E2B::init(unsigned char rom[8]){
	for (int i=0; i<7; i++)
    this->rom[i] = rom[i];

		/*previous = 0;
		old_previous = 0;
		diff = 0;*/

	// Initializes all function pointers to NULL for user-defined functions
	#if E2B_ASYNC_CUSTOM_FUNC
		for(int i=0; i < 256; i++){
		    userFunc[i] = NULL;
		}
	#endif

	#if E2B_CRC
		uint8_t romUINT[7];
		for (int i=0; i<7; i++)
			romUINT[i] = rom[i];
		this->rom[7] = crc8(romUINT,7);
		//this->rom[7] = crc8(this->rom,7);
	#endif
}

//Writes the input data to the scratchpad
void E2B::setScratchpad(unsigned char scratchpad[9]){
  for (int i=0; i<8; i++)
    this->scratchpad[i] = scratchpad[i];
	#if E2B_CRC
		uint8_t scratchpadUINT[8];
		for (int i=0; i<8; i++)
	    scratchpadUINT[i] = scratchpad[i];
  	this->scratchpad[8] = crc8(scratchpadUINT,8);
		//this->scratchpad[8] = crc8(this->scratchpad,8);
	#endif
}

//Sets the power of the E2B port
void E2B::setPower(uint8_t power){
  this->power = power;
}

#if E2B_ASYNC_CUSTOM_FUNC
//Enables users to define their own functions for the device to automatically respond with
void E2B::attachUserCommand(uint8_t num, void (*userFunction)(void)){
	userFunc[num] = userFunction;
	#if E2B_CRC
		uint8_t scratchpadUINT[8];
		for (int i=0; i<8; i++)
			scratchpadUINT[i] = scratchpad[i];
		this->scratchpad[8] = crc8(scratchpadUINT,8);
		//this->scratchpad[8] = crc8(this->scratchpad,8);
	#endif
}
#endif

//Synchronously waits for data to be received
bool E2B::waitForRequest(bool ignore_errors){
  errnum = E2B_NO_ERROR;

  while(1){
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

//Interrupt-driven variant of waitForRequest(). Asynchronously waits for data to be reeceived. Triggered by MasterResetPulseDetection()
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

//Receives a command and processes follow-up commands
bool E2B::recvAndProcessCmd(){
	char addr[8];
  uint16_t raw = 0;

  while(1){
    uint8_t cmd = recv_async();
    //Serial.print("cmd: "); Serial.println(cmd,HEX);
		#if defined(ARDUINO_ARCH_ESP32)
			delayMicroseconds(30);
		#endif

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
        for (uint8_t i=0; i<8; i++)
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

//Processes a follow-up command
bool E2B::duty(){
	uint8_t done = recv_async();
	if(FAMILYCODE != FAMILYCODE_TRANSCEIVER){
  	scratchpad[4] = done;
	}
	//Serial.print("done: "); Serial.println(done,HEX);


  if ((secureFlag == 1) && (isLocked == 1) && (done != 0x3A)){
    errnum = E2B_SECURED_AND_LOCKED;
    //#warning "Attempted communication with locked secured device."
    //Serial.println("You shall not pass!");
    return false;
  }/*else{
    isLocked = 1;
  }*/

  switch(done){
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
        if(receivedKey == secureKey){					//Unlocks the device if receivedKey matches the key of the secured device (unlocked for the commands)
          isLocked = 0;
          unlockedState = 3;									//Processes 3 commands before locking again
          //#warning "Secured device unlocked for 3 commands."
        }
  			if (errnum != E2B_NO_ERROR)
  				return false;
			}
      break;
		case 0x4E: // WRITE SCREATCHPAD
			recvData_async(scratchpad,8);
			if (errnum != E2B_NO_ERROR)
				return false;
			break;
		default:
			if(errnum == E2B_NO_ERROR){
				#if E2B_ASYNC_CUSTOM_FUNC
					if (userFunc[done] != NULL){		//Executes the user-defined function at that entry
							userFunc[done]();
							return true;
					}else{													//No user-defined function, so return false
							return false;
					}
				#else
					return true;
				#endif
			}else{
					return false;
			}
	return true;
  }
}

//Returns a byte of data from the specified index in the scratchpad
uint8_t E2B::getScratchpad(uint8_t i){
  return scratchpad[i];
}

//Responds to a search ROM (0xF0) command
bool CRIT_TIMING E2B::searchROM(){
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

//Waits for a reset pulse
bool CRIT_TIMING E2B::waitReset(uint16_t timeout_ms){
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;

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

//Waits for the precense pulse
bool CRIT_TIMING E2B::presence(uint8_t delta){
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;

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
    	if (retries-- == 0)
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

//Asynchronously sends multiple bytes of data
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

//Asynchronously reads multiple bytes of data
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

//Asynchronously sends a byte of data
void E2B::send_async(uint8_t v){
  errnum = E2B_NO_ERROR;
  for (uint8_t bitmask = 0x01; bitmask && (errnum == E2B_NO_ERROR); bitmask <<= 1)
  	send_bit_async((bitmask & v)?1:0);
}

//Asynchronously reads a byte of data
uint8_t E2B::recv_async(){
  uint8_t r = 0;

  errnum = E2B_NO_ERROR;
  DIRECT_MODE_INPUT(baseReg, bitmask);
	for (uint8_t bitmask = 0x01; bitmask && (errnum == E2B_NO_ERROR); bitmask <<= 1)
		if (recv_bit_async())
      r |= bitmask;
  return r;
}

//Asynchronously sends a bit of data
void CRIT_TIMING E2B::send_bit_async(uint8_t v){
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;

  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  uint8_t wt = waitTimeSlot();
  if (wt != 1){					//1 is nominal
    errnum = (wt == 10) ? E2B_READ_TIMESLOT_TIMEOUT_LOW : E2B_READ_TIMESLOT_TIMEOUT_HIGH;
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

//Asynchronously reads a bit of data
uint8_t CRIT_TIMING E2B::recv_bit_async(void){
  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;

  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  uint8_t wt = waitTimeSlotRead();
  //Serial.print("wt: "); Serial.println(wt);
  if (wt != 1){					//1 is nominal
    errnum = (wt == 10) ? E2B_READ_TIMESLOT_TIMEOUT_LOW : E2B_READ_TIMESLOT_TIMEOUT_HIGH;
    interrupts();
    return 0;
  }

	#if defined(ARDUINO_ARCH_ESP32)
	  const uint32_t maxCycles = (F_CPU / 1000000) * 30;		//30us timeout window, initially 3us
	  uint32_t start = esp_cpu_get_cycle_count();

		if (_pin < 32){
			while (!((GPIO.in >> _pin) & 0x1)) {
		    if ((esp_cpu_get_cycle_count() - start) > maxCycles) {
		      interrupts();
		      return 0; // timeout, line stayed low
		    }
		  }
		}else{
			while (!((GPIO.in1.val >> (_pin - 32)) & 0x1)) {
		    if ((esp_cpu_get_cycle_count() - start) > maxCycles) {
		      interrupts();
		      return 0; // timeout, line stayed low
		    }
		  }
		}

	  interrupts();
		DIRECT_MODE_INPUT(baseReg, bitmask);
	  return 1;
	#else
	  delayMicroseconds(30);
	  uint8_t r = DIRECT_READ(reg, mask);
	  interrupts();
		DIRECT_MODE_INPUT(baseReg, bitmask);
	  return r;
	#endif
}

//Waits for a low to high transition followed by a high to low within the time-out
uint8_t CRIT_TIMING E2B::waitTimeSlot(){
  //#define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(120) / 10L
  #define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(20)

	/*#if !F_CPU
	#define TIMESLOT_WAIT_RETRY_COUNT (120*(F_CPU / 1000000L))
	#endif*/
  //#define TIMESLOT_WAIT_RETRY_COUNT (120*(F_CPU / 1000000L)) / 10L

  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
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
  while (DIRECT_READ(reg, mask))
    if (--retries == 0)
      return 20;

  return 1;
}

//Variant of waitTimeSlot used for reading
uint8_t CRIT_TIMING E2B::waitTimeSlotRead(){
  //#define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(120) / 10L
  #define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(20)

	/*#if !F_CPU
	#define TIMESLOT_WAIT_RETRY_COUNT (120*(F_CPU / 1000000L))
	#endif*/
  //#define TIMESLOT_WAIT_RETRY_COUNT (120*(F_CPU / 1000000L)) / 10L
  //It was derived from knowing that the Arduino based master may go up to 130 micros more than our wait after reset
  #define TIMESLOT_WAIT_READ_RETRY_COUNT microsecondsToClockCycles(135)
	/*#if !F_CPU
	#define TIMESLOT_WAIT_RETRY_COUNT (135*(F_CPU / 1000000L))
	#endif*/
  //#define TIMESLOT_WAIT_READ_RETRY_COUNT (135*(F_CPU / 1000000L)) / 10L

  IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;

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
  while (DIRECT_READ(reg, mask))
		if (--retries == 0)
			return 20;

	return 1;
}

#endif    //E2B_ASYNC_RECV



///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// E2B_CRC ////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#if E2B_CRC
//Computes a Dallas Semiconductor 8-bit CRC directly.
//The 1-Wire CRC scheme is described in Maxim Application Note 27: "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// E2B_CHECKSUM //////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#if E2B_CHECKSUM
//Computes a standard XOR checksum
uint8_t E2B::checksum(const uint8_t *addr, uint8_t len){
  uint8_t checksum = 0;
  for (int i=0; i < len; i++) checksum ^= addr[i];
	return checksum;
}

#endif    //E2B_CHECKSUM



// undef defines for no particular reason
#ifdef ARDUINO_ARCH_ESP32
#  undef noInterrupts() {portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;portENTER_CRITICAL(&mux)
#  undef interrupts() portEXIT_CRITICAL(&mux);}
#endif
// for info on this, search "IRAM_ATTR" at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/general-notes.html
#undef CRIT_TIMING
