#ifndef E2B_h
#define E2B_h

#ifdef __cplusplus

#include <stdint.h>

#if defined(__AVR__)
#include <util/crc16.h>
#endif

#if ARDUINO >= 100
#include <Arduino.h>       // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"      // for delayMicroseconds
#include "pins_arduino.h"  // for digitalPinToBitMask, etc
#endif

// Board-specific macros for direct GPIO
#include "util/E2B_direct_regtype.h"


#if defined(__SAM3X8E__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__PIC32MX__)
#define lowmark   325
#define highmark  500
//#define dsslaveassignedint dsslavepin
#elif defined(__AVR__)
#define lowmark   360			//360
#define highmark  900 		//900

/*#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) //Arduino Mega
  #if dsslavepin == 2
  #define dsslaveassignedint 0
  #elif dsslavepin == 3
  #define dsslaveassignedint 1
  #elif dsslavepin == 18
  #define dsslaveassignedint 5
  #elif dsslavepin == 19
  #define dsslaveassignedint 4
  #elif dsslavepin == 20
  #define dsslaveassignedint 3
  #elif dsslavepin == 21
  #define dsslaveassignedint 2
  #else
  #error "Not an Interrupt Pin"
  #error "Use 2, 3, 18, 19, 20 or 21"
  #define dsslaveassignedint -1
  #endif
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) //Sanguino
  #if dsslavepin == 2
  #define dsslaveassignedint 2
  #elif dsslavepin == 10
  #define dsslaveassignedint 0
  #elif dsslavepin == 11
  #define dsslaveassignedint 1
  #else
  #error "Not an Interrupt Pin"
  #error "Use 2, 10 or 11"
  #define dsslaveassignedint -1
  #endif
#elif defined(__AVR_ATmega32U4__) && !defined(CORE_TEENSY) //Arduino Leonardo, Vinciduino
  #if dsslavepin == 0
  #define dsslaveassignedint 2
  #elif dsslavepin == 1
  #define dsslaveassignedint 3
  #elif dsslavepin == 2
  #define dsslaveassignedint 1
  #elif dsslavepin == 3
  #define dsslaveassignedint 0
  #elif dsslavepin == 7
  #define dsslaveassignedint 4
  #else
  #error "Not an Interrupt Pin"
  #error "Use 0, 1, 2, 3 or 7"
  #define dsslaveassignedint -1
  #endif
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)	//Teensy 1.0++ 2.0++
  #if dsslavepin == 0
  #define dsslaveassignedint 0
  #elif dsslavepin == 1
  #define dsslaveassignedint 1
  #elif dsslavepin == 2
  #define dsslaveassignedint 2
  #elif dsslavepin == 3
  #define dsslaveassignedint 3
  #elif dsslavepin == 18
  #define dsslaveassignedint 6
  #elif dsslavepin == 19
  #define dsslaveassignedint 7
  #elif dsslavepin == 36
  #define dsslaveassignedint 4
  #elif dsslavepin == 37
  #define dsslaveassignedint 5
  #else
  #error "Not an Interrupt Pin"
  #error "Use 0, 1, 2, 3, 18, 19, 36 or 37"
  #define dsslaveassignedint -1
  #endif
#elif defined(__AVR_AT90USB162__)	//Teensy 1.0
  #if dsslavepin == 0
  #define dsslaveassignedint 0
  #elif dsslavepin == 1
  #define dsslaveassignedint 1
  #elif dsslavepin == 2
  #define dsslaveassignedint 2
  #elif dsslavepin == 3
  #define dsslaveassignedint 3
  #elif dsslavepin == 4
  #define dsslaveassignedint 5
  #elif dsslavepin == 6
  #define dsslaveassignedint 6
  #elif dsslavepin == 7
  #define dsslaveassignedint 7
  #elif dsslavepin == 16
  #define dsslaveassignedint 4
  #else
  #error "Not an Interrupt Pin"
  #error "Use 0, 1, 2, 3, 4, 6, 7 or 16"
  #define dsslaveassignedint -1
	#endif
#elif defined(__AVR_ATmega32U4__) && defined(CORE_TEENSY) // teensy 2.0
  #if dsslavepin == 2
  #define dsslaveassignedint 6
  #elif dsslavepin == 5
  #define dsslaveassignedint 0
  #elif dsslavepin == 6
  #define dsslaveassignedint 1
  #elif dsslavepin == 7
  #define dsslaveassignedint 2
  #elif dsslavepin == 8
  #define dsslaveassignedint 3
  #else
  #error "Not an Interrupt Pin"
  #error "Use (2 not included in Teensyduino), 5, 6, 7 or 8"
  #define dsslaveassignedint -1
  #endif
#elif defined(__AVR__) //ATTiny
  #if dsslavepin == 0
  #define dsslaveassignedint 0
  #elif dsslavepin == 1
  #define dsslaveassignedint 1
  #elif dsslavepin == 2
  #define dsslaveassignedint 2
  #elif dsslavepin == 3
  #define dsslaveassignedint 3
  #elif dsslavepin == 4
  #define dsslaveassignedint 5
  #elif dsslavepin == 5
  #else
  #error "Not an Interrupt Pin"
  #error "Use 0, 1, 2, 3, 4 or 5"
  #define dsslaveassignedint -1
  #endif
#else
  #if dsslavepin == 2
  #define dsslaveassignedint 0
  #elif dsslavepin == 3
  #define dsslaveassignedint 1
  #else
  #error "Not an Interrupt Pin"
  #error "Use 2 or 3"
  #define dsslaveassignedint -1
  #endif
#endif*/
#else
#define lowmark   360			//360
#define highmark  900 		//900
#endif

/*#if defined(__AVR__)
#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define IO_REG_ASM asm("r30")
#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+2)) |= (mask))

#elif defined(__MK20DX128__) || defined(__MK20DX256__)
#define PIN_TO_BASEREG(pin)             (portOutputRegister(pin))
#define PIN_TO_BITMASK(pin)             (1)
#define IO_REG_TYPE uint8_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (*((base)+512))
#define DIRECT_MODE_INPUT(base, mask)   (*((base)+640) = 0)
#define DIRECT_MODE_OUTPUT(base, mask)  (*((base)+640) = 1)
#define DIRECT_WRITE_LOW(base, mask)    (*((base)+256) = 1)
#define DIRECT_WRITE_HIGH(base, mask)   (*((base)+128) = 1)

#elif defined(__SAM3X8E__)
// Arduino 1.5.1 may have a bug in delayMicroseconds() on Arduino Due.
// http://arduino.cc/forum/index.php/topic,141030.msg1076268.html#msg1076268
// If you have trouble with OneWire on Arduino Due, please check the
// status of delayMicroseconds() before reporting a bug in OneWire!
#define PIN_TO_BASEREG(pin)             (&(digitalPinToPort(pin)->PIO_PER))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (((*((base)+15)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+5)) = (mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+4)) = (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+13)) = (mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+12)) = (mask))
#ifndef PROGMEM
#define PROGMEM
#endif
#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const uint8_t *)(addr))
#endif

#elif defined(__PIC32MX__)
#define PIN_TO_BASEREG(pin)             (portModeRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (((*(base+4)) & (mask)) ? 1 : 0)  //PORTX + 0x10
#define DIRECT_MODE_INPUT(base, mask)   ((*(base+2)) = (mask))            //TRISXSET + 0x08
#define DIRECT_MODE_OUTPUT(base, mask)  ((*(base+1)) = (mask))            //TRISXCLR + 0x04
#define DIRECT_WRITE_LOW(base, mask)    ((*(base+8+1)) = (mask))          //LATXCLR  + 0x24
#define DIRECT_WRITE_HIGH(base, mask)   ((*(base+8+2)) = (mask))          //LATXSET + 0x28

//#else
//#error "Please define I/O register types here"
#endif*/

// Device Power-Source
#define EXTERNAL 1
#define PARASITE 0

// Device Family-Code
#define DS18B20	0x28
#define DS18S20 0x10
#define DS1822  0x22
#define DS1820	0x10
#define DS2423	0x1D

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

#define ONEWIRE_NO_ERROR                   0
#define ONEWIRE_READ_TIMESLOT_TIMEOUT      1
#define ONEWIRE_WRITE_TIMESLOT_TIMEOUT     2
#define ONEWIRE_WAIT_RESET_TIMEOUT         3
#define ONEWIRE_VERY_LONG_RESET            4
#define ONEWIRE_VERY_SHORT_RESET           5
#define ONEWIRE_PRESENCE_LOW_ON_LINE       6
#define ONEWIRE_READ_TIMESLOT_TIMEOUT_LOW  7
#define ONEWIRE_READ_TIMESLOT_TIMEOUT_HIGH 8


class E2B{
  private:
    //E2B
    IO_REG_TYPE bitmask;
    volatile IO_REG_TYPE *baseReg;

    // global search state
    unsigned char ROM_NO[8];
    uint8_t LastDiscrepancy;
    uint8_t LastFamilyDiscrepancy;
    bool LastDeviceFlag;

    //E2Bslave
    bool recvAndProcessCmd();
    uint8_t waitTimeSlot();
    uint8_t waitTimeSlotRead();
    uint8_t power;
    char rom[8];
    char scratchpad[9];
    char temp_scratchpad[3];
    char scratchpadtemperature[2];

  public:
    E2B() { }
    E2B(uint8_t pin) { begin(pin); }
    void begin(uint8_t pin);
    uint8_t reset(void);
    void select(const uint8_t rom[8]);
    void skip(void);
    void write(uint8_t v, uint8_t power = 0);
    void write_bytes(const uint8_t *buf, uint16_t count, bool power = 0);
    uint8_t read(void);
    void read_bytes(uint8_t *buf, uint16_t count);
    void write_bit(uint8_t v);
    uint8_t read_bit(void);
    void depower(void);
    void reset_search();
    void target_search(uint8_t family_code);
    bool search(uint8_t *newAddr, bool search_mode = true);
    static uint8_t crc8(const uint8_t *addr, uint8_t len);
    static bool check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc = 0);
    static uint16_t crc16(const uint8_t* input, uint16_t len, uint16_t crc = 0);

    //E2B(uint8_t pin);
    void init(unsigned char rom[8]);
    void MasterResetPulseDetection();
    static void ISRPIN();
    void setScratchpad(unsigned char scratchpad[9]);
    void setScratchpad_external(char temp_scratchpad[3]);
    void setPower(uint8_t power);
    void setTemperature(unsigned char scratchpadtemperature[2]);
    bool waitForRequest(bool ignore_errors);
    bool waitForRequestInterrupt(bool ignore_errors);
    bool waitReset(uint16_t timeout_ms);
    bool waitReset();
    bool owsprint();
    bool presence(uint8_t delta);
    bool presence();
    bool searchROM();
    bool duty();
    void setResolution(uint8_t resolution);
    uint8_t getResolution();
    void attach44h (void (*)(void));
    void attach48h (void (*)(void));
    void attachB8h (void (*)(void));
    uint8_t sendData(char buf[], uint8_t data_len);
    uint8_t recvData(char buf[], uint8_t data_len);
    void send(uint8_t v);
    uint8_t recv(void);
    void sendBit(uint8_t v);
    uint8_t recvBit(void);
    uint8_t crc8_alt(char addr[], uint8_t len);

    uint8_t errnum;
};

static E2B* static_OWS_instance;

// Prevent this name from leaking into Arduino sketches
#ifdef IO_REG_TYPE
#undef IO_REG_TYPE
#endif

#endif // __cplusplus
#endif // E2B_h
