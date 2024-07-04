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

class E2B{
  private:
    IO_REG_TYPE bitmask;
    volatile IO_REG_TYPE *baseReg;

    // global search state
    unsigned char ROM_NO[8];
    uint8_t LastDiscrepancy;
    uint8_t LastFamilyDiscrepancy;
    bool LastDeviceFlag;

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
};

// Prevent this name from leaking into Arduino sketches
#ifdef IO_REG_TYPE
#undef IO_REG_TYPE
#endif

#endif // __cplusplus
#endif // E2B_h
