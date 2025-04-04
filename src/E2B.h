#ifndef E2B_h
#define E2B_h

//#include <inttypes.h>     //Added 7-30-24. Placed under OneWireSlave_h in OneWireSlave.h so may need this

#ifdef __cplusplus

#include <stdint.h>

#if defined(__AVR__)        //May not need this??????
#include <util/crc16.h>
#endif

#if ARDUINO >= 100
#include <Arduino.h>       // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"      // for delayMicroseconds
#include "pins_arduino.h"  // for digitalPinToBitMask, etc
#endif

// Config file for the library
#include "E2B_config.h"

// Board-specific macros for direct GPIO
#include "util/E2B_direct_regtype.h"

#if defined(__SAM3X8E__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__PIC32MX__) /* || defined(__IMXRT1052__) || defined(__IMXRT1062__)*/
#define lowmark   325
#define highmark  500
//#elif defined(__AVR__)
#else
#define lowmark   360
#define highmark  900
#endif

//Device type
#define DEFAULT 0
#define BUS 0             //BUS (Deafult): Includes error-checking during communication
#define POINTTOPOINT 1    //POINTTOPOINT: Does NOT include error-checking during communication
#define TRANSCEIVER 2     //TRANSCEIVER: Does NOT include error-checking during communication

#define FAMILYCODE_HOST 0x31
#define FAMILYCODE_TRANSCEIVER 0x32

// Device Power-Source
//#define EXTERNAL 1
//#define PARASITE 0

#define E2B_NO_ERROR                   0
#define E2B_READ_TIMESLOT_TIMEOUT      1
#define E2B_WRITE_TIMESLOT_TIMEOUT     2
#define E2B_WAIT_RESET_TIMEOUT         3
#define E2B_VERY_LONG_RESET            4
#define E2B_VERY_SHORT_RESET           5
#define E2B_PRESENCE_LOW_ON_LINE       6
#define E2B_READ_TIMESLOT_TIMEOUT_LOW  7
#define E2B_READ_TIMESLOT_TIMEOUT_HIGH 8
#define E2B_SECURED_AND_LOCKED         9

class E2B{
  private:
    IO_REG_TYPE bitmask;
    volatile IO_REG_TYPE *baseReg;

    unsigned char ROM_NO[8];
    uint8_t LastDiscrepancy;
    uint8_t LastFamilyDiscrepancy;
    bool LastDeviceFlag;

    uint8_t busType;
    bool secureFlag;
    bool isLocked;
    int unlockedState;
    uint8_t secureKey;

    #if E2B_ASYNC_RECV
      bool recvAndProcessCmd();
      uint8_t waitTimeSlot();
      uint8_t waitTimeSlotRead();
      uint8_t power;
      char rom[8];

      volatile long previous;
      volatile long old_previous;
      volatile long diff;
    #endif

  public:
    E2B() { }
    E2B(uint8_t pin) { begin(pin); }
    void begin(uint8_t pin);
    void setBusType(uint8_t type);
    uint8_t getBusType();
    void setSecureFlag(uint8_t level, uint8_t key = 0);
    bool getSecureFlag();
    void generateROM(unsigned char *newAddr);
    bool waitToTransmit(void);
    uint8_t reset(void);
    void select(const uint8_t rom[8]);
    void skip(void);
    void read_scratchpad(void);
    void write_scratchpad(void);
    void unlock(uint8_t key);
    void write(uint8_t v, uint8_t power = 0);
    void write_bytes(const uint8_t *buf, uint16_t count, bool power = 0);
    uint8_t read(void);
    void read_bytes(uint8_t *buf, uint16_t count);
    void write_bit(uint8_t v);
    uint8_t read_bit(void);
    void depower(void);
    #if E2B_SEARCH
      void reset_search();
      void target_search(uint8_t family_code);
      bool search(uint8_t *newAddr, bool search_mode = true);
      bool search_and_log(uint8_t *newAddr, uint8_t *searchLog, bool search_mode = true);
    #endif

    #if E2B_ASYNC_RECV
      void init(unsigned char rom[8]);
      void MasterResetPulseDetection();
      void setScratchpad(unsigned char scratchpad[9]);
      void setPower(uint8_t power);
      bool waitForRequest(bool ignore_errors);
      bool waitForRequestInterrupt(bool ignore_errors);
      bool waitReset(uint16_t timeout_ms);
      bool presence(uint8_t delta);
      bool presence();
      bool searchROM();
      bool duty();
      uint8_t getScratchpad(uint8_t i);
      uint8_t sendData_async(char buf[], uint8_t data_len);
      uint8_t recvData_async(char buf[], uint8_t data_len);
      void send_async(uint8_t v);
      uint8_t recv_async(void);
      void send_bit_async(uint8_t v);
      uint8_t recv_bit_async(void);

      #if E2B_ASYNC_CUSTOM_FUNC
        void attachUserCommand(uint8_t num, void (*)(void));
        typedef void (*FuncPointerArray)(void);
        FuncPointerArray userFunc[256];
      #endif

      uint8_t errnum;
      char scratchpad[9];           //Originally a private variable
    #endif

    #if E2B_CRC
      static uint8_t crc8(const uint8_t *addr, uint8_t len);
      static bool check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc = 0);
      static uint16_t crc16(const uint8_t* input, uint16_t len, uint16_t crc = 0);
    #endif
    #if E2B_CHECKSUM
      static uint8_t checksum(const uint8_t *addr, uint8_t len);
    #endif
};

// Prevent this name from leaking into Arduino sketches
#ifdef IO_REG_TYPE
#undef IO_REG_TYPE
#endif

#endif // __cplusplus
#endif // E2B_h
