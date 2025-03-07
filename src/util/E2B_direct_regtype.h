#ifndef E2B_Direct_RegType_h
#define E2B_Direct_RegType_h

#include <stdint.h>

// Platform specific I/O register type
#if defined(__AVR__)
#define IO_REG_TYPE uint8_t

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny10__)
#define FAMILYCODE 0xA0
#elif defined(__LGT8F__)      //Defined here because LGT328 boards are of AVR architectures (since they're a drop-in replacement to the nanov3.0)
#define IO_REG_TYPE uint8_t
#define FAMILYCODE 0xB1
#else
#define FAMILYCODE 0xA1
#endif

#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK66FX1M0__) || defined(__MK64FX512__)
#define IO_REG_TYPE uint8_t
#define FAMILYCODE 0xA2
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)
#define IO_REG_TYPE uint32_t
#define FAMILYCODE 0xA3
#elif defined(__MKL26Z64__)
#define IO_REG_TYPE uint8_t
#define FAMILYCODE 0xA4
#elif defined(__SAM3X8E__) || defined(__SAM3A8C__) || defined(__SAM3A4C__)
#define IO_REG_TYPE uint32_t
#define FAMILYCODE 0xA5
#elif defined(__PIC32MX__)
#define IO_REG_TYPE uint32_t
#define FAMILYCODE 0xA6
#elif defined(ARDUINO_ARCH_ESP8266)
#define IO_REG_TYPE uint32_t
#define FAMILYCODE 0xA7
#elif defined(ARDUINO_ARCH_ESP32)
#define IO_REG_TYPE uint32_t
#define IO_REG_MASK_ATTR
#define FAMILYCODE 0xA8
#elif defined(ARDUINO_ARCH_STM32)
#define IO_REG_TYPE uint32_t
#define FAMILYCODE 0xA9
#elif defined(__SAMD21G18A__)
#define IO_REG_TYPE uint32_t
#define FAMILYCODE 0xAA
#elif defined(__ASR6501__)
#define IO_REG_TYPE uint32_t
#define FAMILYCODE 0xAB
#elif defined(RBL_NRF51822)
#define IO_REG_TYPE uint32_t
#define FAMILYCODE 0xAC
#elif defined(__arc__) /* Arduino101/Genuino101 specifics */
#define IO_REG_TYPE uint32_t
#define FAMILYCODE 0xAD
#elif defined(CH32V20x) || defined(CH32V203xB) || defined(CH32V208) || defined(CH32V30x) || defined(CH32V30x_C) || defined(CH32V10x) || defined(CH32V00x) || defined(CH32X035)
//#error "CH32 support does not exist (yet)"
#define IO_REG_TYPE uint32_t
#define FAMILYCODE 0xAE
#elif defined(__riscv)
#define IO_REG_TYPE uint32_t
#define FAMILYCODE 0xAF
#else
#define IO_REG_TYPE unsigned int
#define FAMILYCODE 0xB0
#endif
#endif
