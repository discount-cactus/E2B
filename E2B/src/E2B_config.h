#ifndef E2B_Config_h
#define E2B_Config_h

#include <stdint.h>

// you can exclude asynchronous E2B receiving by defining this to 0
#ifndef E2B_ASYNC_RECV
#define E2B_ASYNC_RECV 1
#endif

// you can exclude E2B search by defining this to 0
#ifndef E2B_SEARCH
#define E2B_SEARCH 1
#endif

// you can exclude E2B CRC by defining this to 0
#ifndef E2B_CRC
#define E2B_CRC 1
#endif

// you can exclude E2B Checksum by defining this to 0, defaulted to 0
#ifndef E2B_CHECKSUM
#define E2B_CHECKSUM 0
#endif

// you can exclude E2B Hamming Codes by defining this to 0, defaulted to 0
#ifndef E2B_HAMMING
#define E2B_HAMMING 0
#endif

// you can exclude E2B Low-Density Parity Checks (LPDC) by defining this to 0, defaulted to 0
#ifndef E2B_LPDC
#define E2B_LPDC 0
#endif

// you can exclude E2B Convolution Codes by defining this to 0, defaulted to 0
#if E2B_CONVOLUTION
#define E2B_CONVOLUTION 0
#endif

// you can exclude E2B Parity bits by defining this to 0, defaulted to 0
#if E2B_PARITY
#define E2B_PARITY 0
#endif

// you can exclude E2B Aurora encoding/decoding by defining this to 0, defaulted to 0
#ifndef E2B_AURORA
#define E2B_AURORA 0
#endif

#endif // E2B_Config_h
