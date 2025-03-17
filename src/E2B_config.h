#ifndef E2B_Config_h
#define E2B_Config_h

#include <stdint.h>

// you can exclude asynchronous E2B receiving by defining this to 0
#ifndef E2B_ASYNC_RECV
#define E2B_ASYNC_RECV 1
#endif

// you can exclude asynchronous E2B receiving by defining this to 0
#ifndef E2B_ASYNC_CUSTOM_FUNC
#define E2B_ASYNC_CUSTOM_FUNC 1
#endif

// you can exclude E2B search by defining this to 0
#ifndef E2B_SEARCH
#define E2B_SEARCH 0
#endif

// you can exclude E2B CRC by defining this to 0
#ifndef E2B_CRC
#define E2B_CRC 0
#endif

// you can exclude E2B Checksum by defining this to 0, defaulted to 0
#ifndef E2B_CHECKSUM
#define E2B_CHECKSUM 0
#endif

#endif // E2B_Config_h
