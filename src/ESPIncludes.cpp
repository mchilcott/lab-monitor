#include "ESPIncludes.h"
#include <cstring>

#if defined(ESP32)
char gHostname [10];

const char * hostname(){
    sprintf(gHostname, "ESP%X", getChipId());
    gHostname[sizeof(gHostname)-1] = '\0'; // Always make sure there is a null byte
    return gHostname;
}
#endif
