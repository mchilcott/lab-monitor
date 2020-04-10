
#include "logging.h"

char gLogBuffer[LOG_ENTRIES][LOG_STRING_SIZE];
unsigned char gLogCount (0);
unsigned char gLogNext (0);


void write_log (const char * message)
{
    #ifdef LOG_SERIAL
    Serial.println(message);
    #endif

    strncpy(gLogBuffer[gLogNext], message, LOG_STRING_SIZE-1);
    gLogBuffer[gLogNext][LOG_STRING_SIZE-1] = '\0';

    ++gLogCount;
    ++gLogNext;
    if(gLogNext == LOG_ENTRIES) gLogNext = 0;

}

void write_log (String message){
  write_log(message.c_str());
}
