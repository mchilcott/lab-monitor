#pragma once
#include <Arduino.h>


// Should the logs be printed out the serial port as well?
#define LOG_SERIAL

#define LOG_STRING_SIZE 128
#define LOG_ENTRIES 10

extern char gLogBuffer[LOG_ENTRIES][LOG_STRING_SIZE];
extern char gLogCount;
extern char gLogNext;

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