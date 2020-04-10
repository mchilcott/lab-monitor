#pragma once
#include <Arduino.h>


// Should the logs be printed out the serial port as well?
#define LOG_SERIAL

#define LOG_STRING_SIZE 128
#define LOG_ENTRIES 10

extern char gLogBuffer[LOG_ENTRIES][LOG_STRING_SIZE];
extern unsigned char gLogCount;
extern unsigned char gLogNext;

void write_log (const char * message);
void write_log (String message);
