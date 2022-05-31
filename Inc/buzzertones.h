//FINAL V0.1
#pragma once
#include <stdbool.h>
#include <stdint.h>

extern const void* lowBatTones[];

void set_buzzerStart(unsigned long mainCnt);

void i2cTimeout();
void adcTimeout();
void serialTimeout();
void TempWarning();
void ppmTimeout();
void MotorFail();
void noLCD();
void noSlave();

void beep_long();
void beep_short6_4();
void beep_short_many();
void beep_short_8();
void beep_short_13();
void lowBattery1();
void lowBattery2();
void lowBattery3();

void beep_short_many();

void startUpSound();
void shutDownSound();

void reverseSound();

void buttonRelease();

void resetSound();