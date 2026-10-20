#pragma once
#include <stdint.h>
#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define INPUT_PULLUP 2
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
unsigned long millis(); void delay(unsigned long); void delayMicroseconds(unsigned int);
void pinMode(int,int); int digitalRead(int); int analogRead(int); void analogReadResolution(int);
struct SerialT { void begin(int); void print(const char*); void print(float,int=2); void println(const char*); void println(float,int=2);
  int available(); int read(); int printf(const char*,...); };
extern SerialT Serial;
