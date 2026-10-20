#pragma once
#include <stdint.h>
struct WireT { void begin(int,int); void setClock(uint32_t); void beginTransmission(uint8_t); void write(uint8_t); int endTransmission(bool); int requestFrom(uint8_t,uint8_t); int available(); int read(); };
extern WireT Wire;
