#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>
#include <deque>
#include <cmath>
static uint64_t stubTimeUs=0;
static float stubFsrMv[2]={1000,1000};
constexpr int ADC_11db=3;
inline uint32_t millis(){return uint32_t(stubTimeUs/1000);}
inline void delay(uint32_t ms){stubTimeUs+=uint64_t(ms)*1000;}
inline void analogReadResolution(int){}
inline void analogSetPinAttenuation(int,int){}
inline uint32_t analogReadMilliVolts(int pin){return uint32_t(pin==4?stubFsrMv[0]:pin==5?stubFsrMv[1]:1500);}
struct StubSerial {
  std::string output;std::deque<int> input;bool connected=true;
  void begin(int){} explicit operator bool() const{return connected;}
  int availableForWrite(){return connected?1024:0;}
  size_t write(const uint8_t* p,size_t n){output.append(reinterpret_cast<const char*>(p),n);return n;}
  int available(){return int(input.size());}
  int read(){if(input.empty())return -1;int x=input.front();input.pop_front();return x;}
};
static StubSerial Serial;
