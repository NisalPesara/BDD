#pragma once
#include "Arduino.h"
#include <vector>
#include <utility>
struct StubWire {
  uint8_t regs[128][256]={};uint8_t address=0,reg=0;
  std::vector<uint8_t> tx;std::deque<uint8_t> rx;
  std::deque<std::pair<uint32_t,uint32_t>> fifo;
  bool fail=false,overflow=false,ambient=false;
  uint8_t wr=0,rd=0;
  void begin(int,int) {regs[0x57][0xff]=0x15;regs[0x53][0]=regs[0x1d][0]=0xe5;}
  void setClock(int){} void setTimeOut(int){}
  void beginTransmission(uint8_t a){address=a;tx.clear();}
  void write(uint8_t b){tx.push_back(b);}
  uint8_t endTransmission(bool=true){
    if(fail)return 4;
    if(!tx.empty())reg=tx[0];
    if(tx.size()==2){regs[address][reg]=tx[1];
      if(address==0x57 && reg==9 && tx[1]==0x40)regs[address][reg]=0;
      if(address==0x57 && reg==4){wr=0;fifo.clear();}
      if(address==0x57 && reg==5)overflow=false;
      if(address==0x57 && reg==6)rd=0;
    }
    return 0;
  }
  size_t requestFrom(uint8_t a,size_t n){
    rx.clear();if(fail)return 0;
    if(a==0x57 && reg==7 && n==6){
      if(fifo.empty())return 0;
      auto v=fifo.front();fifo.pop_front();rd=(rd+1)&31;
      for(auto z:{v.second,v.first}){rx.push_back((z>>16)&255);rx.push_back((z>>8)&255);rx.push_back(z&255);}
    }else for(size_t i=0;i<n;++i){
      uint8_t r=uint8_t(reg+i), v=regs[a][r];
      if(a==0x57){if(r==0)v=ambient?0x20:0;if(r==4)v=wr;if(r==5)v=overflow?1:0;if(r==6)v=rd;}
      else {if(r==0x30)v=0x80;if(r==0x37)v=1;}
      rx.push_back(v);
    }
    return rx.size();
  }
  int available(){return int(rx.size());}
  int read(){if(rx.empty())return -1;int x=rx.front();rx.pop_front();return x;}
  void push(uint32_t ir,uint32_t red){fifo.emplace_back(ir,red);wr=(wr+1)&31;}
};
static StubWire Wire;
