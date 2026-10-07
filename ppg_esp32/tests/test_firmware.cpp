#include <cstdio>
#include <cstdlib>
#include <string>
#include <sstream>
#include <map>
#include <algorithm>
#include "firmware.h"
#define REQUIRE(x) do{if(!(x)){std::fprintf(stderr,"FIRMWARE FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
int main(){
  ppgapp::setup();REQUIRE(ppgapp::sensorReady);
  REQUIRE(Wire.regs[0x57][0x08]==0x60);REQUIRE(Wire.regs[0x57][0x0a]==0x35);
  REQUIRE(Wire.regs[0x57][0x0c]==100);REQUIRE(Wire.regs[0x57][0x09]==3);
  uint64_t base=stubTimeUs;
  for(int i=0;i<1000;++i){stubTimeUs=base+uint64_t(i+1)*8000;Wire.push(100000,80000);ppgapp::loop();}
  REQUIRE(ppgapp::samples==1000);REQUIRE(ppgapp::gaps==0);
  uint32_t prev=ppgapp::epoch;stubTimeUs+=8000;Wire.push(100000,80000);Wire.push(100000,80000);ppgapp::loop();
  REQUIRE(ppgapp::epoch==prev+1);REQUIRE(ppgapp::discarded==2);REQUIRE(!ppgapp::perfusion.baselineReady);
  stubTimeUs+=8000;Wire.push(100000,80000);ppgapp::loop();REQUIRE(!ppgapp::latest.valid);
  stubTimeUs+=8000;Wire.push(0,0);ppgapp::loop();REQUIRE(!ppgapp::latest.optical);REQUIRE(!ppgapp::spo2.isSpO2Valid());
  stubTimeUs+=8000;Wire.overflow=true;ppgapp::loop();REQUIRE(ppgapp::gaps==2);
  Wire.fail=true;stubTimeUs+=8000;ppgapp::loop();REQUIRE(std::string(ppgapp::fault)=="I2C_ERROR");Wire.fail=false;
  stubTimeUs+=200000;ppgapp::loop();REQUIRE(!ppgapp::latest.valid);
  Serial.connected=false;
  for(int i=0;i<2000;++i){stubTimeUs+=8000;Wire.push(100000,80000);ppgapp::loop();}
  REQUIRE(ppgapp::logger.dropped>0); // backpressure never blocks sampling
  REQUIRE(ppgapp::samples>=3000);
  Serial.connected=true;
  for(int i=0;i<1000;++i)ppgapp::logger.pump();
  // Validate the live CSV schema against firmware headers for every emitted type.
  std::map<std::string,int> cols;std::istringstream stream(Serial.output);std::string row;
  while(std::getline(stream,row)) {
    if(row.compare(0,2,"# ")==0) {
      auto r=row.substr(2);auto comma=r.find(',');if(comma!=std::string::npos)
        cols[r.substr(0,comma)]=int(std::count(r.begin(),r.end(),','))+1;
    } else {
      auto comma=row.find(',');if(comma!=std::string::npos) {
        auto kind=row.substr(0,comma);
        if(cols.count(kind))REQUIRE(int(std::count(row.begin(),row.end(),','))+1==cols[kind]);
      }
    }
  }
  REQUIRE(cols.count("SAMPLE") && cols.count("PULSE"));
  std::printf("PASS: sketch API compile + mocked FIFO pairs/backlog/overflow/I2C errors/timeouts/contact loss/log backpressure\n");
  std::printf("Host object sizes: pipeline=%zu, wave=%zu, logger=%zu, SpO2=%zu bytes\n",sizeof(PpgPipeline),sizeof(WaveAnalyzer),sizeof(SerialLogger),sizeof(SpO2Processor));
}
