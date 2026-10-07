// Build: g++ -std=c++11 -O2 -I. tools/replay.cpp -o replay
// Run:   ./replay session.csv > replay_pulses.csv
// Requires full-rate SAMPLE records (STREAM_EVERY_N=1). Uses the compiled config.
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "pipeline.h"
#include "wave_detection.h"
#include "spo2_processor.h"
#include "measurement_context.h"
int main(int argc,char** argv){
  if(argc!=2){std::cerr<<"usage: replay session.csv\n";return 2;}
  std::ifstream file(argv[1]);if(!file){std::cerr<<"Cannot open recording\n";return 2;}
  PpgPipeline pl;WaveAnalyzer wave;SpO2Processor spo;MeasurementContext ctx;
  pl.begin();spo.begin();ctx.begin();
  uint64_t previous=0,epoch=UINT64_MAX;int samples=0,gaps=0;bool spacingChecked=false;
  std::string line;
  std::cout<<"epoch,onset_us,bpm,amplitude,pi,auc,crest_s,width50_s,spo2_estimate\n";
  while(std::getline(file,line)){
    if(line.compare(0,7,"SAMPLE,")!=0)continue;
    std::vector<std::string> c;std::stringstream ss(line);std::string field;
    while(std::getline(ss,field,','))c.push_back(field);
    if(c.size()!=31){std::cerr<<"Malformed SAMPLE row\n";return 2;}
    try{
      uint64_t e=std::stoull(c[1]),t=std::stoull(c[2]);
      if(e==epoch && previous && !spacingChecked){
        spacingChecked=true;
        if(t-previous!=PPG_PERIOD_US){std::cerr<<"Not contiguous 125 Hz data: set STREAM_EVERY_N=1 before recording\n";return 2;}
      }
      if(e!=epoch || (previous && t-previous!=PPG_PERIOD_US)){
        pl.begin();wave.begin();spo.begin();ctx.begin();++gaps;
      }
      epoch=e;previous=t;++samples;
      float acc[6];for(int k=0;k<6;++k)acc[k]=std::stof(c[20+k]);
      float mv[2]={std::stof(c[16]),std::stof(c[17])};bool refs=std::stoi(c[26])!=0;
      ctx.process(mv,acc,refs);
      const auto ir=uint32_t(std::stoul(c[4])),red=uint32_t(std::stoul(c[5]));
      auto s=pl.process(float(ir),refs?acc:nullptr,t,ctx.contact,ctx.stable,ctx.motionQuiet);
      spo.pushSample(ir,red,t,s.sqi_good);
      if(wave.process(s)){
        const auto& f=wave.last;
        std::cout<<e<<','<<f.onset_us<<','<<f.bpm<<','<<f.amp<<','<<f.pi<<','<<f.area<<','<<f.crest<<','<<f.pw50<<','<<spo.getSpO2()<<'\n';
      }
    }catch(const std::exception& e){std::cerr<<"Invalid number in SAMPLE: "<<e.what()<<'\n';return 2;}
  }
  if(!samples){std::cerr<<"No SAMPLE records\n";return 2;}
  std::cerr<<"Replayed "<<samples<<" samples, "<<gaps<<" epochs/gaps\n";
}
