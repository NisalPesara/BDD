// Desktop test of the pipeline (no ESP32 needed):  g++ -O2 -I.. native_test.cpp -o t && ./t > out.csv
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "../pipeline.h"
static float gauss(float t,float mu,float sd){return expf(-0.5f*powf((t-mu)/sd,2));}
int main(){
  PpgPipeline pipe; pipe.begin();
  const float fs=PPG_FS; const int N=60*PPG_FS; srand(1);
  int beats=0, goodWin=0, badWin=0; int badInMotion=0, winInMotion=0, goodClean=0, winClean=0;
  printf("t,raw,ppg,vpg,apg,sqi,good,beat\n");
  for(int i=0;i<N;i++){
    float t=i/fs; float hr=1.2f; float ph=fmodf(t*hr,1.0f);
    float pulse=gauss(ph,0.20f,0.07f)+0.45f*gauss(ph,0.55f,0.10f);
    float base=800*sinf(2*M_PI*0.2f*t);                       // respiration drift
    float noise=6*((rand()/(float)RAND_MAX)-0.5f);
    float motion=(t>35&&t<45)? 900*sinf(2*M_PI*3.1f*t)*sinf(2*M_PI*0.7f*t)+400*((rand()/(float)RAND_MAX)-0.5f):0;
    float raw=2000+base-500*pulse+noise+motion;   // inverted like a reflective MAX30102 (INVERT_SIGNAL=1 flips it back)
    PpgSample s=pipe.process(raw,0,(uint32_t)(t*1000));
    if(s.beat) beats++;
    if(s.sqi.fresh){ bool mot=(t>39&&t<49); if(mot){winInMotion++; if(!s.sqi.good) badInMotion++;} else if(t>10&&t<34){winClean++; if(s.sqi.good) goodClean++;}
       fprintf(stderr,"t=%5.1f good=%d per=%.2f kurt=%.2f ent=%.2f skew=%.2f pi=%.2f\n",t,s.sqi.good,s.sqi.periodicity,s.sqi.kurt,s.sqi.entropy,s.sqi.skew,s.sqi.pi);}
    printf("%.3f,%.1f,%.3f,%.3f,%.3f,%d,%d,%d\n",t,raw,s.ppg,s.vpg,s.apg,s.sqi_good,s.valid,s.beat);
  }
  fprintf(stderr,"beats detected=%d (expected ~72 minus warmup/motion)\n",beats);
  fprintf(stderr,"clean windows flagged good: %d/%d ; motion windows flagged bad: %d/%d\n",goodClean,winClean,badInMotion,winInMotion);
}
