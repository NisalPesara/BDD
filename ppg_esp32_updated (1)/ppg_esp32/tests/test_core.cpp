#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>
#include "pipeline.h"
#include "wave_detection.h"
#include "spo2_processor.h"
#include "perfusion_monitor.h"
#include "measurement_context.h"
#include "m04_smoothing.h"

static int checks=0;
#define CHECK(x) do { ++checks; if (!(x)) { std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); std::exit(1); } } while(0)
static float g(float t,float c,float w) { const float z=(t-c)/w; return expf(-.5f*z*z); }
static float pulse(float t,float period=.85f,float dia=.3f) {
  const int center=int(floorf(t/period)); float s=0;
  for (int k=center-1;k<=center+1;++k) {
    const float q=(t-k*period)/period;
    s+=g(q,.20f,.065f)+dia*g(q,.58f,.09f);
  }
  return s;
}
static void filters() {
  MovingAverage<64> m; CHECK(m.begin(125,40)==5);
  std::vector<float> x; double sum=0;
  for(int i=0;i<10000;++i) {
    float v=50000.f+float(i%13); x.push_back(v);sum+=v;if(i>=5)sum-=x[i-5];
    CHECK(fabs(m.process(v)-sum/std::min(i+1,5))<.02);
  }
  ppgnr::SavGolDeriv<7> sg; CHECK(sg.begin(125,3));
  CHECK(!sg.begin(0,3)); CHECK(sg.begin(125,3));
  for(int i=0;i<100;++i) {
    const double t=i/125., c=(i-7)/125.; float s,v,a;
    if(sg.process(float(2+3*t-.5*t*t+.1*t*t*t),s,v,a)) {
      CHECK(fabs(s-(2+3*c-.5*c*c+.1*c*c*c))<1e-4);
      CHECK(fabs(v-(3-c+.3*c*c))<.01);CHECK(fabs(a-(-1+.6*c))<.1);
    }
  }
  ppgnr::Hampel<5> h;
  float out=0;
  for(int i=0;i<30;++i) { out=h.process(i==15?1000.f:100.f); if(i>5)CHECK(fabs(out-100)<.01); }
  CHECK(h.spikes==1);
  ppgnr::ButterFilter<4> lp;CHECK(lp.begin(125,10,false));
  double input=0,output=0;
  for(int i=0;i<5000;++i) {float v=sinf(2*3.14159265f*10*i/125), y=lp.process(v);if(i>2500){input+=v*v;output+=y*y;}}
  CHECK(fabs(sqrt(output/input)-sqrt(.5))<.005);
  ppgnr::Sqi<125,5,1> sq;
  sq.begin(); ppgnr::SqiOut r;
  for(int i=0;i<625;++i) { r=sq.push(100,50000); CHECK(r.fresh==(i==624)); }
  CHECK(!r.good && r.fail==ppgnr::SQI_F_FLAT);
  r=sq.push(NAN,50000);CHECK(!r.good && r.fail==ppgnr::SQI_F_NONFINITE);
  ppgnr::ContactMonitor cm;cm.highForce=20;cm.onForce=5;cm.offForce=3;cm.holdSamples=3;
  CHECK(cm.update(50,0)==ppgnr::CONTACT_HIGH);CHECK(cm.update(0,0)==ppgnr::CONTACT_NONE);
  CHECK(cm.update(10,0)==ppgnr::CONTACT_OK);cm.update(10,0);cm.update(10,0);CHECK(cm.stable());
  cm.update(NAN,0);CHECK(!cm.stable());
  const float tri[]={0,1,2,1,0};
  CHECK(fabs(PulseAuc::integrate(tri,5,2,0,4)-2)<1e-6);
  CHECK(fabs(PulseAuc::integrate(tri,5,2,0,2)+PulseAuc::integrate(tri,5,2,2,4)-2)<1e-6);
  const float shifted[]={10,12,14,14,14};
  CHECK(fabs(PulseAuc::integrate(shifted,5,2,0,4)-2)<1e-6);
  std::puts("PASS: moving average, SG derivatives, spikes, filter gain, SQI, force, analytic AUC");
}
static void peaks() {
  for (float period : {1.5f,1.45f,1.f,.85f,.5f,.35f,1.f/3.f}) {
    AdaptivePeakDetector p; p.begin(125);int beats=0;double hr=0;int nh=0;
    for(int i=0;i<60*125;++i) {
      bool b=p.process(100*pulse(i/125.f,period,.45f));
      if(i>10*125 && b) {++beats;if(isfinite(p.bpm())){hr+=p.bpm();++nh;}}
    }
    std::printf("peak regression: expected %.2f bpm, observed %.2f (%d beats)\n",60/period,hr/nh,beats);
    CHECK(nh>10);CHECK(fabs(hr/nh-60/period)<2);CHECK(abs(beats-int(50/period))<=2);
    for(int i=0;i<5*125;++i) { p.process(0); }
    CHECK(!isfinite(p.bpm()));
  }
}
static void anc() {
  ppgnr::Nlms<3,8> a;a.mu=.03;a.pmin=.05;
  std::mt19937 gen(42);std::normal_distribution<float> norm;
  double before=0,after=0;float prev[3]={};
  for(int i=0;i<20000;++i) {
    float r[3];for(int j=0;j<3;++j)r[j]=.8f*prev[j]+.3f*norm(gen);
    float truth=5*sinf(i*.07f), d=truth+20*r[0]-10*r[1];float y=a.process(d,r);
    if(i>10000){before+=(d-truth)*(d-truth);after+=(y-truth)*(y-truth);}
    std::copy(r,r+3,prev);
  }
  CHECK(after<.2*before);
  ppgnr::Nlms<1,4> gated; gated.pmin=100;float ref=1;
  for(int i=0;i<100;++i)CHECK(gated.process(10,&ref)==10);
  std::puts("PASS: correlated-artifact cancellation and no-motion adaptation gate");
}
static void fullPipeline() {
  PpgPipeline pl;CHECK(pl.begin());WaveAnalyzer w;int n=0;double bpm=0;int withNotch=0;
  std::mt19937 gen(5);std::normal_distribution<float> norm;
  for(int i=0;i<80*125;++i) {
    const float t=i/125.f;
    float raw=100000-700*pulse(t,.85f,.45f)+60*sinf(2*3.14159265f*.08f*t)+2*norm(gen);
    if(i==40*125) raw+=7000;
    PpgSample s=pl.process(raw,nullptr,uint64_t(i+1)*8000);
    if(w.process(s)) {
      ++n;bpm+=w.last.bpm;CHECK(w.last.pi>0);CHECK(w.last.amp>0);
      CHECK(w.last.area>0);CHECK(w.last.pw50>0 && w.last.pw50<.6f);
      if(w.last.notchType) {++withNotch;CHECK(fabs((w.last.sysArea+w.last.diaArea)-w.last.area)<.002);}
    }
  }
  std::printf("full pipeline: %d accepted pulses, mean %.3f bpm, %d notches\n",n,bpm/n,withNotch);
  CHECK(n>55); CHECK(fabs(bpm/n-60/.85)<2);
  auto lost=pl.process(0,nullptr,81*1000000ULL);CHECK(!lost.valid && !lost.sqi_good);
  auto back=pl.process(100000,nullptr,81*1000000ULL+8000);CHECK(!back.valid);
  // Test waveform analysis rejects EVERY segment touching an immediate quality gap.
  PpgPipeline gap;gap.begin();WaveAnalyzer wg;int rejectedRange=0;
  for(int i=0;i<40*125;++i) {
    auto s=gap.process(100000-700*pulse(i/125.f),nullptr,uint64_t(i+1)*8000);
    if(i>=20*125 && i<21*125)s.sqi_good=false;
    if(wg.process(s)) {
      uint64_t end=wg.last.onsetAbs+wg.last.len-1;
      CHECK(end<20*125 || wg.last.onsetAbs>=21*125);++rejectedRange;
    }
  }
  CHECK(rejectedRange>20);
  std::puts("PASS: integrated raw-to-features chain, strong diastolic wave, spike and quality gap");
}
static void spo2Test() {
  SpO2Processor s;s.begin();int updates=0;uint64_t first=0;
  for(int i=0;i<10*125;++i) {
    float q=pulse(i/125.f,1.0f,.2f);
    s.pushSample(uint32_t(120000-6000*q),uint32_t(100000-2500*q),uint64_t(i+1)*8000,true);
    if(s.fresh){if(!first)first=uint64_t(i+1)*8000;++updates;}
  }
  CHECK(first==4000000);CHECK(updates==7);
  std::printf("SpO2 synthetic: %d%%, ratio %.3f, correlation %.3f, %d updates\n",s.getSpO2(),s.last.ratio,s.last.correlation,updates);
  CHECK(s.isSpO2Valid());CHECK(s.getSpO2()>=90);
  s.pushSample(120000,100000,10008000,false);CHECK(!s.isSpO2Valid());
  uint32_t a[100],b[100];for(int i=0;i<100;++i){a[i]=120000;b[i]=100000;}
  CHECK(!ppgspo2::calculate(a,b,99).valid);CHECK(!ppgspo2::calculate(a,b,100).valid);
  CHECK(!ppgspo2::calculate(nullptr,b,100).valid);
  // Tail plateau formerly read beyond the end of the buffer.
  for(int i=0;i<100;++i){a[i]=120000-uint32_t(3000*pulse(i/25.f,1));b[i]=100000-uint32_t(1200*pulse(i/25.f,1));}
  for(int i=90;i<100;++i){a[i]=100000;b[i]=90000;}
  (void)ppgspo2::calculate(a,b,100);
  a[25]=UINT32_MAX;CHECK(!ppgspo2::calculate(a,b,100).valid);
  std::puts("PASS: 125-to-25 Hz paired SpO2, 4-second window, one-second updates, invalidation and bounds");
}
static void trends() {
  PerfusionMonitor p;PulseFeatures f;f.valid=true;f.amp=100;f.pi=1;f.area=20;
  p.captureBaseline();for(int i=0;i<10;++i)p.push(f,uint64_t(i+1)*1000000);
  CHECK(p.baselineReady);CHECK(fabs(p.amplitudeRatio-1)<1e-6);
  PpgSample s;s.optical=s.contact=s.forceStable=s.motionQuiet=s.valid=s.sqi_good=true;
  f.amp=40;f.pi=.4;f.area=8;
  for(int i=0;i<3;++i)p.push(f,11000000+i*1000000);
  CHECK(std::string(p.status(s,13000000))=="REDUCED_PULSE_TREND");
  CHECK(std::string(p.status(s,18000000))=="NO_RELIABLE_PULSE");
  p.reset();CHECK(!p.baselineReady);
  std::puts("PASS: explicit fixed baseline, persistence and pulse timeout");
}
int main(){filters();peaks();anc();fullPipeline();spo2Test();trends();std::printf("ALL %d CHECKS PASSED\n",checks);}
