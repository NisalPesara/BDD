// ANC test with two accelerometers (finger + band). Compile twice to compare reference modes:
//   g++ -O2 -std=c++17 -I.. -DANC_MODE=0 anc_test.cpp -o a0 && ./a0     (ABS6: raw axes of both sensors)
//   g++ -O2 -std=c++17 -I.. -DANC_MODE=1 anc_test.cpp -o a1 && ./a1     (REL9: finger acc + relative acc + relative velocity)
// Synthetic model: band = hand motion h;  finger = h + f (finger moves relative to hand).
//   scenario 1: artifact ~ RELATIVE VELOCITY (f integrated)     scenario 2: artifact ~ ABSOLUTE finger acceleration
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include "../pipeline.h"
static float rnd(){return (rand()/(float)RAND_MAX)-0.5f;}
static float gauss(float t,float mu,float sd){return expf(-0.5f*powf((t-mu)/sd,2));}
static double corr(std::vector<float>&x,std::vector<float>&y){double mx=0,my=0;int n=x.size();for(int i=0;i<n;i++){mx+=x[i];my+=y[i];}mx/=n;my/=n;double sxy=0,sxx=0,syy=0;for(int i=0;i<n;i++){sxy+=(x[i]-mx)*(y[i]-my);sxx+=(x[i]-mx)*(x[i]-mx);syy+=(y[i]-my)*(y[i]-my);}return sxy/sqrt(sxx*syy);}
static void run(int scen){
  const float fs=PPG_FS; const int N=120*PPG_FS; srand(7+scen);
  PipelineConfig off; off.useAnc=false; PipelineConfig on; on.useAnc=true; PipelineConfig ref; ref.useAnc=false;
  PpgPipeline pOff,pOn,pRef; pOff.begin(off); pOn.begin(on); pRef.begin(ref);
  float hl[3]={0},fl[3]={0},vt[3]={0}; float wv[3]={250,-180,120}, wa[3]={200,-140,90};
  float leak=expf(-6.2831853f*0.3f/fs);
  std::vector<float> y0,y1,yr;
  for(int i=0;i<N;i++){
    float t=i/fs; float ph=fmodf(t*1.15f,1.0f);
    float pulse=gauss(ph,0.2f,0.07f)+0.45f*gauss(ph,0.55f,0.10f);
    bool mot=(t>30&&t<110);
    float h[3],f[3],fa[3];
    for(int k=0;k<3;k++){ hl[k]+=0.15f*((mot?rnd()*6.f:0)-hl[k]); fl[k]+=0.15f*((mot?rnd()*6.f:0)-fl[k]);
      h[k]=hl[k]*3.f; f[k]=fl[k]*3.f; fa[k]=h[k]+f[k]; vt[k]=vt[k]*leak+f[k]/fs; }
    float acc[6]; for(int k=0;k<3;k++){ float g=(k==2)?9.81f:0; acc[k]=fa[k]+g; acc[3+k]=h[k]+g; }
    float m=0; for(int k=0;k<3;k++) m+= (scen==1)? wv[k]*vt[k]*ACC_VEL_SCALE : wa[k]*fa[k];
    float clean=60000-500*pulse;
    float raw=clean+m+3*rnd(), rawClean=clean+3*rnd();
    PpgSample s0=pOff.process(raw,acc,i*8), s1=pOn.process(raw,acc,i*8), sr=pRef.process(rawClean,acc,i*8);
    if(t>90&&t<110){ y0.push_back(s0.ppg); y1.push_back(s1.ppg); yr.push_back(sr.ppg);}
  }
  printf("  scenario %d (%s): ANC off %.3f -> ANC on %.3f\n",scen,scen==1?"artifact ~ relative velocity":"artifact ~ absolute finger accel",corr(y0,yr),corr(y1,yr));
}
int main(){ printf("ANC_MODE=%d (%s), refs=%d, taps=%d\ncorrelation with motion-free signal, t=90-110 s:\n",ANC_MODE,ANC_MODE==2?"REL6":ANC_MODE==1?"REL9":"ABS6",ANC_NUM_REFS,ANC_TAPS); run(1); run(2); }
