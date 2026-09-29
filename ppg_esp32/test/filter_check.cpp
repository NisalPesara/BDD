#include <cstdio>
#include <cmath>
#include "../m01_frequency_filters.h"
int main(){ SosFilter f; selectFrequencyFilter(f,FM_CHEBY2_PYPPG);
 for(int i=0;i<1250;i++){ float t=i/125.0f; float x=sinf(6.2831853f*1.2f*t)+0.5f*sinf(6.2831853f*2.4f*t)+0.7f*sinf(6.2831853f*0.1f*t)+0.3f*sinf(6.2831853f*40*t); printf("%.7f %.7f\n",x,f.process(x)); } }
