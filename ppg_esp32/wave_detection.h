// ======================================================================
//  WAVE-PROPERTIES FEATURE EXTRACTION (per-pulse analysis)
//  The pipeline calls waveDetectionProcess() once per sample (125 Hz) with the
//  purified signal. Only analyses samples where s.valid && s.sqi_good.
//  Buffers one complete pulse (beat-to-beat), then extracts fiducial points
//  and computes morphological features matching the original ppg:: namespace.
//
//  Inputs available in `s`:
//     s.ppg (clean PPG), s.vpg, s.apg, s.jpg (derivatives), s.upslope,
//     s.beat / s.bpm (Shin-2009 adaptive peak detector), s.sqi.*, s.t_ms, s.raw
// ======================================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include "pipeline.h"
#include "config.h"

// Declared in ppg_esp32.ino — posture accelerometer readings
extern float lastAccelX, lastAccelY, lastAccelZ;

class WaveAnalyzer {
private:
    static const int MAX_PULSE_LEN = 250;   // up to 2 s at 125 Hz (~30 bpm minimum)
    PpgSample buf[MAX_PULSE_LEN];
    int count;
    bool inPulse;

    // Slow DC tracker for perfusion index (tracks raw IR level)
    float dcLevel;
    bool dcInit;

    // Report counter
    int reportNum;

    // ---------- Helper: print one feature line ----------
    static void printFeature(const char* name, float value, const char* unit) {
        if (isnan(value)) Serial.printf("%-38s %10s %s\n", name, "NaN", unit);
        else              Serial.printf("%-38s %10.5f %s\n", name, (double)value, unit);
    }

    // ---------- Find PPG fiducial points ----------
    // onset = start of pulse (index 0 by definition — buffer starts at beat boundary)
    // peak  = systolic peak (global max)
    // notch = dicrotic notch (local minimum between peak and end)
    // offset = end of pulse (last index)
    struct PpgFid { int onset, peak, notch, offset; };

    PpgFid findPpgFiducials() {
        PpgFid p = {0, 0, count - 1, count - 1};
        // Systolic peak = global maximum
        for (int i = 1; i < count; i++)
            if (buf[i].ppg > buf[p.peak].ppg) p.peak = i;
        // Dicrotic notch = minimum between peak and end
        if (p.peak + 2 < count) {
            p.notch = p.peak + 1;
            for (int i = p.peak + 2; i < count; i++)
                if (buf[i].ppg < buf[p.notch].ppg) p.notch = i;
        } else {
            p.notch = p.offset;   // fallback: no notch found
        }
        return p;
    }

    // ---------- Find VPG fiducial points ----------
    // u = max upstroke velocity, v = max downstroke velocity, w = secondary peak after v
    // z1, z2 = 1st and 2nd downward zero-crossings of VPG
    struct VpgFid { int u, v, w, z1, z2; };

    VpgFid findVpgFiducials(const PpgFid& ppg) {
        VpgFid f = {0, 0, 0, -1, -1};
        // u: maximum VPG between onset and peak
        for (int i = 0; i <= ppg.peak && i < count; i++)
            if (buf[i].vpg > buf[f.u].vpg) f.u = i;
        // v: minimum VPG between u and offset
        f.v = f.u;
        for (int i = f.u + 1; i < count; i++)
            if (buf[i].vpg < buf[f.v].vpg) f.v = i;
        // w: first local maximum of VPG after v
        f.w = f.v;
        for (int i = f.v + 1; i + 1 < count; i++)
            if (buf[i].vpg >= buf[i - 1].vpg && buf[i].vpg > buf[i + 1].vpg) { f.w = i; break; }
        // Downward zero-crossings
        int crossings = 0;
        for (int i = 1; i < count && crossings < 2; i++) {
            if (buf[i - 1].vpg >= 0.0f && buf[i].vpg < 0.0f) {
                if (crossings == 0) f.z1 = i;
                else                f.z2 = i;
                crossings++;
            }
        }
        return f;
    }

    // ---------- Find APG fiducial points (a-b-c-d-e-f waves, Takazawa 1998) ----------
    struct ApgFid { int a, b, c, d, e, f; };

    int apgExtremum(int start, int end, bool maximum) {
        if (start < 0 || end < 0 || start >= count || end >= count || start > end) return start < count ? start : 0;
        int result = start;
        for (int i = start + 1; i <= end && i < count; i++)
            if ((maximum && buf[i].apg > buf[result].apg) || (!maximum && buf[i].apg < buf[result].apg))
                result = i;
        return result;
    }

    ApgFid findApgFiducials(const PpgFid& ppg) {
        ApgFid f;
        int mid = (ppg.peak + ppg.offset) / 2;
        f.a = apgExtremum(ppg.onset, ppg.peak, true);        // 'a': max before systolic peak
        f.b = apgExtremum(f.a, mid, false);                   // 'b': min after 'a'
        f.e = apgExtremum(ppg.peak, ppg.offset, true);        // 'e': max after systolic peak
        f.c = apgExtremum(f.b, f.e, true);                    // 'c': max between 'b' and 'e'
        f.d = apgExtremum(f.c, f.e, false);                   // 'd': min between 'c' and 'e'
        f.f = apgExtremum(f.e, ppg.offset, false);            // 'f': min after 'e'
        return f;
    }

    // ---------- Trapezoidal area between two indices (above baseline) ----------
    float areaAboveBaseline(int start, int end) {
        if (start < 0 || end <= start || end >= count) return NAN;
        float baseline = buf[start].ppg;
        float dt = 1.0f / PPG_FS;
        float area = 0.0f;
        for (int i = start; i < end; i++)
            area += 0.5f * ((buf[i].ppg - baseline) + (buf[i + 1].ppg - baseline)) * dt;
        return area;
    }

    // ---------- Pulse Width at 50% amplitude ----------
    float computePW50(const PpgFid& ppg) {
        float onsetVal = buf[ppg.onset].ppg;
        float amp = buf[ppg.peak].ppg - onsetVal;
        if (amp <= 0.0f) return NAN;
        float level = onsetVal + 0.5f * amp;
        float dt = 1.0f / PPG_FS;
        int left = ppg.onset, right = ppg.offset;
        for (int i = ppg.onset; i <= ppg.peak; i++)
            if (buf[i].ppg >= level) { left = i; break; }
        for (int i = ppg.peak; i <= ppg.offset; i++)
            if (buf[i].ppg <= level) { right = i; break; }
        return (right - left) * dt;
    }

    // ---------- Analyse one buffered pulse and print the report ----------
    void analyzePulse() {
        if (count < 15) return;   // too short to be a real pulse

        PpgFid ppg = findPpgFiducials();
        VpgFid vpg = findVpgFiducials(ppg);
        ApgFid apg = findApgFiducials(ppg);

        float dt = 1.0f / PPG_FS;

        // ---- PPG morphological features ----
        float sysAmp = buf[ppg.peak].ppg - buf[ppg.onset].ppg;

        // Perfusion index: AC/DC * 100 (AC = peak-to-trough of clean PPG; DC = raw IR level)
        float lo = buf[0].ppg, hi = buf[0].ppg;
        for (int i = 1; i < count; i++) { if (buf[i].ppg < lo) lo = buf[i].ppg; if (buf[i].ppg > hi) hi = buf[i].ppg; }
        float pi = (fabsf(dcLevel) > 1e-6f) ? 100.0f * (hi - lo) / fabsf(dcLevel) : NAN;

        float pw50 = computePW50(ppg);

        float pulseArea    = areaAboveBaseline(ppg.onset, ppg.offset);
        float systolicArea = areaAboveBaseline(ppg.onset, ppg.notch);
        float diastolicArea = areaAboveBaseline(ppg.notch, ppg.offset);
        float ipa = (fabsf(diastolicArea) > 1e-6f) ? systolicArea / diastolicArea : NAN;

        // Augmentation index: (reflection point - notch) / systolic peak amplitude
        float aix = NAN;
        if (apg.e >= 0 && ppg.notch >= 0 && ppg.peak >= 0 && fabsf(buf[ppg.peak].ppg) > 1e-6f)
            aix = (buf[apg.e].ppg - buf[ppg.notch].ppg) / buf[ppg.peak].ppg;

        // ---- VPG timing features ----
        float crestTimeSec = (vpg.z1 >= 0) ? (vpg.z1 - ppg.onset) * dt : NAN;
        float deltaTSec    = (vpg.z1 >= 0 && vpg.z2 > vpg.z1) ? (vpg.z2 - vpg.z1) * dt : NAN;

        // Large artery stiffness index: body_height / deltaT  (default height 1.7 m)
        float lasi = (!isnan(deltaTSec) && deltaTSec > 1e-6f) ? 1.7f / deltaTSec : NAN;

        // ---- SDPTG ratios (APG a-b-c-d-e waves) ----
        float aVal = buf[apg.a].apg;
        float bOverA = NAN, cOverA = NAN, dOverA = NAN, eOverA = NAN;
        float bMinusEOverA = NAN, bMinusCMinusDOverA = NAN, agingIdx = NAN;

        if (fabsf(aVal) > 1e-6f) {
            float bVal = buf[apg.b].apg;
            float cVal = buf[apg.c].apg;
            float dVal = buf[apg.d].apg;
            float eVal = buf[apg.e].apg;
            bOverA = bVal / aVal;
            cOverA = cVal / aVal;
            dOverA = dVal / aVal;
            eOverA = eVal / aVal;
            bMinusEOverA      = (bVal - eVal) / aVal;
            bMinusCMinusDOverA = (bVal - cVal - dVal) / aVal;
            agingIdx           = (bVal - cVal - dVal - eVal) / aVal;
        }

        // Pulse-to-pulse interval (duration of this pulse in seconds)
        float ppiSec = count * dt;

        // ---- Print formatted report ----
        reportNum++;
        Serial.println();
        Serial.println("==================================================");
        Serial.printf( "       PULSE ANALYSIS REPORT #%d  (%.1f BPM)\n", reportNum, buf[0].bpm);
        Serial.println("==================================================");
        Serial.printf("ADXL345 Posture: X: %.2f | Y: %.2f | Z: %.2f\n", lastAccelX, lastAccelY, lastAccelZ);
        Serial.printf("Pulse length: %d samples (%.0f ms)\n", count, ppiSec * 1000.0f);

        // ---- FIDUCIAL POINTS (indices) ----
        Serial.println("\n-- Fiducial Points (sample index) --");
        Serial.printf("  p->onset:              %d\n",   ppg.onset);
        Serial.printf("  p->peak:               %d\n",   ppg.peak);
        Serial.printf("  p->notch:              %d\n",   ppg.notch);
        Serial.printf("  p->offset:             %d\n",   ppg.offset);
        Serial.printf("  p->u (VPG max):        %d\n",   vpg.u);
        Serial.printf("  p->v (VPG min):        %d\n",   vpg.v);
        Serial.printf("  p->w (VPG 2nd peak):   %d\n",   vpg.w);
        Serial.printf("  p->zero1 (VPG cross):  %d\n",   vpg.z1);
        Serial.printf("  p->zero2 (VPG cross):  %d\n",   vpg.z2);
        Serial.printf("  p->a (APG):            %d\n",   apg.a);
        Serial.printf("  p->b (APG):            %d\n",   apg.b);
        Serial.printf("  p->c (APG):            %d\n",   apg.c);
        Serial.printf("  p->d (APG):            %d\n",   apg.d);
        Serial.printf("  p->e (APG):            %d\n",   apg.e);
        Serial.printf("  p->f (APG):            %d\n",   apg.f);

        // ---- FIDUCIAL POINT SIGNAL VALUES ----
        Serial.println("\n-- Fiducial Point Values --");
        Serial.printf("  PPG[onset]:            %.5f\n",  buf[ppg.onset].ppg);
        Serial.printf("  PPG[peak]:             %.5f\n",  buf[ppg.peak].ppg);
        Serial.printf("  PPG[notch]:            %.5f\n",  buf[ppg.notch].ppg);
        Serial.printf("  PPG[offset]:           %.5f\n",  buf[ppg.offset].ppg);
        Serial.printf("  VPG[u]:                %.5f\n",  buf[vpg.u].vpg);
        Serial.printf("  VPG[v]:                %.5f\n",  buf[vpg.v].vpg);
        Serial.printf("  VPG[w]:                %.5f\n",  buf[vpg.w].vpg);
        Serial.printf("  APG[a]:                %.5f\n",  buf[apg.a].apg);
        Serial.printf("  APG[b]:                %.5f\n",  buf[apg.b].apg);
        Serial.printf("  APG[c]:                %.5f\n",  buf[apg.c].apg);
        Serial.printf("  APG[d]:                %.5f\n",  buf[apg.d].apg);
        Serial.printf("  APG[e]:                %.5f\n",  buf[apg.e].apg);
        Serial.printf("  APG[f]:                %.5f\n",  buf[apg.f].apg);

        // ---- TIMING FEATURES ----
        Serial.println("\n-- Timing Features --");
        printFeature("p->crestTimeSeconds",              crestTimeSec,    "s");
        printFeature("p->deltaTSeconds",                 deltaTSec,       "s");
        printFeature("r->pulseToPulseIntervalSeconds",   ppiSec,          "s");

        // ---- COMPUTED FEATURES ----
        Serial.println("\n-- Morphological Features --");
        printFeature("r->systolicAmplitude",             sysAmp,          "a.u.");
        printFeature("r->perfusionIndexPercent",         pi,              "%");
        printFeature("r->pulseWidth50Seconds",           pw50,            "s");
        printFeature("r->pulseArea",                     pulseArea,       "a.u.*s");
        printFeature("r->systolicArea",                  systolicArea,    "a.u.*s");
        printFeature("r->diastolicArea",                 diastolicArea,   "a.u.*s");
        printFeature("r->inflectionPointAreaRatio",      ipa,             "ratio");
        printFeature("r->augmentationIndex",             aix,             "ratio");
        printFeature("r->deltaTSeconds",                 deltaTSec,       "s");
        printFeature("r->largeArteryStiffnessIndex",     lasi,            "m/s");
        printFeature("r->pulseTransitTimeSeconds",       NAN,             "s");

        // ---- SDPTG RATIOS ----
        Serial.println("\n-- SDPTG Ratios (APG) --");
        printFeature("r->bOverA",                        bOverA,          "ratio");
        printFeature("r->cOverA",                        cOverA,          "ratio");
        printFeature("r->dOverA",                        dOverA,          "ratio");
        printFeature("r->eOverA",                        eOverA,          "ratio");
        printFeature("r->bMinusEOverA",                  bMinusEOverA,    "ratio");
        printFeature("r->bMinusCMinusDOverA",            bMinusCMinusDOverA, "ratio");
        printFeature("r->agingIndex",                    agingIdx,        "ratio");

        Serial.println("==================================================\n");
    }

public:
    WaveAnalyzer() : count(0), inPulse(false), dcLevel(0), dcInit(false), reportNum(0) {}

    void begin() {
        count = 0;
        inPulse = false;
        dcLevel = 0;
        dcInit = false;
        reportNum = 0;
    }

    void process(const PpgSample& s) {
        if (!s.valid || !s.sqi_good) return;

        // Track DC level from raw IR for perfusion index
        if (!dcInit) { dcLevel = s.raw; dcInit = true; }
        else dcLevel += 0.001f * (s.raw - dcLevel);   // very slow tracker (~0.02 Hz at 125 Hz)

        // Beat flag marks the boundary between pulses
        if (s.beat) {
            if (inPulse && count >= 15) {
                analyzePulse();                 // analyse the PREVIOUS complete pulse
            }
            inPulse = true;
            count = 0;
        }

        // Buffer the current pulse's samples
        if (inPulse && count < MAX_PULSE_LEN) {
            buf[count++] = s;
        }
    }
};

inline void waveDetectionProcess(const PpgSample& s) {
    static WaveAnalyzer analyzer;
    analyzer.process(s);
}
