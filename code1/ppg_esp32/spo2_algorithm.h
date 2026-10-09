/** \file spo2_algorithm.h **************************************************
* Header-only SpO2 & heart-rate calculation (Maxim Integrated reference design
* algorithm, MAXREFDES117), CORRECTED. Original algorithm (c) Maxim Integrated;
* keep Maxim's license notice when you redistribute.
*
* Input : FreqS-Hz raw IR and RED counts (DC + AC, NOT filtered/inverted/derivative),
*         exactly BUFFER_SIZE = 4 s of samples.
*         The MAX30102 runs at 125 Hz in this project: m11_spo2_processor.h low-passes
*         and decimates it 5:1 to 25 Hz before calling this function.
*
* Corrections to the version that was in the project (see README "SpO2 algorithm"):
*  1. n_x_ac used the RED max index (n_y_dc_max_idx) -> now n_x_dc_max_idx (IR AC at the IR peak).
*  2. valley boundary check '> BUFFER_SIZE' let index == BUFFER_SIZE through -> now '>= BUFFER_SIZE'.
*  3. peak search read pn_x[i + n_width] one element past the end -> bounds checked.
*  4. MA4 smoothing left the last 3 samples unsmoothed but they were still used for the
*     threshold and the valley search -> only the BUFFER_SIZE-MA4_SIZE+1 smoothed samples are used,
*     valley index shifted to the window centre and refined on the raw IR minimum.
*  5. n_ir_buffer_length was ignored (BUFFER_SIZE used everywhere) -> a wrong length is rejected.
*  6. 32-bit products (AC x DC can exceed 2^31 with 18-bit data) -> 64-bit.
*  7. "median" averaged the wrong pair for odd counts -> true median.
*  8. the table has 183 entries but was declared [184] (last entry silently 0 -> SpO2 0 %) ->
*     size taken from the table itself.
*  9. minimum valley distance 4 samples (160 ms = 375 bpm, catches dicrotic notches) -> FreqS*60/MAX_BPM.
* 10. negative AC (no real pulse between two valleys) produced negative ratios -> skipped.
* 11. "[cite: ..]" text after #include broke compilation -> removed.
******************************************************************************/
#ifndef SPO2_ALGORITHM_H_
#define SPO2_ALGORITHM_H_

#include <stdint.h>
#include "config.h"

#define FreqS        SPO2_FS                  // 25 Hz, derived from PPG_FS / SPO2_DECIMATION in config.h
#define BUFFER_SIZE  (FreqS * SPO2_WINDOW_S)  // 100 samples = 4 s
#define MA4_SIZE     4                        // moving average size (do not change)
#define SPO2_MAX_PEAKS 15
#define SPO2_MIN_PEAK_DIST ((FreqS * 60) / (int)MAX_BPM)   // 8 samples at 25 Hz / 180 bpm

// Lookup table: SpO2 = table[round(100 * R)], R = (ACred/DCred) / (ACir/DCir)
// (Maxim's empirical calibration ~ 104 - 17R for its fingertip reference design).
const uint8_t uch_spo2_table[] = {
    95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
    99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
    97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
    90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
    80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
    66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
    49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
    28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
    3, 2, 1
};
static const int32_t SPO2_TABLE_SIZE = (int32_t)(sizeof(uch_spo2_table) / sizeof(uch_spo2_table[0]));

static int32_t an_x[BUFFER_SIZE];   // working array for IR (single-threaded use only)
static int32_t an_y[BUFFER_SIZE];   // working array for RED

inline void maxim_sort_ascend(int32_t *pn_x, int32_t n_size) {
  for (int32_t i = 1; i < n_size; i++) {
    const int32_t n_temp = pn_x[i]; int32_t j;
    for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--) pn_x[j] = pn_x[j - 1];
    pn_x[j] = n_temp;
  }
}

inline void maxim_sort_indices_descend(const int32_t *pn_x, int32_t *pn_indx, int32_t n_size) {
  for (int32_t i = 1; i < n_size; i++) {
    const int32_t n_temp = pn_indx[i]; int32_t j;
    for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--) pn_indx[j] = pn_indx[j - 1];
    pn_indx[j] = n_temp;
  }
}

// Local maxima above n_min_height (plateaus count once). FIX 3: never reads past n_size-1.
inline void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *n_npks, const int32_t *pn_x,
                                         int32_t n_size, int32_t n_min_height) {
  int32_t i = 1;
  *n_npks = 0;
  while (i < n_size - 1) {
    if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1]) {
      int32_t n_width = 1;
      while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width]) n_width++;
      if (i + n_width < n_size && pn_x[i] > pn_x[i + n_width] && (*n_npks) < SPO2_MAX_PEAKS) {
        pn_locs[(*n_npks)++] = i;
        i += n_width + 1;
      } else {
        i += n_width;
      }
    } else {
      i++;
    }
  }
}

// Keep the strongest peaks, drop any peak closer than n_min_distance to a stronger one.
inline void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *n_npks, const int32_t *pn_x, int32_t n_min_distance) {
  maxim_sort_indices_descend(pn_x, pn_locs, *n_npks);
  for (int32_t i = -1; i < *n_npks; i++) {
    const int32_t n_old_npks = *n_npks;
    *n_npks = i + 1;
    for (int32_t j = i + 1; j < n_old_npks; j++) {
      const int32_t n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]);
      if (n_dist > n_min_distance || n_dist < -n_min_distance) pn_locs[(*n_npks)++] = pn_locs[j];
    }
  }
  maxim_sort_ascend(pn_locs, *n_npks);
}

inline void maxim_find_peaks(int32_t *pn_locs, int32_t *n_npks, const int32_t *pn_x, int32_t n_size,
                             int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num) {
  maxim_peaks_above_min_height(pn_locs, n_npks, pn_x, n_size, n_min_height);
  maxim_remove_close_peaks(pn_locs, n_npks, pn_x, n_min_distance);
  if (*n_npks > n_max_num) *n_npks = n_max_num;
}

// Outputs: *pn_spo2 (%), *pch_spo2_valid, *pn_heart_rate (bpm), *pch_hr_valid. -999 = not available.
// Optional *pn_ratio_x100 = median 100*R (for logging), may be nullptr.
inline void maxim_heart_rate_and_oxygen_saturation(
    const uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length,
    const uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid,
    int32_t *pn_heart_rate, int8_t *pch_hr_valid, int32_t *pn_ratio_x100 = nullptr) {

  *pn_spo2 = -999; *pch_spo2_valid = 0; *pn_heart_rate = -999; *pch_hr_valid = 0;
  if (pn_ratio_x100) *pn_ratio_x100 = -999;
  if (!pun_ir_buffer || !pun_red_buffer || n_ir_buffer_length != BUFFER_SIZE) return;   // FIX 5

  // ---- 1. IR valleys = peaks of the inverted, DC-removed, 4-point smoothed IR ----
  int64_t sum = 0;
  for (int32_t k = 0; k < BUFFER_SIZE; k++) sum += pun_ir_buffer[k];
  const int32_t un_ir_mean = (int32_t)(sum / BUFFER_SIZE);
  for (int32_t k = 0; k < BUFFER_SIZE; k++) an_x[k] = -((int32_t)pun_ir_buffer[k] - un_ir_mean);

  const int32_t n_smooth = BUFFER_SIZE - MA4_SIZE + 1;                 // FIX 4: valid smoothed samples
  for (int32_t k = 0; k < n_smooth; k++)
    an_x[k] = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / 4;

  int32_t n_th1 = 0;
  for (int32_t k = 0; k < n_smooth; k++) n_th1 += an_x[k];
  n_th1 /= n_smooth;
  if (n_th1 < 30) n_th1 = 30;
  if (n_th1 > 60) n_th1 = 60;

  int32_t an_ir_valley_locs[SPO2_MAX_PEAKS] = {0};
  int32_t n_npks = 0;
  maxim_find_peaks(an_ir_valley_locs, &n_npks, an_x, n_smooth, n_th1, SPO2_MIN_PEAK_DIST, SPO2_MAX_PEAKS);

  // MA4 output k is centred at k+1.5: refine each valley on the raw IR minimum in [k, k+3]
  for (int32_t k = 0; k < n_npks; k++) {
    const int32_t lo = an_ir_valley_locs[k], hi = lo + MA4_SIZE - 1;
    int32_t best = lo;
    for (int32_t i = lo; i <= hi && i < BUFFER_SIZE; i++) if (pun_ir_buffer[i] < pun_ir_buffer[best]) best = i;
    an_ir_valley_locs[k] = best;
  }

  // ---- 2. heart rate from the mean valley interval ----
  if (n_npks >= 2) {
    const int32_t span = an_ir_valley_locs[n_npks - 1] - an_ir_valley_locs[0];
    if (span > 0) {
      *pn_heart_rate = (int32_t)((60 * FreqS * (n_npks - 1) + span / 2) / span);   // rounded
      *pch_hr_valid = (*pn_heart_rate >= (int32_t)MIN_BPM && *pn_heart_rate <= (int32_t)MAX_BPM) ? 1 : 0;
    }
  }

  // ---- 3. ratio of ratios, beat by beat between consecutive valleys ----
  for (int32_t k = 0; k < BUFFER_SIZE; k++) { an_x[k] = (int32_t)pun_ir_buffer[k]; an_y[k] = (int32_t)pun_red_buffer[k]; }

  for (int32_t k = 0; k < n_npks; k++)
    if (an_ir_valley_locs[k] < 0 || an_ir_valley_locs[k] >= BUFFER_SIZE) return;            // FIX 2

  int32_t an_ratio[5] = {0}, n_i_ratio_count = 0;
  for (int32_t k = 0; k < n_npks - 1; k++) {
    const int32_t v0 = an_ir_valley_locs[k], v1 = an_ir_valley_locs[k + 1], dv = v1 - v0;
    if (dv <= 3) continue;
    int32_t n_x_dc_max = -16777216, n_y_dc_max = -16777216, n_x_dc_max_idx = v0, n_y_dc_max_idx = v0;
    for (int32_t i = v0; i < v1; i++) {
      if (an_x[i] > n_x_dc_max) { n_x_dc_max = an_x[i]; n_x_dc_max_idx = i; }
      if (an_y[i] > n_y_dc_max) { n_y_dc_max = an_y[i]; n_y_dc_max_idx = i; }
    }
    // AC = peak minus the straight valley-to-valley line at the peak position (64-bit, FIX 6)
    int64_t n_y_ac = (int64_t)(an_y[v1] - an_y[v0]) * (n_y_dc_max_idx - v0);
    n_y_ac = an_y[v0] + n_y_ac / dv;
    n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;
    int64_t n_x_ac = (int64_t)(an_x[v1] - an_x[v0]) * (n_x_dc_max_idx - v0);
    n_x_ac = an_x[v0] + n_x_ac / dv;
    n_x_ac = an_x[n_x_dc_max_idx] - n_x_ac;                                                 // FIX 1
    if (n_y_ac <= 0 || n_x_ac <= 0) continue;                                               // FIX 10
    const int64_t n_nume  = n_y_ac * n_x_dc_max;
    const int64_t n_denom = n_x_ac * n_y_dc_max;
    if (n_denom > 0 && n_i_ratio_count < 5) {
      an_ratio[n_i_ratio_count++] = (int32_t)((n_nume * 100) / n_denom);
    }
  }
  if (n_i_ratio_count == 0) return;

  maxim_sort_ascend(an_ratio, n_i_ratio_count);                                              // FIX 7
  const int32_t mid = n_i_ratio_count / 2;
  const int32_t n_ratio_average = (n_i_ratio_count & 1) ? an_ratio[mid] : (an_ratio[mid - 1] + an_ratio[mid]) / 2;
  if (pn_ratio_x100) *pn_ratio_x100 = n_ratio_average;

  if (n_ratio_average > 2 && n_ratio_average < SPO2_TABLE_SIZE) {                            // FIX 8
    *pn_spo2 = uch_spo2_table[n_ratio_average];
    *pch_spo2_valid = 1;
  }
}

#endif /* SPO2_ALGORITHM_H_ */
