/* AUTO-GENERATED — do not edit by hand.
 * Baseline rule-based classifier for LoRa cognitive radio.
 *   0 = Clear/Nominal      -> no action
 *   1 = Jammed/PU active   -> hop channel
 *   2 = Severe link fading -> SF+1, CR=4/8
 * features=10, rules use 4 (meanSNR, meanRSSI, PLR, CRC)
 * Verified decision-identical to Python on all dataset rows.
 */
#ifndef BASELINE_MODEL_H
#define BASELINE_MODEL_H
#include <stdint.h>

/* feature vector index map — fill x[] in THIS order */
#define F_meanRSSI 0
#define F_varRSSI 1
#define F_meanSNR 2
#define F_varSNR 3
#define F_CFO 4
#define F_PLR 5
#define F_CRC 6
#define F_SF 7
#define F_CR 8
#define F_link_lost 9
#define BL_N_FEATURES 10

/* tuned thresholds */
#define TH_SNR_CLEAR    3.0f
#define TH_PLR_CLEAR    0.12f
#define TH_PLR_JAMMED   0.50f
#define TH_RSSI_FADING  -118.0f
#define TH_SNR_FADING   2.0f

static uint8_t baseline_predict(const float *x) {
    /* link lost -> hop (deterministic fallback) */
    if (x[F_link_lost] >= 0.5f) return 1;

    /* Rule 1: healthy signal -> clear */
    if (x[F_meanSNR] > TH_SNR_CLEAR && x[F_PLR] < TH_PLR_CLEAR)
        return 0;

    /* Rule 2: CRC failures or very high PLR -> jammed */
    if (x[F_CRC] > 0.0f || x[F_PLR] > TH_PLR_JAMMED)
        return 1;

    /* Rule 3: very weak signal, no CRC -> fading */
    if (x[F_meanRSSI] < TH_RSSI_FADING && x[F_meanSNR] < TH_SNR_FADING
        && x[F_CRC] <= 0.0f)
        return 2;

    /* Rule 4: ambiguous -> default jammed */
    return 1;
}

#endif /* BASELINE_MODEL_H */
