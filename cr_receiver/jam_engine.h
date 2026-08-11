/***********************************************************************
 * jam_engine.h  —  Cognitive Jamming-Response Engine (Receiver side)
 *
 * A self-contained multi-armed-bandit channel/SF selector with memory.
 *
 * ARM SPACE:
 *   frequency ∈ {433000..434500 kHz, 250 kHz spacing}  (7 channels)
 *   SF        ∈ {7,8,9,10,11,12}                         (6 SFs)
 *   => 42 arms, each remembering its own quality history.
 *
 * WHAT IT DOES
 *   - Records the outcome (PLR / CRC) of every window into the CURRENT arm.
 *   - On a jammer verdict, blacklists the current arm for a cooldown and
 *     selects the best-scoring alternative arm, preferring:
 *        * historically clean channels        (EWMA of packet-loss)
 *        * a CHANGE of SF (breaks same-SF collision — the dominant kill)
 *        * frequencies FAR from the last-jammed frequency (never walk
 *          blindly toward the jammer)
 *        * arms not used recently             (anti-thrash)
 *   - Epsilon-greedy exploration keeps the map fresh so recovered
 *     channels come back into use after a jammer moves away.
 *   - Blacklists and scores DECAY over time -> self-healing.
 *
 * USAGE (receiver .ino):
 *   #include "jam_engine.h"
 *   setup():          jamEngineInit(currentFrequencyKHz, currentSF);
 *   every window:     jamEngineRecordWindow(features.PLR,
 *                                           features.consecutiveCRCFailures,
 *                                           currentFrequencyKHz, currentSF);
 *   executeDecision() case 1:
 *       uint32_t f; uint8_t sf;
 *       jamEngineSelect(currentFrequencyKHz, currentSF, &f, &sf);
 *       beginControlCommand(CMD_CHANNEL_HOP, f, sf, currentCR);
 *
 *   (Reset streak on non-jam windows: jamEngineResetStreak();)
 ***********************************************************************/

#ifndef JAM_ENGINE_H
#define JAM_ENGINE_H

#include <Arduino.h>

// ======================= ARM SPACE DEFINITION ========================
static const uint32_t JE_FREQS[] = {
    433000, 433250, 433500, 433750, 434000, 434250, 434500
};
static const uint8_t  JE_N_FREQ = sizeof(JE_FREQS) / sizeof(JE_FREQS[0]);

static const uint8_t  JE_SFS[]   = { 7, 8, 9, 10, 11, 12 };
static const uint8_t  JE_N_SF    = sizeof(JE_SFS) / sizeof(JE_SFS[0]);

// ======================= TUNABLE WEIGHTS =============================
// Scoring weights (higher score = better arm to pick).
static const float JE_W_CLEAN   = 1.00f;  // reward low historical PLR
static const float JE_W_SFCHG   = 0.80f;  // reward changing SF (break collision)
static const float JE_W_FARJAM  = 0.60f;  // reward distance from last-jammed freq
static const float JE_W_RECENCY = 0.40f;  // penalize recently-used arms

static const float    JE_EWMA_ALPHA      = 0.30f;   // PLR smoothing
static const float    JE_EPSILON         = 0.10f;   // exploration probability
static const uint32_t JE_BLACKLIST_MS    = 45000UL; // cooldown after a bad arm
static const uint32_t JE_RECENCY_WINDOW  = 20000UL; // "recent" horizon for penalty
static const float    JE_JAM_PLR_THRESH  = 0.30f;   // PLR above this = treat as jammed
static const uint16_t JE_JAM_CRC_THRESH  = 3;       // CRC fails above this = jammed

// ======================= PER-ARM MEMORY =============================
struct JE_Arm {
    float    ewmaPLR;        // smoothed packet-loss (init optimistic = 0)
    uint16_t crcFails;       // last recorded CRC failures on this arm
    uint32_t lastUsedMs;     // recency
    uint16_t timesTried;
    uint32_t blacklistUntil; // 0 = not blacklisted
};

static JE_Arm    je_arms[JE_N_FREQ][JE_N_SF];
static uint32_t  je_lastJammedFreqKHz = 0;   // where we most recently got jammed
static uint8_t   je_jamStreak         = 0;   // consecutive jam verdicts

// ======================= INDEX HELPERS ==============================
static int je_freqIndex(uint32_t freqKHz) {
    int best = 0; uint32_t bestD = 0xFFFFFFFF;
    for (uint8_t i = 0; i < JE_N_FREQ; i++) {
        uint32_t d = (JE_FREQS[i] > freqKHz) ? (JE_FREQS[i] - freqKHz)
                                             : (freqKHz - JE_FREQS[i]);
        if (d < bestD) { bestD = d; best = i; }
    }
    return best;   // nearest legal channel to an arbitrary current freq
}
static int je_sfIndex(uint8_t sf) {
    for (uint8_t i = 0; i < JE_N_SF; i++) if (JE_SFS[i] == sf) return i;
    return 0;
}

// ======================= INIT ======================================
static inline void jamEngineInit(uint32_t startFreqKHz, uint8_t startSF) {
    for (uint8_t f = 0; f < JE_N_FREQ; f++)
        for (uint8_t s = 0; s < JE_N_SF; s++) {
            je_arms[f][s].ewmaPLR        = 0.0f;  // optimistic start
            je_arms[f][s].crcFails       = 0;
            je_arms[f][s].lastUsedMs     = 0;
            je_arms[f][s].timesTried     = 0;
            je_arms[f][s].blacklistUntil = 0;
        }
    (void)startFreqKHz; (void)startSF;
    je_lastJammedFreqKHz = 0;
    je_jamStreak = 0;
    randomSeed(micros());
}

// ================== RECORD A WINDOW OUTCOME =========================
// Call once per completed feature window with the CURRENT arm's stats.
static inline void jamEngineRecordWindow(int aiClass, float plr, uint16_t crcFails,
                                         uint32_t curFreqKHz, uint8_t curSF) {
    int fi = je_freqIndex(curFreqKHz);
    int si = je_sfIndex(curSF);
    JE_Arm &a = je_arms[fi][si];
    
    a.ewmaPLR = JE_EWMA_ALPHA * plr + (1-JE_EWMA_ALPHA) * a.ewmaPLR;  // quality metric only
    
    if (aiClass == 1) {                 // AI said "jammer" — not a threshold
        je_lastJammedFreqKHz = curFreqKHz;
        a.blacklistUntil = millis() + JE_BLACKLIST_MS;
    }
}

static inline void jamEngineResetStreak() { je_jamStreak = 0; }
static inline uint8_t jamEngineStreak()    { return je_jamStreak; }

// ======================= SCORING ===================================
static float je_scoreArm(uint8_t fi, uint8_t si,
                         uint32_t curFreqKHz, uint8_t curSF, uint32_t nowMs) {
    JE_Arm &a = je_arms[fi][si];

    // Hard exclusion: blacklisted (in cooldown) arms are unusable.
    if (a.blacklistUntil != 0 && (int32_t)(nowMs - a.blacklistUntil) < 0)
        return -1e9f;

    // 1) Cleanliness: reward low smoothed PLR.
    float sClean = JE_W_CLEAN * (1.0f - a.ewmaPLR);

    // 2) SF change: reward moving OFF the current SF (breaks collision).
    //    Larger SF gap = stronger orthogonality, so scale by the gap.
    uint8_t sfGap = abs((int)JE_SFS[si] - (int)curSF);
    float sSF = JE_W_SFCHG * (sfGap == 0 ? 0.0f
                             : (0.5f + 0.1f * (float)sfGap));

    // 3) Distance from the last-jammed frequency: reward being FAR from
    //    the jammer; never reward walking toward it.
    float sFar = 0.0f;
    if (je_lastJammedFreqKHz != 0) {
        uint32_t d = (JE_FREQS[fi] > je_lastJammedFreqKHz)
                       ? (JE_FREQS[fi] - je_lastJammedFreqKHz)
                       : (je_lastJammedFreqKHz - JE_FREQS[fi]);
        // normalize by full band span (~1.5 MHz) -> 0..1
        sFar = JE_W_FARJAM * ((float)d / 1500000.0f);
    }

    // 4) Recency penalty: discourage thrashing the same arm repeatedly.
    float sRec = 0.0f;
    uint32_t age = nowMs - a.lastUsedMs;
    if (a.lastUsedMs != 0 && age < JE_RECENCY_WINDOW)
        sRec = -JE_W_RECENCY * (1.0f - (float)age / (float)JE_RECENCY_WINDOW);

    return sClean + sSF + sFar + sRec;
}

// ======================= SELECT NEXT ARM ===========================
// Fills *outFreq / *outSF with the chosen arm. Increments jam streak.
// Returns true always (a config is always chosen).
static inline bool jamEngineSelect(uint32_t curFreqKHz, uint8_t curSF,
                                   uint32_t *outFreq, uint8_t *outSF) {
    je_jamStreak++;
    uint32_t nowMs = millis();

    // Quarantine the current (failing) arm immediately.
    int cfi = je_freqIndex(curFreqKHz);
    int csi = je_sfIndex(curSF);
    je_arms[cfi][csi].blacklistUntil = nowMs + JE_BLACKLIST_MS;
    je_lastJammedFreqKHz = curFreqKHz;

    // --- Epsilon-greedy exploration: occasionally pick a random valid arm ---
    if ((random(1000) / 1000.0f) < JE_EPSILON) {
        for (uint8_t tries = 0; tries < 20; tries++) {
            uint8_t fi = random(JE_N_FREQ);
            uint8_t si = random(JE_N_SF);
            JE_Arm &a = je_arms[fi][si];
            bool bl = (a.blacklistUntil != 0 &&
                       (int32_t)(nowMs - a.blacklistUntil) < 0);
            if (!bl && !(JE_FREQS[fi] == curFreqKHz && JE_SFS[si] == curSF)) {
                *outFreq = JE_FREQS[fi];
                *outSF   = JE_SFS[si];
                a.lastUsedMs = nowMs;
                return true;
            }
        }
        // fall through to greedy if exploration couldn't find a free arm
    }

    // --- Greedy exploit: highest-scoring non-blacklisted arm ---
    float bestScore = -1e18f;
    int   bestFi = -1, bestSi = -1;
    for (uint8_t fi = 0; fi < JE_N_FREQ; fi++)
        for (uint8_t si = 0; si < JE_N_SF; si++) {
            if (JE_FREQS[fi] == curFreqKHz && JE_SFS[si] == curSF) continue;
            float sc = je_scoreArm(fi, si, curFreqKHz, curSF, nowMs);
            if (sc > bestScore) { bestScore = sc; bestFi = fi; bestSi = si; }
        }

    // --- Degraded mode: every arm blacklisted / no candidate. ---
    // The jammer is likely sweeping or following. Fall back to the arm
    // FARTHEST in frequency from the last jam, at maximum robustness,
    // and clear the oldest blacklist so we don't lock up permanently.
    if (bestFi < 0) {
        uint32_t bestD = 0; int ffi = 0;
        for (uint8_t fi = 0; fi < JE_N_FREQ; fi++) {
            uint32_t d = (JE_FREQS[fi] > je_lastJammedFreqKHz)
                           ? (JE_FREQS[fi] - je_lastJammedFreqKHz)
                           : (je_lastJammedFreqKHz - JE_FREQS[fi]);
            if (d >= bestD) { bestD = d; ffi = fi; }
        }
        *outFreq = JE_FREQS[ffi];
        *outSF   = 12;                 // max processing gain + robustness
        // relieve the map so future selects have options again
        for (uint8_t fi = 0; fi < JE_N_FREQ; fi++)
            for (uint8_t si = 0; si < JE_N_SF; si++)
                je_arms[fi][si].blacklistUntil = 0;
        je_arms[ffi][je_sfIndex(12)].lastUsedMs = nowMs;
        return true;
    }

    // Escalation nudge: if we've been jammed repeatedly, bias toward
    // maximum robustness on the chosen channel.
    uint8_t chosenSF = JE_SFS[bestSi];
    if (je_jamStreak >= 3 && chosenSF < 12) chosenSF = 12;

    *outFreq = JE_FREQS[bestFi];
    *outSF   = chosenSF;
    je_arms[bestFi][bestSi].lastUsedMs = nowMs;
    return true;
}

#endif // JAM_ENGINE_H