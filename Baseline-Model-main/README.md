# Baseline Rule-Based Model — LoRa Cognitive Radio

Offline evaluation pipeline for the **rule-based threshold baseline**,
structured identically to the Random Forest (AI) project so the two
can be compared side by side.

## Run it

```bash
bash run_all.sh          # cleaning -> EDA -> train -> export -> verify
```

Deterministic (`seed=42`), identical 80/20 split as the RF project.

| Script | Purpose |
|---|---|
| `prepare_data.py` | Cleaning + feature selection (shared with RF project) |
| `eda.py` | EDA plots (shared with RF project) |
| `train_baseline.py` | Threshold tuning, evaluation, CV, feature analysis |
| `export_firmware.py` | Emit `baseline_model.h`, verify against Python |
| `test_baseline_model.c` | Replay every row through compiled C |

## Results

| Metric | Baseline (rules) | Random Forest (AI) |
|---|---|---|
| Test accuracy | **93.22 %** | **98.66 %** |
| Test macro-F1 | **93.31 %** | **98.66 %** |
| Critical (1↔2) error | **3.77 % ❌** | **0.87 % ✅** |
| 5-fold CV macro-F1 | 0.9416 | 0.9901 |
| Safety gate (<1%) | FAIL | PASS |
| Features used | 4 / 10 | 10 / 10 |
| Firmware size | ~1.5 KB | ~73 KB |

## The rules

```
if SNR > 3.0  and PLR < 0.12           → Class 0 (clear)
if CRC > 0    or  PLR > 0.50           → Class 1 (jammed)
if RSSI < -118 and SNR < 2 and CRC==0  → Class 2 (fading)
else                                    → Class 1 (default)
```

## Why this baseline matters

This is what the ESP32 would run **without** any AI — hand-tuned
threshold rules. The comparison demonstrates that:

1. Fixed rules achieve ~93% accuracy but **fail the safety gate**
   (3.77% critical error, nearly 4× worse than the RF's 0.87%)
2. The failure is **structural**, not a tuning problem — even with
   thresholds optimized on the training set, rules cannot capture
   the non-linear jam/fading boundary
3. Rules use only 4 of 10 features; the RF leverages all 10 and
   their interactions
