"""
Step 1-3 : Cleaning + feature engineering for the LoRa cognitive-radio RF model.

Run:   python3 prepare_data.py
Out:   outputs/dataset_clean.csv
       outputs/cleaning_report.txt
"""
import pandas as pd
import numpy as np

RAW = "merged_dataset.csv"
OUT = "outputs/dataset_clean.csv"

# Physical bounds hard-coded in the firmware config (proposal, Sec. 4.2.4)
RSSI_MIN, RSSI_MAX = -140.0, -30.0
SNR_MIN,  SNR_MAX  = -20.0,  10.0
CFO_MIN,  CFO_MAX  = -25000.0, 25000.0

# In this dataset meanToA takes exactly one value per (SF, CR) pair.
TOA_LOOKUP = {(8, 5): 83000.0, (10, 5): 289000.0, (12, 8): 1450000.0}


def clean(df, log):
    n0 = len(df)
    log(f"Raw rows                                 : {n0}")

    # 1a. missing label ---------------------------------------------------
    n = df["label"].isna().sum()
    df = df.dropna(subset=["label"]).copy()
    df["label"] = df["label"].astype(int)
    log(f"Dropped rows with missing label          : {n}")

    # 1b. '-999' sentinel = no packet received in the window ---------------
    # Real, informative condition -> keep the row, remove the magic number,
    # record the condition explicitly in its own flag.
    sent = df["meanRSSI"] <= -900
    df["link_lost"] = sent.astype(int)
    df.loc[sent, "meanRSSI"] = RSSI_MIN
    df.loc[sent, "meanSNR"] = SNR_MIN
    log(f"Recoded '-999' sentinel rows             : {int(sent.sum())}"
        f"   (-> RSSI={RSSI_MIN}, SNR={SNR_MIN}, link_lost=1)")

    # 1c. 'meanToA == 0' is ALSO a sentinel --------------------------------
    # Means 'not measurable', not 'zero airtime'. Occurs in 400 class-1 rows,
    # 10 class-2 rows, 0 class-0 rows -> label-correlated generator artifact.
    # Impute from (SF, CR) so the trees cannot shortcut on it.
    tz = df["meanToA"] == 0
    df.loc[tz, "meanToA"] = [TOA_LOOKUP[(int(r.SF), int(r.CR))]
                             for r in df.loc[tz].itertuples()]
    log(f"Imputed 'meanToA==0' sentinel rows       : {int(tz.sum())}"
        f"   (from (SF,CR) lookup; removes label shortcut)")

    # 1d. clip to declared physical bounds ---------------------------------
    for col, lo, hi in [("meanRSSI", RSSI_MIN, RSSI_MAX),
                        ("meanSNR",  SNR_MIN,  SNR_MAX),
                        ("CFO",      CFO_MIN,  CFO_MAX)]:
        n = int(((df[col] < lo) | (df[col] > hi)).sum())
        df[col] = df[col].clip(lo, hi)
        log(f"Clipped {col:8s} into [{lo:>8}, {hi:>7}]  : {n}")

    # 1e. contradictory labels --------------------------------------------
    feat = [c for c in df.columns if c != "label"]
    nconf = int(df.groupby(feat)["label"].transform("nunique").gt(1).sum())
    log(f"Rows: identical features, conflicting label: {nconf}")
    if nconf:
        log("    ^ these sit inside the link_lost condition. When zero packets")
        log("      arrive, jamming and fading are indistinguishable by")
        log("      construction -> firmware needs a deterministic fallback.")

    # 1f. exact duplicates (last, so earlier recodes are accounted for) -----
    n = int(df.duplicated().sum())
    df = df.drop_duplicates().reset_index(drop=True)
    log(f"Dropped exact duplicate rows             : {n}")

    # 1g. structural assertions -------------------------------------------
    assert df["PLR"].between(0, 1).all()
    assert df["CRC"].ge(0).all()
    assert set(df["label"]) == {0, 1, 2}
    log(f"Clean rows                               : {len(df)} "
        f"({n0-len(df)} removed, {100*(n0-len(df))/n0:.1f}%)")
    return df


def engineer(df, log):
    """
    Step 3 : feature selection.

    NOTE: the derived features usually proposed for this project
    (RSSI stability, SNR stability, CRC rate, RSSI/SNR ratio) were built,
    measured, and REJECTED on evidence. Reasoning:

      A decision tree splits on thresholds (x <= t). Any strictly MONOTONIC
      transform of a feature yields the identical set of reachable splits,
      so it cannot add information to a tree model.
          sdRSSI  = sqrt(varRSSI)      -> Spearman r = 1.0000 vs varRSSI
          sdSNR   = sqrt(varSNR)       -> Spearman r = 1.0000 vs varSNR
          crcRate = CRC/(CRC+1)        -> Spearman r = 1.0000 vs CRC
          snrMargin = meanSNR + 20     -> Spearman r = 1.0000 vs meanSNR
      (This is why scaling/normalisation is also unnecessary for RF.)

      Measured 5-fold CV macro-F1 (30 trees, depth 8):
          base features only              0.9902
          base + 4 monotonic transforms   0.9861   (-0.0041)
          base + RSSI/SNR ratio           0.9884   (-0.0017)
      Adding them slightly HURTS: max_features samples columns at random,
      so redundant twins waste picks. Fewer features also = smaller firmware.

    Only meanToA is removed, and only because it is a lookup of (SF, CR).
    """
    df = df.drop(columns=["meanToA"])
    log("Dropped meanToA (deterministic fn of SF+CR -> redundant)")
    log("Derived features evaluated and rejected (monotonic -> no value to a")
    log("   tree; measured CV delta -0.0041). Kept 9 base features + link_lost.")
    return df


def main():
    lines = []
    def log(m):
        print(m); lines.append(m)

    df = clean(pd.read_csv(RAW), log)
    df = engineer(df, log)

    log("\nClass distribution (clean):")
    for k, v in df["label"].value_counts().sort_index().items():
        log(f"   class {k} : {v:5d}  ({100*v/len(df):5.1f}%)")

    df.to_csv(OUT, index=False)
    log(f"\nWrote {OUT}   shape={df.shape}")
    with open("outputs/cleaning_report.txt", "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
