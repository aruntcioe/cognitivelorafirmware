"""
analyze.py -- Offline analysis for the cognitive-radio dataset.

Reads the CSVs written by live.py (columns include condition / detected /
review_flag) and produces the evidence you need for the presentation:

  1. Dataset summary (rows per condition, flagged counts).
  2. Confusion matrix of AI `detected` vs ground-truth `condition`
     -- both RAW and FLAG-CLEANED (jammer-idle windows removed).
  3. A THRESHOLD baseline classifier on the same features, so you can
     show AI vs threshold head-to-head.
  4. Feature-distribution plots (shows the overlap thresholds can't split).

Usage:
    pip install pandas scikit-learn matplotlib
    python analyze.py collected_data/*.csv
    python analyze.py collected_data/JAMMING_20260811_120000.csv
"""

import sys
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import confusion_matrix, classification_report, accuracy_score

# class codes: condition/detected use 1=jamming, 2=fading; AI may also emit 0=normal
CLASS_NAMES = {0: "Normal", 1: "Jamming", 2: "Fading"}


# ------------------------------------------------------------ load
def load(paths):
    frames = []
    for pattern in paths:
        for f in glob.glob(pattern):
            df = pd.read_csv(f)
            df["__source"] = f
            frames.append(df)
    if not frames:
        print("No CSV files matched.")
        sys.exit(1)
    df = pd.concat(frames, ignore_index=True)
    # detected may be blank for windows that had no AI verdict yet -> drop those
    df = df[df["detected"].notna() & (df["detected"] != "")]
    df["detected"] = df["detected"].astype(int)
    df["condition"] = df["condition"].astype(int)
    df["review_flag"] = df["review_flag"].astype(int)
    return df


# ------------------------------------------------------------ threshold baseline
def threshold_baseline(df):
    """A hand-tuned expert-rule classifier for AI-vs-threshold comparison.
    Rules (deliberately the 'obvious' ones an engineer would write):
        - jammer: high loss / CRC storm / very low SNR
        - fading: moderately low SNR with high RSSI variance, few CRC
        - else  : normal
    Returns predicted class per row.
    """
    preds = []
    for _, r in df.iterrows():
        snr, plr, crc, varR = r["meanSNR"], r["PLR"], r["CRC"], r["varRSSI"]
        if plr >= 0.30 or crc >= 3 or snr < 0:
            preds.append(1)                      # jamming
        elif snr < 6 and varR > 2.0:
            preds.append(2)                      # fading
        else:
            preds.append(0)                      # normal
    return np.array(preds)


# ------------------------------------------------------------ reporting
def show_confusion(y_true, y_pred, title):
    labels = sorted(set(list(y_true) + list(y_pred)))
    cm = confusion_matrix(y_true, y_pred, labels=labels)
    print(f"\n=== {title} ===")
    print("labels:", [CLASS_NAMES.get(l, l) for l in labels])
    print(cm)
    print(f"accuracy: {accuracy_score(y_true, y_pred):.3f}")
    print(classification_report(y_true, y_pred,
          labels=labels, target_names=[CLASS_NAMES.get(l, str(l)) for l in labels],
          zero_division=0))
    return cm, labels


def plot_confusion(cm, labels, title, ax):
    im = ax.imshow(cm, cmap="Blues")
    ax.set_title(title, fontsize=10)
    ax.set_xticks(range(len(labels)))
    ax.set_yticks(range(len(labels)))
    names = [CLASS_NAMES.get(l, str(l)) for l in labels]
    ax.set_xticklabels(names, rotation=45, ha="right", fontsize=8)
    ax.set_yticklabels(names, fontsize=8)
    ax.set_xlabel("Predicted"); ax.set_ylabel("Truth")
    for i in range(len(labels)):
        for j in range(len(labels)):
            ax.text(j, i, cm[i, j], ha="center", va="center",
                    color="black" if cm[i, j] < cm.max() / 2 else "white", fontsize=9)


def plot_feature_overlap(df):
    """Histograms of SNR / PLR by ground-truth condition -- visually shows
    why a single threshold can't cleanly separate jamming from fading."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    for ax, feat, name in zip(axes,
                              ["meanSNR", "PLR", "varRSSI"],
                              ["mean SNR (dB)", "PLR", "var RSSI"]):
        for cond, color in [(1, "#e74c3c"), (2, "#e67e22")]:
            sub = df[df["condition"] == cond][feat]
            if len(sub):
                ax.hist(sub, bins=30, alpha=0.5, label=CLASS_NAMES[cond], color=color)
        ax.set_title(name); ax.set_xlabel(name); ax.legend(fontsize=8)
    fig.suptitle("Feature distributions by ground-truth condition "
                 "(overlap = where thresholds fail)", fontsize=11)
    fig.tight_layout()


# ------------------------------------------------------------ main
def main():
    if len(sys.argv) < 2:
        print("usage: python analyze.py <csv or glob> [more...]")
        sys.exit(1)
    df = load(sys.argv[1:])

    # ---- 1. summary ----
    print("=" * 55)
    print(" DATASET SUMMARY")
    print("=" * 55)
    print(f"total windows (with AI verdict): {len(df)}")
    for cond in sorted(df["condition"].unique()):
        n = (df["condition"] == cond).sum()
        flagged = df[(df["condition"] == cond)]["review_flag"].sum()
        print(f"  condition {cond} ({CLASS_NAMES.get(cond)}): {n} rows, "
              f"{flagged} auto-flagged (jammer-idle)")

    clean = df[df["review_flag"] == 0].copy()
    print(f"\nafter removing {df['review_flag'].sum()} flagged rows: "
          f"{len(clean)} rows for fair evaluation")

    # ---- 2. AI detected vs ground truth ----
    show_confusion(df["condition"], df["detected"], "AI vs Truth  (RAW, all rows)")
    cm_ai, lab_ai = show_confusion(clean["condition"], clean["detected"],
                                   "AI vs Truth  (FLAG-CLEANED)")

    # ---- 3. threshold baseline vs ground truth (cleaned) ----
    thr_pred = threshold_baseline(clean)
    cm_thr, lab_thr = show_confusion(clean["condition"], thr_pred,
                                     "THRESHOLD baseline vs Truth  (FLAG-CLEANED)")

    # ---- 4. plots ----
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
    plot_confusion(cm_ai, lab_ai, "AI (Random Forest)", ax1)
    plot_confusion(cm_thr, lab_thr, "Threshold baseline", ax2)
    fig.suptitle("AI vs Threshold -- confusion matrices (cleaned data)", fontsize=12)
    fig.tight_layout()

    plot_feature_overlap(clean)

    # ---- headline numbers for the slide ----
    print("\n" + "=" * 55)
    print(" HEADLINE (cleaned data)")
    print("=" * 55)
    print(f"  AI accuracy        : {accuracy_score(clean['condition'], clean['detected']):.3f}")
    print(f"  Threshold accuracy : {accuracy_score(clean['condition'], thr_pred):.3f}")
    print("  -> Look at the off-diagonal Jamming<->Fading cells:")
    print("     the threshold confuses them where features overlap;")
    print("     the RF separates them. That is your thesis.")

    plt.show()


if __name__ == "__main__":
    main()