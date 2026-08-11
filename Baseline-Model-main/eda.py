"""
Step 2 : Exploratory Data Analysis on the CLEANED data.

Running EDA on the raw file is misleading: the -999 sentinels force a
spurious ~1.00 RSSI/SNR correlation. Always run this after prepare_data.py.

Run:  python3 eda.py
Out:  figures/*.png, outputs/eda_report.txt
"""
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CLEAN = "outputs/dataset_clean.csv"
CLASS_NAMES = {0: "Clear/Nominal", 1: "Jammed/PU", 2: "Severe fading"}
COLORS = {0: "#2a9d8f", 1: "#e76f51", 2: "#e9c46a"}


def main():
    df = pd.read_csv(CLEAN)
    y = df["label"]
    feats = [c for c in df.columns if c != "label"]
    lines = []
    def log(m):
        print(m); lines.append(m)

    # --- class balance ---------------------------------------------------
    fig, ax = plt.subplots(figsize=(6, 4))
    counts = y.value_counts().sort_index()
    ax.bar([CLASS_NAMES[i] for i in counts.index], counts.values,
           color=[COLORS[i] for i in counts.index])
    for i, v in enumerate(counts.values):
        ax.text(i, v + 15, str(v), ha="center", fontsize=10)
    ax.set_ylabel("samples"); ax.set_title("Class distribution (cleaned)")
    fig.tight_layout(); fig.savefig("figures/class_distribution.png", dpi=150)
    plt.close(fig)
    imb = counts.max() / counts.min()
    log(f"Class balance ratio (max/min) = {imb:.3f}  "
        f"-> {'balanced, no resampling needed' if imb < 1.5 else 'IMBALANCED'}")

    # --- histograms per class --------------------------------------------
    ncol = 4
    nrow = int(np.ceil(len(feats) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(4 * ncol, 2.8 * nrow))
    for ax, f in zip(axes.ravel(), feats):
        for c in sorted(y.unique()):
            ax.hist(df.loc[y == c, f], bins=40, alpha=.55,
                    label=CLASS_NAMES[c], color=COLORS[c])
        ax.set_title(f, fontsize=10)
    for ax in axes.ravel()[len(feats):]:
        ax.axis("off")
    axes.ravel()[0].legend(fontsize=8)
    fig.suptitle("Feature distributions by class", y=1.005)
    fig.tight_layout(); fig.savefig("figures/histograms.png", dpi=150,
                                    bbox_inches="tight")
    plt.close(fig)

    # --- boxplots ---------------------------------------------------------
    fig, axes = plt.subplots(nrow, ncol, figsize=(4 * ncol, 2.8 * nrow))
    for ax, f in zip(axes.ravel(), feats):
        data = [df.loc[y == c, f].values for c in sorted(y.unique())]
        bp = ax.boxplot(data, patch_artist=True,
                        tick_labels=[str(c) for c in sorted(y.unique())])
        for patch, c in zip(bp["boxes"], sorted(y.unique())):
            patch.set_facecolor(COLORS[c])
        ax.set_title(f, fontsize=10); ax.set_xlabel("class", fontsize=8)
    for ax in axes.ravel()[len(feats):]:
        ax.axis("off")
    fig.suptitle("Feature boxplots by class", y=1.005)
    fig.tight_layout(); fig.savefig("figures/boxplots.png", dpi=150,
                                    bbox_inches="tight")
    plt.close(fig)

    # --- correlation matrix ----------------------------------------------
    corr = df[feats].corr()
    fig, ax = plt.subplots(figsize=(9, 7.5))
    im = ax.imshow(corr, cmap="RdBu_r", vmin=-1, vmax=1)
    ax.set_xticks(range(len(feats))); ax.set_xticklabels(feats, rotation=90)
    ax.set_yticks(range(len(feats))); ax.set_yticklabels(feats)
    for i in range(len(feats)):
        for j in range(len(feats)):
            ax.text(j, i, f"{corr.iloc[i, j]:.2f}", ha="center", va="center",
                    fontsize=7,
                    color="white" if abs(corr.iloc[i, j]) > .6 else "black")
    fig.colorbar(im); ax.set_title("Correlation matrix (cleaned data)")
    fig.tight_layout(); fig.savefig("figures/correlation_matrix.png", dpi=150)
    plt.close(fig)

    # --- highly correlated pairs -----------------------------------------
    log("\nHighly correlated feature pairs (|r| > 0.85):")
    hi = []
    for i in range(len(feats)):
        for j in range(i + 1, len(feats)):
            r = corr.iloc[i, j]
            if abs(r) > .85:
                hi.append((feats[i], feats[j], r))
                log(f"   {feats[i]:10s} ~ {feats[j]:10s}  r = {r:+.3f}")
    if not hi:
        log("   none")
    log("   (RF is robust to collinearity for accuracy, but it SPLITS the")
    log("    importance between correlated twins -> use permutation")
    log("    importance, not just Gini, when reading Step 9.)")

    # --- pairplot on the top discriminative features ----------------------
    key = ["meanRSSI", "meanSNR", "PLR", "varRSSI"]
    fig, axes = plt.subplots(len(key), len(key), figsize=(11, 10))
    for i, fi in enumerate(key):
        for j, fj in enumerate(key):
            ax = axes[i, j]
            if i == j:
                for c in sorted(y.unique()):
                    ax.hist(df.loc[y == c, fi], bins=30, alpha=.55,
                            color=COLORS[c])
            else:
                for c in sorted(y.unique()):
                    ax.scatter(df.loc[y == c, fj], df.loc[y == c, fi], s=3,
                               alpha=.35, color=COLORS[c])
            if i == len(key) - 1: ax.set_xlabel(fj, fontsize=9)
            if j == 0: ax.set_ylabel(fi, fontsize=9)
            ax.tick_params(labelsize=6)
    fig.suptitle("Pairplot - key features by class", y=.995)
    fig.tight_layout(); fig.savefig("figures/pairplot.png", dpi=150)
    plt.close(fig)

    log("\nFigures written to figures/")
    with open("outputs/eda_report.txt", "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
