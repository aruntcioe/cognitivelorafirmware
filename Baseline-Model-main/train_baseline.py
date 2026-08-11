"""
Baseline Rule-Based Model : train (tune thresholds), evaluate, report.

Mirrors train_model.py from the RF project so the two can be compared
directly. Uses the IDENTICAL cleaned dataset and the IDENTICAL 80/20
stratified split (seed=42).

Run:  python3 train_baseline.py
Out:  outputs/baseline_model.pkl, outputs/model_meta.json,
      outputs/training_report.txt, figures/{confusion_matrix,baseline_rules}.png
"""
import json
import numpy as np
import pandas as pd
import joblib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sklearn.model_selection import (train_test_split, StratifiedKFold,
                                     cross_val_score)
from sklearn.base import BaseEstimator, ClassifierMixin
from sklearn.metrics import (accuracy_score, f1_score, classification_report,
                             confusion_matrix)

CLEAN = "outputs/dataset_clean.csv"
SEED = 42
CLASS_NAMES = {0: "Clear/Nominal", 1: "Jammed/PU", 2: "Severe fading"}

LINES = []
def log(m=""):
    print(m); LINES.append(str(m))


# =====================================================================
#  RULE-BASED CLASSIFIER (sklearn-compatible for cross_val_score)
# =====================================================================

class RuleBasedClassifier(BaseEstimator, ClassifierMixin):
    """
    Fixed if/else rules using SNR, RSSI, PLR, and CRC.
    Wrapped as a sklearn estimator so we can use cross_val_score,
    classification_report, and all the same evaluation tools.

    Parameters (thresholds) can be set at init or tuned via grid search.
    """
    def __init__(self, snr_clear=5.0, plr_clear=0.10, plr_jammed=0.50,
                 rssi_fading=-120.0, snr_fading=0.0):
        self.snr_clear   = snr_clear
        self.plr_clear   = plr_clear
        self.plr_jammed  = plr_jammed
        self.rssi_fading = rssi_fading
        self.snr_fading  = snr_fading

    def fit(self, X, y=None):
        """No-op — rules don't learn. Required by sklearn API."""
        self.classes_ = np.array([0, 1, 2])
        self.feature_names_ = list(X.columns) if hasattr(X, 'columns') else None
        return self

    def predict(self, X):
        if isinstance(X, pd.DataFrame):
            return np.array([self._classify_row(row) for _, row in X.iterrows()])
        else:
            cols = self.feature_names_ or [
                'meanRSSI','varRSSI','meanSNR','varSNR','CFO',
                'PLR','CRC','SF','CR','link_lost']
            df = pd.DataFrame(X, columns=cols)
            return np.array([self._classify_row(row) for _, row in df.iterrows()])

    def _classify_row(self, row):
        snr  = row["meanSNR"]
        rssi = row["meanRSSI"]
        plr  = row["PLR"]
        crc  = row["CRC"]

        # link lost -> hop (deterministic fallback)
        if "link_lost" in row and row["link_lost"] == 1:
            return 1

        # Rule 1: healthy signal -> clear
        if snr > self.snr_clear and plr < self.plr_clear:
            return 0

        # Rule 2: CRC failures or very high packet loss -> jammed
        if crc > 0 or plr > self.plr_jammed:
            return 1

        # Rule 3: very weak signal, no CRC errors -> fading
        if rssi < self.rssi_fading and snr < self.snr_fading and crc == 0:
            return 2

        # Rule 4: degraded but ambiguous -> default jammed
        return 1


def critical_error(y_true, y_pred):
    cm = confusion_matrix(y_true, y_pred, labels=[0, 1, 2])
    return (cm[1][2] + cm[2][1]) / (cm[1].sum() + cm[2].sum())


def tune_thresholds(Xtr, ytr, cv):
    """Sweep thresholds on training data to find the best fixed rules."""
    log("sweeping thresholds on training data (no test leakage)...")
    best_f1, best_params = 0, None
    count = 0
    for snr_th in np.arange(3.0, 8.0, 1.0):
        for plr_c in [0.04, 0.08, 0.12]:
            for plr_j in [0.15, 0.30, 0.50]:
                for rssi_f in [-128, -124, -120, -118]:
                    for snr_f in [-6, -3, 0, 2]:
                        m = RuleBasedClassifier(
                            snr_clear=snr_th, plr_clear=plr_c,
                            plr_jammed=plr_j, rssi_fading=rssi_f,
                            snr_fading=snr_f)
                        s = cross_val_score(m, Xtr, ytr, cv=cv,
                                            scoring='f1_macro', n_jobs=-1)
                        if s.mean() > best_f1:
                            best_f1 = s.mean()
                            best_params = dict(snr_clear=snr_th, plr_clear=plr_c,
                                               plr_jammed=plr_j, rssi_fading=rssi_f,
                                               snr_fading=snr_f)
                        count += 1
    log(f"evaluated {count} threshold combinations")
    log(f"best CV macro-F1: {best_f1:.4f}")
    return best_params, best_f1


def main():
    df = pd.read_csv(CLEAN)
    feats = [c for c in df.columns if c != "label"]
    X, y = df[feats], df["label"]

    log("=" * 68)
    log("STEP 1 : TRAIN / TEST SPLIT")
    log("=" * 68)
    Xtr, Xte, ytr, yte = train_test_split(
        X, y, test_size=.20, stratify=y, random_state=SEED)
    log(f"features ({len(feats)}) : {feats}")
    log(f"train {Xtr.shape}   test {Xte.shape}   stratified, seed={SEED}")
    log("(identical split to the Random Forest project)")

    cv = StratifiedKFold(5, shuffle=True, random_state=SEED)

    # ----- Naive baseline (no tuning) ------------------------------------
    log()
    log("=" * 68)
    log("STEP 2 : NAIVE EXPERT RULES (domain knowledge only)")
    log("=" * 68)
    naive = RuleBasedClassifier()
    naive.fit(Xtr, ytr)
    naive_pred = naive.predict(Xte)
    log("thresholds (from radio engineering domain knowledge):")
    log(f"  if SNR > {naive.snr_clear} and PLR < {naive.plr_clear}      -> clear")
    log(f"  if CRC > 0 or PLR > {naive.plr_jammed}                 -> jammed")
    log(f"  if RSSI < {naive.rssi_fading} and CRC == 0             -> fading")
    log(f"  else                                        -> jammed (default)")
    log(f"\ntest accuracy  : {accuracy_score(yte, naive_pred):.4f}")
    log(f"test macro-F1  : {f1_score(yte, naive_pred, average='macro'):.4f}")
    log(f"critical error : {critical_error(yte, naive_pred)*100:.2f}%")

    # ----- Optimized baseline (tuned on training set) --------------------
    log()
    log("=" * 68)
    log("STEP 3 : THRESHOLD TUNING (on training set only)")
    log("=" * 68)
    best_params, best_cv = tune_thresholds(Xtr, ytr, cv)
    log(f"\noptimal thresholds:")
    for k, v in best_params.items():
        log(f"  {k:15s} = {v}")

    model = RuleBasedClassifier(**best_params)
    model.fit(Xtr, ytr)

    log()
    log("=" * 68)
    log("STEP 4 : EVALUATION (on held-out test set)")
    log("=" * 68)
    pred = model.predict(Xte)
    log(f"test accuracy  : {accuracy_score(yte, pred):.4f}")
    log(f"test macro-F1  : {f1_score(yte, pred, average='macro'):.4f}")
    log(f"\n{classification_report(yte, pred, digits=3, target_names=[CLASS_NAMES[i] for i in [0,1,2]])}")

    cm = confusion_matrix(yte, pred, labels=[0, 1, 2])
    log("confusion matrix (rows=actual, cols=predicted):")
    log(str(cm))

    # safety gate
    log("\n--- SAFETY GATE : class 1 <-> class 2 cross-misclassification ---")
    j_as_f, f_as_j = int(cm[1][2]), int(cm[2][1])
    n1, n2 = int(cm[1].sum()), int(cm[2].sum())
    log(f"jammed  -> predicted fading : {j_as_f} / {n1}  ({100*j_as_f/n1:.2f}%)")
    log(f"fading  -> predicted jammed : {f_as_j} / {n2}  ({100*f_as_j/n2:.2f}%)")
    gate = (j_as_f + f_as_j) / (n1 + n2)
    log(f"combined critical error rate: {gate*100:.2f}%   "
        f"{'PASS (<1%)' if gate < .01 else 'FAIL (>1%)'}")

    # confusion matrix figure
    fig, ax = plt.subplots(figsize=(6.2, 5.2))
    im = ax.imshow(cm, cmap="Oranges")
    ticks = [CLASS_NAMES[i] for i in [0, 1, 2]]
    ax.set_xticks(range(3)); ax.set_xticklabels(ticks, rotation=20)
    ax.set_yticks(range(3)); ax.set_yticklabels(ticks)
    for i in range(3):
        for j in range(3):
            ax.text(j, i, cm[i, j], ha="center", va="center", fontsize=13,
                    color="white" if cm[i, j] > cm.max() / 2 else "black")
    ax.set_xlabel("predicted"); ax.set_ylabel("actual")
    ax.set_title("Confusion matrix — Baseline Rule-Based Model (test set)")
    fig.colorbar(im); fig.tight_layout()
    fig.savefig("figures/confusion_matrix.png", dpi=150); plt.close(fig)

    log()
    log("=" * 68)
    log("STEP 5 : CROSS-VALIDATION")
    log("=" * 68)
    for k in (5, 10):
        s = cross_val_score(model, X, y, cv=StratifiedKFold(k, shuffle=True,
                            random_state=SEED), scoring="f1_macro", n_jobs=-1)
        log(f"{k:2d}-fold macro-F1 : {s.mean():.4f} +/- {s.std():.4f}   "
            f"folds {np.round(s, 4)}")

    log()
    log("=" * 68)
    log("STEP 6 : FEATURE USAGE ANALYSIS")
    log("=" * 68)
    log("The rule-based model uses only 4 of the 10 features:")
    log("  USED    : meanSNR, meanRSSI, PLR, CRC")
    log("  IGNORED : varRSSI, varSNR, CFO, SF, CR, link_lost")
    log("(link_lost triggers a deterministic fallback, not a threshold rule)")
    log("")
    log("This is the fundamental limitation: fixed rules cannot combine")
    log("features non-linearly. The Random Forest uses all 10 features")
    log("and captures interactions between them (e.g., low SNR + high")
    log("variance + low CRC -> fading, not jamming).")

    # rules visualization
    fig, ax = plt.subplots(figsize=(8, 5))
    used = ['meanSNR', 'PLR', 'CRC', 'meanRSSI']
    unused = ['varRSSI', 'varSNR', 'CFO', 'SF', 'CR', 'link_lost']
    all_f = used + unused
    colors = ['#e76f51' if f in used else '#ccc' for f in all_f]
    ax.barh(all_f, [1]*len(used) + [0]*len(unused), color=colors)
    ax.set_xlabel("used by baseline (1 = yes)")
    ax.set_title("Features used by baseline rules (4 / 10)")
    for i, f in enumerate(all_f):
        ax.text(0.5, i, "USED" if f in used else "ignored",
                ha="center", va="center", fontsize=9,
                color="white" if f in used else "#999")
    fig.tight_layout()
    fig.savefig("figures/feature_usage.png", dpi=150); plt.close(fig)

    log()
    log("=" * 68)
    log("STEP 7 : SAVE")
    log("=" * 68)
    joblib.dump({"model": model, "features": feats, "type": "rule-based"},
                "outputs/baseline_model.pkl")
    meta = {
        "model_type": "rule-based-threshold",
        "features_used": ["meanSNR", "meanRSSI", "PLR", "CRC"],
        "features_available": feats,
        "thresholds": best_params,
        "test_accuracy": float(accuracy_score(yte, pred)),
        "test_macro_f1": float(f1_score(yte, pred, average="macro")),
        "critical_error_rate": float(gate),
        "safety_gate_passed": bool(gate < 0.01),
        "n_train": int(len(Xtr)),
        "n_test": int(len(Xte)),
    }
    json.dump(meta, open("outputs/model_meta.json", "w"), indent=2)
    log("wrote outputs/baseline_model.pkl")
    log("wrote outputs/model_meta.json")

    with open("outputs/training_report.txt", "w") as f:
        f.write("\n".join(LINES) + "\n")


if __name__ == "__main__":
    main()
