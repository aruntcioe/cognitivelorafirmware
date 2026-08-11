#!/usr/bin/env bash
# Full baseline pipeline, start to finish. Deterministic (seed=42).
set -e
mkdir -p outputs figures

echo "### 1/4  cleaning + feature selection"
python3 prepare_data.py

echo; echo "### 2/4  exploratory data analysis"
python3 eda.py

echo; echo "### 3/4  train baseline / tune thresholds / evaluate"
python3 train_baseline.py

echo; echo "### 4/4  firmware export + equivalence proof"
python3 export_firmware.py

# Compile and verify the C header against all dataset rows
if command -v gcc >/dev/null 2>&1; then
  echo; echo "### verifying compiled C against Python"
  python3 - <<'PY'
import sys; sys.path.insert(0,'.')
from train_baseline import RuleBasedClassifier
import joblib, pandas as pd, numpy as np
b = joblib.load('outputs/baseline_model.pkl')
df = pd.read_csv('outputs/dataset_clean.csv')
X = df[b['features']]
pred = b['model'].predict(X)
np.savetxt('outputs/_vectors.csv',
           np.column_stack([X.to_numpy(), pred]),
           delimiter=',', fmt='%.8g')
PY
  gcc -O2 -std=c99 -I outputs test_baseline_model.c -o /tmp/test_bl
  /tmp/test_bl outputs/_vectors.csv
  rm -f outputs/_vectors.csv
else
  echo "gcc not found — skipping compiled-C verification"
fi

echo; echo "### done. deliverables in outputs/ and figures/"
