#!/usr/bin/env python3
"""train_model.py

Train a classifier on captured IMU movements saved by `pi_tcp_server.py`.

Directory layout expected (default --data ./captures):
  ./captures/0/*.csv
  ./captures/1/*.csv
  ...

Each CSV contains rows with columns: ax,ay,az,gx,gy,gz

The script extracts simple statistics per file, trains a RandomForest
classifier, prints a test accuracy, and saves the trained pipeline to
`models/model.pkl` by default.

Requires: scikit-learn
  pip install scikit-learn
"""
try:
    import numpy as _np
    if not hasattr(_np, 'int64'):
        _np.int64 = int
    # prefer the numpy scalar name `bool_` to avoid using the deprecated
    # `np.bool` alias which triggers a DeprecationWarning on attribute access
    if not hasattr(_np, 'bool_'):
        _np.bool_ = bool
except Exception:
    pass
import argparse
import os
import csv
import time
import joblib
from sklearn.ensemble import RandomForestClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from sklearn.model_selection import train_test_split
from features import extract_features


def load_dataset(data_dir):
    X = []
    y = []
    for label in sorted(os.listdir(data_dir)):
        label_dir = os.path.join(data_dir, label)
        if not os.path.isdir(label_dir):
            continue
        # only numeric labels
        if not label.isdigit():
            continue
        for fname in os.listdir(label_dir):
            if not fname.lower().endswith('.csv'):
                continue
            path = os.path.join(label_dir, fname)
            rows = []
            try:
                with open(path, 'r', newline='') as f:
                    r = csv.reader(f)
                    hdr = next(r, None)
                    for row in r:
                        if len(row) >= 6:
                            rows.append([float(x) for x in row[:6]])
            except Exception as e:
                print('Failed to read', path, e)
                continue
            if not rows:
                continue
            feats = extract_features(rows)
            if not feats:
                continue
            X.append(feats)
            y.append(int(label))
    return X, y


def main():
    p = argparse.ArgumentParser(description='Train IMU movement classifier')
    p.add_argument('--data', '-d', default='./captures', help='Captured data directory')
    p.add_argument('--out', '-o', default='./models/model.pkl', help='Output model path')
    args = p.parse_args()
    X, y = load_dataset(args.data)
    if not X:
        print('No training data found in', args.data)
        return
    print(f'Loaded {len(X)} samples from {args.data}')
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42, stratify=y)
    pipe = Pipeline([
        ('scaler', StandardScaler()),
        ('clf', RandomForestClassifier(n_estimators=200, random_state=42))
    ])
    print('Training...')
    pipe.fit(X_train, y_train)
    acc = pipe.score(X_test, y_test)
    print(f'Test accuracy: {acc:.3f} ({len(X_test)} samples)')
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    joblib.dump(pipe, args.out)
    print('Saved model to', args.out)


if __name__ == '__main__':
    main()
