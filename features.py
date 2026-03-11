"""features.py

Simple feature extraction for IMU movement CSV files or in-memory buffers.
Input: sequence of rows [ax,ay,az,gx,gy,gz]
Output: 1D feature vector (means, stds, mins, maxs, ranges per channel)
"""
import math
def extract_features(rows):
    # rows: list of iterables length>=6
    if not rows:
        return []
    cols = list(zip(*rows))
    features = []
    for c in cols[:6]:
        vals = [float(x) for x in c]
        n = len(vals)
        mean = sum(vals)/n
        var = sum((v-mean)**2 for v in vals)/n
        std = math.sqrt(var)
        mn = min(vals)
        mx = max(vals)
        rng = mx - mn
        features.extend([mean, std, mn, mx, rng])
    return features
