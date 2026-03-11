#!/usr/bin/env python3
"""pi_tcp_server.py

Threaded TCP server for receiving IMU movement CSV blocks from the device.
Protocol (lines):
  MOVEMENT_START,<N>\n
  ax,ay,az,gx,gy,gz\n  (repeated N times)

  MOVEMENT_END\n
The server writes each completed movement to movement_YYYYMMDD_HHMMSS.csv in --out directory.
"""
import argparse
import socketserver
import threading
import csv
import time
import os
import joblib
import numpy as np
from features import extract_features
# Monkeypatch numpy deprecated aliases used by older sklearn versions
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

import threading
import sys
import select
try:
    import msvcrt
    _HAS_MS = True
except Exception:
    _HAS_MS = False


class IMUHandler(socketserver.StreamRequestHandler):
    def handle(self):
        addr = self.client_address
        print(f"Connection from {addr}")
        buf = []
        expecting = None
        while True:
            line = self.rfile.readline()
            if not line:
                break
            try:
                s = line.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            if not s:
                continue
            if s.startswith('MOVEMENT_START'):
                parts = s.split(',')
                try:
                    expecting = int(parts[1]) if len(parts) > 1 else None
                except Exception:
                    expecting = None
                buf = []
                print(f"[{addr}] Movement start (expecting={expecting})")
                continue
            if s == 'MOVEMENT_END':
                if buf:
                    # If training mode: save to label folder; if identifying: classify using model
                    mode = getattr(self.server, 'mode', 'identifying')
                    if mode == 'training' and getattr(self.server, 'label', None) is not None:
                        label = str(self.server.label)
                        label_dir = os.path.join(self.server.out_dir, label)
                        os.makedirs(label_dir, exist_ok=True)
                        ts = time.strftime('%Y%m%d_%H%M%S')
                        fname = os.path.join(label_dir, f"movement_{ts}.csv")
                        with open(fname, 'w', newline='') as f:
                            w = csv.writer(f)
                            w.writerow(['ax','ay','az','gx','gy','gz'])
                            w.writerows(buf)
                        print(f"[{addr}] Saved {len(buf)} samples to {fname} (label={label})")
                    elif mode == 'identifying':
                        # lazy-load model
                        model = getattr(self.server, 'model', None)
                        if model is None:
                            try:
                                model = joblib.load(self.server.model_path)
                                self.server.model = model
                                print(f"Loaded model from {self.server.model_path}")
                            except Exception as e:
                                print(f"Failed to load model: {e}")
                                model = None
                        if model is not None:
                            feats = extract_features(buf)
                            if feats:
                                X = np.array(feats).reshape(1, -1)
                                try:
                                    pred = model.predict(X)[0]
                                    prob = None
                                    if hasattr(model, 'predict_proba'):
                                        p = model.predict_proba(X)
                                        # probability for predicted class
                                        idx = list(model.classes_).index(pred)
                                        prob = p[0, idx]
                                    if prob is not None:
                                        print(f"[{addr}] IDENTIFIED => {pred} (p={prob:.3f})")
                                    else:
                                        print(f"[{addr}] IDENTIFIED => {pred}")
                                except Exception as e:
                                    print(f"Classification error: {e}")
                            else:
                                print(f"[{addr}] No features extracted; cannot classify")
                        else:
                            print(f"[{addr}] No model available; cannot classify")
                    else:
                        print(f"[{addr}] Movement received but not saved (mode={mode})")
                else:
                    print(f"[{addr}] Movement end (empty buffer)")
                buf = []
                expecting = None
                continue
            # parse data line
            parts = [p.strip() for p in s.split(',') if p.strip()!='']
            if len(parts) >= 6:
                try:
                    row = [float(x) for x in parts[:6]]
                    buf.append(row)
                except Exception:
                    print(f"[{addr}] Failed to parse data line: {s}")
            else:
                # ignore other info lines
                pass


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True


def key_listener(server):
    """Listen for single-key presses to switch modes.

    - 't' or 'T' -> training mode
    - 'i' or 'I' -> identifying mode
    - '0'..'9'    -> set current training label (only used when in training mode)
    """
    print("Key controls: 't' = training, 'i' = identifying, 0-9 to select training label")
    while True:
        try:
            if _HAS_MS:
                if msvcrt.kbhit():
                    ch = msvcrt.getwch()
                else:
                    time.sleep(0.1)
                    continue
            else:
                dr, dw, de = select.select([sys.stdin], [], [], 0.1)
                if not dr:
                    continue
                ch = sys.stdin.read(1)
            if not ch:
                continue
            ch = ch.lower()
            if ch == 't':
                server.mode = 'training'
                print("Mode -> TRAINING (label=", server.label, ")")
            elif ch == 'i':
                server.mode = 'identifying'
                print("Mode -> IDENTIFYING")
            elif ch.isdigit():
                server.label = ch
                print(f"Selected label -> {server.label}")
            else:
                # ignore other keys
                pass
        except Exception:
            # keep listener alive on errors
            time.sleep(0.1)


def run(host, port, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    server = ThreadedTCPServer((host, port), IMUHandler)
    server.out_dir = out_dir
    # operational state: default to training mode with label '0' (state 0)
    server.mode = 'training'
    server.label = '0'
    # model loading state for identifying mode
    server.model = None
    server.model_path = os.path.join('.', 'models', 'model.pkl')
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    # Start key listener thread
    kl = threading.Thread(target=key_listener, args=(server,), daemon=True)
    kl.start()
    print(f"Listening on {host}:{port}, writing to {out_dir}")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print('Shutting down')
        server.shutdown()
        server.server_close()


def main():
    p = argparse.ArgumentParser(description='Receive IMU CSV blocks over TCP and save to CSV')
    p.add_argument('--host', '-H', default='0.0.0.0')
    p.add_argument('--port', '-p', type=int, default=5005)
    p.add_argument('--out', '-o', default='.', help='Output directory for CSV files')
    args = p.parse_args()
    run(args.host, args.port, args.out)


if __name__ == '__main__':
    main()
