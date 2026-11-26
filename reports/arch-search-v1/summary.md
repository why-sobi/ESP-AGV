# Architecture search (Optuna) with measured ESP latency

`scripts/models/search_architectures.py`: multi-objective TPE search per detector, maximizing
detections on the *tuning* half of the fault runs (A 2.4, B 3.4, C 2.6, D 3.1 rad/s; minus 0.05 x
median delay as a tie-break) and minimizing the architecture's per-frame time on the ESP, looked up
in `reports/esp-latency-v1/latency.csv` (967 architectures measured on the chip with the firmware's
`B` command, `scripts/models/esp_latency.py`). Thresholds use the deployed rule (no persistent alarm
on any healthy validation run; GRU x2). Trials train with a reduced budget; the deployed
architecture is trial 0. Trials: motor conv 40, IR conv 40, GRU 12.

Candidates from each Pareto front were retrained with the deployed models' full budget and
compared with the deployed models on the **held-out** half of the faults (the other speed of each
route) and the healthy test runs, neither seen by the search.

## Held-out results

| Detector | Version | Architecture | Held-out detected | Median delay | Healthy test alarms | ESP time | Weights |
|---|---|---|---|---|---|---|---|
| Motor conv | deployed | window 20, 16 ch, kernel 3, bottleneck 8, hidden 32 | 25/28 | 0.70 s | 2/18 | 307 us | 20.3 KB |
| Motor conv | candidate 0 | window 10, 8 ch, kernel 3, bottleneck 12, hidden 16 | 28/28 | 0.40 s | 2/18 | 100 us | 6.6 KB |
| Motor conv | candidate 1 | window 10, 8 ch, kernel 3, bottleneck 16, hidden 16 | 27/28 | 0.45 s | 1/18 | 104 us | 7.5 KB |
| IR conv | deployed | window 20, 16 ch, kernel 3, bottleneck 8, hidden 32 | 17/32 | 2.75 s | 0/18 | 294 us | 17.8 KB |
| IR conv | candidate 0 | window 10, 8 ch, kernel 3, bottleneck 4, hidden 32 | 22/32 | 2.48 s | 2/18 | 100 us | 6.3 KB |
| IR conv | candidate 1 | window 10, 24 ch, kernel 3, bottleneck 4, hidden 32 | 16/32 | 2.42 s | 2/18 | 263 us | 14.4 KB |
| Motor GRU | deployed | hidden 32, score window 62 | 27/28 | 0.16 s | 0/18 | 267 us | 14.5 KB |
| Motor GRU | candidate 0 | hidden 16, score window 31 | 28/28 | 0.13 s | 0/18 | 127 us | 4.3 KB |
| Motor GRU | candidate 1 | hidden 8, score window 47 | 27/28 | 0.13 s | 0/18 | 63 us | 1.4 KB |

No version raised an alarm on a healthy validation run or before a fault's onset.

## Findings

- **Motor conv:** candidate 0 (window 10, 8 channels) is better on every measure: 28/28 vs 25/28,
  0.40 s vs 0.70 s median delay, 3x faster (100 vs 307 us), 3x smaller, same healthy test alarms.
- **IR conv:** candidate 0 (window 10, 8 channels) detects 22/32 vs 17/32 and is 3x faster, but
  raises 2/18 healthy test alarms where the deployed model has none: a trade-off, not a clear win.
  Candidate 1 looked best in the search (21/32) but reached only 16/32 after full retraining.
- **Motor GRU:** candidate 0 (16 hidden, 1 s score window) detects 28/28 vs 27/28, 2x faster (127 vs
  267 us), 3x smaller, no healthy alarms. The 8-hidden candidate matches the deployed GRU at 63 us.
- **Pattern:** shorter windows (10 samples, 0.32 s) and fewer channels/units than the deployed
  choices are both faster and at least as good here; the deployed sizes were larger than needed.
- Not yet deployed: the firmware's conv forward pass has the window (20) and channel count (16)
  fixed at compile time; deploying needs those made configurable in `export_c.py` /
  `anomaly.cpp`, then a replay and live runs.
