# INT8 / INT16 benchmark (ESP-DL) vs float32 (esp-dsp)

Deployed models (`models/esp-conv-v1`, `models/esp-gru-v1`) quantized with esp-ppq for the ESP32-S3
(`scripts/models/quantize_int8.py`, calibrated on 1024 healthy training windows / GRU steps) and run
with ESP-DL on the ESP (`firmware/esp-dl-bench`, `scripts/esp/int8_bench.py`). w8a8 = INT8 weights
and activations, w16a16 = INT16. Float32 is the deployed esp-dsp code.

## On the ESP (median of 101 runs, 160 MHz)

| Model | Precision | Time per inference | Weights | ESP output vs esp-ppq simulation |
|---|---|---|---|---|
| Motor conv | float32 (esp-dsp) | 307 us | 20.3 KB | - |
| Motor conv | INT8 | 747 us | 5.6 KB | 0 max abs |
| Motor conv | INT16 | 737 us | 11.1 KB | 5e-07 max abs |
| IR conv | float32 (esp-dsp) | 294 us | 17.8 KB | - |
| IR conv | INT8 | 743 us | 4.8 KB | 0 max abs |
| IR conv | INT16 | 768 us | 9.6 KB | 5e-06 max abs |
| Motor GRU | float32 (esp-dsp) | 267 us | 14.5 KB | - |
| Motor GRU | INT8 | 813 us | 5.0 KB | - |
| Motor GRU | INT16 | 821 us | 20.2 KB | - |

INT8 and INT16 are ~2.5x slower than float32: these models do ~17k (conv) or ~3k (GRU)
multiply-adds per frame, so ESP-DL's fixed per-layer runtime cost (module dispatch, layout
conversion, requantization, reshape/transpose copies, LUT activations) dominates; INT8 and INT16
take the same time and all three models ~0.75-0.8 ms regardless of size. The ESP's INT8 outputs
equal esp-ppq's simulation exactly, so the accuracy below (simulated on every run) holds on the chip.

## Accuracy (simulated, all runs)

Detected = fault runs of the model's own fault family (motor 56, IR 64). Thresholds: the float
model's, or re-derived from the quantized validation scores with the same no-alarm rule.

| Model | Precision | Score error p99 / threshold | Detected (float threshold) | Healthy runs alarming (float threshold) | Detected (own threshold) | Healthy test alarms (own threshold) |
|---|---|---|---|---|---|---|
| Motor conv | float32 | - | 49 | 0/18 val, 2/18 test | 49 | 2/18 |
| Motor conv | INT8 | 8.86 | 56 | 18/18 val, 18/18 test | 12 | 0/18 |
| Motor conv | INT16 | 0.00 | 49 | 1/18 val, 2/18 test | 49 | 2/18 |
| IR conv | float32 | - | 33 | 0/18 val, 0/18 test | 33 | 0/18 |
| IR conv | INT8 | 0.49 | 36 | 3/18 val, 0/18 test | 31 | 0/18 |
| IR conv | INT16 | 0.50 | 36 | 3/18 val, 0/18 test | 35 | 0/18 |
| Motor GRU | float32 | - | 52 | 0/18 val, 0/18 test | 52 | 0/18 |
| Motor GRU | INT8 | 2.88 | 52 | 0/18 val, 0/18 test | 52 | 0/18 |
| Motor GRU | INT16 | 0.05 | 52 | 0/18 val, 0/18 test | 52 | 0/18 |

## Conclusion

- **Motor conv: INT8 is unusable.** Its healthy reconstruction error (~0.03) is below INT8's
  rounding noise: at the float threshold every healthy run alarms; with its own threshold it
  detects 12 of 56 instead of 49. INT16 matches float32.
- **IR conv:** INT8 loses a little (31 vs 33 with its own threshold); INT16 matches float32.
- **Motor GRU:** INT8 and INT16 both match float32 (its fault scores are >=25x the threshold).
- **Decision: keep float32 on esp-dsp.** It is ~2.5x faster than ESP-DL at this model size, matches
  Python to ~1e-5, and the motor model needs more than 8 bits anyway. INT8's only gain is ~4x
  smaller weights (5 KB vs 20 KB per model), and RAM is not a constraint (165 KB free).
