#pragma once

#include <cstddef>
#include <cstdint>

// Latency benchmark for candidate detector architectures (serial command
// 'B', see firmware/README.md). Runs the same esp-dsp forward passes as
// anomaly.cpp, but with runtime layer sizes and random weights in internal
// RAM, so scripts/models/esp_latency.py can measure any architecture in the
// search space without reflashing.
namespace bench {

struct Result {
  bool ok = false;
  uint32_t median_us = 0;
  uint32_t max_us = 0;
  size_t weight_bytes = 0;  // Padded float32 weights, as anomaly.cpp keeps them.
};

// Conv autoencoder: Conv1d(F,C,k) ReLU, Conv1d(C,C,k,stride 2) ReLU, Flatten,
// Linear(C*ceil(W/2),Z), Linear(Z,H) ReLU, Linear(H,W*F); score = MSE.
Result conv(int features, int window, int channels, int kernel, int bottleneck, int hidden,
            int repeats);

// One GRU step (gates r, z, n) plus a linear head.
Result gru(int inputs, int hidden, int outputs, int repeats);

}  // namespace bench
