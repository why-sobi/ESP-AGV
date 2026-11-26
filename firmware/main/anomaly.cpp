#include "anomaly.h"

#include <algorithm>

#include "esp_timer.h"

namespace {

using namespace anomaly_model;

int popcount(uint32_t bits) {
  int count = 0;
  for (; bits != 0; bits &= bits - 1) {
    ++count;
  }
  return count;
}

}  // namespace

void AnomalyDetector::reset() {
  next_ = 0;
  count_ = 0;
  exceed_history_ = 0;
  has_score_ = false;
  score_ = 0.0F;
  alarm_ = false;
}

void AnomalyDetector::add_sample(const float *raw, uint32_t elapsed_ms, bool monitored) {
  for (int f = 0; f < model_.features; ++f) {
    window_[next_][f] = (raw[f] - model_.mean[f]) / model_.scale[f];
  }
  sample_monitored_[next_] = monitored;
  sample_ms_[next_] = elapsed_ms;
  next_ = (next_ + 1) % kWindow;
  count_ = std::min(count_ + 1, kWindow);
  if (count_ < kWindow) {
    return;
  }

  const int64_t started = esp_timer_get_time();
  score_ = compute_score();
  last_inference_us_ = static_cast<uint32_t>(esp_timer_get_time() - started);
  max_inference_us_ = std::max(max_inference_us_, last_inference_us_);
  has_score_ = true;

  // The oldest sample is at next_ once the ring is full.
  bool window_monitored = sample_ms_[next_] >= static_cast<uint32_t>(kWarmupMs);
  for (int i = 0; i < kWindow; ++i) {
    window_monitored = window_monitored && sample_monitored_[i];
  }
  const bool exceed = window_monitored && score_ > model_.threshold;
  exceed_history_ = ((exceed_history_ << 1) | (exceed ? 1U : 0U)) & ((1U << kPersistN) - 1U);
  alarm_ = popcount(exceed_history_) >= kPersistK;
}

// Forward pass of ConvAutoencoder in scripts/models/train_autoencoder.py:
// Conv1d(F,16,k3,p1) ReLU, Conv1d(16,16,k3,s2,p1) ReLU, Flatten (channel-major),
// Linear(160,8), Linear(8,32) ReLU, Linear(32,20*F); score = MSE vs the input.
float AnomalyDetector::compute_score() const {
  const int features = model_.features;
  float x[kWindow][kMaxFeatures];  // Time-ordered (oldest first) scaled window.
  for (int t = 0; t < kWindow; ++t) {
    const int slot = (next_ + t) % kWindow;
    for (int f = 0; f < features; ++f) {
      x[t][f] = window_[slot][f];
    }
  }

  float h1[kChannels][kWindow];
  for (int c = 0; c < kChannels; ++c) {
    for (int t = 0; t < kWindow; ++t) {
      float sum = model_.conv1_b[c];
      for (int i = 0; i < features; ++i) {
        for (int k = 0; k < 3; ++k) {
          const int source = t + k - 1;
          if (source >= 0 && source < kWindow) {
            sum += model_.conv1_w[(c * features + i) * 3 + k] * x[source][i];
          }
        }
      }
      h1[c][t] = std::max(sum, 0.0F);
    }
  }

  float h2[kChannels * kReduced];  // Flattened channel-major, as torch Flatten.
  for (int c = 0; c < kChannels; ++c) {
    for (int t = 0; t < kReduced; ++t) {
      float sum = model_.conv2_b[c];
      for (int i = 0; i < kChannels; ++i) {
        for (int k = 0; k < 3; ++k) {
          const int source = 2 * t + k - 1;
          if (source >= 0 && source < kWindow) {
            sum += model_.conv2_w[(c * kChannels + i) * 3 + k] * h1[i][source];
          }
        }
      }
      h2[c * kReduced + t] = std::max(sum, 0.0F);
    }
  }

  float z[kBottleneck];
  for (int j = 0; j < kBottleneck; ++j) {
    float sum = model_.enc_b[j];
    for (int i = 0; i < kChannels * kReduced; ++i) {
      sum += model_.enc_w[j * kChannels * kReduced + i] * h2[i];
    }
    z[j] = sum;
  }

  float hidden[kHidden];
  for (int j = 0; j < kHidden; ++j) {
    float sum = model_.dec1_b[j];
    for (int i = 0; i < kBottleneck; ++i) {
      sum += model_.dec1_w[j * kBottleneck + i] * z[i];
    }
    hidden[j] = std::max(sum, 0.0F);
  }

  // Output is time-major [t0 f0, t0 f1, ...], like the flattened input.
  const int outputs = kWindow * features;
  float squared_error = 0.0F;
  for (int j = 0; j < outputs; ++j) {
    float sum = model_.dec2_b[j];
    for (int i = 0; i < kHidden; ++i) {
      sum += model_.dec2_w[j * kHidden + i] * hidden[i];
    }
    const float error = sum - x[j / features][j % features];
    squared_error += error * error;
  }
  return squared_error / static_cast<float>(outputs);
}
