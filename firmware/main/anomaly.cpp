#include "anomaly.h"

#include <algorithm>
#include <cmath>

#include "esp_dsp.h"
#include "esp_timer.h"

namespace {

using namespace anomaly_model;

// esp-dsp takes its ESP32-S3 SIMD path only for 16-byte-aligned data and
// lengths (and matrix sizes) that are multiples of 4; export_c.py pads the
// weights accordingly.
constexpr int kEncInputs = kChannels * kReduced;
static_assert(kEncInputs % 4 == 0 && kBottleneck % 4 == 0 && kHidden % 4 == 0, "padding");
static_assert(kWindow % 4 == 0 && kConv1Patch % 4 == 0 && kReducedPadded % 4 == 0, "padding");
static_assert(kGruHidden % 4 == 0 && kGruInputs % 4 == 0, "padding");
static_assert(WindowGate::kCapacity >= kGruWindow && WindowGate::kCapacity >= kWindow &&
                  WindowGate::kCapacity >= IrStuckDetector::kSpan,
              "gate capacity");

int popcount(uint32_t bits) {
  int count = 0;
  for (; bits != 0; bits &= bits - 1) {
    ++count;
  }
  return count;
}

// out[r] = weights[r] . input + bias[r]; rows are ``length`` floats.
void dense(const float *weights, const float *bias, const float *input, float *out, int rows,
           int length) {
  for (int r = 0; r < rows; ++r) {
    dsps_dotprod_f32(weights + r * length, input, &out[r], length);
    out[r] += bias[r];
  }
}

float sigmoid(float value) { return 1.0F / (1.0F + std::exp(-value)); }

template <size_t N>
void copy_weights(const float *source, float (&target)[N], size_t count = N) {
  std::copy(source, source + count, target);
}

uint32_t elapsed_us(int64_t started) {
  return static_cast<uint32_t>(esp_timer_get_time() - started);
}

}  // namespace

void WindowGate::reset() {
  next_ = 0;
  count_ = 0;
}

void WindowGate::push(uint32_t elapsed_ms, bool monitored) {
  sample_ms_[next_] = elapsed_ms;
  sample_monitored_[next_] = monitored;
  next_ = (next_ + 1) % kCapacity;
  count_ = std::min(count_ + 1, kCapacity);
}

bool WindowGate::monitored(int window) const {
  if (count_ < window) {
    return false;
  }
  const int oldest = (next_ - window + kCapacity) % kCapacity;
  if (sample_ms_[oldest] < static_cast<uint32_t>(kWarmupMs)) {
    return false;
  }
  for (int i = 0; i < window; ++i) {
    if (!sample_monitored_[(oldest + i) % kCapacity]) {
      return false;
    }
  }
  return true;
}

void Persistence::reset() {
  history_ = 0;
  alarm_ = false;
}

bool Persistence::update(bool exceed) {
  history_ = ((history_ << 1) | (exceed ? 1U : 0U)) & ((1U << kPersistN) - 1U);
  alarm_ = popcount(history_) >= kPersistK;
  return alarm_;
}

// ---------------------------------------------------------------- conv

AnomalyDetector::AnomalyDetector(const ConvAutoencoder &model) : model_(model) {
  ConvWeights &w = weights_;
  copy_weights(model.conv1_wt, w.conv1_wt);
  copy_weights(model.conv1_b, w.conv1_b);
  copy_weights(model.conv2_wt, w.conv2_wt);
  copy_weights(model.conv2_b, w.conv2_b);
  copy_weights(model.enc_w, w.enc_w);
  copy_weights(model.enc_b, w.enc_b);
  copy_weights(model.dec1_w, w.dec1_w);
  copy_weights(model.dec1_b, w.dec1_b);
  copy_weights(model.dec2_w, w.dec2_w, kWindow * model.features * kHidden);
  copy_weights(model.dec2_b, w.dec2_b, kWindow * model.features);
}

void AnomalyDetector::reset() {
  next_ = 0;
  gate_.reset();
  persistence_.reset();
  has_score_ = false;
  score_ = 0.0F;
}

void AnomalyDetector::add_sample(const float *raw, uint32_t elapsed_ms, bool monitored) {
  for (int f = 0; f < model_.features; ++f) {
    window_[next_][f] = (raw[f] - model_.mean[f]) / model_.scale[f];
  }
  next_ = (next_ + 1) % kWindow;
  gate_.push(elapsed_ms, monitored);
  if (gate_.count() < kWindow) {
    return;
  }

  const int64_t started = esp_timer_get_time();
  score_ = compute_score();
  last_inference_us_ = elapsed_us(started);
  max_inference_us_ = std::max(max_inference_us_, last_inference_us_);
  has_score_ = true;
  persistence_.update(gate_.monitored(kWindow) && score_ > model_.threshold);
}

// Forward pass of ConvAutoencoder in scripts/models/train_autoencoder.py:
// Conv1d(F,16,k3,p1) ReLU, Conv1d(16,16,k3,s2,p1) ReLU, Flatten,
// Linear(160,8), Linear(8,32) ReLU, Linear(32,20*F); score = MSE vs the input.
// Each conv is one esp-dsp matrix multiply of an im2col patch matrix
// [time][tap * channels] by the exported [tap * channels][out channel] weight.
float AnomalyDetector::compute_score() const {
  const int features = model_.features;
  float x[kWindow][kMaxFeatures];  // Time-ordered (oldest first) scaled window.
  for (int t = 0; t < kWindow; ++t) {
    const int slot = (next_ + t) % kWindow;
    for (int f = 0; f < features; ++f) {
      x[t][f] = window_[slot][f];
    }
  }

  alignas(16) float patch1[kWindow][kConv1Patch] = {};
  for (int t = 0; t < kWindow; ++t) {
    for (int k = 0; k < 3; ++k) {
      const int source = t + k - 1;
      if (source >= 0 && source < kWindow) {
        std::copy(x[source], x[source] + features, &patch1[t][k * features]);
      }
    }
  }
  alignas(16) float h1[kWindow][kChannels];
  dspm_mult_f32(&patch1[0][0], weights_.conv1_wt, &h1[0][0], kWindow, kConv1Patch, kChannels);
  for (int t = 0; t < kWindow; ++t) {
    for (int c = 0; c < kChannels; ++c) {
      h1[t][c] = std::max(h1[t][c] + weights_.conv1_b[c], 0.0F);
    }
  }

  // Stride-2 conv; rows past kReduced are zero padding for the SIMD path.
  alignas(16) float patch2[kReducedPadded][3 * kChannels] = {};
  for (int t = 0; t < kReduced; ++t) {
    for (int k = 0; k < 3; ++k) {
      const int source = 2 * t + k - 1;
      if (source >= 0 && source < kWindow) {
        std::copy(h1[source], h1[source] + kChannels, &patch2[t][k * kChannels]);
      }
    }
  }
  // Rows 0..kReduced-1 flattened are the time-major encoder input.
  alignas(16) float h2[kReducedPadded][kChannels];
  dspm_mult_f32(&patch2[0][0], weights_.conv2_wt, &h2[0][0], kReducedPadded, 3 * kChannels,
                kChannels);
  for (int t = 0; t < kReduced; ++t) {
    for (int c = 0; c < kChannels; ++c) {
      h2[t][c] = std::max(h2[t][c] + weights_.conv2_b[c], 0.0F);
    }
  }

  alignas(16) float z[kBottleneck];
  dense(weights_.enc_w, weights_.enc_b, &h2[0][0], z, kBottleneck, kEncInputs);
  alignas(16) float hidden[kHidden];
  dense(weights_.dec1_w, weights_.dec1_b, z, hidden, kHidden, kBottleneck);
  for (float &value : hidden) {
    value = std::max(value, 0.0F);
  }

  // Output is time-major [t0 f0, t0 f1, ...], like the flattened input.
  const int outputs = kWindow * features;
  alignas(16) float reconstruction[kWindow * kMaxFeatures];
  dense(weights_.dec2_w, weights_.dec2_b, hidden, reconstruction, outputs, kHidden);
  float squared_error = 0.0F;
  for (int j = 0; j < outputs; ++j) {
    const float error = reconstruction[j] - x[j / features][j % features];
    squared_error += error * error;
  }
  return squared_error / static_cast<float>(outputs);
}

// ---------------------------------------------------------------- motor GRU

MotorGruDetector::MotorGruDetector() {
  GruWeights &w = weights_;
  copy_weights(kGru_w_ih, w.w_ih);
  copy_weights(kGru_w_hh, w.w_hh);
  copy_weights(kGru_b_ih, w.b_ih);
  copy_weights(kGru_b_hh, w.b_hh);
  copy_weights(kGru_head_w, w.head_w);
  copy_weights(kGru_head_b, w.head_b);
}

void MotorGruDetector::reset() {
  std::fill(std::begin(state_), std::end(state_), 0.0F);
  next_ = 0;
  gate_.reset();
  persistence_.reset();
  has_score_ = false;
  score_ = 0.0F;
}

// One step of torch's GRU (gates r, z, n) and the linear head.
void MotorGruDetector::add_sample(const float *motor, uint32_t elapsed_ms, bool monitored) {
  const int64_t started = esp_timer_get_time();
  alignas(16) float input[kGruInputs] = {};
  for (int i = 0; i < 2; ++i) {
    input[i] = (motor[i] - kMotorMean[i]) / kMotorScale[i];
  }
  constexpr int kGates = 3 * kGruHidden;
  alignas(16) float from_input[kGates];
  alignas(16) float from_state[kGates];
  dense(weights_.w_ih, weights_.b_ih, input, from_input, kGates, kGruInputs);
  dense(weights_.w_hh, weights_.b_hh, state_, from_state, kGates, kGruHidden);
  for (int j = 0; j < kGruHidden; ++j) {
    const float reset = sigmoid(from_input[j] + from_state[j]);
    const float update = sigmoid(from_input[kGruHidden + j] + from_state[kGruHidden + j]);
    const float candidate =
        std::tanh(from_input[2 * kGruHidden + j] + reset * from_state[2 * kGruHidden + j]);
    state_[j] = (1.0F - update) * candidate + update * state_[j];
  }
  float predicted[kGruOutputs];
  dense(weights_.head_w, weights_.head_b, state_, predicted, kGruOutputs, kGruHidden);
  for (int o = 0; o < kGruOutputs; ++o) {
    const float actual = (motor[2 + o] - kMotorMean[2 + o]) / kMotorScale[2 + o];
    const float error = predicted[o] - actual;
    errors_[next_][o] = error * error;
  }
  next_ = (next_ + 1) % kGruWindow;
  gate_.push(elapsed_ms, monitored);
  if (gate_.count() >= kGruWindow) {
    float worst = 0.0F;
    for (int o = 0; o < kGruOutputs; ++o) {
      float sum = 0.0F;
      for (int t = 0; t < kGruWindow; ++t) {
        sum += errors_[t][o];
      }
      worst = std::max(worst, sum / static_cast<float>(kGruWindow));
    }
    score_ = worst;
    has_score_ = true;
    persistence_.update(gate_.monitored(kGruWindow) && score_ > kGruThreshold);
  }
  last_inference_us_ = elapsed_us(started);
  max_inference_us_ = std::max(max_inference_us_, last_inference_us_);
}

// ---------------------------------------------------------------- stuck IR

void IrStuckDetector::reset() {
  next_ = 0;
  gate_.reset();
  persistence_.reset();
  stuck_ = false;
}

void IrStuckDetector::add_sample(const float *raw, float wheel_speed, uint32_t elapsed_ms,
                                 bool monitored) {
  std::copy(raw, raw + 3, readings_[next_]);
  wheel_speeds_[next_] = wheel_speed;
  next_ = (next_ + 1) % kSpan;
  gate_.push(elapsed_ms, monitored);
  if (gate_.count() < kSpan) {
    return;
  }
  float speed_sum = 0.0F;
  for (float speed : wheel_speeds_) {
    speed_sum += speed;
  }
  const bool driving = speed_sum / static_cast<float>(kSpan) > kStuckSpeed;
  stuck_ = false;
  for (int s = 0; s < 3 && driving; ++s) {
    float largest_change = 0.0F;
    for (int i = 1; i < kSpan; ++i) {
      const float now = readings_[(next_ + i) % kSpan][s];  // next_: oldest reading.
      const float before = readings_[(next_ + i - 1) % kSpan][s];
      largest_change = std::max(largest_change, std::fabs(now - before));
    }
    stuck_ = stuck_ || largest_change < kStuckEps;
  }
  persistence_.update(gate_.monitored(kSpan) && stuck_);
}
