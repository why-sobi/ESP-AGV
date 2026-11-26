#pragma once

#include <cstdint>

#include "anomaly_model.h"

// The ESP's anomaly detectors. They mirror scripts/models/train_autoencoder.py,
// scripts/models/gru_experiment.py and scripts/faults/evaluate_faults.py: one
// sample per sensor frame, a score per window of recent samples, and a
// persistent alarm when kPersistK of the last kPersistN monitored windows
// exceed the threshold. A window is monitored when its oldest sample is at
// least kWarmupMs into the route and none of its samples is in a commanded
// manoeuvre. Matrix work runs on esp-dsp (ESP32-S3 SIMD).

// Monitoring flags of the last kCapacity samples.
class WindowGate {
 public:
  static constexpr int kCapacity = 64;

  void reset();
  void push(uint32_t elapsed_ms, bool monitored);
  int count() const { return count_; }
  // True when the last ``window`` samples are all monitorable.
  bool monitored(int window) const;

 private:
  uint32_t sample_ms_[kCapacity]{};
  bool sample_monitored_[kCapacity]{};
  int next_ = 0;
  int count_ = 0;
};

// Model weights live in flash; the detectors copy them into internal RAM once,
// so the per-frame SIMD routines do not stream them through the flash cache
// (which the WiFi stack on the other core shares).
struct ConvWeights {
  alignas(16) float conv1_wt[anomaly_model::kConv1Patch * anomaly_model::kChannels];
  alignas(16) float conv1_b[anomaly_model::kChannels];
  alignas(16) float conv2_wt[3 * anomaly_model::kChannels * anomaly_model::kChannels];
  alignas(16) float conv2_b[anomaly_model::kChannels];
  alignas(16) float enc_w[anomaly_model::kBottleneck * anomaly_model::kChannels *
                          anomaly_model::kReduced];
  alignas(16) float enc_b[anomaly_model::kBottleneck];
  alignas(16) float dec1_w[anomaly_model::kHidden * anomaly_model::kBottleneck];
  alignas(16) float dec1_b[anomaly_model::kHidden];
  alignas(16) float dec2_w[anomaly_model::kWindow * anomaly_model::kMaxFeatures *
                           anomaly_model::kHidden];
  alignas(16) float dec2_b[anomaly_model::kWindow * anomaly_model::kMaxFeatures];
};

struct GruWeights {
  alignas(16) float w_ih[3 * anomaly_model::kGruHidden * anomaly_model::kGruInputs];
  alignas(16) float w_hh[3 * anomaly_model::kGruHidden * anomaly_model::kGruHidden];
  alignas(16) float b_ih[3 * anomaly_model::kGruHidden];
  alignas(16) float b_hh[3 * anomaly_model::kGruHidden];
  alignas(16) float head_w[anomaly_model::kGruOutputs * anomaly_model::kGruHidden];
  alignas(16) float head_b[anomaly_model::kGruOutputs];
};

// k-of-n persistence over consecutive windows.
class Persistence {
 public:
  void reset();
  bool update(bool exceed);
  bool alarm() const { return alarm_; }

 private:
  uint32_t history_ = 0;  // Bit i: window i frames ago exceeded.
  bool alarm_ = false;
};

// Conv autoencoder (motor or ground IR): reconstruction MSE of the last
// kWindow samples.
class AnomalyDetector {
 public:
  explicit AnomalyDetector(const anomaly_model::ConvAutoencoder &model);

  // Forget the window and alarm history (a new route was configured).
  void reset();

  // Add one raw (unscaled) sample. ``elapsed_ms`` is the time since the
  // route started; ``monitored`` is false during commanded manoeuvres.
  void add_sample(const float *raw, uint32_t elapsed_ms, bool monitored);

  bool has_score() const { return has_score_; }
  float score() const { return score_; }
  bool alarm() const { return persistence_.alarm(); }
  uint32_t last_inference_us() const { return last_inference_us_; }
  uint32_t max_inference_us() const { return max_inference_us_; }

 private:
  float compute_score() const;

  const anomaly_model::ConvAutoencoder &model_;
  ConvWeights weights_;  // RAM copy of model_'s layers.
  float window_[anomaly_model::kWindow][anomaly_model::kMaxFeatures]{};  // Scaled, ring.
  int next_ = 0;
  WindowGate gate_;
  Persistence persistence_;
  bool has_score_ = false;
  float score_ = 0.0F;
  uint32_t last_inference_us_ = 0;
  uint32_t max_inference_us_ = 0;
};

// Motor GRU plant model: predicts both encoder wheel speeds from the wheel
// targets, one step per frame from the start of the route. Score: per wheel,
// the mean squared prediction error over the last kGruWindow samples; the
// larger wheel.
class MotorGruDetector {
 public:
  MotorGruDetector();
  void reset();
  // ``motor``: target left/right, encoder speed left/right (raw), as for the
  // motor conv model.
  void add_sample(const float *motor, uint32_t elapsed_ms, bool monitored);

  bool has_score() const { return has_score_; }
  float score() const { return score_; }
  bool alarm() const { return persistence_.alarm(); }
  uint32_t last_inference_us() const { return last_inference_us_; }
  uint32_t max_inference_us() const { return max_inference_us_; }

 private:
  GruWeights weights_;  // RAM copy of the exported GRU.
  alignas(16) float state_[anomaly_model::kGruHidden]{};
  float errors_[anomaly_model::kGruWindow][anomaly_model::kGruOutputs]{};  // Ring.
  int next_ = 0;
  WindowGate gate_;
  Persistence persistence_;
  bool has_score_ = false;
  float score_ = 0.0F;
  uint32_t last_inference_us_ = 0;
  uint32_t max_inference_us_ = 0;
};

// Stuck ground sensor: one sensor's reading has not changed by kStuckEps for
// kStuckSamples frames while the robot drives (mean wheel speed over the
// same frames above kStuckSpeed). Driving, not another sensor moving: a
// sensor stuck at the on-the-line value takes the robot off the track, where
// the healthy sensors read flat floor too.
class IrStuckDetector {
 public:
  static constexpr int kSpan = anomaly_model::kStuckSamples + 1;  // Readings checked.

  void reset();
  // ``raw``: the three ground sensors; ``wheel_speed``: (|left| + |right|) / 2
  // of the encoder wheel speeds, rad/s.
  void add_sample(const float *raw, float wheel_speed, uint32_t elapsed_ms, bool monitored);

  bool stuck() const { return stuck_; }  // This frame's rule, before persistence.
  bool alarm() const { return persistence_.alarm(); }

 private:
  float readings_[kSpan][3]{};  // Raw, ring.
  float wheel_speeds_[kSpan]{};  // Ring, same slots.
  int next_ = 0;
  WindowGate gate_;
  Persistence persistence_;
  bool stuck_ = false;
};
