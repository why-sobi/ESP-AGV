#pragma once

#include <cstdint>

#include "anomaly_model.h"

// Streaming anomaly detector for one conv autoencoder (motor or ground IR).
// It mirrors scripts/models/train_autoencoder.py and evaluate_faults.py: one
// sample per sensor frame, a sliding window of anomaly_model::kWindow samples,
// a reconstruction-MSE score per window, and a persistent alarm when
// kPersistK of the last kPersistN monitored windows exceed the threshold.
class AnomalyDetector {
 public:
  explicit AnomalyDetector(const anomaly_model::ConvAutoencoder &model) : model_(model) {}

  // Forget the window and alarm history (a new route was configured).
  void reset();

  // Add one raw (unscaled) sample. ``elapsed_ms`` is the time since the
  // route started; ``monitored`` is false during commanded manoeuvres.
  void add_sample(const float *raw, uint32_t elapsed_ms, bool monitored);

  bool has_score() const { return has_score_; }
  float score() const { return score_; }
  bool alarm() const { return alarm_; }
  uint32_t last_inference_us() const { return last_inference_us_; }
  uint32_t max_inference_us() const { return max_inference_us_; }

 private:
  float compute_score() const;

  const anomaly_model::ConvAutoencoder &model_;
  float window_[anomaly_model::kWindow][anomaly_model::kMaxFeatures]{};  // Scaled, ring.
  bool sample_monitored_[anomaly_model::kWindow]{};
  uint32_t sample_ms_[anomaly_model::kWindow]{};
  int next_ = 0;
  int count_ = 0;
  uint32_t exceed_history_ = 0;  // Bit i: window i frames ago exceeded.
  bool has_score_ = false;
  float score_ = 0.0F;
  bool alarm_ = false;
  uint32_t last_inference_us_ = 0;
  uint32_t max_inference_us_ = 0;
};
