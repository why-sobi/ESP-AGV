#include "bench.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "esp_dsp.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

namespace bench {
namespace {

int round4(int value) { return (value + 3) / 4 * 4; }

// 16-byte-aligned internal RAM, freed on scope exit.
class Buffer {
 public:
  explicit Buffer(int floats)
      : size_(static_cast<size_t>(std::max(floats, 4)) * sizeof(float)),
        data_(static_cast<float *>(heap_caps_aligned_alloc(16, size_, MALLOC_CAP_INTERNAL))) {
    if (data_ != nullptr) {
      std::memset(data_, 0, size_);
    }
  }
  ~Buffer() { heap_caps_free(data_); }
  Buffer(const Buffer &) = delete;
  Buffer &operator=(const Buffer &) = delete;

  float *get() const { return data_; }
  size_t bytes() const { return size_; }

 private:
  size_t size_;
  float *data_;
};

void fill_random(Buffer &buffer, uint32_t &seed) {
  const size_t count = buffer.bytes() / sizeof(float);
  for (size_t i = 0; i < count; ++i) {
    seed = seed * 1664525U + 1013904223U;
    buffer.get()[i] = (static_cast<float>(seed >> 8) / 16777216.0F - 0.5F) * 0.2F;
  }
}

void dense(const float *weights, const float *bias, const float *input, float *out, int rows,
           int length) {
  for (int r = 0; r < rows; ++r) {
    dsps_dotprod_f32(weights + r * length, input, &out[r], length);
    out[r] += bias[r];
  }
}

Result timed(std::vector<uint32_t> &samples, size_t weight_bytes) {
  std::sort(samples.begin(), samples.end());
  Result result;
  result.ok = true;
  result.median_us = samples[samples.size() / 2];
  result.max_us = samples.back();
  result.weight_bytes = weight_bytes;
  return result;
}

}  // namespace

Result conv(int features, int window, int channels, int kernel, int bottleneck, int hidden,
            int repeats) {
  if (features < 1 || features > 8 || window < 4 || window > 64 || channels % 4 != 0 ||
      channels < 4 || channels > 64 || kernel % 2 == 0 || kernel < 1 || kernel > 7 ||
      bottleneck % 4 != 0 || bottleneck < 4 || bottleneck > 64 || hidden % 4 != 0 ||
      hidden < 4 || hidden > 128 || repeats < 1 || repeats > 200) {
    return {};
  }
  const int pad = kernel / 2;
  const int rows1 = round4(window);
  const int patch1 = round4(kernel * features);
  const int reduced = (window + 1) / 2;
  const int rows2 = round4(reduced);
  const int patch2 = kernel * channels;
  const int enc_inputs = channels * reduced;  // channels % 4 == 0.
  const int outputs = window * features;

  Buffer w1(patch1 * channels), b1(channels), w2(patch2 * channels), b2(channels);
  Buffer we(bottleneck * enc_inputs), be(bottleneck), wd1(hidden * bottleneck), bd1(hidden);
  Buffer wd2(outputs * hidden), bd2(outputs);
  Buffer x(window * features), p1(rows1 * patch1), h1(rows1 * channels);
  Buffer p2(rows2 * patch2), h2(rows2 * channels), z(bottleneck), hid(hidden), out(outputs);
  for (Buffer *b : {&w1, &b1, &w2, &b2, &we, &be, &wd1, &bd1, &wd2, &bd2, &x, &p1, &h1, &p2,
                    &h2, &z, &hid, &out}) {
    if (b->get() == nullptr) {
      return {};
    }
  }
  uint32_t seed = 1;
  for (Buffer *b : {&w1, &b1, &w2, &b2, &we, &be, &wd1, &bd1, &wd2, &bd2, &x}) {
    fill_random(*b, seed);
  }
  const size_t weight_bytes = w1.bytes() + b1.bytes() + w2.bytes() + b2.bytes() + we.bytes() +
                              be.bytes() + wd1.bytes() + bd1.bytes() + wd2.bytes() + bd2.bytes();

  std::vector<uint32_t> samples(repeats);
  volatile float sink = 0.0F;
  for (int run = 0; run < repeats; ++run) {
    const int64_t started = esp_timer_get_time();
    std::memset(p1.get(), 0, p1.bytes());
    for (int t = 0; t < window; ++t) {
      for (int k = 0; k < kernel; ++k) {
        const int source = t + k - pad;
        if (source >= 0 && source < window) {
          std::memcpy(p1.get() + t * patch1 + k * features, x.get() + source * features,
                      features * sizeof(float));
        }
      }
    }
    dspm_mult_f32(p1.get(), w1.get(), h1.get(), rows1, patch1, channels);
    for (int i = 0; i < window * channels; ++i) {
      h1.get()[i] = std::max(h1.get()[i] + b1.get()[i % channels], 0.0F);
    }
    std::memset(p2.get(), 0, p2.bytes());
    for (int t = 0; t < reduced; ++t) {
      for (int k = 0; k < kernel; ++k) {
        const int source = 2 * t + k - pad;
        if (source >= 0 && source < window) {
          std::memcpy(p2.get() + t * patch2 + k * channels, h1.get() + source * channels,
                      channels * sizeof(float));
        }
      }
    }
    dspm_mult_f32(p2.get(), w2.get(), h2.get(), rows2, patch2, channels);
    for (int i = 0; i < reduced * channels; ++i) {
      h2.get()[i] = std::max(h2.get()[i] + b2.get()[i % channels], 0.0F);
    }
    dense(we.get(), be.get(), h2.get(), z.get(), bottleneck, enc_inputs);
    dense(wd1.get(), bd1.get(), z.get(), hid.get(), hidden, bottleneck);
    for (int i = 0; i < hidden; ++i) {
      hid.get()[i] = std::max(hid.get()[i], 0.0F);
    }
    dense(wd2.get(), bd2.get(), hid.get(), out.get(), outputs, hidden);
    float squared = 0.0F;
    for (int j = 0; j < outputs; ++j) {
      const float error = out.get()[j] - x.get()[j];
      squared += error * error;
    }
    sink = sink + squared / static_cast<float>(outputs);
    samples[run] = static_cast<uint32_t>(esp_timer_get_time() - started);
  }
  return timed(samples, weight_bytes);
}

Result gru(int inputs, int hidden, int outputs, int repeats) {
  if (inputs < 1 || inputs > 16 || hidden % 4 != 0 || hidden < 4 || hidden > 128 ||
      outputs < 1 || outputs > 8 || repeats < 1 || repeats > 200) {
    return {};
  }
  const int in_padded = round4(inputs);
  const int gates = 3 * hidden;
  Buffer w_ih(gates * in_padded), w_hh(gates * hidden), b_ih(gates), b_hh(gates);
  Buffer head_w(outputs * hidden), head_b(outputs);
  Buffer x(in_padded), state(hidden), from_input(gates), from_state(gates), out(outputs);
  for (Buffer *b : {&w_ih, &w_hh, &b_ih, &b_hh, &head_w, &head_b, &x, &state, &from_input,
                    &from_state, &out}) {
    if (b->get() == nullptr) {
      return {};
    }
  }
  uint32_t seed = 2;
  for (Buffer *b : {&w_ih, &w_hh, &b_ih, &b_hh, &head_w, &head_b, &x}) {
    fill_random(*b, seed);
  }
  const size_t weight_bytes = w_ih.bytes() + w_hh.bytes() + b_ih.bytes() + b_hh.bytes() +
                              head_w.bytes() + head_b.bytes();
  std::vector<uint32_t> samples(repeats);
  for (int run = 0; run < repeats; ++run) {
    const int64_t started = esp_timer_get_time();
    dense(w_ih.get(), b_ih.get(), x.get(), from_input.get(), gates, in_padded);
    dense(w_hh.get(), b_hh.get(), state.get(), from_state.get(), gates, hidden);
    float *fi = from_input.get();
    float *fs = from_state.get();
    for (int j = 0; j < hidden; ++j) {
      const float reset = 1.0F / (1.0F + std::exp(-(fi[j] + fs[j])));
      const float update = 1.0F / (1.0F + std::exp(-(fi[hidden + j] + fs[hidden + j])));
      const float candidate = std::tanh(fi[2 * hidden + j] + reset * fs[2 * hidden + j]);
      state.get()[j] = (1.0F - update) * candidate + update * state.get()[j];
    }
    dense(head_w.get(), head_b.get(), state.get(), out.get(), outputs, hidden);
    samples[run] = static_cast<uint32_t>(esp_timer_get_time() - started);
  }
  return timed(samples, weight_bytes);
}

}  // namespace bench
