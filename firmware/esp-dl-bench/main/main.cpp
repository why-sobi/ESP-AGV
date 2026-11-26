// ESP-DL benchmark for the quantized detectors (scripts/esp/int8_bench.py).
//
// Serial, 115200 8-N-1, one command per line:
//   "T,<model>,<repeats>"  -> "t,<ok>,<median_us>,<max_us>,<param_bytes>,<heap_used_bytes>"
//       times quantize inputs + Model::run + dequantize outputs, zero inputs.
//   "R,<model>,<v0>,<v1>,..." -> "r,<ok>,<o0>,<o1>,..."
//       runs float inputs (all model inputs concatenated, in name order) and
//       returns the dequantized outputs (in name order).
// <model> is e.g. motor_w8a8, ir_w16a16, motor_gru_w8a16.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "dl_model_base.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP-DL's prebuilt model loader (libfbs_model.a) calls esp_log_write, which
// this IDF 6.0 build does not provide; logging is off here anyway. Weak, so a
// real one wins.
extern "C" __attribute__((weak)) void esp_log_write(esp_log_level_t, const char *, const char *, ...) {}

namespace {

constexpr uart_port_t kSerialPort = UART_NUM_0;
constexpr size_t kLineBufferSize = 4096;

#define EMBEDDED(name)                                                        \
  extern const uint8_t name##_start[] asm("_binary_" #name "_espdl_start");
EMBEDDED(motor_w8a8)
EMBEDDED(ir_w8a8)
EMBEDDED(motor_gru_w8a8)
EMBEDDED(motor_w8a16)
EMBEDDED(ir_w8a16)
EMBEDDED(motor_gru_w8a16)
EMBEDDED(motor_w16a16)
EMBEDDED(ir_w16a16)
EMBEDDED(motor_gru_w16a16)

const std::map<std::string, const uint8_t *> kModels = {
    {"motor_w8a8", motor_w8a8_start},         {"ir_w8a8", ir_w8a8_start},
    {"motor_gru_w8a8", motor_gru_w8a8_start}, {"motor_w8a16", motor_w8a16_start},
    {"ir_w8a16", ir_w8a16_start},             {"motor_gru_w8a16", motor_gru_w8a16_start},
    {"motor_w16a16", motor_w16a16_start},     {"ir_w16a16", ir_w16a16_start},
    {"motor_gru_w16a16", motor_gru_w16a16_start},
};

struct Loaded {
  dl::Model *model = nullptr;
  size_t heap_used = 0;
};
std::map<std::string, Loaded> g_loaded;

void write(const char *text) { uart_write_bytes(kSerialPort, text, std::strlen(text)); }

Loaded *load(const std::string &name) {
  auto found = g_loaded.find(name);
  if (found != g_loaded.end()) {
    return &found->second;
  }
  auto model = kModels.find(name);
  if (model == kModels.end()) {
    return nullptr;
  }
  const size_t before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  // param_copy: weights in internal RAM, as the float detectors keep theirs.
  Loaded loaded;
  loaded.model = new dl::Model(reinterpret_cast<const char *>(model->second),
                               fbs::MODEL_LOCATION_IN_FLASH_RODATA, 0,
                               dl::MEMORY_MANAGER_GREEDY, nullptr, true);
  loaded.heap_used = before - heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  return &(g_loaded[name] = loaded);
}

float scale_of(dl::TensorBase *tensor) { return std::ldexp(1.0F, tensor->get_exponent()); }

void set_inputs(dl::Model *model, const float *values) {
  size_t offset = 0;
  for (auto &[name, tensor] : model->get_inputs()) {
    const float inverse = 1.0F / scale_of(tensor);
    const int size = tensor->get_size();
    if (tensor->get_dtype() == dl::DATA_TYPE_INT8) {
      int8_t *data = tensor->get_element_ptr<int8_t>();
      for (int i = 0; i < size; ++i) {
        data[i] = static_cast<int8_t>(
            std::clamp(std::lround(values[offset + i] * inverse), -128L, 127L));
      }
    } else {
      int16_t *data = tensor->get_element_ptr<int16_t>();
      for (int i = 0; i < size; ++i) {
        data[i] = static_cast<int16_t>(
            std::clamp(std::lround(values[offset + i] * inverse), -32768L, 32767L));
      }
    }
    offset += size;
  }
}

size_t input_count(dl::Model *model) {
  size_t count = 0;
  for (auto &[name, tensor] : model->get_inputs()) {
    count += tensor->get_size();
  }
  return count;
}

void get_outputs(dl::Model *model, std::vector<float> &out) {
  out.clear();
  for (auto &[name, tensor] : model->get_outputs()) {
    const float scale = scale_of(tensor);
    for (int i = 0; i < tensor->get_size(); ++i) {
      out.push_back(scale * (tensor->get_dtype() == dl::DATA_TYPE_INT8
                                 ? tensor->get_element_ptr<int8_t>()[i]
                                 : tensor->get_element_ptr<int16_t>()[i]));
    }
  }
}

void process(char *line) {
  char *save = nullptr;
  const char *command = strtok_r(line, ",", &save);
  const char *name = strtok_r(nullptr, ",", &save);
  if (command == nullptr || name == nullptr) {
    write("e,BAD_COMMAND\n");
    return;
  }
  Loaded *loaded = load(name);
  if (loaded == nullptr) {
    write(command[0] == 'T' ? "t,0\n" : "r,0\n");
    return;
  }
  dl::Model *model = loaded->model;
  std::vector<float> inputs(input_count(model), 0.0F);
  std::vector<float> outputs;
  char reply[64];
  if (command[0] == 'T') {
    const char *text = strtok_r(nullptr, ",", &save);
    const int repeats = text != nullptr ? std::clamp(std::atoi(text), 1, 500) : 51;
    std::vector<uint32_t> samples(repeats);
    for (int i = 0; i < repeats; ++i) {
      const int64_t started = esp_timer_get_time();
      set_inputs(model, inputs.data());
      model->run();
      get_outputs(model, outputs);
      samples[i] = static_cast<uint32_t>(esp_timer_get_time() - started);
    }
    std::sort(samples.begin(), samples.end());
    size_t params = 0;
    for (auto &[key, info] : model->get_memory_info()) {
      if (key == "parameter_copy") {
        params += info.internal + info.psram + info.flash;
      }
    }
    std::snprintf(reply, sizeof(reply), "t,1,%lu,%lu,%u,%u\n",
                  static_cast<unsigned long>(samples[repeats / 2]),
                  static_cast<unsigned long>(samples.back()), static_cast<unsigned>(params),
                  static_cast<unsigned>(loaded->heap_used));
    write(reply);
    return;
  }
  for (float &value : inputs) {
    const char *text = strtok_r(nullptr, ",", &save);
    if (text == nullptr) {
      write("r,0\n");
      return;
    }
    value = std::strtof(text, nullptr);
  }
  set_inputs(model, inputs.data());
  model->run();
  get_outputs(model, outputs);
  write("r,1");
  for (float value : outputs) {
    std::snprintf(reply, sizeof(reply), ",%.6g", value);
    write(reply);
  }
  write("\n");
}

void serial_task(void *) {
  static char line[kLineBufferSize];
  size_t used = 0;
  uint8_t byte = 0;
  while (true) {
    if (uart_read_bytes(kSerialPort, &byte, 1, pdMS_TO_TICKS(100)) != 1) {
      continue;
    }
    if (byte == '\n') {
      line[used] = '\0';
      if (used > 0 && line[used - 1] == '\r') {
        line[used - 1] = '\0';
      }
      process(line);
      used = 0;
    } else if (used + 1 < sizeof(line)) {
      line[used++] = static_cast<char>(byte);
    }
  }
}

}  // namespace

extern "C" void app_main() {
  uart_config_t config = {};
  config.baud_rate = 115200;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_param_config(kSerialPort, &config));
  ESP_ERROR_CHECK(uart_set_pin(kSerialPort, GPIO_NUM_43, GPIO_NUM_44, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(kSerialPort, 8192, 8192, 0, nullptr, 0));
  // Same core as the AGV control task.
  xTaskCreatePinnedToCore(serial_task, "bench", 16384, nullptr, 5, nullptr, 1);
}
