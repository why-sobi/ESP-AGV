#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr uart_port_t kSerialPort = UART_NUM_0;
constexpr int kBaudRate = 115200;
constexpr gpio_num_t kSerialTxPin = GPIO_NUM_43;
constexpr gpio_num_t kSerialRxPin = GPIO_NUM_44;
constexpr size_t kRxBufferSize = 1024;
constexpr size_t kLineBufferSize = 128;
constexpr size_t kMaxRouteLength = 64;

// These PD settings match the current Webots line follower and still need
// tuning across the project's supported simulated routes and speeds.
constexpr float kLineThreshold = 400.0F;       // Below this, a sensor sees dark track.
constexpr float kMinLineStrength = 30.0F;      // Reject weak threshold crossings/noise.
constexpr float kKp = 1.2F;                    // Steering response to line offset.
constexpr float kKd = 0.015F;                  // Damping response to changing line offset.
constexpr float kMaxSpeed = 3.5F;              // Straight-line wheel speed, rad/s.
constexpr float kMinSpeed = 2.5F;              // Slow speed for sharp corrections, rad/s.
constexpr float kMaxWheelSpeed = 5.0F;         // E-puck-compatible output cap, rad/s.
constexpr float kMaxSteering = 1.5F;           // Bound differential wheel correction.
constexpr float kDerivativeLimit = 5.0F;       // Limit noise amplification in derivative.
constexpr float kDerivativeSmoothing = 0.25F;  // Low-pass weight for derivative estimate.
constexpr unsigned kJunctionConfirmFrames = 2; // Require stable junction readings.
constexpr unsigned kJunctionReleaseFrames = 3; // Require branch reacquisition to unlatch.
constexpr uint32_t kTurnSearchLimitMs = 750;   // Stop if a commanded turn loses the line.
constexpr uint32_t kSensorTimeoutMs = 250;     // Stop if the host stops sending sensor data.

enum class State {
  kNoRoute,
  kFollowing,
  kJunctionLeft,
  kJunctionStraight,
  kJunctionRight,
  kLineLost,
  kAtDestination,
  kLinkLost,
};

struct Controller {
  char route[kMaxRouteLength + 1]{};
  size_t route_length = 0;
  size_t route_index = 0;
  State state = State::kNoRoute;
  bool route_loaded = false;
  bool junction_latched = false;
  unsigned junction_frames = 0;
  unsigned junction_clear_frames = 0;
  float previous_error = 0.0F;
  float filtered_error_rate = 0.0F;
  float last_turn_direction = -1.0F;
  uint32_t turn_search_ms = 0;
  uint32_t last_sensor_seq = 0;
  int64_t last_sensor_us = 0;
};

Controller g_controller;

const char *state_name(State state) {
  switch (state) {
    case State::kNoRoute:
      return "NO_ROUTE";
    case State::kFollowing:
      return "FOLLOWING";
    case State::kJunctionLeft:
      return "TURN_LEFT";
    case State::kJunctionStraight:
      return "STRAIGHT";
    case State::kJunctionRight:
      return "TURN_RIGHT";
    case State::kLineLost:
      return "LINE_LOST";
    case State::kAtDestination:
      return "AT_DEST";
    case State::kLinkLost:
      return "LINK_LOST";
  }
  return "ERROR";
}

void send_response(uint32_t sequence, float left, float right) {
  char response[96];
  const int length = std::snprintf(
      response, sizeof(response), "M,%lu,%.3f,%.3f,%s,%u\n",
      static_cast<unsigned long>(sequence), left, right, state_name(g_controller.state),
      static_cast<unsigned>(g_controller.route_index));
  if (length > 0 && static_cast<size_t>(length) < sizeof(response)) {
    uart_write_bytes(kSerialPort, response, length);
  }
}

void send_error(const char *reason) {
  char response[48];
  const int length = std::snprintf(response, sizeof(response), "E,%s\n", reason);
  if (length > 0 && static_cast<size_t>(length) < sizeof(response)) {
    uart_write_bytes(kSerialPort, response, length);
  }
}

bool configure_route(const char *actions) {
  const size_t length = std::strlen(actions);
  if (length == 0 || length > kMaxRouteLength || actions[length - 1] != 'D') {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    const char action = actions[i];
    if (action != 'L' && action != 'S' && action != 'R' && action != 'D') {
      return false;
    }
    if (action == 'D' && i != length - 1) {
      return false;
    }
  }

  std::memcpy(g_controller.route, actions, length + 1);
  g_controller.route_length = length;
  g_controller.route_index = 0;
  g_controller.route_loaded = true;
  g_controller.junction_latched = false;
  g_controller.junction_frames = 0;
  g_controller.junction_clear_frames = 0;
  g_controller.previous_error = 0.0F;
  g_controller.filtered_error_rate = 0.0F;
  g_controller.turn_search_ms = 0;
  g_controller.last_sensor_us = esp_timer_get_time();
  g_controller.state = State::kFollowing;
  uart_write_bytes(kSerialPort, "A,ROUTE_OK\n", sizeof("A,ROUTE_OK\n") - 1);
  return true;
}

void maybe_enter_junction(bool left_active, bool right_active) {
  if (g_controller.junction_latched || !g_controller.route_loaded) {
    return;
  }

  if (left_active && right_active) {
    ++g_controller.junction_frames;
    if (g_controller.junction_frames >= kJunctionConfirmFrames) {
      const char action = g_controller.route[g_controller.route_index];
      ++g_controller.route_index;
      g_controller.junction_latched = true;
      g_controller.junction_clear_frames = 0;
      g_controller.turn_search_ms = 0;

      switch (action) {
        case 'L':
          g_controller.state = State::kJunctionLeft;
          break;
        case 'S':
          g_controller.state = State::kJunctionStraight;
          break;
        case 'R':
          g_controller.state = State::kJunctionRight;
          break;
        case 'D':
          g_controller.state = State::kAtDestination;
          break;
      }
    }
  } else {
    g_controller.junction_frames = 0;
  }
}

void maybe_leave_junction(bool left_active, bool right_active) {
  if (!g_controller.junction_latched || g_controller.state == State::kAtDestination) {
    return;
  }

  if (left_active && right_active) {
    g_controller.junction_clear_frames = 0;
  } else {
    ++g_controller.junction_clear_frames;
    if (g_controller.junction_clear_frames >= kJunctionReleaseFrames) {
      g_controller.junction_latched = false;
      g_controller.junction_clear_frames = 0;
      g_controller.state = State::kFollowing;
      g_controller.turn_search_ms = 0;
    }
  }
}

void process_sensor_frame(char *line) {
  unsigned sequence = 0;
  float dt_ms = 0.0F;
  float readings[3]{};
  char extra = '\0';
  if (std::sscanf(line, "S,%u,%f,%f,%f,%f%c", &sequence, &dt_ms, &readings[0],
                  &readings[1], &readings[2], &extra) != 5 ||
      !std::isfinite(dt_ms) || dt_ms <= 0.0F || dt_ms > 250.0F ||
      !std::isfinite(readings[0]) || !std::isfinite(readings[1]) ||
      !std::isfinite(readings[2])) {
    send_error("BAD_SENSOR_FRAME");
    return;
  }

  g_controller.last_sensor_seq = sequence;
  g_controller.last_sensor_us = esp_timer_get_time();
  const float dt = std::clamp(dt_ms / 1000.0F, 0.005F, 0.100F);
  const float left = std::max(0.0F, kLineThreshold - readings[0]);
  const float center = std::max(0.0F, kLineThreshold - readings[1]);
  const float right = std::max(0.0F, kLineThreshold - readings[2]);
  const float strength = left + center + right;
  const bool left_active = left >= kMinLineStrength;
  const bool right_active = right >= kMinLineStrength;

  if (!g_controller.route_loaded || g_controller.state == State::kAtDestination ||
      g_controller.state == State::kLinkLost) {
    send_response(sequence, 0.0F, 0.0F);
    return;
  }

  maybe_enter_junction(left_active, right_active);
  if (g_controller.junction_latched) {
    maybe_leave_junction(left_active, right_active);
  }

  if (g_controller.state == State::kAtDestination) {
    send_response(sequence, 0.0F, 0.0F);
    return;
  }

  float error = 0.0F;
  if (g_controller.junction_latched) {
    if (g_controller.state == State::kJunctionLeft) {
      error = -1.0F;
      g_controller.last_turn_direction = -1.0F;
    } else if (g_controller.state == State::kJunctionRight) {
      error = 1.0F;
      g_controller.last_turn_direction = 1.0F;
    }
    g_controller.filtered_error_rate = 0.0F;
  } else if (strength >= kMinLineStrength) {
    error = (right - left) / strength;
    const float raw_error_rate = (error - g_controller.previous_error) / dt;
    const float limited_rate =
        std::clamp(raw_error_rate, -kDerivativeLimit, kDerivativeLimit);
    g_controller.filtered_error_rate =
        kDerivativeSmoothing * limited_rate +
        (1.0F - kDerivativeSmoothing) * g_controller.filtered_error_rate;
    g_controller.previous_error = error;
    if (error < 0.0F) {
      g_controller.last_turn_direction = -1.0F;
    } else if (error > 0.0F) {
      g_controller.last_turn_direction = 1.0F;
    }
    g_controller.state = State::kFollowing;
    g_controller.turn_search_ms = 0;
  } else if (g_controller.state == State::kJunctionLeft ||
             g_controller.state == State::kJunctionRight) {
    g_controller.turn_search_ms += static_cast<uint32_t>(dt_ms);
    if (g_controller.turn_search_ms > kTurnSearchLimitMs) {
      g_controller.state = State::kLineLost;
      g_controller.route_loaded = false;
      send_response(sequence, 0.0F, 0.0F);
      return;
    }
    const float search_steering = 0.75F * g_controller.last_turn_direction;
    const float search_left = std::clamp(1.0F + search_steering, 0.0F, kMaxWheelSpeed);
    const float search_right = std::clamp(1.0F - search_steering, 0.0F, kMaxWheelSpeed);
    send_response(sequence, search_left, search_right);
    return;
  } else {
    g_controller.state = State::kLineLost;
    send_response(sequence, 0.0F, 0.0F);
    return;
  }

  const float turn_amount = std::min(1.0F, std::fabs(error));
  const float base_speed = kMaxSpeed - (kMaxSpeed - kMinSpeed) * turn_amount;
  float steering = kKp * error + kKd * g_controller.filtered_error_rate;
  steering = std::clamp(steering, -kMaxSteering, kMaxSteering);
  const float left_target =
      std::clamp(base_speed + steering, 0.0F, kMaxWheelSpeed);
  const float right_target =
        std::clamp(base_speed - steering, 0.0F, kMaxWheelSpeed);
  send_response(sequence, left_target, right_target);
}

void process_command(char *line) {
  if (std::strncmp(line, "R,", 2) == 0) {
    if (!configure_route(line + 2)) {
      send_error("BAD_ROUTE");
    }
    return;
  }
  if (line[0] == 'S' && line[1] == ',') {
    process_sensor_frame(line);
    return;
  }
  send_error("UNKNOWN_COMMAND");
}

void check_sensor_timeout() {
  if (!g_controller.route_loaded || g_controller.state == State::kAtDestination ||
      g_controller.state == State::kLinkLost || g_controller.last_sensor_us == 0) {
    return;
  }

  const int64_t age_us = esp_timer_get_time() - g_controller.last_sensor_us;
  if (age_us > static_cast<int64_t>(kSensorTimeoutMs) * 1000) {
    g_controller.state = State::kLinkLost;
    send_response(g_controller.last_sensor_seq, 0.0F, 0.0F);
  }
}

void initialize_serial() {
  uart_config_t config = {};
  config.baud_rate = kBaudRate;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_DEFAULT;

  ESP_ERROR_CHECK(uart_param_config(kSerialPort, &config));
  ESP_ERROR_CHECK(uart_set_pin(kSerialPort, kSerialTxPin, kSerialRxPin,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(kSerialPort, kRxBufferSize, 0, 0, nullptr, 0));
}

void serial_task(void *) {
  char line[kLineBufferSize]{};
  size_t used = 0;
  uint8_t input[64];

  while (true) {
    const int count = uart_read_bytes(kSerialPort, input, sizeof(input), pdMS_TO_TICKS(50));
    if (count > 0) {
      for (int i = 0; i < count; ++i) {
        const char byte = static_cast<char>(input[i]);
        if (byte == '\n') {
          line[used] = '\0';
          if (used > 0 && line[used - 1] == '\r') {
            line[used - 1] = '\0';
          }
          process_command(line);
          used = 0;
        } else if (used + 1 < sizeof(line)) {
          line[used++] = byte;
        } else {
          used = 0;
          send_error("FRAME_TOO_LONG");
        }
      }
    }
    check_sensor_timeout();
  }
}

}  // namespace

extern "C" void app_main() {
  initialize_serial();
  xTaskCreate(serial_task, "serial_control", 4096, nullptr, 5, nullptr);
}
