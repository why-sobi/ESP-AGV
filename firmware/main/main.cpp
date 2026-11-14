#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
constexpr size_t kLineBufferSize = 512;
constexpr size_t kMaxRouteLength = 32;

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

// Odometry. Three ground sensors cannot tell a junction from a line wide
// enough to cover all of them, so a junction pattern only counts within
// kJunctionWindowM of the distance the host gave for that junction.
constexpr float kWheelRadiusM = 0.02F;     // E-puck wheel radius.
constexpr float kJunctionWindowM = 0.10F;  // Accept a junction this far either side.

// U-turn: spin left in place, measuring the turn from the encoders, until at
// least kUTurnMinRad has turned and the center sensor is back on the line.
constexpr float kWheelBaseM = 0.052F;     // E-puck distance between the wheels.
constexpr float kUTurnSpeed = 2.0F;       // Wheel speed while spinning, rad/s.
constexpr float kUTurnMinRad = 2.79F;     // 160 degrees: ignore lines crossed earlier.
constexpr float kUTurnMaxRad = 4.71F;     // 270 degrees: give up, the line was not found.

enum class State {
  kNoRoute,
  kFollowing,
  kJunctionLeft,
  kJunctionStraight,
  kJunctionRight,
  kUTurn,
  kLineLost,
  kAtDestination,
  kLinkLost,
  kJunctionMissed,
};

struct RouteStep {
  char action = 'D';
  float distance_m = 0.0F;
};

struct Controller {
  RouteStep route[kMaxRouteLength]{};
  size_t route_length = 0;
  size_t route_index = 0;
  State state = State::kNoRoute;
  State state_before_link_loss = State::kNoRoute;
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
  bool odometry_started = false;
  float encoder_start_left = 0.0F;
  float encoder_start_right = 0.0F;
  float distance_m = 0.0F;
  float heading_rad = 0.0F;  // Left-positive turn since the route started.
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
    case State::kUTurn:
      return "U_TURN";
    case State::kLineLost:
      return "LINE_LOST";
    case State::kAtDestination:
      return "AT_DEST";
    case State::kLinkLost:
      return "LINK_LOST";
    case State::kJunctionMissed:
      return "JUNCTION_MISSED";
  }
  return "ERROR";
}

void send_response(uint32_t sequence, float left, float right) {
  char response[112];
  const int length = std::snprintf(
      response, sizeof(response), "M,%lu,%.3f,%.3f,%s,%u,%.4f\n",
      static_cast<unsigned long>(sequence), left, right, state_name(g_controller.state),
      static_cast<unsigned>(g_controller.route_index), g_controller.distance_m);
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

// Parse "<action>@<metres>" steps separated by commas, e.g. "R@0.265,D@0.779".
// Distances are odometry distances from the start of the route; they must not
// decrease, and the last step must be the destination 'D'. A 'U' (U-turn in
// place) may only be the first step, e.g. "U@0.000,L@0.514,D@0.779".
bool configure_route(char *steps) {
  RouteStep parsed[kMaxRouteLength];
  size_t length = 0;
  float previous_distance = 0.0F;
  char *save = nullptr;
  for (char *token = strtok_r(steps, ",", &save); token != nullptr;
       token = strtok_r(nullptr, ",", &save)) {
    if (length == kMaxRouteLength) {
      return false;
    }
    const char action = token[0];
    if ((action != 'L' && action != 'S' && action != 'R' && action != 'D' &&
         action != 'U') ||
        token[1] != '@' || (action == 'U' && length != 0)) {
      return false;
    }
    char *end = nullptr;
    const float distance = std::strtof(token + 2, &end);
    if (end == token + 2 || *end != '\0' || !std::isfinite(distance) ||
        distance < previous_distance) {
      return false;
    }
    parsed[length++] = RouteStep{action, distance};
    previous_distance = distance;
  }
  if (length == 0 || parsed[length - 1].action != 'D') {
    return false;
  }
  for (size_t i = 0; i + 1 < length; ++i) {
    if (parsed[i].action == 'D') {
      return false;
    }
  }

  std::copy(parsed, parsed + length, g_controller.route);
  g_controller.route_length = length;
  g_controller.route_index = 0;
  g_controller.route_loaded = true;
  g_controller.junction_latched = false;
  g_controller.junction_frames = 0;
  g_controller.junction_clear_frames = 0;
  g_controller.previous_error = 0.0F;
  g_controller.filtered_error_rate = 0.0F;
  g_controller.turn_search_ms = 0;
  // Arm the link timeout and zero the odometry on the first sensor frame: the
  // host may take a while to start streaming after it configures the route.
  g_controller.last_sensor_seq = 0;
  g_controller.last_sensor_us = 0;
  g_controller.odometry_started = false;
  g_controller.distance_m = 0.0F;
  g_controller.heading_rad = 0.0F;
  g_controller.state = parsed[0].action == 'U' ? State::kUTurn : State::kFollowing;
  uart_write_bytes(kSerialPort, "A,ROUTE_OK\n", sizeof("A,ROUTE_OK\n") - 1);
  return true;
}

void update_odometry(float encoder_left, float encoder_right) {
  if (!g_controller.odometry_started) {
    g_controller.encoder_start_left = encoder_left;
    g_controller.encoder_start_right = encoder_right;
    g_controller.odometry_started = true;
  }
  const float left_rad = encoder_left - g_controller.encoder_start_left;
  const float right_rad = encoder_right - g_controller.encoder_start_right;
  g_controller.distance_m = kWheelRadiusM * 0.5F * (left_rad + right_rad);
  g_controller.heading_rad = kWheelRadiusM * (right_rad - left_rad) / kWheelBaseM;
}

// Spin left in place for a pending 'U' step. Returns true while still
// turning (the response has been sent); false once the line is reacquired
// and normal following should handle this frame.
bool run_u_turn(uint32_t sequence, bool center_active) {
  if (g_controller.route[g_controller.route_index].action != 'U') {
    return false;
  }
  if (g_controller.heading_rad >= kUTurnMinRad && center_active) {
    ++g_controller.route_index;
    g_controller.state = State::kFollowing;
    g_controller.previous_error = 0.0F;
    g_controller.filtered_error_rate = 0.0F;
    return false;
  }
  if (g_controller.heading_rad > kUTurnMaxRad) {
    g_controller.state = State::kLineLost;
    g_controller.route_loaded = false;
    send_response(sequence, 0.0F, 0.0F);
    return true;
  }
  g_controller.state = State::kUTurn;
  send_response(sequence, -kUTurnSpeed, kUTurnSpeed);
  return true;
}

// Stop at the destination distance, or if the next junction's window has
// passed without the junction pattern being seen.
void check_route_distance() {
  if (g_controller.junction_latched) {
    return;
  }
  const RouteStep &next = g_controller.route[g_controller.route_index];
  if (next.action == 'D') {
    if (g_controller.distance_m >= next.distance_m) {
      ++g_controller.route_index;
      g_controller.state = State::kAtDestination;
    }
  } else if (g_controller.distance_m > next.distance_m + kJunctionWindowM) {
    g_controller.state = State::kJunctionMissed;
    g_controller.route_loaded = false;
  }
}

void maybe_enter_junction(bool left_active, bool right_active) {
  if (g_controller.junction_latched || !g_controller.route_loaded) {
    return;
  }
  const RouteStep &next = g_controller.route[g_controller.route_index];
  const bool in_window =
      next.action != 'D' &&
      std::fabs(g_controller.distance_m - next.distance_m) <= kJunctionWindowM;

  if (in_window && left_active && right_active) {
    ++g_controller.junction_frames;
    if (g_controller.junction_frames >= kJunctionConfirmFrames) {
      ++g_controller.route_index;
      g_controller.junction_latched = true;
      g_controller.junction_clear_frames = 0;
      g_controller.turn_search_ms = 0;

      switch (next.action) {
        case 'L':
          g_controller.state = State::kJunctionLeft;
          break;
        case 'S':
          g_controller.state = State::kJunctionStraight;
          break;
        case 'R':
          g_controller.state = State::kJunctionRight;
          break;
      }
    }
  } else {
    g_controller.junction_frames = 0;
  }
}

void maybe_leave_junction(bool left_active, bool right_active) {
  if (!g_controller.junction_latched) {
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
  float encoders[2]{};
  char extra = '\0';
  if (std::sscanf(line, "S,%u,%f,%f,%f,%f,%f,%f%c", &sequence, &dt_ms, &readings[0],
                  &readings[1], &readings[2], &encoders[0], &encoders[1], &extra) != 7 ||
      !std::isfinite(dt_ms) || dt_ms <= 0.0F || dt_ms > 250.0F ||
      !std::isfinite(readings[0]) || !std::isfinite(readings[1]) ||
      !std::isfinite(readings[2]) || !std::isfinite(encoders[0]) ||
      !std::isfinite(encoders[1])) {
    send_error("BAD_SENSOR_FRAME");
    return;
  }

  g_controller.last_sensor_seq = sequence;
  g_controller.last_sensor_us = esp_timer_get_time();

  // The simulation pauses with the host, so resume once frames arrive again.
  // Wheel targets were zero while the link was lost.
  if (g_controller.state == State::kLinkLost) {
    g_controller.state = g_controller.state_before_link_loss;
  }

  if (!g_controller.route_loaded || g_controller.state == State::kAtDestination) {
    send_response(sequence, 0.0F, 0.0F);
    return;
  }

  update_odometry(encoders[0], encoders[1]);
  const float dt = std::clamp(dt_ms / 1000.0F, 0.005F, 0.100F);
  const float left = std::max(0.0F, kLineThreshold - readings[0]);
  const float center = std::max(0.0F, kLineThreshold - readings[1]);
  const float right = std::max(0.0F, kLineThreshold - readings[2]);
  const float strength = left + center + right;
  const bool left_active = left >= kMinLineStrength;
  const bool right_active = right >= kMinLineStrength;

  if (run_u_turn(sequence, center >= kMinLineStrength)) {
    return;
  }

  maybe_enter_junction(left_active, right_active);
  maybe_leave_junction(left_active, right_active);
  check_route_distance();

  if (g_controller.state == State::kAtDestination ||
      g_controller.state == State::kJunctionMissed) {
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
  } else {
    // Line lost: turn slowly towards where it was last seen, for a bounded time.
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
    g_controller.state_before_link_loss = g_controller.state;
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
  static char line[kLineBufferSize]{};
  size_t used = 0;
  uint8_t input[64];

  while (true) {
    // Wait for one byte, then drain what is already buffered. Asking for the
    // whole buffer would block until 64 bytes or the timeout arrived, delaying
    // every ~35-byte sensor frame by the full 50 ms.
    int count = uart_read_bytes(kSerialPort, input, 1, pdMS_TO_TICKS(50));
    if (count > 0) {
      size_t buffered = 0;
      uart_get_buffered_data_len(kSerialPort, &buffered);
      buffered = std::min(buffered, sizeof(input) - 1);
      if (buffered > 0) {
        const int more = uart_read_bytes(kSerialPort, input + 1, buffered, 0);
        if (more > 0) {
          count += more;
        }
      }
    }
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
