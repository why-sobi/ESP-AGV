#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "anomaly.h"
#include "bench.h"
#include "notify.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr uart_port_t kSerialPort = UART_NUM_0;
constexpr int kBaudRate = 115200;
constexpr gpio_num_t kSerialTxPin = GPIO_NUM_43;
constexpr gpio_num_t kSerialRxPin = GPIO_NUM_44;
constexpr size_t kRxBufferSize = 1024;
constexpr size_t kLineBufferSize = 1024;  // Routes with place labels are a few hundred bytes.
constexpr size_t kMaxRouteLength = 32;
constexpr size_t kMaxPlaceLength = 64;  // Per-step place label for notifications.

// These PD settings match the current Webots line follower and still need
// tuning across the project's supported simulated routes and speeds.
constexpr float kLineThreshold = 400.0F;       // Below this, a sensor sees dark track.
constexpr float kMinLineStrength = 30.0F;      // Reject weak threshold crossings/noise.
constexpr float kKp = 1.2F;                    // Steering response to line offset.
constexpr float kKd = 0.015F;                  // Damping response to changing line offset.
constexpr float kDefaultMaxSpeed = 3.5F;       // Straight-line wheel speed, rad/s.
constexpr float kMinSpeedRatio = 2.5F / 3.5F;  // Sharp-correction speed / straight speed.
constexpr float kLowestMaxSpeed = 1.0F;        // Accepted range for a route's 'V' speed.
constexpr float kHighestMaxSpeed = 4.5F;
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
// A junction pattern already present when the window opens (all three sensors
// dark along a line as wide as the sensor spread, e.g. horizontal lines on the
// stretched track image) says nothing about where the junction is: act on it
// only this close to the expected distance, where junctions are taken.
constexpr float kStaleJunctionLeadM = 0.03F;
// A corner ('l'/'r': the map's turn is ~90 degrees onto a crossing line): slow
// down, drive the wheel axle onto the junction, spin in place like the U-turn
// until at least kCornerMinRad has turned and the center sensor is on a line,
// and keep the speed down this far after it while the PD loop settles.
constexpr float kCornerSpeed = 1.5F;       // rad/s approaching and settling.
constexpr float kCornerSettleM = 0.04F;
constexpr float kCornerSpinSpeed = 2.0F;   // rad/s, as the U-turn.
constexpr float kCornerMinRad = 1.05F;     // 60 degrees.
constexpr float kCornerMaxRad = 2.36F;     // 135 degrees: the line was not found.
// When the sensors run off the far side of the crossing line, the axle is
// still this far short of its centre (sensors ~3 cm ahead of the axle, minus
// half a line width): drive on this far, then spin.
constexpr float kCornerAxleM = 0.02F;
// A bear turn ('<'/'>') onto a Y branch: the junction pattern clears while the
// branch is still beside the old line, so the turn is held until the robot has
// turned this far (encoder heading), at most kBearMaxRad.
constexpr float kBearMinRad = 0.44F;  // 25 degrees.
constexpr float kBearMaxRad = 1.4F;   // 80 degrees.
// When the junction pattern appears, the sensors are at the crossing line's
// near side: the axle is ~3 cm plus half a line width short of its centre.
constexpr float kCornerEntryAxleM = 0.04F;
// Within this distance of the destination, losing the line is arriving (the
// sensors run past the end of the line or across a T's crossbar first): drive
// straight on to the destination distance instead of searching.
constexpr float kArrivalSlackM = 0.05F;
constexpr float kArrivalSpeed = 1.0F;           // rad/s while running in blind.
constexpr uint32_t kArrivalLimitMs = 3000;      // Then stop with LINE_LOST anyway.

// U-turn: spin left in place, measuring the turn from the encoders, until at
// least kUTurnMinRad has turned and the center sensor is back on the line.
constexpr float kWheelBaseM = 0.052F;     // E-puck distance between the wheels.
constexpr float kUTurnSpeed = 2.0F;       // Wheel speed while spinning, rad/s.
constexpr float kUTurnMinRad = 2.79F;     // 160 degrees: ignore lines crossed earlier.
constexpr float kUTurnMaxRad = 4.71F;     // 270 degrees: give up, the line was not found.

// Fault stop, only for routes with 'F@<s>': once an anomaly alarm has been
// raised continuously for that long, spin about 90 degrees in place (right,
// or the side the route's 'P@' option gives for the current step), drive
// forward off the line, stop with FAULT_STOP and send a notification.
constexpr float kLowestFaultHoldS = 0.1F;    // Accepted range for a route's 'F' hold.
constexpr float kHighestFaultHoldS = 10.0F;
constexpr float kPullOverSpeed = 2.0F;       // Wheel speed while pulling over, rad/s.
constexpr float kPullOverTurnRad = 1.571F;   // 90 degrees.
constexpr float kPullOverDriveM = 0.08F;     // Then this far forward.
constexpr uint32_t kPullOverLimitMs = 6000;  // Healthy ~2.8 s, 50% weak motor ~4.1 s; then stop anyway.

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
  kFaultPullOver,
  kFaultStop,
};

struct RouteStep {
  char action = 'D';
  float distance_m = 0.0F;
};

struct Controller {
  RouteStep route[kMaxRouteLength]{};
  size_t route_length = 0;
  float max_speed = kDefaultMaxSpeed;  // Straight-line speed for this route.
  size_t route_index = 0;
  State state = State::kNoRoute;
  State state_before_link_loss = State::kNoRoute;
  bool route_loaded = false;
  bool junction_latched = false;
  unsigned junction_frames = 0;
  unsigned junction_clear_frames = 0;
  // The next junction's window: open, and whether its pattern was already
  // present when it opened and has not cleared since (kStaleJunctionLeadM).
  bool junction_window_open = false;
  bool junction_pattern_stale = false;
  bool junction_started_dark = false;  // The pattern was present when the window opened.
  float corner_exit_m = -1.0F;         // Where the sensors left the crossing line.
  float corner_white_m = 0.0F;         // Start of the current all-white run.
  float corner_entry_m = -1.0F;        // Where the junction pattern first appeared.
  unsigned corner_white_frames = 0;
  unsigned junction_gap_frames = 0;
  bool bear_turn = false;        // Latched junction turn is a '<'/'>' bear.
  float junction_heading_rad = 0.0F;  // Heading when the junction latched.
  bool corner_turn = false;      // Corner turn under way or settling (kCornerSpeed).
  bool corner_spinning = false;  // Spinning in place onto the crossing line.
  float corner_heading_rad = 0.0F;
  float corner_end_m = 0.0F;     // Speed limit lifts here once the turn is done.
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
  uint32_t fault_hold_ms = 0;  // 0: alarms are only reported.
  float pull_over_heading_rad = 0.0F;
  float pull_over_distance_m = 0.0F;
  uint32_t pull_over_ms = 0;
  bool pull_over_turned = false;
  bool pulling_over_left = false;
  // Optional 'P@' sides: pull over to the left (true) or right per step; the
  // Supervisor picks the side clear of other lines from the map.
  bool pull_over_left[kMaxRouteLength]{};
  size_t pull_over_side_count = 0;
  // The alarm that started the pull-over, for the notification.
  bool fault_motor_alarm = false;
  bool fault_ir_alarm = false;
  // Optional 'E@' labels: where the robot is while heading for each step,
  // e.g. "between Junction 1 and Junction 4".
  char places[kMaxRouteLength][kMaxPlaceLength]{};
  size_t place_count = 0;
};

Controller g_controller;

// Anomaly detection runs on every sensor frame of a route. The motor sample is
// (left target, right target, left speed, right speed) and the IR sample the
// three ground sensors, exactly as logged by epuck_route and used in training.
// The motor GRU (slow motor faults) and the stuck-sensor rule pair with the
// conv models: the motor alarm is either motor detector's, the IR alarm
// either IR detector's (scripts/models/gru_experiment.py).
AnomalyDetector g_motor_detector(anomaly_model::kMotor);
AnomalyDetector g_ir_detector(anomaly_model::kIr);
MotorGruDetector g_motor_gru;
IrStuckDetector g_ir_stuck;

bool motor_alarm() { return g_motor_detector.alarm() || g_motor_gru.alarm(); }
bool ir_alarm() { return g_ir_detector.alarm() || g_ir_stuck.alarm(); }

struct FrameInputs {
  bool pending = false;  // A sensor frame awaits its response (and samples).
  float dt_ms = 0.0F;
  float readings[3]{};
  float encoders[2]{};
};

struct DetectionTiming {
  uint32_t elapsed_ms = 0;  // Since the route was configured.
  bool has_previous_encoders = false;
  float previous_encoders[2]{};
  bool u_turn_seen = false;
  uint32_t last_u_turn_ms = 0;
  bool alarm_on = false;  // Either model's alarm, raised since alarm_since_ms.
  uint32_t alarm_since_ms = 0;
};

FrameInputs g_frame;
DetectionTiming g_detection;

// Recent alarms for the LINE_LOST/JUNCTION_MISSED notification. Unlike
// g_detection these survive a new route (e.g. the return leg's U-turn).
struct AlarmMemory {
  uint32_t clock_ms = 0;  // Sensor-frame time, never reset.
  bool motor_seen = false;
  bool ir_seen = false;
  uint32_t last_motor_ms = 0;
  uint32_t last_ir_ms = 0;
};
AlarmMemory g_alarms;
constexpr uint32_t kRecentAlarmMs = 10000;

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
    case State::kFaultPullOver:
      return "FAULT_PULLOVER";
    case State::kFaultStop:
      return "FAULT_STOP";
  }
  return "ERROR";
}

float round_to_thousandths(float value) {
  return std::round(value * 1000.0F) / 1000.0F;
}

// Add this frame's motor and IR samples. The wheel targets are rounded as
// they are sent, so the ESP scores exactly what the host logs.
void update_detectors(float left, float right) {
  DetectionTiming &timing = g_detection;
  timing.elapsed_ms += static_cast<uint32_t>(std::lround(g_frame.dt_ms));
  const State state = g_controller.state;
  // Spins in place: the U-turn and a corner turn (the frame's wheel targets
  // have opposite signs; scripts/telemetry/prepare_telemetry.py tells them
  // apart that way).
  const bool spinning = state == State::kUTurn ||
                        ((state == State::kJunctionLeft || state == State::kJunctionRight) &&
                         (left < 0.0F) != (right < 0.0F));
  if (spinning) {
    timing.u_turn_seen = true;
    timing.last_u_turn_ms = timing.elapsed_ms;
  }
  // Commanded manoeuvres, and the line follower settling after a spin, are
  // not monitored (scripts/telemetry/prepare_telemetry.py).
  const bool settling = timing.u_turn_seen &&
      timing.elapsed_ms - timing.last_u_turn_ms < static_cast<uint32_t>(anomaly_model::kSettleMs);
  const bool monitored = state != State::kAtDestination && !spinning &&
                         state != State::kFaultPullOver && state != State::kFaultStop &&
                         !settling;

  // A sample needs a wheel speed, i.e. a previous frame; the IR window starts
  // with the same frame so both cover the same rows as in training.
  if (timing.has_previous_encoders) {
    g_ir_detector.add_sample(g_frame.readings, timing.elapsed_ms, monitored);
    const float dt = g_frame.dt_ms / 1000.0F;
    const float motor[4] = {
        round_to_thousandths(left),
        round_to_thousandths(right),
        (g_frame.encoders[0] - timing.previous_encoders[0]) / dt,
        (g_frame.encoders[1] - timing.previous_encoders[1]) / dt,
    };
    const float wheel_speed = (std::fabs(motor[2]) + std::fabs(motor[3])) * 0.5F;
    g_ir_stuck.add_sample(g_frame.readings, wheel_speed, timing.elapsed_ms, monitored);
    g_motor_detector.add_sample(motor, timing.elapsed_ms, monitored);
    g_motor_gru.add_sample(motor, timing.elapsed_ms, monitored);
  }
  const bool alarm = motor_alarm() || ir_alarm();
  if (alarm && !timing.alarm_on) {
    timing.alarm_since_ms = timing.elapsed_ms;
  }
  timing.alarm_on = alarm;
  g_alarms.clock_ms += static_cast<uint32_t>(std::lround(g_frame.dt_ms));
  if (motor_alarm()) {
    g_alarms.motor_seen = true;
    g_alarms.last_motor_ms = g_alarms.clock_ms;
  }
  if (ir_alarm()) {
    g_alarms.ir_seen = true;
    g_alarms.last_ir_ms = g_alarms.clock_ms;
  }
  timing.previous_encoders[0] = g_frame.encoders[0];
  timing.previous_encoders[1] = g_frame.encoders[1];
  timing.has_previous_encoders = true;
}

void reset_detectors() {
  g_motor_detector.reset();
  g_ir_detector.reset();
  g_motor_gru.reset();
  g_ir_stuck.reset();
  g_detection = DetectionTiming{};
}

// Motor frame: wheel targets, controller state, then the motor and IR conv
// scores (-1 until a full window exists), the motor and IR alarms (conv or
// GRU; conv or stuck rule), the motor GRU score (-1 until its first window),
// the GRU alarm and the stuck-sensor alarm.
void send_response(uint32_t sequence, float left, float right) {
  if (g_frame.pending) {
    g_frame.pending = false;
    update_detectors(left, right);
  }
  const float motor_score = g_motor_detector.has_score() ? g_motor_detector.score() : -1.0F;
  const float ir_score = g_ir_detector.has_score() ? g_ir_detector.score() : -1.0F;
  const float gru_score = g_motor_gru.has_score() ? g_motor_gru.score() : -1.0F;
  char response[192];
  const int length = std::snprintf(
      response, sizeof(response), "M,%lu,%.3f,%.3f,%s,%u,%.4f,%.5f,%.5f,%d,%d,%.6f,%d,%d\n",
      static_cast<unsigned long>(sequence), left, right, state_name(g_controller.state),
      static_cast<unsigned>(g_controller.route_index), g_controller.distance_m, motor_score,
      ir_score, motor_alarm() ? 1 : 0, ir_alarm() ? 1 : 0, gru_score,
      g_motor_gru.alarm() ? 1 : 0, g_ir_stuck.alarm() ? 1 : 0);
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
// 'l'/'r' are corners (a sharp turn onto a crossing line, from the map): the
// robot slows down and spins in place onto the new line at the junction.
// '<'/'>' bear left/right onto a Y branch (a gentle turn, from the map): the
// turn is held until the robot has turned kBearMinRad.
// Distances are odometry distances from the start of the route; they must not
// decrease, and the last step must be the destination 'D'. A 'U' (U-turn in
// place) may only be the first step, e.g. "U@0.000,L@0.514,D@0.779". An
// optional leading "V@<rad/s>" sets the straight-line speed for the route, an
// optional leading "F@<seconds>" enables the fault stop with that alarm hold,
// an optional leading "E@<place>|<place>|..." names where the robot is
// while heading for each step (one per step, for notifications), and an
// optional leading "P@<R|L>..." gives the pull-over side per step.
bool configure_route(char *steps) {
  static char places[kMaxRouteLength][kMaxPlaceLength];
  RouteStep parsed[kMaxRouteLength];
  bool sides_left[kMaxRouteLength]{};
  size_t length = 0;
  size_t place_count = 0;
  size_t side_count = 0;
  float previous_distance = 0.0F;
  float max_speed = kDefaultMaxSpeed;
  float fault_hold_s = 0.0F;
  bool options_allowed = true;
  char *save = nullptr;
  for (char *token = strtok_r(steps, ",", &save); token != nullptr;
       token = strtok_r(nullptr, ",", &save)) {
    if (options_allowed && token[0] == 'E' && token[1] == '@') {
      char *place_save = nullptr;
      for (char *place = strtok_r(token + 2, "|", &place_save); place != nullptr;
           place = strtok_r(nullptr, "|", &place_save)) {
        if (place_count == kMaxRouteLength || std::strlen(place) >= kMaxPlaceLength) {
          return false;
        }
        std::strcpy(places[place_count++], place);
      }
      continue;
    }
    if (options_allowed && token[0] == 'P' && token[1] == '@') {
      side_count = 0;
      for (const char *side = token + 2; *side != '\0'; ++side) {
        if ((*side != 'R' && *side != 'L') || side_count == kMaxRouteLength) {
          return false;
        }
        sides_left[side_count++] = *side == 'L';
      }
      if (side_count == 0) {
        return false;
      }
      continue;
    }
    if (options_allowed && (token[0] == 'V' || token[0] == 'F') && token[1] == '@') {
      char *end = nullptr;
      const float value = std::strtof(token + 2, &end);
      if (end == token + 2 || *end != '\0') {
        return false;
      }
      if (token[0] == 'V') {
        if (!(value >= kLowestMaxSpeed) || value > kHighestMaxSpeed) {
          return false;
        }
        max_speed = value;
      } else {
        if (!(value >= kLowestFaultHoldS) || value > kHighestFaultHoldS) {
          return false;
        }
        fault_hold_s = value;
      }
      continue;
    }
    options_allowed = false;
    if (length == kMaxRouteLength) {
      return false;
    }
    const char action = token[0];
    if ((action != 'L' && action != 'S' && action != 'R' && action != 'l' && action != 'r' &&
         action != '<' && action != '>' && action != 'D' && action != 'U') ||
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
  if ((place_count != 0 && place_count != length) ||
      (side_count != 0 && side_count != length)) {
    return false;
  }

  std::copy(parsed, parsed + length, g_controller.route);
  std::copy(sides_left, sides_left + side_count, g_controller.pull_over_left);
  g_controller.pull_over_side_count = side_count;
  for (size_t i = 0; i < place_count; ++i) {
    std::strcpy(g_controller.places[i], places[i]);
  }
  g_controller.place_count = place_count;
  g_controller.route_length = length;
  g_controller.max_speed = max_speed;
  g_controller.fault_hold_ms = static_cast<uint32_t>(std::lround(fault_hold_s * 1000.0F));
  g_controller.route_index = 0;
  g_controller.route_loaded = true;
  g_controller.junction_latched = false;
  g_controller.junction_frames = 0;
  g_controller.junction_clear_frames = 0;
  g_controller.junction_window_open = false;
  g_controller.corner_turn = false;
  g_controller.corner_spinning = false;
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
  reset_detectors();
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

// Notifications are for whoever has to go and fetch the robot: what went
// wrong in plain words, and where it is (from the route's 'E@' labels).

const char *problem_name(bool motor, bool ir) {
  if (motor && ir) {
    return "motor and line sensor problem";
  }
  return ir ? "line sensor problem" : "motor problem";
}

// Where the robot is: the label of the step it was heading for, or nullptr.
const char *current_place() {
  const Controller &c = g_controller;
  if (c.place_count == 0) {
    return nullptr;
  }
  return c.places[std::min(c.route_index, c.place_count - 1)];
}

// LINE_LOST and JUNCTION_MISSED also leave the robot stopped mid-route; a
// motor fault often ends that way before its alarm has lasted the fault hold.
// Like the fault stop, only on routes with 'F@'.
void notify_safety_stop() {
  if (g_controller.fault_hold_ms == 0) {
    return;
  }
  const AlarmMemory &a = g_alarms;
  const bool motor = a.motor_seen && a.clock_ms - a.last_motor_ms <= kRecentAlarmMs;
  const bool ir = a.ir_seen && a.clock_ms - a.last_ir_ms <= kRecentAlarmMs;
  const bool missed = g_controller.state == State::kJunctionMissed;
  char cause[64] = "";
  if (motor || ir) {
    std::snprintf(cause, sizeof(cause), ", probably because of a %s,", problem_name(motor, ir));
  }
  const char *place = current_place();
  char where[96] = "";
  if (place != nullptr) {
    std::snprintf(where, sizeof(where), " %s", place);
  }
  char message[256];
  std::snprintf(message, sizeof(message),
                "The robot %s%s and stopped%s. It could not move off the track and may be "
                "blocking it.",
                missed ? "missed a turn" : "lost the track", cause, where);
  notify_send(missed ? "AGV stopped: missed a turn" : "AGV stopped: lost the track", message);
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
    notify_safety_stop();
    send_response(sequence, 0.0F, 0.0F);
    return true;
  }
  g_controller.state = State::kUTurn;
  send_response(sequence, -kUTurnSpeed, kUTurnSpeed);
  return true;
}

void notify_fault_stop(bool completed) {
  const Controller &c = g_controller;
  const char *problem = problem_name(c.fault_motor_alarm, c.fault_ir_alarm);
  const char *place = current_place();
  char message[256];
  if (completed) {
    std::snprintf(message, sizeof(message),
                  "The robot found a %s, moved off the track and stopped. It is next to the "
                  "track%s%s.",
                  problem, place != nullptr ? " " : "", place != nullptr ? place : "");
  } else {
    std::snprintf(message, sizeof(message),
                  "The robot found a %s and stopped, but could not move fully off the track. "
                  "It may be blocking the track%s%s.",
                  problem, place != nullptr ? " " : "", place != nullptr ? place : "");
  }
  char title[64];
  std::snprintf(title, sizeof(title), "AGV stopped: %s", problem);
  notify_send(title, message);
}

// Pull over once an alarm has lasted the route's fault hold: spin in place
// towards the step's 'P@' side (right without it), drive forward off the
// line, then stop. Returns true while pulling over or once stopped (the
// response has been sent).
bool run_fault_stop(uint32_t sequence, float dt_ms) {
  Controller &c = g_controller;
  if (c.state != State::kFaultPullOver) {
    const DetectionTiming &timing = g_detection;
    if (c.fault_hold_ms == 0 || !timing.alarm_on ||
        timing.elapsed_ms - timing.alarm_since_ms < c.fault_hold_ms) {
      return false;
    }
    c.state = State::kFaultPullOver;
    c.junction_latched = false;
    c.pull_over_heading_rad = c.heading_rad;
    c.pull_over_distance_m = c.distance_m;
    c.pull_over_ms = 0;
    c.pull_over_turned = false;
    c.pulling_over_left =
        c.pull_over_side_count != 0 &&
        c.pull_over_left[std::min(c.route_index, c.pull_over_side_count - 1)];
    c.fault_motor_alarm = motor_alarm();
    c.fault_ir_alarm = ir_alarm();
  }
  c.pull_over_ms += static_cast<uint32_t>(dt_ms);
  // heading_rad is left-positive.
  const float turned = c.pulling_over_left ? c.heading_rad - c.pull_over_heading_rad
                                           : c.pull_over_heading_rad - c.heading_rad;
  if (!c.pull_over_turned && turned >= kPullOverTurnRad) {
    c.pull_over_turned = true;
    c.pull_over_distance_m = c.distance_m;
  }
  const bool completed =
      c.pull_over_turned && c.distance_m - c.pull_over_distance_m >= kPullOverDriveM;
  if (completed || c.pull_over_ms > kPullOverLimitMs) {
    c.state = State::kFaultStop;
    c.route_loaded = false;
    notify_fault_stop(completed);
    send_response(sequence, 0.0F, 0.0F);
  } else if (c.pull_over_turned) {
    send_response(sequence, kPullOverSpeed, kPullOverSpeed);
  } else if (c.pulling_over_left) {
    send_response(sequence, -kPullOverSpeed, kPullOverSpeed);
  } else {
    send_response(sequence, kPullOverSpeed, -kPullOverSpeed);
  }
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
    notify_safety_stop();
  }
}

// Heading for a corner: the next step is a corner ('l'/'r', from the map).
// (A pattern already present at an 'L'/'R' window, e.g. on the base track's
// circle, is an ordinary turn delayed to kStaleJunctionLeadM.)
bool approaching_corner() {
  const Controller &c = g_controller;
  if (c.junction_latched || !c.junction_window_open || !c.route_loaded) {
    return false;
  }
  const char action = c.route[c.route_index].action;
  return action == 'l' || action == 'r';
}

// ``line_seen``: any sensor on a line (all three white: past the line's end).
void maybe_enter_junction(bool left_active, bool right_active, bool line_seen) {
  if (g_controller.junction_latched || !g_controller.route_loaded) {
    return;
  }
  const RouteStep &next = g_controller.route[g_controller.route_index];
  const bool in_window =
      next.action != 'D' &&
      std::fabs(g_controller.distance_m - next.distance_m) <= kJunctionWindowM;
  const bool pattern = left_active && right_active;

  if (in_window && !g_controller.junction_window_open) {
    g_controller.junction_window_open = true;
    g_controller.junction_pattern_stale = pattern;
    g_controller.junction_started_dark = pattern;
    g_controller.corner_exit_m = -1.0F;
    g_controller.corner_entry_m = -1.0F;
    g_controller.corner_white_frames = 0;
    g_controller.junction_gap_frames = 0;
  }
  if (in_window && approaching_corner()) {
    // A corner: spin once the axle is on the crossing line, from odometry or
    // from where the sensors ran off its far side, whichever comes first.
    // Off the line for kJunctionReleaseFrames in a row (a single white frame
    // is drift on a wide line): it started where the white run started.
    if (line_seen) {
      g_controller.corner_white_frames = 0;
    } else if (g_controller.corner_white_frames++ == 0) {
      g_controller.corner_white_m = g_controller.distance_m;
    }
    if (g_controller.corner_white_frames >= kJunctionReleaseFrames &&
        g_controller.corner_exit_m < 0.0F) {
      g_controller.corner_exit_m = g_controller.corner_white_m;
    }
    // A pattern appearing in the window: the sensors reached the crossing
    // line's near side (confirmed like a junction).
    if (!g_controller.junction_started_dark && pattern) {
      if (++g_controller.junction_frames >= kJunctionConfirmFrames &&
          g_controller.corner_entry_m < 0.0F) {
        g_controller.corner_entry_m = g_controller.distance_m;
      }
    } else {
      g_controller.junction_frames = 0;
    }
    const bool on_corner =
        g_controller.distance_m >= next.distance_m ||
        (g_controller.corner_exit_m >= 0.0F &&
         g_controller.distance_m >= g_controller.corner_exit_m + kCornerAxleM) ||
        (g_controller.corner_entry_m >= 0.0F &&
         g_controller.distance_m >= g_controller.corner_entry_m + kCornerEntryAxleM);
    if (on_corner) {
      ++g_controller.route_index;
      g_controller.junction_window_open = false;
      g_controller.junction_latched = true;
      g_controller.junction_clear_frames = 0;
      g_controller.turn_search_ms = 0;
      g_controller.corner_turn = true;
      g_controller.corner_spinning = true;
      g_controller.corner_heading_rad = g_controller.heading_rad;
      g_controller.corner_end_m = 1e9F;
      g_controller.junction_frames = 0;
      g_controller.state = next.action == 'L' || next.action == 'l' ? State::kJunctionLeft
                                                                      : State::kJunctionRight;
    }
    return;
  }
  if (pattern) {
    g_controller.junction_gap_frames = 0;
  } else if (++g_controller.junction_gap_frames >= kJunctionReleaseFrames) {
    g_controller.junction_pattern_stale = false;  // A new pattern can start.
  }
  // A Y branch ('<'/'>') appears gradually, beside the old line: like a stale
  // pattern, its turn waits for the junction distance (less the lead).
  const bool bear = next.action == '<' || next.action == '>';
  const bool due = (!g_controller.junction_pattern_stale && !bear) ||
                   g_controller.distance_m >= next.distance_m - kStaleJunctionLeadM;

  if (in_window && pattern && due) {
    ++g_controller.junction_frames;
    if (g_controller.junction_frames >= kJunctionConfirmFrames) {
      ++g_controller.route_index;
      g_controller.junction_window_open = false;
      g_controller.junction_latched = true;
      g_controller.junction_clear_frames = 0;
      g_controller.turn_search_ms = 0;

      g_controller.bear_turn = next.action == '<' || next.action == '>';
      g_controller.junction_heading_rad = g_controller.heading_rad;
      switch (next.action) {
        case 'L':
        case '<':
          g_controller.state = State::kJunctionLeft;
          break;
        case 'S':
          g_controller.state = State::kJunctionStraight;
          break;
        case 'R':
        case '>':
          g_controller.state = State::kJunctionRight;
          break;
      }
    }
  } else {
    g_controller.junction_frames = 0;
  }
}

// Spin in place for a corner turn; true when it answered the frame.
bool run_corner_turn(uint32_t sequence, bool center_active) {
  Controller &c = g_controller;
  if (!c.corner_spinning) {
    return false;
  }
  const float turned = std::fabs(c.heading_rad - c.corner_heading_rad);
  if (turned >= kCornerMinRad && center_active) {
    c.corner_spinning = false;
    c.junction_latched = false;
    c.junction_clear_frames = 0;
    c.corner_end_m = c.distance_m + kCornerSettleM;
    c.state = State::kFollowing;
    c.previous_error = 0.0F;
    c.filtered_error_rate = 0.0F;
    c.turn_search_ms = 0;
    return false;  // Follow the new line from this frame.
  }
  if (turned > kCornerMaxRad) {
    c.corner_spinning = false;
    c.state = State::kLineLost;
    c.route_loaded = false;
    notify_safety_stop();
    send_response(sequence, 0.0F, 0.0F);
    return true;
  }
  const float direction = c.state == State::kJunctionLeft ? 1.0F : -1.0F;  // Left-positive.
  send_response(sequence, -direction * kCornerSpinSpeed, direction * kCornerSpinSpeed);
  return true;
}

void maybe_leave_junction(bool left_active, bool right_active) {
  if (!g_controller.junction_latched || g_controller.corner_spinning) {
    return;
  }

  const float turned = std::fabs(g_controller.heading_rad - g_controller.junction_heading_rad);
  const bool bearing = g_controller.bear_turn && turned < kBearMinRad;
  if (g_controller.bear_turn && turned > kBearMaxRad) {
    g_controller.junction_clear_frames = kJunctionReleaseFrames;  // Release now.
  } else if ((left_active && right_active) || bearing) {
    g_controller.junction_clear_frames = 0;
  } else {
    ++g_controller.junction_clear_frames;
  }
  if (g_controller.junction_clear_frames >= kJunctionReleaseFrames) {
    g_controller.bear_turn = false;
    g_controller.corner_end_m = g_controller.distance_m + kCornerSettleM;
    g_controller.junction_latched = false;
    g_controller.junction_clear_frames = 0;
    g_controller.state = State::kFollowing;
    g_controller.turn_search_ms = 0;
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
  g_frame.pending = true;
  g_frame.dt_ms = dt_ms;
  std::copy(readings, readings + 3, g_frame.readings);
  std::copy(encoders, encoders + 2, g_frame.encoders);

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

  if (run_u_turn(sequence, center >= kMinLineStrength) || run_fault_stop(sequence, dt_ms)) {
    return;
  }

  maybe_enter_junction(left_active, right_active, strength >= kMinLineStrength);
  if (run_corner_turn(sequence, center >= kMinLineStrength)) {
    return;
  }
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
    const RouteStep &next = g_controller.route[g_controller.route_index];
    const bool arriving =
        next.action == 'D' && g_controller.distance_m >= next.distance_m - kArrivalSlackM;
    if (arriving && g_controller.turn_search_ms <= kArrivalLimitMs) {
      send_response(sequence, kArrivalSpeed, kArrivalSpeed);
      return;
    }
    if (approaching_corner() && g_controller.turn_search_ms <= kTurnSearchLimitMs) {
      send_response(sequence, kCornerSpeed, kCornerSpeed);  // Onto the corner.
      return;
    }
    if (g_controller.turn_search_ms > kTurnSearchLimitMs) {
      g_controller.state = State::kLineLost;
      g_controller.route_loaded = false;
      notify_safety_stop();
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
  float max_speed = g_controller.max_speed;
  // Slow down for a 90-degree corner: approaching one (a turn at a junction
  // whose pattern was already present) and until it is settled.
  if (g_controller.corner_turn && g_controller.distance_m > g_controller.corner_end_m) {
    g_controller.corner_turn = false;
  }
  if (approaching_corner() || g_controller.corner_turn) {
    max_speed = std::min(max_speed, kCornerSpeed);
  }
  const float base_speed = max_speed - max_speed * (1.0F - kMinSpeedRatio) * turn_amount;
  float steering = kKp * error + kKd * g_controller.filtered_error_rate;
  steering = std::clamp(steering, -kMaxSteering, kMaxSteering);
  const float left_target =
      std::clamp(base_speed + steering, 0.0F, kMaxWheelSpeed);
  const float right_target =
      std::clamp(base_speed - steering, 0.0F, kMaxWheelSpeed);
  send_response(sequence, left_target, right_target);
}

// Resource usage: "U,<free>,<min_free>,<largest>,<internal_free>,
// <internal_min_free>,<psram_total>,<psram_free>,<heap_total>,<tasks>,<window_us>"
// (bytes), then one "T,<name>,<core>,<cpu_permille>,<stack_free_min>" line per
// task. cpu_permille is the task's share of one core since the previous "U"
// (core -1: not pinned); stack_free_min is the lowest free stack in bytes.
void send_resource_usage() {
  constexpr int kMaxTasks = 32;
  struct Previous {
    TaskHandle_t handle;
    configRUN_TIME_COUNTER_TYPE run_time;
  };
  static Previous previous[kMaxTasks];
  static int previous_count = 0;
  static int64_t previous_us = 0;

  static TaskStatus_t tasks[kMaxTasks];
  const UBaseType_t count = uxTaskGetSystemState(tasks, kMaxTasks, nullptr);
  const int64_t now_us = esp_timer_get_time();
  const int64_t window_us = now_us - previous_us;

  char response[192];
  int length = std::snprintf(
      response, sizeof(response), "U,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lld\n",
      static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
      static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT)),
      static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
      static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
      static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)),
      static_cast<unsigned>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM)),
      static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
      static_cast<unsigned>(heap_caps_get_total_size(MALLOC_CAP_8BIT)),
      static_cast<unsigned>(count), static_cast<long long>(window_us));
  if (length > 0 && static_cast<size_t>(length) < sizeof(response)) {
    uart_write_bytes(kSerialPort, response, length);
  }

  for (UBaseType_t i = 0; i < count; ++i) {
    configRUN_TIME_COUNTER_TYPE before = 0;
    for (int j = 0; j < previous_count; ++j) {
      if (previous[j].handle == tasks[i].xHandle) {
        before = previous[j].run_time;
        break;
      }
    }
    // Run time counts esp_timer microseconds.
    const long long permille =
        window_us > 0 ? static_cast<long long>(tasks[i].ulRunTimeCounter - before) * 1000 / window_us
                      : 0;
    const BaseType_t pinned = xTaskGetCoreID(tasks[i].xHandle);
    const int core = pinned == tskNO_AFFINITY ? -1 : static_cast<int>(pinned);
    length = std::snprintf(response, sizeof(response), "T,%s,%d,%lld,%u\n", tasks[i].pcTaskName,
                           core, permille,
                           static_cast<unsigned>(tasks[i].usStackHighWaterMark));
    if (length > 0 && static_cast<size_t>(length) < sizeof(response)) {
      uart_write_bytes(kSerialPort, response, length);
    }
  }

  previous_count = static_cast<int>(count);
  for (UBaseType_t i = 0; i < count; ++i) {
    previous[i] = {tasks[i].xHandle, tasks[i].ulRunTimeCounter};
  }
  previous_us = now_us;
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
  if (line[0] == 'Q' && line[1] == '\0') {
    // Inference timing: last and maximum microseconds per frame, for the
    // motor conv, IR conv and motor GRU models.
    char response[112];
    const int length = std::snprintf(
        response, sizeof(response), "I,%lu,%lu,%lu,%lu,%lu,%lu\n",
        static_cast<unsigned long>(g_motor_detector.last_inference_us()),
        static_cast<unsigned long>(g_motor_detector.max_inference_us()),
        static_cast<unsigned long>(g_ir_detector.last_inference_us()),
        static_cast<unsigned long>(g_ir_detector.max_inference_us()),
        static_cast<unsigned long>(g_motor_gru.last_inference_us()),
        static_cast<unsigned long>(g_motor_gru.max_inference_us()));
    if (length > 0 && static_cast<size_t>(length) < sizeof(response)) {
      uart_write_bytes(kSerialPort, response, length);
    }
    return;
  }
  if (line[0] == 'U' && line[1] == '\0') {
    send_resource_usage();
    return;
  }
  if (line[0] == 'B' && line[1] == ',') {
    // Latency benchmark: "B,C,<features>,<window>,<channels>,<kernel>,
    // <bottleneck>,<hidden>,<repeats>" (conv autoencoder) or
    // "B,G,<inputs>,<hidden>,<outputs>,<repeats>" (GRU step); replies
    // "b,<ok>,<median_us>,<max_us>,<weight_bytes>". Not for use during a route.
    int v[7] = {};
    bench::Result result;
    if (line[2] == 'C' &&
        std::sscanf(line + 3, ",%d,%d,%d,%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5],
                    &v[6]) == 7) {
      result = bench::conv(v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
    } else if (line[2] == 'G' &&
               std::sscanf(line + 3, ",%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3]) == 4) {
      result = bench::gru(v[0], v[1], v[2], v[3]);
    }
    char response[80];
    const int length = std::snprintf(
        response, sizeof(response), "b,%d,%lu,%lu,%lu\n", result.ok ? 1 : 0,
        static_cast<unsigned long>(result.median_us), static_cast<unsigned long>(result.max_us),
        static_cast<unsigned long>(result.weight_bytes));
    if (length > 0 && static_cast<size_t>(length) < sizeof(response)) {
      uart_write_bytes(kSerialPort, response, length);
    }
    return;
  }
  if (line[0] == 'W' && line[1] == '\0') {
    // WiFi and notification status.
    char status[96];
    notify_status(status, sizeof(status));
    char response[112];
    const int length = std::snprintf(response, sizeof(response), "W,%s\n", status);
    if (length > 0 && static_cast<size_t>(length) < sizeof(response)) {
      uart_write_bytes(kSerialPort, response, length);
    }
    return;
  }
  if (line[0] == 'N' && line[1] == '\0') {
    if (notify_send("AGV test", "Test notification from the ESP32-S3.")) {
      uart_write_bytes(kSerialPort, "A,NOTIFY_QUEUED\n", sizeof("A,NOTIFY_QUEUED\n") - 1);
    } else {
      send_error("NOTIFY_OFF");
    }
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
  notify_start();
  // The anomaly forward pass keeps ~3 KB of activations on the stack. Core 1
  // keeps the control loop away from WiFi, which runs on core 0.
  xTaskCreatePinnedToCore(serial_task, "serial_control", 12288, nullptr, 5, nullptr, 1);
}
