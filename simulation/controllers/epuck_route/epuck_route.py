import csv
import json
import math
import os
import random
import re
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

from controller import Supervisor


CONTROLLER_DIR = Path(__file__).resolve().parent
ROOT_DIR = CONTROLLER_DIR.parent.parent.parent
MAP_PATH = ROOT_DIR / "maps" / "paths" / "base track.json"
# Environment overrides let scripts/telemetry/collect_route_runs.py drive batch runs
# without editing the committed mission or route files.
MISSION_PATH = Path(
    os.environ.get("AGV_MISSION_PATH", ROOT_DIR / "maps" / "routes" / "mission.json")
)
ROUTE_PATH = Path(
    os.environ.get("AGV_ROUTE_PATH", ROOT_DIR / "maps" / "routes" / "active_route.json")
)
LOG_DIR = Path(os.environ.get("AGV_LOG_DIR", ROOT_DIR / "data" / "route_runs"))


def _read_inputs():
    with MAP_PATH.open(encoding="utf-8") as map_file:
        track_map = json.load(map_file)
    with MISSION_PATH.open(encoding="utf-8") as mission_file:
        mission = json.load(mission_file)

    transform = track_map.get("world_transform")
    if not isinstance(transform, dict):
        raise ValueError(f"Map has no world_transform: {MAP_PATH}")
    if not transform.get("calibrated", False) and not mission.get(
        "allow_uncalibrated_map_for_simulation", False
    ):
        raise ValueError(
            "Map-to-world transform is uncalibrated. Set the explicit "
            "simulation-only override in the mission after reviewing alignment."
        )

    route, route_paths, esp_route = _plan_leg(
        track_map, mission, mission["start_node"], mission["destination_node"]
    )
    ROUTE_PATH.parent.mkdir(parents=True, exist_ok=True)
    ROUTE_PATH.write_text(
        json.dumps(route.to_dict(), indent=2) + "\n",
        encoding="utf-8",
    )
    nodes = {node["id"]: node for node in track_map["nodes"]}
    return track_map, mission, route, nodes, route_paths, transform, esp_route


def _plan_leg(track_map, mission, start, destination, u_turn_first=False):
    """Plan one leg; return the route, its world-space edge paths and ESP route.

    ``u_turn_first`` prefixes the ESP route with a U-turn, for a leg that
    starts facing back along the previous leg.
    """
    sys.path.insert(0, str(ROOT_DIR))
    from scripts.esp.route_actions import format_route_command, route_action_plan
    from scripts.maps.route_planner import TrackMap, shortest_route

    transform = track_map["world_transform"]
    route = shortest_route(TrackMap.from_dict(track_map), start, destination)
    edges = {edge["id"]: edge for edge in track_map["edges"]}

    route_paths = []
    for step in route.steps:
        edge = edges[step.edge_id]
        geometry = edge.get("geometry")
        if not isinstance(geometry, list) or len(geometry) < 2:
            raise ValueError(f"Route edge {step.edge_id!r} has no centerline geometry.")
        if edge["from"] == step.source and edge["to"] == step.target:
            ordered_geometry = geometry
        elif (
            edge.get("bidirectional", True)
            and edge["to"] == step.source
            and edge["from"] == step.target
        ):
            ordered_geometry = list(reversed(geometry))
        else:
            raise ValueError(f"Route traverses edge {step.edge_id!r} in a forbidden direction.")

        route_paths.append(
            {
                "edge_id": step.edge_id,
                "target_node": step.target,
                "speed_limit": step.speed_limit,
                "points": [
                    _pixel_to_world(point, transform) for point in ordered_geometry
                ],
            }
        )

    # Junction and destination distances for the ESP, as expected wheel
    # odometry: the robot cuts inside curves, so it drives less than the map
    # centerline (about 0.92 of it on the base track's healthy runs).
    plan = route_action_plan(
        track_map,
        route,
        distance_scale=float(mission.get("esp_odometry_scale", 0.92)),
    )
    if u_turn_first:
        plan = [("U", 0.0)] + plan
    # Optional straight-line speed for the ESP's PD loop (firmware default
    # 3.5 rad/s); it must come before every other step.
    if mission.get("esp_speed_rad_s") is not None:
        plan = [("V", float(mission["esp_speed_rad_s"]))] + plan
    return route, route_paths, format_route_command(plan)


def _pixel_to_world(point, transform):
    width, height = transform["image_size_px"]
    origin_x, origin_y = transform["world_origin"]
    scale_x, scale_y = transform["world_units_per_pixel"]
    return origin_x + point[0] * scale_x, origin_y + point[1] * scale_y


def _polyline_length(points):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def _point_at_distance(points, distance):
    for start, end in zip(points, points[1:]):
        segment_length = math.hypot(end[0] - start[0], end[1] - start[1])
        if distance <= segment_length:
            fraction = distance / segment_length if segment_length else 0.0
            return (
                start[0] + (end[0] - start[0]) * fraction,
                start[1] + (end[1] - start[1]) * fraction,
            )
        distance -= segment_length
    return points[-1]


def _nearest_progress(position, points):
    best_progress = 0.0
    best_distance = math.inf
    progress = 0.0
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.hypot(dx, dy)
        if segment_length == 0:
            continue
        fraction = max(
            0.0,
            min(
                1.0,
                ((position[0] - start[0]) * dx + (position[1] - start[1]) * dy)
                / (segment_length * segment_length),
            ),
        )
        projected = (start[0] + fraction * dx, start[1] + fraction * dy)
        distance = math.hypot(position[0] - projected[0], position[1] - projected[1])
        if distance < best_distance:
            best_distance = distance
            best_progress = progress + fraction * segment_length
        progress += segment_length
    return best_progress, best_distance


def _wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _line_error(values, threshold):
    left = max(0.0, threshold - values[0])
    center = max(0.0, threshold - values[1])
    right = max(0.0, threshold - values[2])
    strength = left + center + right
    if strength <= 30.0:
        return None
    return (right - left) / strength


MOTOR_FAULTS = ("motor_gain", "motor_stall", "motor_intermittent")
ENCODER_FAULTS = ("encoder_stuck", "encoder_noise")
IR_FAULTS = ("ir_stuck", "ir_dead", "ir_noise", "ir_drift")
IR_TARGETS = ("gs_left", "gs_center", "gs_right")


class FaultInjector:
    """Apply one optional mission ``fault`` from ``onset_s`` onwards.

    Motor faults change the velocity applied to a wheel (the logged target is
    the controller's command). Encoder faults change only the logged wheel
    position. IR faults change the ground-sensor value seen by both the
    controller and the log, like a real faulty sensor.

    Spec keys: ``type``, ``target`` (``left``/``right`` or a ground sensor
    name), ``onset_s``, ``severity`` and ``seed``; ``motor_intermittent`` also
    takes ``period_s`` and ``burst_s``.
    """

    def __init__(self, spec):
        self.spec = spec or None
        self.type = ""
        if not self.spec:
            return
        self.type = spec.get("type", "")
        self.target = spec.get("target")
        self.onset = float(spec.get("onset_s", 0.0))
        self.severity = float(spec.get("severity", 0.0))
        self.period = float(spec.get("period_s", 1.0))
        self.burst = float(spec.get("burst_s", 0.15))
        self.rng = random.Random(int(spec.get("seed", 0)))
        self.frozen = None
        if self.type in MOTOR_FAULTS + ENCODER_FAULTS:
            if self.target not in ("left", "right"):
                raise ValueError(f"{self.type} needs target 'left' or 'right'")
            self.index = 0 if self.target == "left" else 1
        elif self.type in IR_FAULTS:
            if self.target not in IR_TARGETS:
                raise ValueError(f"{self.type} needs target in {IR_TARGETS}")
            self.index = IR_TARGETS.index(self.target)
        else:
            raise ValueError(f"Unknown fault type: {self.type!r}")

    def active(self, time):
        return self.spec is not None and time >= self.onset

    def motors(self, time, commands):
        commands = list(commands)
        if self.type not in MOTOR_FAULTS or not self.active(time):
            return commands
        if self.type == "motor_gain":
            commands[self.index] *= self.severity
        elif self.type == "motor_stall":
            commands[self.index] = 0.0
        elif (time - self.onset) % self.period < self.burst:  # motor_intermittent
            commands[self.index] = 0.0
        return commands

    def encoders(self, time, positions):
        positions = list(positions)
        if self.type not in ENCODER_FAULTS or not self.active(time):
            return positions
        if self.type == "encoder_stuck":
            if self.frozen is None:
                self.frozen = positions[self.index]
            positions[self.index] = self.frozen
        else:  # encoder_noise
            positions[self.index] += self.rng.gauss(0.0, self.severity)
        return positions

    def sensors(self, time, values):
        values = list(values)
        if self.type not in IR_FAULTS or not self.active(time):
            return values
        if self.type == "ir_stuck":
            if self.frozen is None:
                self.frozen = values[self.index]
            values[self.index] = self.frozen
        elif self.type == "ir_dead":
            values[self.index] = self.severity
        elif self.type == "ir_noise":
            values[self.index] += self.rng.gauss(0.0, self.severity)
        else:  # ir_drift: severity is sensor units per second
            values[self.index] += self.severity * (time - self.onset)
        return values


def _stop(motors):
    motors[0].setVelocity(0.0)
    motors[1].setVelocity(0.0)


def _read_esp_line(link, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        response = link.readline()
        if response:
            return response.decode("ascii", errors="replace").strip()
    raise TimeoutError("Timed out waiting for an ESP response.")


def _open_esp_link(port_name, baud_rate):
    import serial

    link = serial.Serial(
        port=None,
        baudrate=baud_rate,
        timeout=0.05,
        write_timeout=1.0,
        dsrdtr=False,
        rtscts=False,
    )
    link.port = port_name
    link.dtr = False
    link.rts = False
    link.open()
    time.sleep(0.5)
    link.reset_input_buffer()
    return link


ESP_STATES = (
    "FOLLOWING",
    "TURN_LEFT",
    "STRAIGHT",
    "TURN_RIGHT",
    "U_TURN",
    "AT_DEST",
    "LINE_LOST",
    "LINK_LOST",
    "JUNCTION_MISSED",
    "NO_ROUTE",
)
ESP_STOP_STATES = ("AT_DEST", "LINE_LOST", "LINK_LOST", "JUNCTION_MISSED")


def _configure_esp_route(link, esp_route):
    link.write(f"R,{esp_route}\n".encode("ascii"))
    response = _read_esp_line(link, 2.0)
    if response != "A,ROUTE_OK":
        raise RuntimeError(f"ESP rejected route {esp_route!r}: {response!r}")


def _step_esp(link, sequence, time_step, sensor_values, encoder_values):
    frame = (
        f"S,{sequence},{time_step},"
        f"{sensor_values[0]:.3f},{sensor_values[1]:.3f},{sensor_values[2]:.3f},"
        f"{encoder_values[0]:.6f},{encoder_values[1]:.6f}\n"
    )
    link.write(frame.encode("ascii"))
    while True:
        response = _read_esp_line(link, 0.2)
        if response.startswith("E,"):
            raise RuntimeError(f"ESP rejected sensor frame: {response}")

        fields = response.split(",")
        # 7 fields from firmware without anomaly detection, 11 with it.
        if len(fields) not in (7, 11) or fields[0] != "M":
            raise RuntimeError(f"Malformed ESP motor response: {response!r}")
        if int(fields[1]) >= sequence:
            break
        # The ESP reports a sensor timeout unprompted, repeating the last
        # sequence number. Skip it and wait for the reply to this frame.
        print(f"Skipped stale ESP response {response!r} while waiting for {sequence}")
    if int(fields[1]) != sequence:
        raise RuntimeError(
            f"ESP response sequence mismatch: expected {sequence}, got {fields[1]!r}"
        )

    # Wheels only reverse during a U-turn, where they spin in opposite
    # directions; the firmware caps every target at 5 rad/s either way.
    left_target, right_target = float(fields[2]), float(fields[3])
    if (
        not math.isfinite(left_target)
        or not math.isfinite(right_target)
        or not -5.0 <= left_target <= 5.0
        or not -5.0 <= right_target <= 5.0
    ):
        raise RuntimeError(f"Unsafe ESP wheel targets: {response!r}")
    state = fields[4]
    if state not in ESP_STATES:
        raise RuntimeError(f"Unknown ESP controller state: {state!r}")
    # Motor/IR anomaly scores (-1 before the first full window) and alarms.
    anomaly = tuple(fields[7:11]) if len(fields) == 11 else ("", "", "", "")
    return left_target, right_target, state, int(fields[5]), float(fields[6]), anomaly


def main():
    robot = Supervisor()
    time_step = int(robot.getBasicTimeStep())

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    motors = (left_motor, right_motor)
    for motor in motors:
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    left_encoder = robot.getDevice("left wheel sensor")
    right_encoder = robot.getDevice("right wheel sensor")
    left_encoder.enable(time_step)
    right_encoder.enable(time_step)

    ground_sensors = [robot.getDevice(f"gs{i}") for i in range(3)]
    for sensor in ground_sensors:
        sensor.enable(time_step)

    log_file = None
    esp_link = None
    try:
        track_map, mission, route, nodes, route_paths, transform, esp_route = _read_inputs()
        if not route_paths:
            print(f"Already at destination {route.destination}; stopping.")
            return
        control_mode = os.environ.get(
            "AGV_CONTROL_MODE", mission.get("control_mode", "supervisor")
        ).lower()
        if control_mode not in ("supervisor", "esp32"):
            raise ValueError(f"Unsupported control_mode: {control_mode!r}")
        # The return leg needs a U-turn at the destination, which only the
        # ESP controller implements.
        return_to_start = bool(mission.get("return_to_start", False))
        if return_to_start and control_mode != "esp32":
            raise ValueError("return_to_start requires control_mode 'esp32'.")
        if control_mode == "esp32":
            esp_port = os.environ.get(
                "AGV_ESP_PORT", mission.get("esp_serial_port", "COM3")
            )
            esp_baud = int(mission.get("esp_baud", 115200))
            esp_link = _open_esp_link(esp_port, esp_baud)
            _configure_esp_route(esp_link, esp_route)
            print(
                f"ESP32 control enabled on {esp_port} at {esp_baud} baud; "
                f"route actions: {esp_route}"
            )
        else:
            print("Supervisor control enabled.")

        self_node = robot.getSelf()
        translation_field = self_node.getField("translation")
        rotation_field = self_node.getField("rotation")

        first_node = nodes[route.start]
        start_world = _pixel_to_world([first_node["x"], first_node["y"]], transform)
        first_points = route_paths[0]["points"]
        # Aim at a point a lookahead distance along the edge rather than the
        # first pixel step: skeletonised junctions often start with a short
        # diagonal kink that would point the ground sensors off the line.
        heading_point = _point_at_distance(
            first_points,
            min(
                _polyline_length(first_points),
                float(mission.get("lookahead_m", 0.045)),
            ),
        )
        first_direction = (
            heading_point[0] - start_world[0],
            heading_point[1] - start_world[1],
        )
        if math.hypot(*first_direction) <= 1e-8:
            first_direction = None

        if mission.get("spawn_at_start_node", False):
            if first_direction is None:
                raise ValueError("First route edge has no usable direction.")
            heading = math.atan2(first_direction[1], first_direction[0])
            spawn_x, spawn_y = start_world
            # Optional seeded start-pose variation, so repeated runs of one
            # route and speed are not identical (Webots is deterministic).
            jitter = mission.get("spawn_jitter")
            if jitter:
                jitter_rng = random.Random(int(jitter.get("seed", 0)))
                lateral = jitter_rng.uniform(-1.0, 1.0) * float(jitter.get("lateral_m", 0.0))
                heading += math.radians(
                    jitter_rng.uniform(-1.0, 1.0) * float(jitter.get("heading_deg", 0.0))
                )
                spawn_x -= lateral * math.sin(heading)
                spawn_y += lateral * math.cos(heading)
                print(f"Spawn jitter: {lateral * 1000:.1f} mm lateral, heading {math.degrees(heading):.1f} deg")
            current_z = self_node.getPosition()[2]
            translation_field.setSFVec3f([spawn_x, spawn_y, current_z])
            rotation_field.setSFRotation([0.0, 0.0, 1.0, heading])
            self_node.resetPhysics()

        print(
            f"Route {route.start} -> {route.destination}: "
            f"{' -> '.join(route.nodes)} ({len(route.steps)} edges)"
        )
        if not transform.get("calibrated", False):
            print("WARNING: using uncalibrated image-to-world transform for simulation only.")

        log_dir = LOG_DIR
        log_dir.mkdir(parents=True, exist_ok=True)
        label = re.sub(r"[^A-Za-z0-9_.-]+", "-", str(mission.get("run_label", ""))).strip("-")
        log_path = log_dir / (
            f"route-{datetime.now():%Y%m%d-%H%M%S}" + (f"-{label}" if label else "") + ".csv"
        )
        log_file = log_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(log_file)
        header = [
            "time",
            "route_start",
            "route_destination",
            "edge_id",
            "pose_x",
            "pose_y",
            "centerline_error_m",
            "line_error",
            "target_vel_left",
            "target_vel_right",
            "actual_pos_left",
            "actual_pos_right",
            "gs_left",
            "gs_center",
            "gs_right",
            "fault_type",
            "fault_active",
        ]
        if control_mode == "esp32":
            header.extend((
                "esp_state", "esp_route_index", "esp_distance_m",
                "esp_motor_score", "esp_ir_score", "esp_motor_alarm", "esp_ir_alarm",
            ))
        writer.writerow(header)
        fault = FaultInjector(mission.get("fault"))
        if fault.spec:
            print(f"FAULT INJECTION: {fault.spec}")

        edge_index = 0
        progress = 0.0
        lookahead = float(mission.get("lookahead_m", 0.045))
        node_radius = float(mission.get("node_radius_m", 0.025))
        wheel_radius = 0.02
        wheel_base = 0.052
        default_wheel_speed = float(mission.get("wheel_speed_rad_s", 2.5))
        max_wheel_speed = 5.0
        line_threshold = float(mission.get("line_threshold", 400.0))
        sensor_heading_gain = float(mission.get("sensor_heading_gain", 0.2))
        max_line_loss_s = float(mission.get("max_line_loss_s", 2.0))
        batch_size = 100
        buffered_rows = 0
        completed = False
        failure = None
        line_lost_since = None
        esp_sequence = 0
        on_return_leg = False
        unloading_until = None

        while robot.step(time_step) != -1:
            position = self_node.getPosition()
            pose_x, pose_y = position[0], position[1]
            pose_matrix = self_node.getOrientation()
            heading = math.atan2(pose_matrix[3], pose_matrix[0])
            now = robot.getTime()
            sensor_values = fault.sensors(
                now, [sensor.getValue() for sensor in ground_sensors]
            )
            sensor_error = _line_error(sensor_values, line_threshold)

            if unloading_until is not None and now >= unloading_until:
                # Plan the return leg from the destination and hand it to the
                # ESP; it starts with a U-turn because the robot faces away.
                route, route_paths, esp_route = _plan_leg(
                    track_map,
                    mission,
                    route.destination,
                    route.start,
                    u_turn_first=True,
                )
                _configure_esp_route(esp_link, esp_route)
                print(
                    f"Return leg {route.start} -> {route.destination}: "
                    f"{' -> '.join(route.nodes)}; route actions: {esp_route}"
                )
                edge_index = 0
                progress = 0.0
                line_lost_since = None
                unloading_until = None
                on_return_leg = True

            # Advance past every edge whose target node has been reached, then
            # control and log this step on the new edge. Skipping the step here
            # would leave a 2x-timestep gap in the telemetry at each junction.
            # In ESP mode the ESP decides when it has arrived, so stay on the
            # final edge and check its stop against the destination below.
            last_edge = len(route_paths) - 1 if control_mode == "esp32" else len(route_paths)
            while edge_index < last_edge:
                target_node_id = route_paths[edge_index]["target_node"]
                target_node = _pixel_to_world(
                    [nodes[target_node_id]["x"], nodes[target_node_id]["y"]],
                    transform,
                )
                if math.dist((pose_x, pose_y), target_node) > node_radius:
                    break
                edge_index += 1
                progress = 0.0
            if edge_index >= len(route_paths):
                _stop(motors)
                completed = True
                break

            edge = route_paths[edge_index]
            points = edge["points"]
            total_length = _polyline_length(points)
            measured_progress, distance_from_path = _nearest_progress(
                (pose_x, pose_y), points
            )
            progress = max(progress, measured_progress)
            target_point = _point_at_distance(
                points, min(total_length, progress + lookahead)
            )

            encoder_left, encoder_right = fault.encoders(
                now, (left_encoder.getValue(), right_encoder.getValue())
            )

            esp_state = ""
            esp_route_index = -1
            esp_distance = ""
            esp_anomaly = ("", "", "", "")
            if unloading_until is not None:
                # Stopped at the destination; the ESP gets no frames until
                # the return route is sent.
                _stop(motors)
                left_target = right_target = 0.0
                esp_state = "UNLOADING"
            elif control_mode == "esp32":
                esp_sequence += 1
                (
                    left_target,
                    right_target,
                    esp_state,
                    esp_route_index,
                    esp_distance,
                    esp_anomaly,
                ) = _step_esp(
                    esp_link,
                    esp_sequence,
                    time_step,
                    sensor_values,
                    (encoder_left, encoder_right),
                )
                applied_left, applied_right = fault.motors(
                    now, (left_target, right_target)
                )
                left_motor.setVelocity(applied_left)
                right_motor.setVelocity(applied_right)
            elif sensor_error is None:
                _stop(motors)
                left_target = right_target = 0.0
            else:
                target_heading = math.atan2(
                    target_point[1] - pose_y, target_point[0] - pose_x
                )
                heading_error = _wrap_angle(target_heading - heading)
                corrected_heading_error = heading_error - sensor_heading_gain * sensor_error
                curvature = 2.0 * math.sin(corrected_heading_error) / max(lookahead, 1e-4)

                wheel_speed = min(
                    default_wheel_speed,
                    edge["speed_limit"] or default_wheel_speed,
                    max_wheel_speed,
                )
                linear_speed = wheel_speed * wheel_radius
                angular_speed = linear_speed * curvature
                left_target = (linear_speed - angular_speed * wheel_base * 0.5) / wheel_radius
                right_target = (linear_speed + angular_speed * wheel_base * 0.5) / wheel_radius
                left_target = max(0.0, min(max_wheel_speed, left_target))
                right_target = max(0.0, min(max_wheel_speed, right_target))
                applied_left, applied_right = fault.motors(now, (left_target, right_target))
                left_motor.setVelocity(applied_left)
                right_motor.setVelocity(applied_right)

            row = [
                now,
                route.start,
                route.destination,
                edge["edge_id"],
                pose_x,
                pose_y,
                distance_from_path,
                "" if sensor_error is None else sensor_error,
                left_target,
                right_target,
                encoder_left,
                encoder_right,
                *sensor_values,
                fault.type,
                int(fault.active(now)),
            ]
            if control_mode == "esp32":
                row.extend((esp_state, esp_route_index, esp_distance, *esp_anomaly))
            writer.writerow(row)
            buffered_rows += 1
            if buffered_rows >= batch_size:
                log_file.flush()
                buffered_rows = 0

            if control_mode == "esp32" and esp_state in ESP_STOP_STATES:
                _stop(motors)
                if esp_state == "AT_DEST":
                    destination = _pixel_to_world(
                        [nodes[route.destination]["x"], nodes[route.destination]["y"]],
                        transform,
                    )
                    destination_tolerance = float(
                        mission.get("esp_destination_tolerance_m", 0.05)
                    )
                    stop_error = math.dist((pose_x, pose_y), destination)
                    print(
                        f"ESP stopped {stop_error:.3f} m from {route.destination} "
                        f"after {esp_distance:.3f} m of odometry"
                    )
                    if stop_error > destination_tolerance:
                        failure = (
                            f"ESP stopped {stop_error:.3f} m from the destination node "
                            f"(tolerance {destination_tolerance:.3f} m)"
                        )
                    elif return_to_start and not on_return_leg:
                        unloading_until = now + float(mission.get("unload_s", 1.0))
                        print(f"Unloading at {route.destination}")
                        continue
                    else:
                        completed = True
                else:
                    failure = f"ESP controller reported {esp_state}"
                break

            # A U-turn sweeps the sensors off the line on purpose, and the
            # robot is stopped while unloading.
            if esp_state in ("U_TURN", "UNLOADING"):
                line_lost_since = None
            elif sensor_error is None:
                if line_lost_since is None:
                    line_lost_since = robot.getTime()
                elif robot.getTime() - line_lost_since > max_line_loss_s:
                    _stop(motors)
                    failure = (
                        f"line lost for more than {max_line_loss_s:.1f} s on "
                        f"{edge['edge_id']} at t={robot.getTime():.2f} s"
                    )
                    break
            else:
                line_lost_since = None

            if distance_from_path > float(mission.get("max_path_error_m", 0.12)):
                # Stop and close the log normally so the telemetry up to the
                # safety stop is kept (fault runs often end this way).
                _stop(motors)
                failure = (
                    f"robot {distance_from_path:.3f} m from planned edge "
                    f"{edge['edge_id']} at t={now:.2f} s; stopped for safety"
                )
                break

        log_file.flush()
        log_file.close()
        print(f"Route telemetry saved to {log_path}")
        if failure is not None:
            print(f"RUN FAILED: {failure}. Do not use this run as healthy data.")
        if completed or failure is not None:
            if completed:
                print(f"Destination reached: {route.destination}")
            if mission.get("quit_webots_on_complete", False):
                robot.simulationQuit(0 if completed else 1)
            else:
                # Keep the controller alive at the destination so Webots does
                # not restart it and immediately repeat the same mission.
                while robot.step(time_step) != -1:
                    _stop(motors)
    except Exception as error:
        _stop(motors)
        print(f"Route controller error: {error}", file=sys.stderr)
        # Webots drops the controller's last console lines when it quits, so
        # keep the traceback where headless runs can find it.
        try:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            with (LOG_DIR / "controller-errors.log").open("a", encoding="utf-8") as error_log:
                error_log.write(f"{datetime.now():%Y-%m-%d %H:%M:%S} t={robot.getTime():.3f}\n")
                error_log.write(traceback.format_exc() + "\n")
        except OSError:
            pass
        robot.simulationQuit(1)
    finally:
        if log_file is not None and not log_file.closed:
            log_file.close()
        if esp_link is not None and esp_link.is_open:
            esp_link.close()


if __name__ == "__main__":
    main()
