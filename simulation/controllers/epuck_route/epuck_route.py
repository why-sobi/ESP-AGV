import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path

from controller import Supervisor


CONTROLLER_DIR = Path(__file__).resolve().parent
ROOT_DIR = CONTROLLER_DIR.parent.parent.parent
MAP_PATH = ROOT_DIR / "maps" / "paths" / "base track.json"
MISSION_PATH = ROOT_DIR / "maps" / "routes" / "mission.json"
ROUTE_PATH = ROOT_DIR / "maps" / "routes" / "active_route.json"
LOG_DIR = ROOT_DIR / "data" / "route_runs"


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

    sys.path.insert(0, str(ROOT_DIR / "scripts"))
    from route_planner import TrackMap, shortest_route

    parsed_map = TrackMap.from_dict(track_map)
    route = shortest_route(
        parsed_map, mission["start_node"], mission["destination_node"]
    )
    ROUTE_PATH.parent.mkdir(parents=True, exist_ok=True)
    ROUTE_PATH.write_text(
        json.dumps(route.to_dict(), indent=2) + "\n",
        encoding="utf-8",
    )
    edges = {edge["id"]: edge for edge in track_map["edges"]}
    nodes = {node["id"]: node for node in track_map["nodes"]}

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

    return track_map, mission, route, nodes, route_paths, transform


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


def _stop(motors):
    motors[0].setVelocity(0.0)
    motors[1].setVelocity(0.0)


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

    try:
        track_map, mission, route, nodes, route_paths, transform = _read_inputs()
        if not route_paths:
            print(f"Already at destination {route.destination}; stopping.")
            return

        self_node = robot.getSelf()
        translation_field = self_node.getField("translation")
        rotation_field = self_node.getField("rotation")

        first_node = nodes[route.start]
        start_world = _pixel_to_world([first_node["x"], first_node["y"]], transform)
        first_points = route_paths[0]["points"]
        first_direction = next(
            (
                (b[0] - a[0], b[1] - a[1])
                for a, b in zip(first_points, first_points[1:])
                if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-8
            ),
            None,
        )

        if mission.get("spawn_at_start_node", False):
            if first_direction is None:
                raise ValueError("First route edge has no usable direction.")
            heading = math.atan2(first_direction[1], first_direction[0])
            current_z = self_node.getPosition()[2]
            translation_field.setSFVec3f([start_world[0], start_world[1], current_z])
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
        log_path = log_dir / f"route-{datetime.now():%Y%m%d-%H%M%S}.csv"
        log_file = log_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(log_file)
        writer.writerow(
            [
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
            ]
        )

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
        batch_size = 100
        buffered_rows = 0
        completed = False

        while robot.step(time_step) != -1:
            position = self_node.getPosition()
            pose_x, pose_y = position[0], position[1]
            pose_matrix = self_node.getOrientation()
            heading = math.atan2(pose_matrix[3], pose_matrix[0])
            sensor_values = [sensor.getValue() for sensor in ground_sensors]
            sensor_error = _line_error(sensor_values, line_threshold)

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
            target_node = _pixel_to_world(
                [nodes[edge["target_node"]]["x"], nodes[edge["target_node"]]["y"]],
                transform,
            )

            if math.dist((pose_x, pose_y), target_node) <= node_radius:
                edge_index += 1
                progress = 0.0
                if edge_index >= len(route_paths):
                    _stop(motors)
                    completed = True
                    break
                continue

            if sensor_error is None:
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
                left_motor.setVelocity(left_target)
                right_motor.setVelocity(right_target)

            writer.writerow(
                [
                    robot.getTime(),
                    route.start,
                    route.destination,
                    edge["edge_id"],
                    pose_x,
                    pose_y,
                    distance_from_path,
                    "" if sensor_error is None else sensor_error,
                    left_target,
                    right_target,
                    left_encoder.getValue(),
                    right_encoder.getValue(),
                    *sensor_values,
                ]
            )
            buffered_rows += 1
            if buffered_rows >= batch_size:
                log_file.flush()
                buffered_rows = 0

            if distance_from_path > float(mission.get("max_path_error_m", 0.12)):
                _stop(motors)
                raise RuntimeError(
                    f"Robot is {distance_from_path:.3f} m from planned edge "
                    f"{edge['edge_id']}; stopped for safety."
                )

        log_file.flush()
        log_file.close()
        print(f"Route telemetry saved to {log_path}")
        if completed:
            print(f"Destination reached: {route.destination}")
            if mission.get("quit_webots_on_complete", False):
                robot.simulationQuit(0)
            else:
                # Keep the controller alive at the destination so Webots does
                # not restart it and immediately repeat the same mission.
                while robot.step(time_step) != -1:
                    _stop(motors)
    except Exception as error:
        _stop(motors)
        print(f"Route controller error: {error}", file=sys.stderr)
        robot.simulationQuit(1)


if __name__ == "__main__":
    main()
