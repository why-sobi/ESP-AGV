"""Render a graph map JSON as a standalone, editable-workflow HTML preview."""

from __future__ import annotations

import argparse
import base64
import html
import json
import math
import mimetypes
from pathlib import Path
from typing import Any


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite number.")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be a finite number.")
    return result


def _load_map(map_path: Path) -> tuple[dict[str, Any], dict[str, tuple[float, float]]]:
    try:
        with map_path.open(encoding="utf-8") as map_file:
            data = json.load(map_file)
    except json.JSONDecodeError as error:
        raise ValueError(f"Invalid JSON at line {error.lineno}: {error.msg}") from error

    if not isinstance(data, dict):
        raise ValueError("Map JSON must contain an object.")

    raw_nodes = data.get("nodes")
    raw_edges = data.get("edges")
    if not isinstance(raw_nodes, list) or not raw_nodes:
        raise ValueError("Map must contain a non-empty 'nodes' array.")
    if not isinstance(raw_edges, list):
        raise ValueError("Map 'edges' must be an array.")

    nodes: dict[str, tuple[float, float]] = {}
    for index, node in enumerate(raw_nodes):
        if not isinstance(node, dict):
            raise ValueError(f"Node at index {index} must be an object.")
        node_id = node.get("id")
        if not isinstance(node_id, str) or not node_id.strip():
            raise ValueError(f"Node at index {index} needs a non-empty 'id'.")
        if node_id in nodes:
            raise ValueError(f"Duplicate node id: {node_id!r}.")
        nodes[node_id] = (
            _number(node.get("x"), f"node {node_id!r} x"),
            _number(node.get("y"), f"node {node_id!r} y"),
        )

    edge_ids: set[str] = set()
    for index, edge in enumerate(raw_edges):
        if not isinstance(edge, dict):
            raise ValueError(f"Edge at index {index} must be an object.")
        edge_id = edge.get("id")
        if not isinstance(edge_id, str) or not edge_id.strip():
            raise ValueError(f"Edge at index {index} needs a non-empty 'id'.")
        if edge_id in edge_ids:
            raise ValueError(f"Duplicate edge id: {edge_id!r}.")
        edge_ids.add(edge_id)
        source = edge.get("from")
        target = edge.get("to")
        if (
            not isinstance(source, str)
            or not isinstance(target, str)
            or source not in nodes
            or target not in nodes
        ):
            raise ValueError(f"Edge {edge_id!r} references an unknown node.")

        geometry = edge.get("geometry")
        if geometry is not None:
            if not isinstance(geometry, list) or len(geometry) < 2:
                raise ValueError(
                    f"Edge {edge_id!r} geometry must contain at least two points."
                )
            for point_index, point in enumerate(geometry):
                if not isinstance(point, list) or len(point) != 2:
                    raise ValueError(
                        f"Edge {edge_id!r} geometry point {point_index} "
                        "must be an [x, y] pair."
                    )
                _number(point[0], f"edge {edge_id!r} point {point_index} x")
                _number(point[1], f"edge {edge_id!r} point {point_index} y")

    return data, nodes


def _find_background(map_path: Path, map_data: dict[str, Any]) -> Path | None:
    source_name = map_data.get("source_image")
    if not isinstance(source_name, str) or not source_name:
        return None

    candidates = [
        map_path.parent / source_name,
        map_path.parent.parent / "image" / source_name,
        map_path.parent.parent / "textures" / source_name,
    ]
    return next((candidate for candidate in candidates if candidate.is_file()), None)


def _image_data_uri(image_path: Path) -> str:
    mime_type = mimetypes.guess_type(image_path.name)[0] or "application/octet-stream"
    encoded = base64.b64encode(image_path.read_bytes()).decode("ascii")
    return f"data:{mime_type};base64,{encoded}"


def _path_midpoint(points: list[tuple[float, float]]) -> tuple[float, float]:
    lengths = [
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    ]
    half_length = sum(lengths) / 2
    for (x1, y1), (x2, y2), length in zip(points, points[1:], lengths):
        if half_length <= length:
            fraction = half_length / length if length else 0
            return x1 + (x2 - x1) * fraction, y1 + (y2 - y1) * fraction
        half_length -= length
    return points[-1]


def _render_html(
    map_path: Path,
    map_data: dict[str, Any],
    nodes: dict[str, tuple[float, float]],
    background: Path | None,
) -> str:
    coordinate_system = map_data.get("coordinate_system")
    if not isinstance(coordinate_system, dict):
        coordinate_system = {}
    units = str(coordinate_system.get("units", "map units"))
    origin = coordinate_system.get("origin", "bottom_left")
    scale = coordinate_system.get("pixels_per_meter")
    if scale is not None:
        units += f" ({scale} px/m)"

    all_points = list(nodes.values())
    edge_geometries: dict[str, list[tuple[float, float]]] = {}
    for edge in map_data["edges"]:
        geometry = edge.get("geometry")
        if geometry is None:
            points = [nodes[edge["from"]], nodes[edge["to"]]]
        else:
            points = [(float(point[0]), float(point[1])) for point in geometry]
        if origin == "bottom_left":
            points = [(x, -y if y else 0.0) for x, y in points]
        edge_geometries[edge["id"]] = points
        all_points.extend(points)

    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    span_x = max_x - min_x
    span_y = max_y - min_y
    padding = max(span_x, span_y, 1.0) * 0.06
    view_x = min_x - padding
    view_y = min_y - padding
    view_width = max(span_x + 2 * padding, 1.0)
    view_height = max(span_y + 2 * padding, 1.0)

    labels = []
    if background is not None and origin != "bottom_left":
        image_width, image_height = _image_dimensions(map_data)
        if image_width is not None and image_height is not None:
            uri = _image_data_uri(background)
            labels.append(
                f'<image href="{uri}" x="0" y="0" width="{image_width}" '
                f'height="{image_height}" opacity="0.24" preserveAspectRatio="none" />'
            )

    for edge in map_data["edges"]:
        points = edge_geometries[edge["id"]]
        point_text = " ".join(f"{x:.3f},{y:.3f}" for x, y in points)
        edge_id = html.escape(edge["id"])
        edge_title = html.escape(
            f'{edge["id"]}: {edge["from"]} -> {edge["to"]}; '
            f'distance={edge.get("distance", "unspecified")} {units}'
        )
        middle_x, middle_y = _path_midpoint(points)
        labels.append(
            f'<polyline class="edge-line" points="{point_text}" '
            f'data-edge="{edge_id}"><title>{edge_title}</title></polyline>'
        )
        labels.append(
            f'<text class="edge-label" x="{middle_x:.3f}" y="{middle_y:.3f}" '
            f'data-edge-label="{edge_id}">{edge_id}</text>'
        )

    for node in map_data["nodes"]:
        x, original_y = nodes[node["id"]]
        y = original_y
        if origin == "bottom_left":
            y = -y if y else 0.0
        node_id = html.escape(node["id"])
        node_name = html.escape(str(node.get("name", node["id"])))
        labels.append(
            f'<g class="node" data-node="{node_id}">'
            f'<circle cx="{x:.3f}" cy="{y:.3f}" r="{padding * 0.08:.3f}">'
            f'<title>{node_name} ({node_id}) at {x:.2f}, {original_y:.2f} {units}</title>'
            f'</circle>'
            f'<text x="{x + padding * 0.12:.3f}" y="{y - padding * 0.10:.3f}">'
            f'{node_id}</text></g>'
        )

    map_id = html.escape(str(map_data.get("map_id", map_path.stem)))
    status = map_data.get("conversion", {}).get("status", "map")
    status = html.escape(str(status))
    svg_content = "\n".join(labels)
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Map preview: {map_id}</title>
<style>
  :root {{ color-scheme: light; font-family: system-ui, sans-serif; }}
  body {{ margin: 0; color: #172033; background: #f3f5f8; }}
  header {{ padding: 14px 20px; background: #172033; color: white; }}
  header h1 {{ margin: 0 0 4px; font-size: 1.15rem; }}
  header p {{ margin: 0; color: #c9d2e3; font-size: .88rem; }}
  main {{ display: grid; grid-template-columns: minmax(0, 1fr) 300px; gap: 12px; padding: 12px; }}
  .canvas, aside {{ border: 1px solid #d5dbe5; border-radius: 8px; background: white; }}
  .canvas {{ min-height: 70vh; overflow: hidden; }}
  svg {{ width: 100%; height: 75vh; display: block; touch-action: none; }}
  .edge-line {{ fill: none; stroke: #1167a8; stroke-width: 4; stroke-linecap: round; stroke-linejoin: round; vector-effect: non-scaling-stroke; }}
  .edge-label {{ fill: #075985; font-size: 18px; font-weight: 700; paint-order: stroke; stroke: white; stroke-width: 5px; stroke-linejoin: round; text-anchor: middle; }}
  .node circle {{ fill: #e64242; stroke: white; stroke-width: 2; vector-effect: non-scaling-stroke; }}
  .node text {{ fill: #801d1d; font-size: 18px; font-weight: 700; paint-order: stroke; stroke: white; stroke-width: 5px; stroke-linejoin: round; }}
  aside {{ padding: 14px; align-self: start; max-height: 75vh; overflow: auto; }}
  h2 {{ margin: 0 0 8px; font-size: 1rem; }}
  .hint {{ color: #4b5563; font-size: .85rem; line-height: 1.45; }}
  .item {{ border-top: 1px solid #e5e9f0; padding: 8px 0; font-size: .85rem; }}
  .item code {{ font-weight: 700; }}
  button {{ margin: 4px 4px 8px 0; }}
  @media (max-width: 850px) {{ main {{ grid-template-columns: 1fr; }} svg {{ height: 60vh; }} }}
</style>
</head>
<body>
<header><h1>{map_id}</h1><p>{status} · coordinates: {html.escape(units)}</p></header>
<main>
  <section class="canvas" aria-label="Track graph visualization">
    <svg id="map" viewBox="{view_x:.3f} {view_y:.3f} {view_width:.3f} {view_height:.3f}" role="img" aria-label="Map graph">
      {svg_content}
    </svg>
  </section>
  <aside>
    <h2>Map elements</h2>
    <button id="reset">Reset view</button>
    <p class="hint">Scroll to zoom; drag to pan. Hover a node or edge for its details. Edit the JSON file, then regenerate this preview.</p>
    <h2>Nodes ({len(map_data["nodes"])})</h2>
    {"".join(f'<div class="item"><code>{html.escape(node["id"])}</code> — {html.escape(str(node.get("name", "")))}<br>x={nodes[node["id"]][0]:.2f}, y={nodes[node["id"]][1]:.2f}</div>' for node in map_data["nodes"])}
    <h2>Edges ({len(map_data["edges"])})</h2>
    {"".join(f'<div class="item"><code>{html.escape(edge["id"])}</code><br>{html.escape(edge["from"])} → {html.escape(edge["to"])}<br>length={html.escape(str(edge.get("distance", "unspecified")))} {html.escape(units)}</div>' for edge in map_data["edges"])}
  </aside>
</main>
<script>
(() => {{
  const svg = document.getElementById("map");
  const initial = svg.viewBox.baseVal;
  const original = {{ x: initial.x, y: initial.y, width: initial.width, height: initial.height }};
  let dragging = false;
  let lastPointer = null;
  const point = (event) => {{
    const matrix = svg.getScreenCTM().inverse();
    return new DOMPoint(event.clientX, event.clientY).matrixTransform(matrix);
  }};
  svg.addEventListener("wheel", (event) => {{
    event.preventDefault();
    const at = point(event);
    const factor = event.deltaY < 0 ? 0.85 : 1.18;
    const box = svg.viewBox.baseVal;
    const width = box.width * factor;
    const height = box.height * factor;
    const rx = (at.x - box.x) / box.width;
    const ry = (at.y - box.y) / box.height;
    svg.setAttribute("viewBox", `${{at.x - width * rx}} ${{at.y - height * ry}} ${{width}} ${{height}}`);
  }}, {{ passive: false }});
  svg.addEventListener("pointerdown", (event) => {{
    dragging = true;
    lastPointer = {{ x: event.clientX, y: event.clientY }};
    svg.setPointerCapture(event.pointerId);
  }});
  svg.addEventListener("pointermove", (event) => {{
    if (!dragging || !lastPointer) return;
    const box = svg.viewBox.baseVal;
    const rect = svg.getBoundingClientRect();
    const dx = (event.clientX - lastPointer.x) * box.width / rect.width;
    const dy = (event.clientY - lastPointer.y) * box.height / rect.height;
    svg.setAttribute("viewBox", `${{box.x - dx}} ${{box.y - dy}} ${{box.width}} ${{box.height}}`);
    lastPointer = {{ x: event.clientX, y: event.clientY }};
  }});
  svg.addEventListener("pointerup", () => {{ dragging = false; lastPointer = null; }});
  svg.addEventListener("pointercancel", () => {{ dragging = false; lastPointer = null; }});
  document.getElementById("reset").addEventListener("click", () => {{
    svg.setAttribute("viewBox", `${{original.x}} ${{original.y}} ${{original.width}} ${{original.height}}`);
  }});
}})();
</script>
</body>
</html>
"""


def _image_dimensions(map_data: dict[str, Any]) -> tuple[float | None, float | None]:
    conversion = map_data.get("conversion")
    if isinstance(conversion, dict):
        dimensions = conversion.get("original_dimensions")
        if (
            isinstance(dimensions, list)
            and len(dimensions) == 2
            and all(isinstance(value, (int, float)) for value in dimensions)
        ):
            return float(dimensions[0]), float(dimensions[1])
    return None, None


def visualize_map(
    map_path: str | Path,
    output_path: str | Path | None = None,
    output_dir: str | Path | None = None,
) -> Path:
    map_path = Path(map_path)
    if not map_path.is_file():
        raise FileNotFoundError(f"Map file not found: {map_path}")
    if output_path is not None and output_dir is not None:
        raise ValueError("Specify either output_path or output_dir, not both.")
    map_data, nodes = _load_map(map_path)
    background = _find_background(map_path, map_data)
    if output_path is None:
        output_dir = (
            Path(output_dir)
            if output_dir is not None
            else map_path.parent.parent / "visualizations"
        )
        output_path = output_dir / map_path.with_suffix(".html").name
    else:
        output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        _render_html(map_path, map_data, nodes, background),
        encoding="utf-8",
    )
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create a standalone HTML visualization of a map JSON."
    )
    parser.add_argument("-map", required=True, type=Path, help="Map JSON path.")
    parser.add_argument(
        "-out",
        type=Path,
        help="HTML output file (defaults to maps/visualizations/<map-name>.html).",
    )
    parser.add_argument(
        "-out-dir",
        type=Path,
        help="HTML output directory (defaults to the sibling visualizations folder).",
    )
    args = parser.parse_args()

    try:
        output_path = visualize_map(args.map, args.out, args.out_dir)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    print(f"Map preview written to {output_path}")
    print("Open the HTML file in a browser; edit the JSON and rerun to refresh.")


if __name__ == "__main__":
    main()
