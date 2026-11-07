"""Convert a dark-path raster image into a draft graph map."""

from __future__ import annotations

import argparse
import json
import math
from collections import deque
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image, ImageOps

Pixel = tuple[int, int]
PixelLink = frozenset[Pixel]


def _neighbors(pixel: Pixel, foreground: set[Pixel]) -> list[Pixel]:
    x, y = pixel
    result = []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            neighbor = (x + dx, y + dy)
            if neighbor not in foreground:
                continue
            if dx and dy and (
                (x + dx, y) in foreground or (x, y + dy) in foreground
            ):
                continue
            result.append(neighbor)
    return result


def _thin(binary: np.ndarray) -> np.ndarray:
    image = binary.copy()
    height, width = image.shape
    max_iterations = max(height, width)

    for _ in range(max_iterations):
        changed = False
        for phase in (0, 1):
            padded = np.pad(image, 1)
            north = padded[:-2, 1:-1]
            northeast = padded[:-2, 2:]
            east = padded[1:-1, 2:]
            southeast = padded[2:, 2:]
            south = padded[2:, 1:-1]
            southwest = padded[2:, :-2]
            west = padded[1:-1, :-2]
            northwest = padded[:-2, :-2]

            neighbors = (
                north.astype(np.uint8)
                + northeast
                + east
                + southeast
                + south
                + southwest
                + west
                + northwest
            )
            transitions = (
                (~north & northeast).astype(np.uint8)
                + (~northeast & east)
                + (~east & southeast)
                + (~southeast & south)
                + (~south & southwest)
                + (~southwest & west)
                + (~west & northwest)
                + (~northwest & north)
            )

            if phase == 0:
                preserve = ~(north & east & south) & ~(east & south & west)
            else:
                preserve = ~(north & east & west) & ~(north & south & west)

            remove = (
                image
                & (neighbors >= 2)
                & (neighbors <= 6)
                & (transitions == 1)
                & preserve
            )
            if np.any(remove):
                image[remove] = False
                changed = True
        if not changed:
            break
    return image


def _components(pixels: Iterable[Pixel], adjacency: dict[Pixel, list[Pixel]]) -> list[set[Pixel]]:
    remaining = set(pixels)
    components = []
    while remaining:
        seed = remaining.pop()
        component = {seed}
        queue = [seed]
        while queue:
            current = queue.pop()
            for neighbor in adjacency[current]:
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)
        components.append(component)
    return components


def trace_path_map(
    image_path: str | Path,
    output_dir: str | Path,
    threshold: int = 128,
    max_dimension: int = 600,
    pixels_per_meter: float | None = None,
) -> Path:
    image_path = Path(image_path)
    output_dir = Path(output_dir)
    if not image_path.is_file():
        raise FileNotFoundError(f"Input image not found: {image_path}")
    if not 0 <= threshold <= 255:
        raise ValueError("Threshold must be between 0 and 255.")
    if max_dimension < 32:
        raise ValueError("Maximum processing dimension must be at least 32 pixels.")
    if pixels_per_meter is not None and (
        not math.isfinite(pixels_per_meter) or pixels_per_meter <= 0
    ):
        raise ValueError("Pixels per meter must be a positive finite number.")

    with Image.open(image_path) as source:
        grayscale = ImageOps.grayscale(source.convert("RGBA"))
        original_width, original_height = grayscale.size
        resize_ratio = min(1.0, max_dimension / max(original_width, original_height))
        if resize_ratio < 1.0:
            processed_size = (
                max(1, round(original_width * resize_ratio)),
                max(1, round(original_height * resize_ratio)),
            )
            grayscale = grayscale.resize(processed_size, Image.Resampling.BOX)
        else:
            processed_size = grayscale.size

        binary = np.asarray(grayscale) < threshold

    if not np.any(binary):
        raise ValueError("The image contains no dark path pixels at this threshold.")

    skeleton = _thin(binary)
    foreground = {
        (int(x), int(y))
        for y, x in np.argwhere(skeleton)
    }
    if not foreground:
        raise ValueError("Could not extract a centerline from the image.")

    adjacency = {pixel: _neighbors(pixel, foreground) for pixel in foreground}
    key_pixels = {
        pixel for pixel, neighbors in adjacency.items() if len(neighbors) != 2
    }

    # A closed track with no intersections still needs an anchor node.
    components = _components(foreground, adjacency)
    for component in components:
        if not any(pixel in key_pixels for pixel in component):
            key_pixels.add(min(component, key=lambda pixel: (pixel[1], pixel[0])))

    key_adjacency = {
        pixel: [neighbor for neighbor in adjacency[pixel] if neighbor in key_pixels]
        for pixel in key_pixels
    }
    node_components = _components(key_pixels, key_adjacency)
    pixel_to_node: dict[Pixel, int] = {}
    for node_index, component in enumerate(node_components):
        for pixel in component:
            pixel_to_node[pixel] = node_index

    edges = []
    visited_links: set[PixelLink] = set()
    scale_to_original = original_width / processed_size[0]

    for source_index, component in enumerate(node_components):
        for start_pixel in sorted(component):
            for first_pixel in adjacency[start_pixel]:
                if first_pixel in component:
                    continue
                first_link = frozenset((start_pixel, first_pixel))
                if first_link in visited_links:
                    continue

                path = [start_pixel, first_pixel]
                visited_links.add(first_link)
                previous = start_pixel
                current = first_pixel

                while current not in pixel_to_node:
                    choices = [p for p in adjacency[current] if p != previous]
                    if not choices:
                        break
                    next_pixel = choices[0]
                    link = frozenset((current, next_pixel))
                    if link in visited_links:
                        break
                    visited_links.add(link)
                    path.append(next_pixel)
                    previous, current = current, next_pixel

                if current not in pixel_to_node:
                    continue
                target_index = pixel_to_node[current]
                if source_index == target_index and len(path) < 4:
                    continue

                length_pixels = sum(
                    math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(path, path[1:])
                ) * scale_to_original
                length = (
                    length_pixels / pixels_per_meter
                    if pixels_per_meter is not None
                    else length_pixels
                )
                if length <= 0:
                    continue

                original_points = [
                    [
                        round(x * scale_to_original, 2),
                        round(y * scale_to_original, 2),
                    ]
                    for x, y in path
                ]
                edges.append(
                    {
                        "id": f"edge-{len(edges) + 1:03d}",
                        "from": f"node-{source_index + 1:03d}",
                        "to": f"node-{target_index + 1:03d}",
                        "distance": round(length, 3),
                        "bidirectional": True,
                        "geometry": original_points,
                    }
                )

    node_records = []
    for index, component in enumerate(node_components):
        x = sum(pixel[0] for pixel in component) / len(component) * scale_to_original
        y = sum(pixel[1] for pixel in component) / len(component) * scale_to_original
        degree = sum(
            1
            for pixel in component
            for neighbor in adjacency[pixel]
            if neighbor not in component
        )
        kind = "junction" if degree > 2 else "endpoint" if degree == 1 else "loop-anchor"
        node_records.append(
            {
                "id": f"node-{index + 1:03d}",
                "name": f"{kind.replace('-', ' ').title()} {index + 1}",
                "x": round(x, 2),
                "y": round(y, 2),
                "degree": degree,
                "kind": kind,
            }
        )

    map_data = {
        "map_id": image_path.stem,
        "version": 1,
        "source_image": image_path.name,
        "coordinate_system": {
            "type": "image_pixels",
            "origin": "top_left",
            "units": "meters" if pixels_per_meter is not None else "pixels",
            "pixels_per_meter": pixels_per_meter,
            "webots_transform_calibrated": False,
        },
        "conversion": {
            "status": "draft_requires_visual_review",
            "threshold": threshold,
            "processed_dimensions": list(processed_size),
            "original_dimensions": [original_width, original_height],
        },
        "nodes": node_records,
        "edges": edges,
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{image_path.stem}.json"
    output_path.write_text(
        json.dumps(map_data, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a dark-path image to a draft node/edge map JSON."
    )
    parser.add_argument("-img", required=True, type=Path, help="Track image path.")
    parser.add_argument("-out", required=True, type=Path, help="Output directory.")
    parser.add_argument(
        "--threshold",
        type=int,
        default=128,
        help="Grayscale threshold from 0 to 255 (default: 128).",
    )
    parser.add_argument(
        "--max-dimension",
        type=int,
        default=600,
        help="Maximum image dimension used for skeletonization (default: 600).",
    )
    parser.add_argument(
        "--pixels-per-meter",
        type=float,
        help="Calibrated image scale; omit to keep edge lengths in pixels.",
    )
    args = parser.parse_args()

    try:
        output_path = trace_path_map(
            args.img,
            args.out,
            threshold=args.threshold,
            max_dimension=args.max_dimension,
            pixels_per_meter=args.pixels_per_meter,
        )
    except (OSError, ValueError) as error:
        parser.error(str(error))

    print(f"Draft map written to {output_path}")
    print("Review the extracted nodes/edges and calibrate the image-to-Webots transform.")


if __name__ == "__main__":
    main()
