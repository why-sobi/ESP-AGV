"""Draw test tracks in the style of maps/image/base track.png (track2, track3).

Black 40 px lines on white, 2400 x 1800 px (the same image-to-world mapping
onto the 1 m x 1 m RectangleArena), rounded corners and right-angle
T-junctions only, which is what the ESP's junction logic expects. Layout: a
rounded rectangle split by a vertical bar, with a horizontal bar from the
middle of that bar to the right side, giving four T-junctions and six edges
(no parallel pair, no circle).

track3 is the hard course: an octagonal outer loop (every corner two
45-degree bends), a Y junction (a branch leaving the top line at 45 degrees),
a hairpin (180 degrees, ~6 cm radius), a tight 90-degree curve (~4 cm radius)
and a cross bar from the left side to the branch: four junctions, six edges.
Angles and radii are true in the world: the image is stretched onto the
square arena (x 0.417 mm/px, y 0.556 mm/px), so x and y pixel lengths are
scaled accordingly.

It also converts the image to a graph (scripts/maps/pathToMap.py) and copies
the base track's calibrated world_transform into it: same image size, same
arena, so the same mapping.

    python -m scripts.maps.make_track                 # track2
    python -m scripts.maps.make_track --track track3
"""

import math

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw

from scripts.maps.pathToMap import trace_path_map

ROOT = Path(__file__).resolve().parents[2]
BASE_MAP = ROOT / "maps" / "paths" / "base track.json"

WIDTH, HEIGHT = 2400, 1800
LINE = 40  # px, as on base track
Y_PER_X = (1.0 / WIDTH) / (1.0 / HEIGHT)  # y px per x px for equal world length (0.75)
SUPERSAMPLE = 4  # drawn larger and downsampled for antialiased edges like base track


def draw_track2(path):
    s = SUPERSAMPLE
    image = Image.new("L", (WIDTH * s, HEIGHT * s), 255)
    draw = ImageDraw.Draw(image)
    half = LINE // 2
    # Outer loop: line centres x 400..2000, y 280..1520; corner radius 170 px.
    left, top, right, bottom, radius = 400, 280, 2000, 1520, 170
    draw.rounded_rectangle(
        [(left - half) * s, (top - half) * s, (right + half) * s, (bottom + half) * s],
        radius=(radius + half) * s, outline=0, width=LINE * s)
    # Vertical bar at x = 950, top line to bottom line.
    bar_x = 950
    draw.rectangle([(bar_x - half) * s, top * s, (bar_x + half) * s, bottom * s], fill=0)
    # Horizontal bar at y = 1000, from the vertical bar to the right side.
    bar_y = 1000
    draw.rectangle([bar_x * s, (bar_y - half) * s, right * s, (bar_y + half) * s], fill=0)
    image = image.resize((WIDTH, HEIGHT), Image.LANCZOS).convert("RGB")
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    return path


def _arc(cx, cy, radius_x_px, start_deg, end_deg, steps=48):
    """Points of a circular (in the world) arc; angles in the image's y-down frame."""
    radius_y_px = radius_x_px * Y_PER_X
    return [(cx + radius_x_px * math.cos(math.radians(a)),
             cy + radius_y_px * math.sin(math.radians(a)))
            for a in (start_deg + (end_deg - start_deg) * i / steps for i in range(steps + 1))]


def draw_track3(path):
    s = SUPERSAMPLE
    image = Image.new("L", (WIDTH * s, HEIGHT * s), 255)
    draw = ImageDraw.Draw(image)

    def polyline(points):
        draw.line([(x * s, y * s) for x, y in points], fill=0, width=LINE * s, joint="curve")
        for x, y in points[:1] + points[-1:]:
            r = LINE * s / 2
            draw.ellipse([x * s - r, y * s - r, x * s + r, y * s + r], fill=0)

    # Octagon: 45-degree chamfers of 240 px in x = 180 px in y (equal in the world).
    left, right, top, bottom, cx, cy = 300, 2100, 300, 1500, 240, 180
    polyline([(left + cx, top), (right - cx, top), (right, top + cy), (right, bottom - cy),
              (right - cx, bottom), (left + cx, bottom), (left, bottom - cy), (left, top + cy),
              (left + cx, top)])
    # Y junction on the top line at x = 800: a branch down-right at 45 degrees,
    # then straight down past the cross bar's junction.
    branch_x = 800 + 320
    branch = [(800, top), (branch_x, top + 320 * Y_PER_X), (branch_x, 1100)]
    # Hairpin: 180 degrees round a ~6.3 cm radius (150 px in x), opening upwards.
    hairpin = _arc(branch_x + 150, 1100, 150, 180, 0)  # Down round the bottom.
    up_x = branch_x + 300
    # Up the other side, then a tight 90-degree right turn (~4 cm, 96 px) to the right side.
    turn_y = 700
    tight = _arc(up_x + 96, turn_y, 96, 180, 270)
    polyline(branch + hairpin[1:] + [(up_x, turn_y)] + tight[1:] + [(right, tight[-1][1])])
    # Cross bar from the left side to the branch's vertical.
    polyline([(left, 820), (branch_x, 820)])

    image = image.resize((WIDTH, HEIGHT), Image.LANCZOS).convert("RGB")
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    return path


TRACKS = {"track2": draw_track2, "track3": draw_track3}


def convert(image_path, map_dir):
    """Graph map of the image with the base track's world transform."""
    map_path = trace_path_map(image_path, map_dir)
    track_map = json.loads(map_path.read_text(encoding="utf-8"))
    transform = dict(json.loads(BASE_MAP.read_text(encoding="utf-8"))["world_transform"])
    if transform["image_size_px"] != list(Image.open(image_path).size):
        raise ValueError("The base track's world transform is for another image size.")
    transform["calibration"] = ("Copied from base track (same 2400 x 1800 px image on the "
                                "same 1 m x 1 m RectangleArena).")
    track_map["world_transform"] = transform
    map_path.write_text(json.dumps(track_map, indent=2) + "\n", encoding="utf-8")
    return map_path


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--track", choices=sorted(TRACKS), default="track2")
    parser.add_argument("--out", type=Path, help="Default: maps/image/<track>.png")
    parser.add_argument("--map-dir", type=Path, default=ROOT / "maps/paths")
    args = parser.parse_args()
    out = args.out or ROOT / "maps" / "image" / f"{args.track}.png"
    print(f"Wrote {TRACKS[args.track](out)}")
    print(f"Wrote {convert(out, args.map_dir)}")


if __name__ == "__main__":
    main()
