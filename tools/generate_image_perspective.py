#!/usr/bin/env python3
"""Generate the 188x120 point-only inverse-perspective table.

The calibration photographs are enlarged screenshots of the MT9V03X image.
This script recovers camera coordinates, extracts the measured longitudinal
markers and straight-road borders, validates the known lateral offsets, and
writes the integer lookup table used by the MM32 image-processing path.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


IMAGE_W = 188
IMAGE_H = 120
TRACK_WIDTH_MM = 450
BIRD_CENTER_COL = 94
BIRD_GRID_MM = 17
BIRD_NEAR_X = 119
SCREEN_BLACK_LIMIT = 20
ROAD_WHITE_LIMIT = 210


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance-dir", type=Path, required=True)
    parser.add_argument("--width-dir", type=Path, required=True)
    parser.add_argument("--output-header", type=Path, required=True)
    parser.add_argument("--report-dir", type=Path, required=True)
    return parser.parse_args()


def load_gray(path: Path) -> np.ndarray:
    return np.asarray(Image.open(path).convert("L"))


def camera_bbox(image: np.ndarray) -> tuple[int, int, int, int]:
    ys, xs = np.where(image > SCREEN_BLACK_LIMIT)
    if xs.size == 0:
        raise ValueError("camera image area was not found")
    left, right = int(xs.min()), int(xs.max()) + 1
    top, bottom = int(ys.min()), int(ys.max()) + 1
    aspect = (right - left) / (bottom - top)
    expected = IMAGE_W / IMAGE_H
    if abs(aspect - expected) > 0.01:
        raise ValueError(f"unexpected camera image aspect ratio: {aspect:.5f}")
    return left, top, right, bottom


def crop_camera(path: Path, bbox: tuple[int, int, int, int]) -> np.ndarray:
    image = load_gray(path)
    left, top, right, bottom = bbox
    return image[top:bottom, left:right]


def runs(mask: np.ndarray, minimum_length: int) -> list[tuple[int, int]]:
    result: list[tuple[int, int]] = []
    start: int | None = None
    for index, enabled in enumerate(mask):
        if enabled and start is None:
            start = index
        if start is not None and (not enabled or index == len(mask) - 1):
            end = index if enabled and index == len(mask) - 1 else index - 1
            if end - start + 1 >= minimum_length:
                result.append((start, end))
            start = None
    return result


def extract_road_edges(
    camera: np.ndarray,
    *,
    fit_straight_lines: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    screen_h, screen_w = camera.shape
    scale_x = screen_w / IMAGE_W
    scale_y = screen_h / IMAGE_H
    left = np.full(IMAGE_H, np.nan, dtype=float)
    right = np.full(IMAGE_H, np.nan, dtype=float)
    previous_center = BIRD_CENTER_COL * scale_x

    for row in range(IMAGE_H - 1, -1, -1):
        screen_y = int(round((row + 0.5) * scale_y - 0.5))
        y0 = max(0, screen_y - 2)
        y1 = min(screen_h, screen_y + 3)
        profile = np.median(camera[y0:y1], axis=0)
        candidates = runs(profile > ROAD_WHITE_LIMIT, max(12, int(scale_x * 2)))
        if not candidates:
            continue
        containing = [item for item in candidates if item[0] <= previous_center <= item[1]]
        if containing:
            chosen = max(containing, key=lambda item: item[1] - item[0])
        else:
            chosen = min(
                candidates,
                key=lambda item: abs((item[0] + item[1]) * 0.5 - previous_center),
            )
        previous_center = (chosen[0] + chosen[1]) * 0.5
        left[row] = chosen[0] / scale_x
        right[row] = chosen[1] / scale_x

    valid = np.isfinite(left) & np.isfinite(right) & ((right - left) > 8)
    if not fit_straight_lines:
        return left, right

    fit_rows = np.where(valid & (np.arange(IMAGE_H) >= 10))[0]
    if fit_rows.size < 60:
        raise ValueError("not enough straight-road rows for lateral calibration")

    # Straight road borders are projective lines. A linear fit suppresses the
    # screenshot interpolation and isolated threshold noise.
    left_fit = np.polyval(np.polyfit(fit_rows, left[fit_rows], 1), np.arange(IMAGE_H))
    right_fit = np.polyval(np.polyfit(fit_rows, right[fit_rows], 1), np.arange(IMAGE_H))
    left_fit = np.clip(left_fit, 0.0, IMAGE_W - 2.0)
    right_fit = np.clip(right_fit, left_fit + 2.0, IMAGE_W - 1.0)
    return left_fit, right_fit


def extract_marker_lower_edge(
    reference: np.ndarray,
    marked: np.ndarray,
    distance_mm: int,
) -> float:
    screen_h, screen_w = reference.shape
    center0 = int(screen_w * 0.42)
    center1 = int(screen_w * 0.58)
    difference = np.maximum(
        reference[:, center0:center1].astype(np.int16)
        - marked[:, center0:center1].astype(np.int16),
        0,
    )
    score = np.mean(difference, axis=1)
    candidate_mask = score > 25.0
    candidate_mask[: max(8, int(screen_h * 0.045))] = False
    candidates = runs(candidate_mask, max(4, int(screen_h / IMAGE_H * 0.5)))
    if not candidates:
        raise ValueError(f"marker was not found for {distance_mm} mm")

    # The actual marker produces the strongest sustained darkening of the
    # otherwise white lane. Select by mean score, then use its lower edge.
    selected = max(
        candidates,
        key=lambda item: float(np.mean(score[item[0] : item[1] + 1])),
    )
    return (selected[1] + 1) * IMAGE_H / screen_h


def linear_extrapolated_interpolation(
    sample_rows: np.ndarray,
    sample_distances: np.ndarray,
) -> np.ndarray:
    order = np.argsort(sample_rows)
    rows = sample_rows[order]
    distances = sample_distances[order]
    output = np.interp(np.arange(IMAGE_H), rows, distances)

    low_slope = (distances[1] - distances[0]) / (rows[1] - rows[0])
    high_slope = (distances[-1] - distances[-2]) / (rows[-1] - rows[-2])
    low_mask = np.arange(IMAGE_H) < rows[0]
    high_mask = np.arange(IMAGE_H) > rows[-1]
    output[low_mask] = distances[0] + (np.arange(IMAGE_H)[low_mask] - rows[0]) * low_slope
    output[high_mask] = distances[-1] + (np.arange(IMAGE_H)[high_mask] - rows[-1]) * high_slope
    return np.clip(output, 0.0, BIRD_NEAR_X * BIRD_GRID_MM)


def extract_shift_validation(
    base_left: np.ndarray,
    base_right: np.ndarray,
    shifted_camera: np.ndarray,
) -> dict[str, float]:
    shifted_left, shifted_right = extract_road_edges(
        shifted_camera, fit_straight_lines=False
    )
    rows_to_use = np.arange(20, 91)
    base_center = (base_left + base_right) * 0.5
    shifted_center = (shifted_left + shifted_right) * 0.5
    valid = (
        np.isfinite(base_left[rows_to_use])
        & np.isfinite(base_right[rows_to_use])
        & np.isfinite(shifted_left[rows_to_use])
        & np.isfinite(shifted_right[rows_to_use])
    )
    rows_to_use = rows_to_use[valid]
    shifts = (
        (shifted_center[rows_to_use] - base_center[rows_to_use])
        * TRACK_WIDTH_MM
        / (base_right[rows_to_use] - base_left[rows_to_use])
    )
    return {
        "median_mm": float(np.median(shifts)),
        "p10_mm": float(np.percentile(shifts, 10)),
        "p90_mm": float(np.percentile(shifts, 90)),
    }


def format_array(values: list[int], per_line: int = 16) -> str:
    lines = []
    for start in range(0, len(values), per_line):
        lines.append("    " + ", ".join(str(value) for value in values[start : start + per_line]))
    return ",\n".join(lines)


def write_header(
    output: Path,
    forward_x: np.ndarray,
    lane_center: np.ndarray,
    lane_width: np.ndarray,
    distances_mm: np.ndarray,
) -> None:
    lut_values: list[int] = []
    for row in range(IMAGE_H):
        center = lane_center[row]
        width = lane_width[row]
        for col in range(IMAGE_W):
            lateral_mm = (col - center) * TRACK_WIDTH_MM / width
            bird_y = int(round(BIRD_CENTER_COL + lateral_mm / BIRD_GRID_MM))
            lut_values.extend((int(forward_x[row]), max(0, min(IMAGE_W - 1, bird_y))))

    inverse_row: list[int] = []
    for bird_x in range(IMAGE_H):
        inverse_row.append(int(np.argmin(np.abs(forward_x.astype(int) - bird_x))))

    center_q8 = [int(round(value * 256.0)) for value in lane_center]
    width_q8 = [int(round(value * 256.0)) for value in lane_width]
    distance_int = [int(round(value)) for value in distances_mm]

    text = f"""#ifndef __IMAGE_PERSPECTIVE_TABLE_H__
#define __IMAGE_PERSPECTIVE_TABLE_H__

// Generated by tools/generate_image_perspective.py from the raised-camera calibration set.
#define IMAGE_PERSPECTIVE_GRID_MM          ({BIRD_GRID_MM}U)
#define IMAGE_PERSPECTIVE_TRACK_WIDTH_MM   ({TRACK_WIDTH_MM}U)
#define IMAGE_PERSPECTIVE_CENTER_COL       ({BIRD_CENTER_COL}U)
#define IMAGE_PERSPECTIVE_NEAR_X           ({BIRD_NEAR_X}U)

static const uint8 image_perspective_lut[MT9V03X_IMAGE_SIZE * 2U] =
{{
{format_array(lut_values, 24)}
}};

static const uint16 image_perspective_distance_mm[MT9V03X_H] =
{{
{format_array(distance_int)}
}};

static const uint16 image_perspective_lane_center_q8[MT9V03X_H] =
{{
{format_array(center_q8)}
}};

static const uint16 image_perspective_lane_width_q8[MT9V03X_H] =
{{
{format_array(width_q8)}
}};

static const uint8 image_perspective_source_row[MT9V03X_H] =
{{
{format_array(inverse_row)}
}};

#endif
"""
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8", newline="\n")


def write_overlay(
    output: Path,
    base_path: Path,
    bbox: tuple[int, int, int, int],
    left: np.ndarray,
    right: np.ndarray,
) -> None:
    image = Image.open(base_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    x0, y0, x1, y1 = bbox
    scale_x = (x1 - x0) / IMAGE_W
    scale_y = (y1 - y0) / IMAGE_H
    for row in range(IMAGE_H):
        y = y0 + (row + 0.5) * scale_y
        draw.ellipse((x0 + left[row] * scale_x - 2, y - 2, x0 + left[row] * scale_x + 2, y + 2), fill=(255, 0, 0))
        draw.ellipse((x0 + right[row] * scale_x - 2, y - 2, x0 + right[row] * scale_x + 2, y + 2), fill=(0, 128, 255))
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def main() -> None:
    args = parse_args()
    base_path = args.width_dir / "完全居中、车身平行.png"
    base_full = load_gray(base_path)
    bbox = camera_bbox(base_full)
    base_camera = crop_camera(base_path, bbox)
    base_left_raw, base_right_raw = extract_road_edges(
        base_camera, fit_straight_lines=False
    )
    left, right = extract_road_edges(base_camera)
    lane_center = (left + right) * 0.5
    lane_width = right - left

    marker_rows: list[float] = []
    marker_distances: list[float] = []
    marker_pattern = re.compile(r"^(\d+)cm$")
    for path in args.distance_dir.glob("*.png"):
        match = marker_pattern.match(path.stem)
        if not match:
            continue
        distance_mm = int(match.group(1)) * 10
        marker_rows.append(extract_marker_lower_edge(base_camera, crop_camera(path, bbox), distance_mm))
        marker_distances.append(float(distance_mm))

    if len(marker_rows) < 8:
        raise ValueError("not enough longitudinal marker images")

    rows_array = np.asarray(marker_rows)
    distances_array = np.asarray(marker_distances)
    distances_mm = linear_extrapolated_interpolation(rows_array, distances_array)
    forward_x = np.rint(BIRD_NEAR_X - distances_mm / BIRD_GRID_MM).astype(int)
    forward_x = np.clip(forward_x, 0, BIRD_NEAR_X).astype(np.uint8)

    validations = {}
    validation_files = {
        "right_10cm": args.width_dir / "向右偏10 cm、仍然平行.png",
        "right_5cm": args.width_dir / "向右偏5 cm、仍然平行.png",
        "left_5cm": args.width_dir / "向左偏5 cm、仍然平行.png",
        "left_10cm": args.width_dir / "向左偏10 cm、仍然平行.png",
    }
    for name, path in validation_files.items():
        validations[name] = extract_shift_validation(
            base_left_raw, base_right_raw, crop_camera(path, bbox)
        )

    report = {
        "camera_bbox": list(bbox),
        "camera_screen_size": [bbox[2] - bbox[0], bbox[3] - bbox[1]],
        "raw_size": [IMAGE_W, IMAGE_H],
        "track_width_mm": TRACK_WIDTH_MM,
        "bird_grid_mm": BIRD_GRID_MM,
        "marker_samples": sorted(
            [
                {"distance_mm": int(distance), "raw_row": float(row)}
                for row, distance in zip(marker_rows, marker_distances)
            ],
            key=lambda item: item["distance_mm"],
        ),
        "lateral_validation": validations,
        "lane_fit": {
            "left_row_20": float(left[20]),
            "right_row_20": float(right[20]),
            "left_row_80": float(left[80]),
            "right_row_80": float(right[80]),
            "left_row_110": float(left[110]),
            "right_row_110": float(right[110]),
        },
    }

    args.report_dir.mkdir(parents=True, exist_ok=True)
    (args.report_dir / "calibration_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    write_overlay(args.report_dir / "straight_edge_fit.png", base_path, bbox, left, right)
    write_header(args.output_header, forward_x, lane_center, lane_width, distances_mm)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
