#!/usr/bin/env python3
"""Offline validation for the embedded HFK point-tracing pipeline."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

import generate_image_perspective as calibration


MAX_POINTS = 100
WHEELBASE_MM = 200
PURE_PURSUIT_MAX_ANGLE_X10 = 300
PURE_PURSUIT_DEADBAND_X10 = 10
STEERING_COMMAND_LIMIT_X10 = 250
DIR_FORWARD = ((-2, 0), (0, 2), (2, 0), (0, -2))
DIR_LEFT = ((-2, -2), (-2, 2), (2, 2), (2, -2))
DIR_RIGHT = ((-2, 2), (2, 2), (2, -2), (-2, -2))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-root", type=Path, required=True)
    parser.add_argument("--distance-dir", type=Path, required=True)
    parser.add_argument("--width-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--steering-near-cm", type=int, default=55)
    parser.add_argument("--steering-gain-percent", type=int, default=150)
    return parser.parse_args()


def make_calibration(distance_dir: Path, width_dir: Path):
    base_path = width_dir / "完全居中、车身平行.png"
    bbox = calibration.camera_bbox(calibration.load_gray(base_path))
    base = calibration.crop_camera(base_path, bbox)
    left, right = calibration.extract_road_edges(base)
    center = (left + right) * 0.5
    width = right - left

    rows = []
    distances = []
    for path in distance_dir.glob("*.png"):
        if not path.stem.endswith("cm"):
            continue
        distance = int(path.stem[:-2]) * 10
        rows.append(calibration.extract_marker_lower_edge(base, calibration.crop_camera(path, bbox), distance))
        distances.append(distance)
    distance_by_row = calibration.linear_extrapolated_interpolation(
        np.asarray(rows), np.asarray(distances)
    )
    forward = np.rint(
        calibration.BIRD_NEAR_X - distance_by_row / calibration.BIRD_GRID_MM
    ).astype(int)
    forward = np.clip(forward, 0, calibration.BIRD_NEAR_X)

    lut = np.zeros((calibration.IMAGE_H, calibration.IMAGE_W, 2), dtype=np.uint8)
    for row in range(calibration.IMAGE_H):
        lut[row, :, 0] = forward[row]
        for col in range(calibration.IMAGE_W):
            lateral_mm = (col - center[row]) * calibration.TRACK_WIDTH_MM / width[row]
            lut[row, col, 1] = np.clip(
                round(calibration.BIRD_CENTER_COL + lateral_mm / calibration.BIRD_GRID_MM),
                0,
                calibration.IMAGE_W - 1,
            )
    inverse_row = np.asarray(
        [int(np.argmin(np.abs(forward - x))) for x in range(calibration.IMAGE_H)]
    )
    return bbox, center, width, lut, inverse_row


def raw_camera(path: Path, bbox) -> np.ndarray:
    camera = Image.fromarray(calibration.crop_camera(path, bbox))
    return np.asarray(
        camera.resize(
            (calibration.IMAGE_W, calibration.IMAGE_H), Image.Resampling.BOX
        )
    )


def local_threshold(image: np.ndarray, row: int, col: int, cap: int = 6) -> int:
    mean = int(image[row - 2 : row + 3, col - 2 : col + 3].sum()) // 25
    return max(0, mean - cap)


def edge_contrast(image: np.ndarray, row: int, col: int) -> bool:
    left = int(image[row, col - 2])
    right = int(image[row, col + 2])
    difference = abs(left - right)
    return difference >= 15 and (left + right) // max(1, difference) <= 7


def seed_thresholds(image: np.ndarray) -> tuple[int, int]:
    white = int(image[70:91, 86:103].mean())
    left_black = int(image[70:91, 3:19].mean())
    right_black = int(image[70:91, 169:185].mean())
    return (white + left_black) // 2, (white + right_black) // 2


def find_seed(image: np.ndarray, threshold: int, left_side: bool):
    for row in range(116, 69, -3):
        if left_side:
            low, high = 3, 94
            while low < high:
                middle = (low + high) // 2
                samples = (middle, max(3, middle - 8), max(3, middle - 25))
                if all(int(image[row, col]) < threshold for col in samples):
                    low = middle + 1
                else:
                    high = middle
            edge = low
            if image[row, edge + 2] >= threshold and edge_contrast(image, row, edge):
                return row, edge + 1
        else:
            low, high = 94, 184
            while low < high:
                middle = (low + high + 1) // 2
                samples = (middle, min(184, middle + 8), min(184, middle + 25))
                if all(int(image[row, col]) < threshold for col in samples):
                    high = middle - 1
                else:
                    low = middle
            edge = high
            if image[row, edge - 2] >= threshold and edge_contrast(image, row, edge):
                return row, edge - 1
    return None


def trace(image: np.ndarray, seed, left_side: bool):
    if seed is None:
        return []
    row, col = seed
    direction = 0
    turns = 0
    update = 0
    threshold = local_threshold(image, row, col)
    output = []
    side_directions = DIR_LEFT if left_side else DIR_RIGHT
    while (
        3 < row < 117
        and 3 < col < 185
        and len(output) < MAX_POINTS
        and turns < 4
    ):
        update += 1
        if update >= 2:
            threshold = local_threshold(image, row, col)
            update = 0
        fr = row + DIR_FORWARD[direction][0]
        fc = col + DIR_FORWARD[direction][1]
        sr = row + side_directions[direction][0]
        sc = col + side_directions[direction][1]
        if image[fr, fc] < threshold:
            direction = (direction + (1 if left_side else 3)) & 3
            turns += 1
        elif image[sr, sc] < threshold:
            row, col = fr, fc
            output.append((row, col))
            turns = 0
        else:
            row, col = sr, sc
            output.append((row, col))
            turns = 0
            direction = (direction + (3 if left_side else 1)) & 3
        if (row, col) == seed and turns == 0:
            break
    return output


def filter_line(points):
    result = [[int(point[0]), int(point[1])] for point in points]
    for index in range(1, len(result)):
        result[index][0] = (result[index - 1][0] + result[index][0]) // 2
        result[index][1] = (result[index - 1][1] + result[index][1]) // 2
    return [tuple(point) for point in result]


def resample(points, distance):
    if len(points) < 2:
        return []
    output = []
    remain = 0
    for first, second in zip(points, points[1:]):
        x0, y0 = int(first[0]), int(first[1])
        dx, dy = int(second[0]) - x0, int(second[1]) - y0
        length = int(np.sqrt(dx * dx + dy * dy))
        if length == 0:
            continue
        while len(output) < MAX_POINTS:
            if remain >= length:
                remain -= length
                break
            output.append((round((x0 * length + dx * remain) / length), round((y0 * length + dy * remain) / length)))
            remain += distance
    return output


def center_from_border(border, from_left):
    if len(border) < 3:
        return []
    output = [border[0]] * len(border)
    half_width = round(225 / calibration.BIRD_GRID_MM)
    for index in range(1, len(border)):
        index0 = max(0, index - 2)
        index1 = min(len(border) - 1, index + 2)
        dx = border[index1][0] - border[index0][0]
        dy = border[index1][1] - border[index0][1]
        length = int(np.sqrt(dx * dx + dy * dy))
        if not length:
            output[index] = border[index]
        elif from_left:
            output[index] = (
                np.clip(border[index][0] + dy * half_width // length, 0, 119),
                np.clip(border[index][1] - dx * half_width // length, 0, 187),
            )
        else:
            output[index] = (
                np.clip(border[index][0] - dy * half_width // length, 0, 119),
                np.clip(border[index][1] + dx * half_width // length, 0, 187),
            )
    output[0] = output[1]
    return [(int(x), int(y)) for x, y in output]


def bird_to_image(point, center, width, inverse_row):
    x, y = point
    row = int(inverse_row[x])
    lateral_mm = (y - calibration.BIRD_CENTER_COL) * calibration.BIRD_GRID_MM
    col = center[row] + lateral_mm * width[row] / calibration.TRACK_WIDTH_MM
    return row, int(np.clip(round(col), 0, 187))


def atan_ratio_x10(numerator: int, denominator: int) -> int:
    tangent_x65536 = (
        0, 1143, 2288, 3434, 4582, 5732,
        6887, 8046, 9212, 10383, 11562, 12749,
        13945, 15151, 16368, 17597, 18840, 20097,
        21369, 22657, 23962, 25286, 26628, 27989,
        29371, 30775, 32203, 33654, 35132, 36637, 37837,
    )
    if numerator == 0 or denominator <= 0:
        return 0
    sign = 1 if numerator > 0 else -1
    scaled_numerator = abs(numerator) * 65536
    if scaled_numerator >= denominator * tangent_x65536[30]:
        return sign * PURE_PURSUIT_MAX_ANGLE_X10
    for degree in range(29, -1, -1):
        lower = denominator * tangent_x65536[degree]
        if scaled_numerator >= lower:
            upper = denominator * tangent_x65536[degree + 1]
            fraction_x10 = (scaled_numerator - lower) * 10 // (upper - lower)
            return sign * (degree * 10 + fraction_x10)
    return 0


def pure_pursuit_angle_x10(target_bird) -> int:
    wheelbase_units = (WHEELBASE_MM + calibration.BIRD_GRID_MM // 2) // calibration.BIRD_GRID_MM
    forward_units = calibration.BIRD_NEAR_X - target_bird[0] + wheelbase_units
    lateral_units = calibration.BIRD_CENTER_COL - target_bird[1]
    angle_x10 = atan_ratio_x10(
        2 * wheelbase_units * lateral_units,
        forward_units * forward_units + lateral_units * lateral_units,
    )
    return 0 if -PURE_PURSUIT_DEADBAND_X10 < angle_x10 < PURE_PURSUIT_DEADBAND_X10 else angle_x10


def steering_command_x10(angle_x10: int, gain_percent: int) -> int:
    scaled = angle_x10 * gain_percent
    scaled = math.trunc((scaled + 50 if scaled >= 0 else scaled - 50) / 100)
    return max(-STEERING_COMMAND_LIMIT_X10, min(STEERING_COMMAND_LIMIT_X10, scaled))


def process(
    path,
    bbox,
    center,
    width,
    lut,
    inverse_row,
    steering_near_cm,
    steering_gain_percent,
):
    image = raw_camera(path, bbox)
    left_threshold, right_threshold = seed_thresholds(image)
    left = trace(image, find_seed(image, left_threshold, True), True)
    right = trace(image, find_seed(image, right_threshold, False), False)
    left_bird = resample(
        filter_line([tuple(map(int, lut[r, c])) for r, c in left]), 3
    ) if len(left) >= 12 else []
    right_bird = resample(
        filter_line([tuple(map(int, lut[r, c])) for r, c in right]), 3
    ) if len(right) >= 12 else []
    if left_bird and (not right_bird or len(left_bird) >= len(right_bird)):
        center_bird = center_from_border(left_bird, True)
        side = "left"
    elif right_bird:
        center_bird = center_from_border(right_bird, False)
        side = "right"
    else:
        center_bird = []
        side = "none"
    center_bird = resample(filter_line(center_bird), 2) if center_bird else []
    center_image = [bird_to_image(point, center, width, inverse_row) for point in center_bird]
    target = None
    target_bird = None
    centerline_length_cm = None
    steering_near_actual_cm = None
    steering_near_reached = False
    pursuit_angle_x10 = None
    command_x10 = None
    if center_bird:
        previous = (calibration.BIRD_NEAR_X, calibration.BIRD_CENTER_COL)
        accumulated_mm = 0
        path_distances_mm = []
        for point in center_bird:
            dx_mm = (point[0] - previous[0]) * calibration.BIRD_GRID_MM
            dy_mm = (point[1] - previous[1]) * calibration.BIRD_GRID_MM
            accumulated_mm += math.isqrt(dx_mm * dx_mm + dy_mm * dy_mm)
            path_distances_mm.append(accumulated_mm)
            previous = point
        requested_mm = steering_near_cm * 10
        target_index = min(
            range(len(center_bird)),
            key=lambda i: abs(path_distances_mm[i] - requested_mm),
        )
        target = center_image[target_index]
        target_bird = center_bird[target_index]
        centerline_length_cm = (path_distances_mm[-1] + 5) // 10
        steering_near_actual_cm = (path_distances_mm[target_index] + 5) // 10
        steering_near_reached = abs(steering_near_actual_cm - steering_near_cm) <= 4
        pursuit_angle_x10 = pure_pursuit_angle_x10(target_bird)
        command_x10 = steering_command_x10(pursuit_angle_x10, steering_gain_percent)
    return (
        image,
        left,
        right,
        center_image,
        target,
        target_bird,
        centerline_length_cm,
        steering_near_actual_cm,
        steering_near_reached,
        pursuit_angle_x10,
        command_x10,
        side,
    )


def save_overlay(output: Path, image, left, right, center, target):
    canvas = Image.fromarray(image).convert("RGB").resize((752, 480), Image.Resampling.NEAREST)
    draw = ImageDraw.Draw(canvas)
    scale = 4
    for row, col in left:
        draw.ellipse((col * scale - 2, row * scale - 2, col * scale + 2, row * scale + 2), fill=(255, 0, 0))
    for row, col in right:
        draw.ellipse((col * scale - 2, row * scale - 2, col * scale + 2, row * scale + 2), fill=(0, 128, 255))
    for row, col in center:
        draw.ellipse((col * scale - 2, row * scale - 2, col * scale + 2, row * scale + 2), fill=(0, 255, 0))
    if target:
        row, col = target
        draw.line((col * scale - 12, row * scale, col * scale + 12, row * scale), fill=(255, 255, 0), width=3)
        draw.line((col * scale, row * scale - 12, col * scale, row * scale + 12), fill=(255, 255, 0), width=3)
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)


def main():
    args = parse_args()
    bbox, center, width, lut, inverse_row = make_calibration(args.distance_dir, args.width_dir)
    report = []
    for path in sorted(args.image_root.rglob("*.png")):
        # Distance-marker frames deliberately interrupt the road and are not ordinary-road cases.
        if path.parent == args.distance_dir:
            continue
        (
            image,
            left,
            right,
            centerline,
            target,
            target_bird,
            centerline_length_cm,
            steering_near_actual_cm,
            steering_near_reached,
            pursuit_angle_x10,
            command_x10,
            side,
        ) = process(
            path,
            bbox,
            center,
            width,
            lut,
            inverse_row,
            args.steering_near_cm,
            args.steering_gain_percent,
        )
        relative = path.relative_to(args.image_root)
        save_overlay(args.output_dir / relative, image, left, right, centerline, target)
        report.append(
            {
                "image": str(relative),
                "left_count": len(left),
                "right_count": len(right),
                "center_count": len(centerline),
                "selected_side": side,
                "target_row": target[0] if target else None,
                "target_col": target[1] if target else None,
                "target_bird_x": target_bird[0] if target_bird else None,
                "target_bird_y": target_bird[1] if target_bird else None,
                "centerline_length_cm": centerline_length_cm,
                "steering_near_set_cm": args.steering_near_cm,
                "steering_near_actual_cm": steering_near_actual_cm,
                "steering_near_reached": steering_near_reached,
                "pure_pursuit_angle_x10": pursuit_angle_x10,
                "steering_gain_percent": args.steering_gain_percent,
                "steering_command_x10": command_x10,
            }
        )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "track_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
