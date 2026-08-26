"""Pure helpers for reconstructing Go2 LiDAR point measurement time."""

from __future__ import annotations

from typing import Iterable, Sequence, Tuple

import numpy as np


def _offset_summary(values: np.ndarray) -> dict:
    if values.size == 0:
        return {"count": 0}
    return {
        "count": int(values.size),
        "minimum": float(values.min()),
        "median": float(np.median(values)),
        "maximum": float(values.max()),
        "range": float(values.max() - values.min()),
    }


def analyze_reconstructed_point_alignment(
    point_time_frames: Iterable[Tuple[float, Sequence[float]]],
    imu_stamps: Sequence[float],
) -> dict:
    """Match every covered ``header + point_time`` against the nearest IMU."""

    frame_arrays = []
    point_count = 0
    for header_stamp, point_times in point_time_frames:
        times = np.asarray(point_times, dtype=np.float64)
        if times.ndim != 1 or not np.isfinite(times).all():
            raise ValueError("point time arrays must be one-dimensional and finite")
        point_count += int(times.size)
        if times.size:
            frame_arrays.append(float(header_stamp) + times)

    imu = np.asarray(sorted(float(value) for value in imu_stamps), dtype=np.float64)
    if not frame_arrays:
        raise ValueError("no point measurement times")
    if imu.size == 0 or not np.isfinite(imu).all():
        raise ValueError("IMU timestamps must be finite and non-empty")

    points = np.concatenate(frame_arrays)
    covered_mask = (points >= imu[0]) & (points <= imu[-1])
    covered = points[covered_mask]
    if covered.size == 0:
        raise ValueError("point and IMU streams have no shared coverage")

    if imu.size == 1:
        nearest = np.full(covered.shape, imu[0], dtype=np.float64)
    else:
        indexes = np.searchsorted(imu, covered, side="left")
        indexes = np.clip(indexes, 1, imu.size - 1)
        before = imu[indexes - 1]
        after = imu[indexes]
        nearest = np.where(
            np.abs(after - covered) < np.abs(covered - before), after, before
        )

    offsets = nearest - covered
    absolute = np.abs(offsets)
    return {
        "point_count": point_count,
        "matched_count": int(covered.size),
        "outside_imu_coverage_count": int(points.size - covered.size),
        "nearest_imu_minus_point_s": _offset_summary(offsets),
        "absolute_offset_s": {
            "median": float(np.median(absolute)),
            "p95": float(np.quantile(absolute, 0.95)),
            "p99": float(np.quantile(absolute, 0.99)),
            "maximum": float(absolute.max()),
        },
        "note": (
            "Static nearest-neighbor coverage does not prove dynamic physical phase."
        ),
    }
