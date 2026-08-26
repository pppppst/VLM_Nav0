"""Pure validation helpers shared by Go2 preflight tools and unit tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from bisect import bisect_left
import math
from statistics import median
from typing import Dict, Iterable, List, Mapping, Optional, Sequence


GO2_SUBNET_PREFIX = "192.168.123."
TBD = "TBD / requires measurement"


class InterfaceSelectionError(RuntimeError):
    pass


class CalibrationError(RuntimeError):
    pass


class FormalPreflightError(RuntimeError):
    pass


def validate_formal_preflight_report(
    report: Mapping[str, object], *, now_unix_s: float, max_age_s: float
) -> None:
    """Require an explicit, successful, recent non-diagnostic sensor report."""

    if report.get("healthy") is not True:
        raise FormalPreflightError("formal sensor preflight is not healthy")
    formal_status = str(report.get("formal_preflight", ""))
    if formal_status != "PASS":
        if "NOT_EVALUATED" in formal_status or "diagnostic" in formal_status.lower():
            raise FormalPreflightError("diagnostic report is not a formal preflight")
        raise FormalPreflightError("formal preflight PASS marker is missing")
    generated = report.get("generated_at_unix_s")
    if not isinstance(generated, (int, float)) or not math.isfinite(float(generated)):
        raise FormalPreflightError("formal preflight generation time is missing")
    if not math.isfinite(now_unix_s) or not math.isfinite(max_age_s) or max_age_s <= 0:
        raise FormalPreflightError("invalid formal preflight freshness parameters")
    age = now_unix_s - float(generated)
    if age < -5.0:
        raise FormalPreflightError("formal preflight timestamp is in the future")
    if age > max_age_s:
        raise FormalPreflightError(
            f"formal preflight is stale ({age:.1f}s > {max_age_s:.1f}s)"
        )


def nearest_stream_offsets(
    reference_stamps: Sequence[float], high_rate_stamps: Sequence[float]
) -> List[float]:
    """Return reference-minus-nearest offsets within the shared time window."""

    high_rate = sorted(float(value) for value in high_rate_stamps)
    if not high_rate:
        raise ValueError("high-rate timestamp stream is empty")
    offsets = []
    for reference in reference_stamps:
        reference = float(reference)
        if reference < high_rate[0] or reference > high_rate[-1]:
            continue
        index = bisect_left(high_rate, reference)
        if index == 0:
            nearest = high_rate[0]
        elif index == len(high_rate):
            nearest = high_rate[-1]
        else:
            before = high_rate[index - 1]
            after = high_rate[index]
            nearest = before if reference - before <= after - reference else after
        offsets.append(reference - nearest)
    if not offsets:
        raise ValueError("timestamp streams have no shared coverage")
    return offsets


def analyze_cloud_points(points: Iterable[Sequence[float]]) -> Dict[str, object]:
    """Analyze x/y/z/time/ring tuples; ring is diagnostic, never a blocker."""

    finite_xyz = True
    finite_time = True
    times: List[float] = []
    rings: Dict[int, int] = {}
    point_count = 0
    for point in points:
        if len(point) < 5:
            raise ValueError("cloud point must contain x, y, z, time, ring")
        x, y, z, point_time, ring = point[:5]
        point_count += 1
        finite_xyz = finite_xyz and all(math.isfinite(float(value)) for value in (x, y, z))
        if math.isfinite(float(point_time)):
            times.append(float(point_time))
        else:
            finite_time = False
        ring_value = int(ring)
        rings[ring_value] = rings.get(ring_value, 0) + 1

    if point_count == 0:
        raise ValueError("cloud has no points")
    point_time_span = max(times) - min(times) if times else 0.0
    point_time_varies = finite_time and len(times) == point_count and point_time_span > 0.0
    return {
        "healthy": finite_xyz and finite_time and point_time_varies,
        "point_count": point_count,
        "finite_xyz": finite_xyz,
        "finite_time": finite_time,
        "point_time_varies": point_time_varies,
        "point_time_span_s": point_time_span,
        "ring_distribution": rings,
        "ring_single_value_diagnostic": len(rings) == 1,
    }


def _has_go2_address(addresses: Iterable[str]) -> bool:
    return any(address.split("/", 1)[0].startswith(GO2_SUBNET_PREFIX) for address in addresses)


def select_go2_interface(
    interfaces: Mapping[str, Sequence[str]], override: Optional[str] = None
) -> str:
    """Select exactly one interface on 192.168.123.0/24, failing closed."""

    if override:
        if override not in interfaces:
            raise InterfaceSelectionError(
                f"explicit interface {override!r} does not exist"
            )
        if not _has_go2_address(interfaces[override]):
            raise InterfaceSelectionError(
                f"explicit interface {override!r} has no 192.168.123.x address"
            )
        return override

    matches = [name for name, addresses in interfaces.items() if _has_go2_address(addresses)]
    if not matches:
        raise InterfaceSelectionError("no interface has a 192.168.123.x address")
    if len(matches) > 1:
        raise InterfaceSelectionError(
            "multiple interfaces have 192.168.123.x addresses; set GO2_NET_IFACE"
        )
    return matches[0]


def _is_numeric_vector(value: object, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(isinstance(item, (int, float)) and math.isfinite(item) for item in value)
    )


def require_calibrated(name: str, config: Mapping[str, object]) -> None:
    """Reject missing, TBD, or unverified identity hardware transforms."""

    if config.get("calibrated") is not True:
        raise CalibrationError(f"{name} is not calibrated")
    source = str(config.get("source", "")).strip()
    if not source or source == TBD or source.lower() == "unverified":
        raise CalibrationError(f"{name} has no verified source")

    if "translation" in config or "rotation_matrix" in config:
        translation = config.get("translation")
        rotation = config.get("rotation_matrix")
        if not _is_numeric_vector(translation, 3) or not _is_numeric_vector(rotation, 9):
            raise CalibrationError(f"{name} has invalid numeric LiDAR-to-IMU values")
        identity = translation == [0.0, 0.0, 0.0] and rotation == [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
    else:
        xyz = config.get("xyz")
        rpy = config.get("rpy")
        if not _is_numeric_vector(xyz, 3) or not _is_numeric_vector(rpy, 3):
            raise CalibrationError(f"{name} has invalid numeric rigid transform values")
        identity = xyz == [0.0, 0.0, 0.0] and rpy == [0.0, 0.0, 0.0]

    if identity and config.get("identity_verified") is not True:
        raise CalibrationError(
            f"{name} is identity but identity_verified is not true"
        )


@dataclass
class TimestampAccumulator:
    offsets: List[float] = field(default_factory=list)
    previous_header_stamp: Optional[float] = None
    timestamp_backward_count: int = 0

    def add(self, *, header_stamp: float, local_receive_time: float) -> None:
        if not math.isfinite(header_stamp) or not math.isfinite(local_receive_time):
            raise ValueError("timestamps must be finite")
        if (
            self.previous_header_stamp is not None
            and header_stamp < self.previous_header_stamp
        ):
            self.timestamp_backward_count += 1
        self.previous_header_stamp = header_stamp
        self.offsets.append(local_receive_time - header_stamp)

    def summary(self) -> Dict[str, float | int]:
        if not self.offsets:
            raise ValueError("no timestamp samples")
        return {
            "minimum_offset_s": min(self.offsets),
            "median_offset_s": median(self.offsets),
            "jitter_s": max(self.offsets) - min(self.offsets),
            "timestamp_backward_count": self.timestamp_backward_count,
            "sample_count": len(self.offsets),
        }


class CameraHealthAccumulator:
    """Accumulate RGB, aligned depth and CameraInfo without decoding pixels."""

    def __init__(
        self,
        *,
        minimum_samples: int,
        sync_slop_s: float,
        clock_threshold_s: float = 0.05,
    ) -> None:
        if minimum_samples <= 0 or sync_slop_s < 0.0 or clock_threshold_s <= 0.0:
            raise ValueError("invalid camera preflight thresholds")
        self.minimum_samples = int(minimum_samples)
        self.sync_slop_s = float(sync_slop_s)
        self.clock_threshold_s = float(clock_threshold_s)
        self.rgb_clock = TimestampAccumulator()
        self.depth_clock = TimestampAccumulator()
        self.info_clock = TimestampAccumulator()
        self.rgb_stamps: List[float] = []
        self.depth_stamps: List[float] = []
        self.info_stamps: List[float] = []
        self.rgb_dimensions = set()
        self.depth_dimensions = set()
        self.info_dimensions = set()
        self.rgb_metadata_healthy = True
        self.depth_metadata_healthy = True
        self.info_metadata_healthy = True

    @staticmethod
    def _valid_image(
        width: int,
        height: int,
        encoding: str,
        step: int,
        data_size: int,
        allowed_encodings: Sequence[str],
    ) -> bool:
        return bool(
            width > 0
            and height > 0
            and step > 0
            and data_size >= height * step
            and encoding in allowed_encodings
        )

    def add_rgb(
        self,
        *,
        stamp: float,
        local_receive_time: float,
        width: int,
        height: int,
        encoding: str,
        step: int,
        data_size: int,
    ) -> None:
        self.rgb_clock.add(header_stamp=stamp, local_receive_time=local_receive_time)
        self.rgb_stamps.append(float(stamp))
        self.rgb_dimensions.add((int(width), int(height)))
        self.rgb_metadata_healthy = self.rgb_metadata_healthy and self._valid_image(
            width, height, encoding, step, data_size, ("rgb8", "bgr8")
        )

    def add_depth(
        self,
        *,
        stamp: float,
        local_receive_time: float,
        width: int,
        height: int,
        encoding: str,
        step: int,
        data_size: int,
    ) -> None:
        self.depth_clock.add(header_stamp=stamp, local_receive_time=local_receive_time)
        self.depth_stamps.append(float(stamp))
        self.depth_dimensions.add((int(width), int(height)))
        self.depth_metadata_healthy = (
            self.depth_metadata_healthy
            and self._valid_image(
                width, height, encoding, step, data_size, ("16UC1", "32FC1")
            )
        )

    def add_camera_info(
        self,
        *,
        stamp: float,
        local_receive_time: float,
        width: int,
        height: int,
        frame_id: str,
        k: Sequence[float],
    ) -> None:
        self.info_clock.add(header_stamp=stamp, local_receive_time=local_receive_time)
        self.info_stamps.append(float(stamp))
        self.info_dimensions.add((int(width), int(height)))
        self.info_metadata_healthy = self.info_metadata_healthy and bool(
            width > 0
            and height > 0
            and frame_id.strip()
            and len(k) == 9
            and all(math.isfinite(float(value)) for value in k)
            and float(k[0]) > 0.0
            and float(k[4]) > 0.0
        )

    @staticmethod
    def _clock_summary(accumulator: TimestampAccumulator) -> Dict[str, object]:
        if not accumulator.offsets:
            return {"sample_count": 0, "timestamp_backward_count": 0}
        return accumulator.summary()

    def summary(self) -> Dict[str, object]:
        rgb_clock = self._clock_summary(self.rgb_clock)
        depth_clock = self._clock_summary(self.depth_clock)
        info_clock = self._clock_summary(self.info_clock)
        dimensions_aligned = bool(
            len(self.rgb_dimensions) == 1
            and self.rgb_dimensions == self.depth_dimensions
            and self.rgb_dimensions == self.info_dimensions
        )
        offset_summary = None
        sync_healthy = False
        try:
            offsets = nearest_stream_offsets(self.rgb_stamps, self.depth_stamps)
            absolute = [abs(value) for value in offsets]
            offset_summary = {
                "minimum": min(offsets),
                "median": median(offsets),
                "maximum": max(offsets),
                "maximum_absolute": max(absolute),
                "sample_count": len(offsets),
            }
            sync_healthy = max(absolute) <= self.sync_slop_s
        except ValueError:
            pass

        counts_healthy = all(
            count >= self.minimum_samples
            for count in (
                len(self.rgb_stamps),
                len(self.depth_stamps),
                len(self.info_stamps),
            )
        )
        clocks_healthy = all(
            clock.get("timestamp_backward_count", 0) == 0
            and abs(float(clock.get("minimum_offset_s", math.inf)))
            <= self.clock_threshold_s
            for clock in (rgb_clock, depth_clock, info_clock)
        )
        healthy = all(
            (
                counts_healthy,
                clocks_healthy,
                self.rgb_metadata_healthy,
                self.depth_metadata_healthy,
                self.info_metadata_healthy,
                dimensions_aligned,
                sync_healthy,
            )
        )
        return {
            "healthy": healthy,
            "rgb_count": len(self.rgb_stamps),
            "depth_count": len(self.depth_stamps),
            "camera_info_count": len(self.info_stamps),
            "rgb_clock": rgb_clock,
            "depth_clock": depth_clock,
            "camera_info_clock": info_clock,
            "rgb_metadata_healthy": self.rgb_metadata_healthy,
            "depth_metadata_healthy": self.depth_metadata_healthy,
            "camera_info_metadata_healthy": self.info_metadata_healthy,
            "dimensions_aligned": dimensions_aligned,
            "rgb_depth_offset_s": offset_summary,
        }


@dataclass
class ImuHealthAccumulator:
    gravity_min: float
    gravity_max: float
    stationary_gyro_max: float
    acceleration_abs_max: float
    gyro_abs_max: float
    jump_max: float
    sample_count: int = 0
    nonfinite_count: int = 0
    extreme_count: int = 0
    gravity_out_of_range_count: int = 0
    gyro_out_of_range_count: int = 0
    jump_count: int = 0
    previous_acceleration: Optional[tuple[float, float, float]] = None

    def add(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float) -> None:
        values = (ax, ay, az, gx, gy, gz)
        self.sample_count += 1
        if not all(math.isfinite(value) for value in values):
            self.nonfinite_count += 1
            return

        acceleration = (ax, ay, az)
        gyro = (gx, gy, gz)
        if any(abs(value) > self.acceleration_abs_max for value in acceleration) or any(
            abs(value) > self.gyro_abs_max for value in gyro
        ):
            self.extreme_count += 1

        acceleration_norm = math.sqrt(sum(value * value for value in acceleration))
        gyro_norm = math.sqrt(sum(value * value for value in gyro))
        if not self.gravity_min <= acceleration_norm <= self.gravity_max:
            self.gravity_out_of_range_count += 1
        if gyro_norm > self.stationary_gyro_max:
            self.gyro_out_of_range_count += 1

        if self.previous_acceleration is not None:
            jump = math.sqrt(
                sum(
                    (current - previous) ** 2
                    for current, previous in zip(
                        acceleration, self.previous_acceleration
                    )
                )
            )
            if jump > self.jump_max:
                self.jump_count += 1
        self.previous_acceleration = acceleration

    def summary(self) -> Dict[str, bool | int]:
        errors = (
            self.nonfinite_count
            + self.extreme_count
            + self.gravity_out_of_range_count
            + self.gyro_out_of_range_count
            + self.jump_count
        )
        return {
            "healthy": self.sample_count > 0 and errors == 0,
            "sample_count": self.sample_count,
            "nonfinite_count": self.nonfinite_count,
            "extreme_count": self.extreme_count,
            "gravity_out_of_range_count": self.gravity_out_of_range_count,
            "gyro_out_of_range_count": self.gyro_out_of_range_count,
            "jump_count": self.jump_count,
        }


@dataclass
class RawImuAccumulator:
    """Collect measurements without assigning unverified health thresholds."""

    sample_count: int = 0
    nonfinite_count: int = 0
    acceleration_norms: List[float] = field(default_factory=list)
    gyro_norms: List[float] = field(default_factory=list)
    maximum_acceleration_abs: float = 0.0
    maximum_gyro_abs: float = 0.0
    maximum_acceleration_jump: float = 0.0
    previous_acceleration: Optional[tuple[float, float, float]] = None

    def add(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float) -> None:
        self.sample_count += 1
        values = (ax, ay, az, gx, gy, gz)
        if not all(math.isfinite(value) for value in values):
            self.nonfinite_count += 1
            return
        acceleration = (ax, ay, az)
        gyro = (gx, gy, gz)
        self.acceleration_norms.append(
            math.sqrt(sum(value * value for value in acceleration))
        )
        self.gyro_norms.append(math.sqrt(sum(value * value for value in gyro)))
        self.maximum_acceleration_abs = max(
            self.maximum_acceleration_abs, *(abs(value) for value in acceleration)
        )
        self.maximum_gyro_abs = max(
            self.maximum_gyro_abs, *(abs(value) for value in gyro)
        )
        if self.previous_acceleration is not None:
            jump = math.sqrt(
                sum(
                    (current - previous) ** 2
                    for current, previous in zip(
                        acceleration, self.previous_acceleration
                    )
                )
            )
            self.maximum_acceleration_jump = max(
                self.maximum_acceleration_jump, jump
            )
        self.previous_acceleration = acceleration

    @staticmethod
    def _distribution(values: Sequence[float]) -> Dict[str, float]:
        if not values:
            return {"minimum": math.nan, "median": math.nan, "maximum": math.nan}
        return {
            "minimum": min(values),
            "median": median(values),
            "maximum": max(values),
        }

    def summary(self) -> Dict[str, object]:
        return {
            "sample_count": self.sample_count,
            "nonfinite_count": self.nonfinite_count,
            "acceleration_norm_mps2": self._distribution(self.acceleration_norms),
            "gyro_norm_radps": self._distribution(self.gyro_norms),
            "maximum_acceleration_abs_mps2": self.maximum_acceleration_abs,
            "maximum_gyro_abs_radps": self.maximum_gyro_abs,
            "maximum_acceleration_jump_mps2": self.maximum_acceleration_jump,
        }
