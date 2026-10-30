#!/usr/bin/env python3
"""Generate arm torque plots and summaries from completed recorder CSV files."""

from __future__ import annotations

import argparse
import csv
import errno
import fcntl
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Dict, Iterable, List, Optional, Tuple


OWNED_CSV = re.compile(r"^arm-torque-\d{8}-\d{6}-\d{6}-\d+\.csv$")
OWNED_PART = re.compile(r"^arm-torque-\d{8}-\d{6}-\d{6}-\d+\.csv\.part$")
GENERATED_BY = "arm_torque_report.py"
REPORT_VERSION = 2
MOTORS = range(1, 5)
# Selected for this arm from historical calibration scales and manufacturer options.
# These are assumptions, not fresh register readings; revisit after a motor change.
INFERRED_GEAR_RATIOS = {1: 160, 2: 160, 3: 160, 4: 101}
RATIO_BASIS = "Inferred from historical calibration; not verified by a fresh register read."
MAX_PLOT_POINTS_PER_MOTOR = 6000
_MATPLOTLIB_CONFIG: Optional[tempfile.TemporaryDirectory] = None


def bootstrap_matplotlib():
    global _MATPLOTLIB_CONFIG
    if "MPLCONFIGDIR" not in os.environ:
        if _MATPLOTLIB_CONFIG is None:
            _MATPLOTLIB_CONFIG = tempfile.TemporaryDirectory(prefix="arm-torque-matplotlib-")
        os.environ["MPLCONFIGDIR"] = _MATPLOTLIB_CONFIG.name
    try:
        import matplotlib
    except Exception:
        dist_packages = "/usr/lib/python3/dist-packages"
        if dist_packages not in sys.path:
            sys.path.insert(0, dist_packages)
        for name in list(sys.modules):
            if name == "matplotlib" or name.startswith("matplotlib."):
                del sys.modules[name]
            if name == "numpy" or name.startswith("numpy."):
                del sys.modules[name]
        import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def parse_float(value: Optional[str]) -> Optional[float]:
    if value is None:
        return None
    value = value.strip()
    if not value:
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    if math.isfinite(number):
        return number
    return None


def calculated_nm(row: Dict[str, str], motor: int) -> Optional[float]:
    permille = parse_float(row.get(f"motor{motor}_permille"))
    rated_mnm = parse_float(row.get(f"motor{motor}_rated_mnm"))
    if permille is not None and rated_mnm is not None and rated_mnm > 0.0:
        return permille * rated_mnm / 1_000_000.0
    return None


def choose_motor_units(path: Path) -> Dict[int, str]:
    """Prefer Nm unless a motor has raw permille that cannot be converted safely."""
    units = {motor: "Nm" for motor in MOTORS}
    for _, row in iter_rows(path):
        for motor in MOTORS:
            permille = parse_float(row.get(f"motor{motor}_permille"))
            rated_mnm = parse_float(row.get(f"motor{motor}_rated_mnm"))
            if permille is not None and (rated_mnm is None or rated_mnm <= 0.0):
                units[motor] = "permille"
    return units


def motor_value(row: Dict[str, str], motor: int, unit: str) -> Optional[float]:
    if unit == "permille":
        return parse_float(row.get(f"motor{motor}_permille"))
    explicit_nm = parse_float(row.get(f"motor{motor}_nm"))
    if explicit_nm is not None:
        return explicit_nm
    return calculated_nm(row, motor)


def iter_rows(path: Path) -> Iterable[Tuple[float, Dict[str, str]]]:
    with path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames or "elapsed_s" not in reader.fieldnames:
            return
        for row in reader:
            elapsed = parse_float(row.get("elapsed_s"))
            if elapsed is None:
                continue
            yield elapsed, row


class Summary:
    def __init__(self, units: Dict[int, str]) -> None:
        self.rows = 0
        self.start_elapsed_s: Optional[float] = None
        self.end_elapsed_s: Optional[float] = None
        self.start_wall_time_s: Optional[float] = None
        self.end_wall_time_s: Optional[float] = None
        self.dropped_samples = 0
        self.motors = {
            str(motor): {
                "unit": units[motor],
                "valid_samples": 0,
                "peak_abs": None,
                "peak_positive": None,
                "peak_negative": None,
            }
            for motor in MOTORS
        }

    def update_time(self, elapsed: float, row: Dict[str, str]) -> None:
        wall_time = parse_float(row.get("wall_time_s"))
        dropped = parse_float(row.get("dropped_samples"))
        self.rows += 1
        if dropped is not None and dropped > 0:
            self.dropped_samples += int(dropped)
        if self.start_elapsed_s is None:
            self.start_elapsed_s = elapsed
            self.start_wall_time_s = wall_time
        self.end_elapsed_s = elapsed
        if wall_time is not None:
            self.end_wall_time_s = wall_time

    def update_motor(self, motor: int, value: Optional[float]) -> None:
        if value is None:
            return
        entry = self.motors[str(motor)]
        entry["valid_samples"] += 1
        entry["peak_abs"] = max_abs(entry["peak_abs"], value)
        entry["peak_positive"] = max_value(entry["peak_positive"], value)
        entry["peak_negative"] = min_value(entry["peak_negative"], value)

    def as_dict(self, source: Path, image: Optional[Path]) -> Dict[str, object]:
        duration = None
        if self.start_elapsed_s is not None and self.end_elapsed_s is not None:
            duration = self.end_elapsed_s - self.start_elapsed_s
        return {
            "generated_by": GENERATED_BY,
            "report_version": REPORT_VERSION,
            "plot_quantity": "estimated_output_torque",
            "output_estimate": {
                "gear_ratio_basis": RATIO_BASIS,
                "efficiency_assumed": 1.0,
                "transmission_losses_included": False,
                "motors": {
                    str(motor): self.output_estimate(motor) for motor in MOTORS
                },
            },
            "source_csv": source.name,
            "image_png": image.name if image else None,
            "rows": self.rows,
            "start_elapsed_s": self.start_elapsed_s,
            "end_elapsed_s": self.end_elapsed_s,
            "duration_s": duration,
            "start_wall_time_s": self.start_wall_time_s,
            "end_wall_time_s": self.end_wall_time_s,
            "dropped_samples": self.dropped_samples,
            "motors": self.motors,
            "note": (
                "CSV and motors retain drive motor-side feedback. Charts show estimated "
                "output torque = motor torque * inferred gear ratio, without transmission "
                "losses; this is not measured output torque. Unconvertible axes show raw "
                "motor feedback explicitly."
            ),
        }

    def output_estimate(self, motor: int) -> Dict[str, object]:
        source = self.motors[str(motor)]
        available = source["unit"] == "Nm" and source["valid_samples"] > 0
        ratio = INFERRED_GEAR_RATIOS[motor]
        return {
            "unit": "Nm",
            "gear_ratio": ratio,
            "available": available,
            "valid_samples": source["valid_samples"] if available else 0,
            **{
                key: source[key] * ratio if available and source[key] is not None else None
                for key in ("peak_abs", "peak_positive", "peak_negative")
            },
        }


def max_value(current: Optional[float], candidate: float) -> float:
    if current is None or candidate > current:
        return candidate
    return current


def min_value(current: Optional[float], candidate: float) -> float:
    if current is None or candidate < current:
        return candidate
    return current


def max_abs(current: Optional[float], candidate: float) -> float:
    if current is None or abs(candidate) > abs(current):
        return candidate
    return current


def summarize(path: Path, units: Optional[Dict[int, str]] = None) -> Summary:
    units = units or choose_motor_units(path)
    summary = Summary(units)
    for elapsed, row in iter_rows(path):
        summary.update_time(elapsed, row)
        for motor in MOTORS:
            value = motor_value(row, motor, units[motor])
            summary.update_motor(motor, value)
    return summary


class Bucket:
    def __init__(self) -> None:
        self.has_missing = False
        self.min_t: Optional[float] = None
        self.min_v: Optional[float] = None
        self.max_t: Optional[float] = None
        self.max_v: Optional[float] = None
        self.first_t: Optional[float] = None

    def add(self, elapsed: float, value: Optional[float]) -> None:
        if self.first_t is None:
            self.first_t = elapsed
        if value is None:
            self.has_missing = True
            return
        if self.min_v is None or value < self.min_v:
            self.min_v = value
            self.min_t = elapsed
        if self.max_v is None or value > self.max_v:
            self.max_v = value
            self.max_t = elapsed

    def points(self) -> List[Tuple[float, float]]:
        if self.min_v is None or self.max_v is None:
            if self.first_t is not None and self.has_missing:
                return [(self.first_t, math.nan)]
            return []
        points = [(self.min_t or 0.0, self.min_v)]
        if self.max_t != self.min_t or self.max_v != self.min_v:
            points.append((self.max_t or 0.0, self.max_v))
        points.sort(key=lambda item: item[0])
        if self.has_missing:
            separated: List[Tuple[float, float]] = []
            for point in points:
                separated.append((point[0], math.nan))
                separated.append(point)
                separated.append((point[0], math.nan))
            return separated
        return points


def collect_plot_series(
    path: Path,
    row_count: int,
    units: Dict[int, str],
) -> Dict[int, Tuple[List[float], List[float], str]]:
    bucket_count = max(1, min(row_count, MAX_PLOT_POINTS_PER_MOTOR // 2))
    buckets = {motor: [Bucket() for _ in range(bucket_count)] for motor in MOTORS}
    for index, (elapsed, row) in enumerate(iter_rows(path)):
        bucket_index = min(bucket_count - 1, int(index * bucket_count / max(1, row_count)))
        for motor in MOTORS:
            value = motor_value(row, motor, units[motor])
            buckets[motor][bucket_index].add(elapsed, value)

    series: Dict[int, Tuple[List[float], List[float], str]] = {}
    for motor in MOTORS:
        xs: List[float] = []
        ys: List[float] = []
        for bucket in buckets[motor]:
            for elapsed, value in bucket.points():
                xs.append(elapsed)
                ys.append(value)
        series[motor] = (xs, ys, units[motor])
    return series


def generate_report(path: Path) -> Optional[Path]:
    units = choose_motor_units(path)
    summary = summarize(path, units)
    if summary.rows == 0:
        write_summary(path, summary, None)
        return None

    image = path.with_suffix(".png")
    assert_output_target(image)
    plt = bootstrap_matplotlib()
    series = collect_plot_series(path, summary.rows, units)
    figure, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    for index, motor in enumerate(MOTORS):
        xs, ys, unit = series[motor]
        axis = axes[index]
        estimate = summary.output_estimate(motor)
        if unit == "Nm":
            ys = [value * INFERRED_GEAR_RATIOS[motor] for value in ys]
            axis.set_ylabel(f"M{motor} output est.\n(N·m)")
            peak = estimate["peak_abs"]
            peak_label = f"{abs(peak):.4g} N·m" if peak is not None else "unavailable"
            annotation = f"Assumed ratio {INFERRED_GEAR_RATIOS[motor]}:1 | |peak| {peak_label}"
        else:
            axis.set_ylabel(f"M{motor} motor\n(permille)")
            annotation = "Output estimate unavailable: rated torque unknown; raw motor feedback"
        axis.plot(xs, ys, linewidth=0.9)
        axis.text(
            0.01, 0.96, annotation, transform=axis.transAxes, va="top", fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"},
        )
        axis.margins(y=0.2)
        axis.grid(True, alpha=0.3)
        axis.axhline(0.0, color="black", linewidth=0.5, alpha=0.4)
    axes[-1].set_xlabel("Elapsed time (s)")
    figure.suptitle("Estimated joint output torque — transmission losses excluded", fontsize=14)
    figure.text(0.5, 0.94, path.stem, ha="center", fontsize=9)
    figure.text(
        0.5,
        0.035,
        "Ratios M1–M4: 160 / 160 / 160 / 101 (inferred from historical calibration, not freshly verified).\n"
        "Estimate = motor torque × ratio; efficiency assumed 100%. Not measured output torque. CSV stays motor-side.",
        ha="center",
        fontsize=8,
    )
    figure.tight_layout(rect=(0.04, 0.08, 1.0, 0.93))
    temporary = unique_temp_path(image)
    try:
        figure.savefig(temporary, dpi=130, format="png", metadata={"Software": GENERATED_BY})
        replace_output(temporary, image)
    finally:
        plt.close(figure)
        if temporary.exists():
            temporary.unlink()
    write_summary(path, summary, image)
    return image


def assert_output_target(path: Path) -> None:
    if not path.exists():
        return
    if path.is_symlink() or not path.is_file():
        raise FileExistsError(f"refusing to overwrite unknown output: {path.name}")
    if path.suffix == ".png" and generated_png_for(path.with_suffix(".json")) != path.name:
        raise FileExistsError(f"refusing to overwrite unknown output: {path.name}")
    if path.suffix == ".json" and not is_generated_json(path):
        raise FileExistsError(f"refusing to overwrite unknown output: {path.name}")


def unique_temp_path(target: Path) -> Path:
    descriptor, name = tempfile.mkstemp(
        prefix=f".{target.name}.",
        suffix=".tmp",
        dir=str(target.parent),
    )
    # The control service runs as root; reports must remain readable by its operator.
    os.fchmod(descriptor, 0o644)
    os.close(descriptor)
    return Path(name)


def replace_output(temporary: Path, target: Path) -> None:
    if target.exists() and target.is_symlink():
        raise FileExistsError(f"refusing to overwrite symlink: {target.name}")
    temporary.replace(target)


def write_summary(path: Path, summary: Summary, image: Optional[Path]) -> Path:
    output = path.with_suffix(".json")
    assert_output_target(output)
    temporary = unique_temp_path(output)
    try:
        temporary.write_text(json.dumps(summary.as_dict(path, image), indent=2, sort_keys=True) + "\n")
        replace_output(temporary, output)
    finally:
        if temporary.exists():
            temporary.unlink()
    return output


def owned_completed_csvs(directory: Path) -> List[Path]:
    files: List[Path] = []
    with os.scandir(directory) as entries:
        for entry in entries:
            if not OWNED_CSV.match(entry.name):
                continue
            if entry.is_file(follow_symlinks=False):
                files.append(directory / entry.name)
    files.sort(key=lambda item: item.name, reverse=True)
    return files


def recover_interrupted_csvs(directory: Path) -> List[Path]:
    recovered: List[Path] = []
    with os.scandir(directory) as entries:
        candidates = [
            directory / entry.name
            for entry in entries
            if OWNED_PART.match(entry.name) and entry.is_file(follow_symlinks=False)
        ]
    for part_path in sorted(candidates):
        target = part_path.with_suffix("")
        if target.exists():
            continue
        fd: Optional[int] = None
        try:
            fd = os.open(part_path, os.O_RDONLY | os.O_NOFOLLOW)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            opened = os.fstat(fd)
            current = os.lstat(part_path)
            if opened.st_ino != current.st_ino or opened.st_dev != current.st_dev:
                continue
            os.link(part_path, target, follow_symlinks=False)
            linked = os.stat(target, follow_symlinks=False)
            if opened.st_ino != linked.st_ino or opened.st_dev != linked.st_dev:
                os.unlink(target)
                continue
            os.unlink(part_path)
            recovered.append(target)
        except BlockingIOError:
            continue
        except OSError as error:
            if error.errno not in (errno.EEXIST, errno.ELOOP, errno.ENOENT):
                raise
        finally:
            if fd is not None:
                os.close(fd)
    return recovered


def is_generated_json(path: Path) -> bool:
    if path.is_symlink():
        return False
    try:
        data = json.loads(path.read_text())
    except Exception:
        return False
    return data.get("generated_by") == GENERATED_BY


def generated_png_for(json_path: Path) -> Optional[str]:
    if not is_generated_json(json_path):
        return None
    try:
        data = json.loads(json_path.read_text())
    except Exception:
        return None
    image = data.get("image_png")
    if isinstance(image, str):
        return image
    return None


def generated_report_complete(csv_path: Path) -> bool:
    png_path = csv_path.with_suffix(".png")
    json_path = csv_path.with_suffix(".json")
    complete = (
        png_path.is_file()
        and not png_path.is_symlink()
        and json_path.is_file()
        and not json_path.is_symlink()
        and generated_png_for(json_path) == png_path.name
    )
    if not complete:
        return False
    try:
        data = json.loads(json_path.read_text())
        estimates = data["output_estimate"]
        return (
            data.get("report_version") == REPORT_VERSION
            and data.get("plot_quantity") == "estimated_output_torque"
            and estimates["efficiency_assumed"] == 1.0
            and estimates["gear_ratio_basis"] == RATIO_BASIS
            and all(
                estimates["motors"][str(motor)]["gear_ratio"] == INFERRED_GEAR_RATIOS[motor]
                for motor in MOTORS
            )
        )
    except (OSError, ValueError, KeyError, TypeError):
        return False


def cleanup_old_reports(directory: Path, keep: int) -> List[Path]:
    if keep < 0:
        raise ValueError("--keep must be >= 0")
    removed: List[Path] = []
    for csv_path in owned_completed_csvs(directory)[keep:]:
        json_path = csv_path.with_suffix(".json")
        png_name = (
            generated_png_for(json_path)
            if json_path.is_file() and not json_path.is_symlink()
            else None
        )
        png_path = csv_path.with_suffix(".png")
        if png_name == png_path.name and png_path.is_file() and not png_path.is_symlink():
            png_path.unlink()
            removed.append(png_path)
        if json_path.is_file() and not json_path.is_symlink() and is_generated_json(json_path):
            json_path.unlink()
            removed.append(json_path)
        csv_path.unlink()
        removed.append(csv_path)
    return removed


def generate_missing(directory: Path) -> Tuple[List[Path], List[Dict[str, str]]]:
    return generate_missing_for(owned_completed_csvs(directory))


def generate_missing_for(csv_paths: List[Path]) -> Tuple[List[Path], List[Dict[str, str]]]:
    generated: List[Path] = []
    failed: List[Dict[str, str]] = []
    for csv_path in reversed(csv_paths):
        if generated_report_complete(csv_path):
            continue
        try:
            output = generate_report(csv_path)
        except Exception as error:
            failed.append({"csv": csv_path.name, "error": str(error)})
            continue
        if output is not None:
            generated.append(output)
    return generated, failed


def run(directory: Path, keep: int) -> Dict[str, object]:
    directory.mkdir(parents=True, exist_ok=True)
    recovered = recover_interrupted_csvs(directory)
    removed = cleanup_old_reports(directory, keep)
    generated, failed = generate_missing_for(owned_completed_csvs(directory)[:keep])
    return {
        "directory": str(directory),
        "recovered": [path.name for path in recovered],
        "generated": [path.name for path in generated],
        "failed": failed,
        "removed": [path.name for path in removed],
        "kept_csv": [path.name for path in owned_completed_csvs(directory)[:keep]],
    }


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", required=True, type=Path)
    parser.add_argument("--keep", default=3, type=int)
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    result = run(args.directory, args.keep)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
