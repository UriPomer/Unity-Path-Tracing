"""Analyze production-render artifacts; independent seeds are the statistical units.

Usage: python Tests/Compare-ReSTIRRenders.py <capture-directory>
Requires numpy. Writes analysis.json and analysis.md next to manifest.json.
"""
import json
import sys
from pathlib import Path

import numpy as np


def analyze(directory):
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    failures, results, images = [], {}, {}
    if manifest.get("failure"):
        failures.append(manifest["failure"])
    width, height = manifest["width"], manifest["height"]
    expected = manifest.get("analyticExpected", -1)
    for mode in ("pt", "di", "gi", "di_gi", "restart"):
        captures = sorted((c for c in manifest["captures"] if c["mode"] == mode), key=lambda c: c["seed"])
        if len(captures) != manifest["repeats"]:
            failures.append(f"{mode}: incomplete captures")
            continue
        if any(c["samples"] != manifest["frames"] or c["nonfinitePixels"] for c in captures):
            failures.append(f"{mode}: invalid sample count or nonfinite output")
        rgb = np.stack([np.fromfile(directory / c["file"], dtype="<f4").reshape(height, width, 4)[..., :3]
                        for c in captures]).astype(np.float64)
        if not np.isfinite(rgb).all():
            failures.append(f"{mode}: nonfinite raw pixels")
        images[mode] = rgb
        luminance = rgb @ np.array([0.2126, 0.7152, 0.0722])
        # Global mean plus 4x4 tiles. Pixels inside one run are correlated.
        regions = [luminance.mean(axis=(1, 2))]
        for row in range(4):
            for col in range(4):
                tile = luminance[:, row*height//4:(row+1)*height//4, col*width//4:(col+1)*width//4]
                regions.append(tile.mean(axis=(1, 2)))
        samples = np.stack(regions, axis=1)
        reference = np.full_like(samples, expected) if expected >= 0 else results.get("pt", {}).get("samples", samples)
        difference = samples - reference
        delta = difference.mean(axis=0)
        sem = difference.std(axis=0, ddof=1) / np.sqrt(len(captures))
        # A conservative discrepancy screen, not an unbiasedness proof or a
        # convergence guarantee for heavy-tailed Monte Carlo estimators.
        tolerance = 5 * sem + 1e-4 * np.maximum(1, np.abs(reference.mean(axis=0)))
        bad_regions = np.flatnonzero(np.abs(delta) > tolerance).tolist()
        if bad_regions:
            failures.append(f"{mode}: mean discrepancy in regions {bad_regions} (0=whole image)")
        results[mode] = dict(samples=samples, mean=float(samples[:, 0].mean()),
                             seed_std=float(samples[:, 0].std(ddof=1)), delta=float(delta[0]),
                             sem=float(sem[0]), bad_regions=bad_regions,
                             peak=float(luminance.max()))
    if "restart" in images and "pt" in images:
        restart_error = float(np.max(np.abs(images["restart"] - images["pt"])))
        if restart_error > 1e-5:
            failures.append(f"renderer restart differs from PT with the same seeds: max={restart_error}")
    else:
        restart_error = None
    for result in results.values():
        del result["samples"]
    report = dict(scene=manifest["scene"], expected=expected, frames=manifest["frames"],
                  repeats=manifest["repeats"], results=results, restart_max_error=restart_error,
                  failures=failures, status="FAIL" if failures else "PASS")
    (directory / "analysis.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    lines = [f"# {report['status']} — {manifest['scene']}", "",
             f"Unity {manifest['unity']} / {manifest['device']} / {width}×{height} / {manifest['frames']} spp × {manifest['repeats']} seeds", "",
             "|模式|均值|跨种子标准差|与参考之差|差值标准误|异常区域|",
             "|---|---:|---:|---:|---:|---|"]
    for mode, result in results.items():
        lines.append(f"|{mode}|{result['mean']:.8f}|{result['seed_std']:.8f}|{result['delta']:.8f}|{result['sem']:.8f}|{result['bad_regions']}|")
    lines += ["", f"重启前后最大绝对像素差：{restart_error}", "",
              "参考：解析场景用已知解；实际场景用相同种子的 PT。检查全图及 4×4 区域均值，阈值为 5 倍跨种子标准误加 1e-4 数值余量。",
              "统计相容只支持本场景、本样本量；相关复用和重尾噪声不能用像素数冒充独立样本数。", ""]
    lines += [f"- {failure}" for failure in failures]
    lines += ["", "![PT](pt_0.png)", "", "![DI+GI](di_gi_0.png)", ""]
    (directory / "analysis.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"{report['status']} {directory}: {len(failures)} discrepancies")
    return not failures


def compare_performance(directory, baseline):
    """Paired full-render A/B, including numerical differences and warm timings."""
    current = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    original = json.loads((baseline / "manifest.json").read_text(encoding="utf-8"))
    failures, results = [], {}
    for key in ("scene", "unity", "device", "width", "height", "frames", "repeats",
                "warmupFrames", "batchFrames", "materialSnapshot"):
        if current.get(key) != original.get(key):
            failures.append(f"Different benchmark input: {key}")
    if current.get("failure") or original.get("failure"):
        failures.append("Incomplete or failed renderer run")
    if failures:
        raise ValueError("; ".join(failures))
    h, w = current["height"], current["width"]
    for mode in ("pt", "di", "gi", "di_gi", "restart"):
        before = {c["seed"]: c for c in original["captures"] if c["mode"] == mode}
        after = {c["seed"]: c for c in current["captures"] if c["mode"] == mode}
        if before.keys() != after.keys() or len(after) != current["repeats"]:
            failures.append(f"{mode}: incomplete seed pairs")
            continue
        deltas, times_a, times_b, squared, count, max_error = [], [], [], 0., 0, 0.
        for seed, b in after.items():
            a = before[seed]
            if any(c["samples"] != current["frames"] or c["nonfinitePixels"] for c in (a, b)):
                failures.append(f"{mode}: invalid samples or output")
            rgb_a = np.fromfile(baseline / a["file"], dtype="<f4").reshape(h, w, 4)[..., :3].astype(float)
            rgb_b = np.fromfile(directory / b["file"], dtype="<f4").reshape(h, w, 4)[..., :3].astype(float)
            difference = rgb_b - rgb_a
            if not np.isfinite(difference).all():
                failures.append(f"{mode}: nonfinite raw output")
            squared += float(np.square(difference).sum())
            count += difference.size
            max_error = max(max_error, float(np.abs(difference).max()))
            lum = difference @ np.array([.2126, .7152, .0722])
            deltas.append([lum.mean()] + [lum[y*h//4:(y+1)*h//4, x*w//4:(x+1)*w//4].mean()
                                         for y in range(4) for x in range(4)])
            times_a.append(a["wallSeconds"] * 1000 / a["samples"])
            times_b.append(b["wallSeconds"] * 1000 / b["samples"])
        d = np.array(deltas)
        sem = d.std(axis=0, ddof=1) / np.sqrt(len(d))
        bad = np.flatnonzero(np.abs(d.mean(axis=0)) > 5 * sem + 1e-4).tolist()
        if bad:
            failures.append(f"{mode}: paired regional discrepancy {bad}")
        results[mode] = dict(baseline_ms=float(np.median(times_a)), current_ms=float(np.median(times_b)),
            reduction_percent=float(100 * (1 - np.median(times_b) / np.median(times_a))),
            baseline_samples_ms=times_a, current_samples_ms=times_b,
            max_abs_error=max_error, rgb_rmse=float(np.sqrt(squared/count)),
            mean_delta=float(d[:, 0].mean()), mean_delta_sem=float(sem[0]), bad_regions=bad)
    report = dict(baseline=str(baseline.resolve()), current=str(directory.resolve()),
                  status="FAIL" if failures else "PASS", results=results, failures=failures)
    (directory / "performance.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    for mode, v in results.items():
        print(f"{mode}: {v['baseline_ms']:.3f} -> {v['current_ms']:.3f} ms "
              f"({v['reduction_percent']:+.1f}% reduction), RGB RMSE {v['rgb_rmse']:.6g}")
    print(report["status"], failures)
    return not failures


if __name__ == "__main__":
    ok = compare_performance(Path(sys.argv[1]), Path(sys.argv[3])) if len(sys.argv) == 4 and sys.argv[2] == "--baseline" else analyze(Path(sys.argv[1]))
    sys.exit(0 if ok else 1)
