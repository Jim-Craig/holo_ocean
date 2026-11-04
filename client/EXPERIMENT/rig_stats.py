"""Compare sensor-rig pilot runs: how much of each camera image can the sonar actually help with?

usage:  python rig_stats.py run_A run_B run_C [--thr 0.02] [--tol 2] [--stride 2] [--max-samples 150]

For every sample the script measures, over the camera image:
  valid     pixels with a usable depth (0.3-50 m; the rest is open water)
  in beam   pixels whose surface lies inside the sonar's beam (angles) and range limits
  echo      of the in-beam pixels, the share whose sonar bin really has an echo
  helpful   in beam AND echo, as a share of ALL pixels  (the headline number)
It assumes the sonar sits exactly where the camera does (identity extrinsics) and that sonar column 0 is the
rightmost direction, as found with check_extrinsics.py.
"""
import argparse, glob, json, os
import numpy as np
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("runs", nargs="+")
ap.add_argument("--thr", type=float, default=0.02, help="sonar value that counts as an echo (use the clean, noise-free sonar)")
ap.add_argument("--tol", type=int, default=2, help="neighbourhood in sonar bins when looking for an echo")
ap.add_argument("--stride", type=int, default=2, help="use every n-th pixel (speed)")
ap.add_argument("--max-samples", type=int, default=150)
args = ap.parse_args()


def dilate(mask, r):
    if r <= 0:
        return mask
    h, w = mask.shape
    p = np.pad(mask, r)
    out = np.zeros_like(mask)
    for dy in range(2 * r + 1):
        for dx in range(2 * r + 1):
            out |= p[dy:dy + h, dx:dx + w]
    return out


def analyse(run):
    meta = json.load(open(f"{run}/meta.json"))
    so = meta["sonar"]
    R_MIN, R_MAX, AZ, EL = so["r_min"], so["r_max"], so["azimuth_deg"], so["elevation_deg"]
    n_r, n_az = so["bins"]
    off = meta.get("sonar_minus_camera", {})
    if any(off.get("translation_m", [0])) or any(off.get("rotation_deg", [0])):
        print(f"  WARNING {run}: the sonar offset in meta.json is not zero; these numbers assume identity.")

    files = sorted(glob.glob(f"{run}/s*.npz"))
    if not files:
        raise SystemExit(f"no samples in {run}")
    if len(files) > args.max_samples:
        keep = np.linspace(0, len(files) - 1, args.max_samples).round().astype(int)
        files = [files[i] for i in keep]

    H, W = np.load(files[0])["depth"].shape
    f = (W / 2) / np.tan(np.radians(meta.get("fov_deg", 90) / 2))
    s = args.stride
    v, u = np.mgrid[0:H:s, 0:W:s]
    xn, yn = (u - W / 2) / f, (v - H / 2) / f
    az = np.degrees(np.arctan2(xn, 1.0))                        # + = right of the optical axis
    el = np.degrees(np.arctan2(yn, np.hypot(xn, 1.0)))          # + = below the optical axis
    ray = np.sqrt(1 + xn**2 + yn**2)                            # range = planar depth * ray
    in_angle = (np.abs(az) <= AZ / 2) & (np.abs(el) <= EL / 2)
    col = np.rint((-az + AZ / 2) / AZ * (n_az - 1)).astype(int) # column 0 = rightmost

    per, ranges = [], []
    for fn in files:
        d = np.load(fn)
        dep = d["depth"][::s, ::s].astype(float)
        sonar = d["sonar"]
        valid = (dep > 0.3) & (dep < 50)
        r = dep * ray
        inb = valid & in_angle & (r >= R_MIN) & (r <= R_MAX)
        row = np.rint((r - R_MIN) / (R_MAX - R_MIN) * (n_r - 1)).astype(int)
        ok = inb & (row >= 0) & (row < n_r) & (col >= 0) & (col < n_az)
        echo_map = dilate(sonar > args.thr, args.tol)
        sup = np.zeros(inb.shape, bool)
        sup[ok] = echo_map[row[ok], col[ok]]
        per.append([valid.mean(), inb.mean(), sup.sum() / max(inb.sum(), 1), sup.mean(), (sonar > args.thr).mean()])
        ranges.append(r[sup])

    per = np.array(per)
    rng = np.concatenate(ranges) if ranges else np.zeros(0)
    bands = [R_MIN, 5, 10, 15, R_MAX]
    band_share = [((rng >= lo) & (rng < hi)).mean() if len(rng) else 0.0 for lo, hi in zip(bands[:-1], bands[1:])]
    name = meta.get("note") or os.path.basename(run.rstrip("/\\"))
    return {"run": run, "label": f"{name} (tilt {meta['tilt_deg']:g}, beam {EL:g})", "n": len(files),
            "tilt": meta["tilt_deg"], "beam": EL, "beam_angular": in_angle.mean(),
            "mean": per.mean(0), "std": per.std(0), "pct": np.percentile(rng, [10, 50, 90]) if len(rng) else np.zeros(3),
            "bands": band_share, "rng": rng, "edges": bands, "R": (R_MIN, R_MAX)}


res = [analyse(r) for r in args.runs]

print(f"\nthreshold for 'echo' = {args.thr}, neighbourhood = +/-{args.tol} bins, every {args.stride}th pixel\n")
hdr = f"{'run':<44}{'n':>4}{'tilt':>6}{'beam':>6}{'beam%':>7}{'valid%':>8}{'inbeam%':>9}{'echo%':>7}{'helpful%':>10}{'echoBins%':>10}"
print(hdr)
print("-" * len(hdr))
for r in res:
    m = r["mean"] * 100
    print(f"{r['label'][:43]:<44}{r['n']:>4}{r['tilt']:>6g}{r['beam']:>6g}{r['beam_angular']*100:>7.1f}"
          f"{m[0]:>8.1f}{m[1]:>9.1f}{m[2]:>7.1f}{m[3]:>10.1f}{m[4]:>10.1f}")
print("\n beam%     = share of the image whose viewing direction is inside the sonar beam (geometry only)")
print(" valid%    = pixels with usable depth      inbeam% = valid pixels inside the beam and range")
print(" echo%     = of those, share with a real sonar echo      helpful% = inbeam and echo, of ALL pixels")
print(" echoBins% = share of all sonar bins above the threshold (if this is near 100, raise --thr)\n")
for r in res:
    p, b = r["pct"], r["bands"]
    print(f"{r['label'][:43]:<44} helpful-pixel ranges 10/50/90th pct: {p[0]:.1f} / {p[1]:.1f} / {p[2]:.1f} m   "
          f"share by band {r['edges'][0]:g}-5 / 5-10 / 10-15 / 15-{r['edges'][4]:g} m: "
          + " / ".join(f"{x*100:.0f}%" for x in b))
print("\nDifferences of a few percent between pilots are within the noise of manual flying; look for big gaps.")

fig, ax = plt.subplots(1, 2, figsize=(15, 5))
names = ["valid depth", "in beam", "echo (of in-beam)", "helpful (of all)"]
x = np.arange(4)
wd = 0.8 / len(res)
for i, r in enumerate(res):
    m = r["mean"][:4] * 100
    sd = r["std"][:4] * 100
    ax[0].bar(x + i * wd - 0.4 + wd / 2, m, wd, yerr=sd, capsize=2, label=r["label"])
ax[0].set_xticks(x); ax[0].set_xticklabels(names)
ax[0].set_ylabel("% of pixels (mean, bar = spread between samples)"); ax[0].set_ylim(0, 105); ax[0].legend(fontsize=8)
ax[0].set_title("What share of the image can the sonar help with?")
for r in res:
    if len(r["rng"]):
        h, e = np.histogram(r["rng"], bins=40, range=r["R"])
        ax[1].plot(0.5 * (e[1:] + e[:-1]), h / max(h.sum(), 1), label=r["label"])
ax[1].set_xlabel("range (m)"); ax[1].set_ylabel("share of helpful pixels")
ax[1].set_title("How far away are the helpful pixels?"); ax[1].legend(fontsize=8)
plt.tight_layout()
fig.savefig("rig_stats.png", dpi=110)
print("saved rig_stats.png")
plt.show()