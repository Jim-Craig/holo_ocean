"""Inspect one recorded sample from experiment_2.py.

usage:  python inspect_run.py run_YYYYMMDD_HHMMSS [sample_index]
"""
import sys, json, glob
import numpy as np
import matplotlib.pyplot as plt

run = sys.argv[1]
idx = int(sys.argv[2]) if len(sys.argv) > 2 else 0
meta = json.load(open(f"{run}/meta.json"))
files = sorted(glob.glob(f"{run}/s*.npz"))
d = np.load(files[idx])
rgb, depth, sonar, pose = d["rgb"], d["depth"], d["sonar"], d["pose"]

so = meta["sonar"]
R_MIN, R_MAX, AZ, ELEV = so["r_min"], so["r_max"], so["azimuth_deg"], so["elevation_deg"]
n_r, n_az = sonar.shape
H, W = depth.shape
f = (W / 2) / np.tan(np.radians(meta.get("fov_deg", 90) / 2))
cx, cy = W / 2, H / 2

print(f"{len(files)} samples in {run}; showing #{idx} ({files[idx]})")
print("arrival ticks (cam, depth, sonar):", d["arrival_ticks"], "| sim time t =", float(d["t"]))
print("pose z (world):", pose[2, 3], "| speed:", np.linalg.norm(d["vel"]), "m/s")

# ---- depth -> camera-frame points (x right, y down, z forward); depth assumed planar ----
v, u = np.mgrid[0:H, 0:W]
xn, yn = (u - cx) / f, (v - cy) / f
X, Y, Z = xn * depth, yn * depth, depth
r = np.sqrt(X**2 + Y**2 + Z**2)
elev = np.degrees(np.arctan2(Y, np.hypot(X, Z)))      # + = below the optical axis
azim = np.degrees(np.arctan2(X, Z))
valid = (depth > 0.3) & (depth < 50)                   # drops open water (~700 m) and anything beyond 50 m
wedge = valid & (np.abs(elev) <= ELEV / 2) & (np.abs(azim) <= AZ / 2) & (r >= R_MIN) & (r <= R_MAX)
print(f"valid depth pixels: {valid.mean():.1%} | pixels inside the sonar's wedge and range: {wedge.mean():.1%}")

# ---- are the black RGB pixels shadow (valid depth) or empty? ----
black = rgb.sum(-1) == 0
if black.any():
    print(f"black RGB pixels: {black.mean():.1%} | of those with valid depth: {(black & valid).sum() / black.sum():.1%}"
          f" | their median depth: {np.median(depth[black]):.2f} m")

# ---- sonar orientation test: which end of the array is the near range? ----
rng_axis = np.linspace(R_MIN, R_MAX, n_r)
prof = sonar.sum(axis=1)
prof_A = prof                 # hypothesis A: row 0 = RangeMin (near)
prof_B = prof[::-1]           # hypothesis B: row 0 = RangeMax (far)
def centroid(p): return float((rng_axis * p).sum() / max(p.sum(), 1e-9))
nz = np.nonzero(prof > 0)[0]
print(f"sonar non-zero rows: {nz.min()}..{nz.max()} of {n_r}")
print(f"sonar mean return range if row 0 = near: {centroid(prof_A):.2f} m | if row 0 = far: {centroid(prof_B):.2f} m")
if wedge.any():
    print(f"depth-camera ranges inside the wedge (5/50/95 pct): {np.percentile(r[wedge], [5, 50, 95]).round(2)} m "
          f"(camera-to-sonar offset not corrected, so expect some mismatch)")

# ---- sonar fan ----
def fan(arr, size=400):
    half = np.deg2rad(AZ) / 2
    Xf, Yf = np.meshgrid(np.linspace(-R_MAX * np.sin(half), R_MAX * np.sin(half), size),
                         np.linspace(R_MAX, 0, size))
    Rr, TH = np.hypot(Xf, Yf), np.arctan2(Xf, Yf)
    ri = ((Rr - R_MIN) / (R_MAX - R_MIN) * (n_r - 1)).round().astype(int)
    ai = ((TH + half) / (2 * half) * (n_az - 1)).round().astype(int)
    ok = (ri >= 0) & (ri < n_r) & (ai >= 0) & (ai < n_az)
    out = np.zeros((size, size))
    out[ok] = arr[ri[ok], ai[ok]]
    return out, [-R_MAX * np.sin(half), R_MAX * np.sin(half), 0, R_MAX]

fanA, ext = fan(sonar)
fanB, _ = fan(sonar[::-1])
vmax = max(float(np.sqrt(sonar).max()), 1e-6)

fig = plt.figure(figsize=(18, 9))
gs = fig.add_gridspec(2, 4)
ax = [fig.add_subplot(gs[0, i]) for i in range(4)]
ax[0].imshow(rgb); ax[0].set_title("RGB"); ax[0].axis("off")
im = ax[1].imshow(np.ma.masked_greater(depth, 50), cmap="viridis"); ax[1].set_title("Depth (m), >50 m masked")
ax[1].axis("off"); plt.colorbar(im, ax=ax[1], fraction=0.046)
ax[2].imshow(black, cmap="gray"); ax[2].set_title("Black RGB pixels (white = black in RGB)"); ax[2].axis("off")
ax[3].imshow(np.sqrt(sonar), cmap="gray", aspect="auto"); ax[3].set_title("Raw sonar array (sqrt), rows = range bins")
for k, (a, t) in enumerate([(fanA, "Fan if row 0 = NEAR"), (fanB, "Fan if row 0 = FAR")]):
    a_ = fig.add_subplot(gs[1, k])
    a_.imshow(np.sqrt(a), cmap="gray", vmin=0, vmax=vmax, extent=ext); a_.set_title(t)
a_ = fig.add_subplot(gs[1, 2:])
if wedge.any():
    hist, edges = np.histogram(r[wedge], bins=60, range=(R_MIN, R_MAX))
    a_.fill_between(0.5 * (edges[1:] + edges[:-1]), hist / max(hist.max(), 1), alpha=0.3, label="depth camera, inside wedge")
a_.plot(rng_axis, prof_A / max(prof_A.max(), 1e-9), label="sonar, row 0 = near")
a_.plot(rng_axis, prof_B / max(prof_B.max(), 1e-9), label="sonar, row 0 = far")
a_.set_xlabel("range (m)"); a_.set_title("Range profiles (normalised)"); a_.legend()
plt.tight_layout()
out = f"{run}/inspect_{idx:05d}.png"
fig.savefig(out, dpi=110)
print("saved", out)
plt.show()