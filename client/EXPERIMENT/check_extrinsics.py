"""Camera-depth <-> imaging-sonar extrinsic consistency check.

usage:  python check_extrinsics.py run_YYYYMMDD_HHMMSS [every_Nth_sample]

Idea: the depth camera tells us where surfaces are. If we place a sonar at a candidate pose, we can predict
which (range, azimuth) bins should contain returns. The right pose makes the predicted occupancy match the
sonar's non-zero bins. We sweep one extrinsic parameter at a time (others at 0) and report the F1 score.

Frames: camera x = right, y = down, z = forward.  Sonar assumed to start with the same axes and be shifted by
t = (tx, ty, tz) metres (in camera axes) and rotated by (rx, ry, rz) degrees about the camera x/y/z axes.
Sonar array: rows = range (row 0 = RangeMin), columns = azimuth.
"""
import sys, glob, json
import numpy as np
import matplotlib.pyplot as plt

run = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 3
meta = json.load(open(f"{run}/meta.json"))
so = meta["sonar"]
R_MIN, R_MAX, AZ, ELEV = so["r_min"], so["r_max"], so["azimuth_deg"], so["elevation_deg"]
files = sorted(glob.glob(f"{run}/s*.npz"))[::step]
print(f"using {len(files)} samples from {run}")

def load(fn):
    d = np.load(fn)
    depth, sonar = d["depth"], d["sonar"]
    H, W = depth.shape
    f = (W / 2) / np.tan(np.radians(meta.get("fov_deg", 90) / 2))
    v, u = np.mgrid[0:H:2, 0:W:2]
    z = depth[::2, ::2]
    ok = (z > 0.3) & (z < 50)                                  # drops open water and far terrain
    xn, yn = (u - W / 2) / f, (v - H / 2) / f
    P = np.stack([xn[ok] * z[ok], yn[ok] * z[ok], z[ok]], 1)   # camera-frame points; depth assumed planar
    return P, sonar > 1e-4

samples = [load(fn) for fn in files]
n_r, n_az = samples[0][1].shape
c0 = int(round((AZ / 2 - 40) / AZ * (n_az - 1)))               # score only azimuths the camera can see (+-40 deg)
c1 = n_az - c0

def rot(rx, ry, rz):
    a, b, c = np.radians([rx, ry, rz])
    Rx = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    Ry = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    Rz = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def dilate(B):
    Pd = np.pad(B, 1)
    out = B.copy()
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            out |= Pd[dy:dy + B.shape[0], dx:dx + B.shape[1]]
    return out

def predict(P, t, R, az_sign):
    Ps = (P - t) @ R                                           # = R^T (p - t) for row vectors
    r = np.linalg.norm(Ps, axis=1)
    az = np.degrees(np.arctan2(Ps[:, 0], Ps[:, 2]))
    el = np.degrees(np.arctan2(Ps[:, 1], np.hypot(Ps[:, 0], Ps[:, 2])))
    ok = (r >= R_MIN) & (r <= R_MAX) & (np.abs(az) <= AZ / 2) & (np.abs(el) <= ELEV / 2)
    row = np.round((r[ok] - R_MIN) / (R_MAX - R_MIN) * (n_r - 1)).astype(int)
    col = np.round((az_sign * az[ok] + AZ / 2) / AZ * (n_az - 1)).astype(int)
    B = np.zeros((n_r, n_az), bool)
    B[row, col] = True
    return dilate(B)

def f1(B, S):
    B, S = B[:, c0:c1], S[:, c0:c1]
    tp = (B & S).sum()
    if tp == 0:
        return 0.0
    p, r = tp / B.sum(), tp / S.sum()
    return 2 * p * r / (p + r)

def score(p6, az_sign):
    t, R = np.array(p6[:3]), rot(*p6[3:])
    return float(np.mean([f1(predict(P, t, R, az_sign), S) for P, S in samples]))

zero = np.zeros(6)
s_pos, s_neg = score(zero, +1), score(zero, -1)
az_sign = +1 if s_pos >= s_neg else -1
print(f"F1 at identity: azimuth columns increasing to the right = {s_pos:.3f} | mirrored = {s_neg:.3f}  -> using {az_sign:+d}")

names = ["tx right (m)", "ty down (m)", "tz forward (m)", "rx about right (deg)", "ry about down / yaw (deg)", "rz about fwd / roll (deg)"]
grids = [np.linspace(-1.5, 1.5, 31)] * 3 + [np.linspace(-8, 8, 33)] * 3
fig, axs = plt.subplots(2, 4, figsize=(18, 8))
print(f"{'parameter':28s} {'best value':>10s} {'F1 best':>8s} {'F1 at 0':>8s} {'width (>=90% of best)':>22s}")
for i, (nm, g) in enumerate(zip(names, grids)):
    sc = []
    for val in g:
        p6 = zero.copy(); p6[i] = val
        sc.append(score(p6, az_sign))
    sc = np.array(sc)
    good = g[sc >= 0.9 * sc.max()]
    print(f"{nm:28s} {g[sc.argmax()]:10.2f} {sc.max():8.3f} {score(zero, az_sign):8.3f} {good.min():10.2f} .. {good.max():.2f}")
    a = axs.flat[i]
    a.plot(g, sc); a.axvline(0, color="k", lw=0.5); a.set_title(nm); a.set_ylabel("F1")

B0 = predict(samples[0][0], np.zeros(3), np.eye(3), az_sign)
S0 = samples[0][1]
ov = np.zeros((n_r, n_az, 3))
ov[..., 0] = B0; ov[..., 2] = B0; ov[..., 1] = S0
axs.flat[6].imshow(ov, aspect="auto"); axs.flat[6].set_title("identity: white=both, magenta=predicted only, green=sonar only")
axs.flat[6].axvline(c0, color="y", lw=0.5); axs.flat[6].axvline(c1, color="y", lw=0.5)
axs.flat[7].axis("off")
plt.tight_layout()
out = f"{run}/extrinsics_check.png"
fig.savefig(out, dpi=100)
print("saved", out)
plt.show()
