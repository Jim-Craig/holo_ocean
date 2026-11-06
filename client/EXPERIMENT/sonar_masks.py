"""Region masks for the depth experiments (numpy only, no PyTorch needed).

For every camera pixel we know its 3D point (from the depth image), so we can ask:
  valid      does the pixel have a usable depth (0.3-50 m; the rest is open water)?
  in_beam    does its surface lie inside the sonar's beam and range limits?
  supported  ...and does the sonar actually have an echo at that spot?
Assumes the sonar sits where the camera does (identity extrinsics) and sonar column 0 is the rightmost direction.
"""
import functools
import numpy as np


@functools.lru_cache(maxsize=8)
def _geometry(H, W, fov_deg, az_total, el_total, n_az):
    f = (W / 2) / np.tan(np.radians(fov_deg / 2))
    v, u = np.mgrid[0:H, 0:W]
    xn, yn = (u - W / 2) / f, (v - H / 2) / f
    az = np.degrees(np.arctan2(xn, 1.0))                         # + = right of the optical axis
    el = np.degrees(np.arctan2(yn, np.hypot(xn, 1.0)))           # + = below the optical axis
    ray = np.sqrt(1 + xn**2 + yn**2).astype(np.float32)          # range = planar depth * ray
    in_angle = (np.abs(az) <= az_total / 2) & (np.abs(el) <= el_total / 2)
    col = np.rint((-az + az_total / 2) / az_total * (n_az - 1)).astype(np.int32)   # column 0 = rightmost
    return ray, in_angle, col


def dilate(mask, r):
    """Grow a boolean mask by r pixels in every direction (square neighbourhood)."""
    if r <= 0:
        return mask
    h, w = mask.shape
    p = np.pad(mask, r)
    out = np.zeros_like(mask)
    for dy in range(2 * r + 1):
        for dx in range(2 * r + 1):
            out |= p[dy:dy + h, dx:dx + w]
    return out


def compute_masks(depth, sonar, meta, thr=0.02, tol=2, max_depth=50.0):
    """Boolean (H, W) masks at full camera resolution. `meta` is the run's meta.json as a dict."""
    so = meta["sonar"]
    r_min, r_max = so["r_min"], so["r_max"]
    az_total, el_total = so["azimuth_deg"], so["elevation_deg"]
    n_r, n_az = so["bins"]
    H, W = depth.shape
    ray, in_angle, col = _geometry(H, W, float(meta.get("fov_deg", 90)), float(az_total), float(el_total), int(n_az))
    valid = (depth > 0.3) & (depth < max_depth)
    r = depth.astype(np.float32) * ray
    in_beam = valid & in_angle & (r >= r_min) & (r <= r_max)
    row = np.rint((r - r_min) / (r_max - r_min) * (n_r - 1)).astype(np.int32)
    ok = in_beam & (row >= 0) & (row < n_r)
    echo = dilate(sonar > thr, tol)
    supported = np.zeros_like(in_beam)
    supported[ok] = echo[row[ok], col[ok]]
    return {"valid": valid, "in_beam": in_beam, "supported": supported}
