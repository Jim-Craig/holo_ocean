"""Read recorded runs written by experiment_3.py / replay_path.py (s00000.npz, key 'depth')
or by record_sessions.py (frame_000000.npz, key 'depth_image')."""
import glob
import os

import numpy as np


def list_samples(run):
    files = sorted(glob.glob(os.path.join(run, "s*.npz"))) or sorted(glob.glob(os.path.join(run, "frame_*.npz")))
    if not files:
        raise FileNotFoundError(f"no samples (s*.npz or frame_*.npz) in {run}")
    return files


def depth_of(npz):
    """Depth image (metres) from an opened npz, whichever key name it uses."""
    return npz["depth"] if "depth" in npz.files else npz["depth_image"]


def has(npz, key):
    return key in npz.files


def load_sample(path):
    """Eagerly load the arrays every tool needs: rgb, depth, sonar, pose."""
    d = np.load(path)                       # object fields such as env_state are never touched
    return {"rgb": d["rgb"], "depth": depth_of(d).astype(np.float32), "sonar": d["sonar"], "pose": d["pose"]}
