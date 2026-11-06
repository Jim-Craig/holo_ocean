"""PyTorch dataset for the depth experiments (reads s*.npz from experiment_3 / replay_path and frame_*.npz from record_sessions).

A "spec" is a run folder, optionally followed by @reference_run:

    run_20261104_104745                      masks come from the run itself
    run_fog_replay@run_20261104_104745       images from the fog replay, masks from the clean reference run

The second form is for condition ablations: replay_path.py records the same viewpoints under a new condition, and the
reference run's masks keep the evaluation regions (in beam / echo support) identical across conditions.

Each item is a dict of tensors (size = (h, w)):
    rgb        3 x h x w   float, 0..1
    depth      h x w       float, metres (0 where unusable)
    valid, in_beam, supported   h x w bool
    sonar      1 x n_r x n_az  float (sqrt of the echo intensity), only if with_sonar=True
    run, idx   where the sample came from
"""
import json
import os
import random

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import Dataset

from run_io import depth_of, list_samples
from sonar_masks import compute_masks


def _split_spec(spec):
    if "@" in spec:
        run, ref = spec.split("@", 1)
        return run, ref
    return spec, None


class DepthSonarDataset(Dataset):
    def __init__(self, specs, size=(240, 320), with_sonar=False, thr=0.02, tol=2, train=False,
                 max_per_run=None):
        self.size, self.with_sonar, self.thr, self.tol, self.train = tuple(size), with_sonar, thr, tol, train
        self.items = []                      # (run, file, ref_file or None, meta, ref_meta)
        for spec in specs:
            run, ref = _split_spec(spec)
            files = list_samples(run)
            meta = json.load(open(os.path.join(run, "meta.json")))
            if ref:
                ref_files = list_samples(ref)
                ref_meta = json.load(open(os.path.join(ref, "meta.json")))
                if len(ref_files) != len(files):
                    raise ValueError(f"{run} has {len(files)} samples but its reference {ref} has {len(ref_files)}")
            else:
                ref_files, ref_meta = [None] * len(files), None
            idx = list(range(len(files)))
            if max_per_run and len(idx) > max_per_run:
                idx = np.linspace(0, len(idx) - 1, max_per_run).round().astype(int).tolist()
            for i in idx:
                self.items.append((run, files[i], ref_files[i], meta, ref_meta))

    def __len__(self):
        return len(self.items)

    def _resize(self, x, mode):
        # x: (C, H, W) tensor
        if tuple(x.shape[-2:]) == self.size:
            return x
        if mode == "bilinear":
            return F.interpolate(x[None], size=self.size, mode="bilinear", align_corners=False, antialias=True)[0]
        return F.interpolate(x[None], size=self.size, mode="nearest")[0]

    def __getitem__(self, i):
        run, path, ref_path, meta, ref_meta = self.items[i]
        d = np.load(path)
        rgb = d["rgb"]
        depth = depth_of(d).astype(np.float32)
        sonar = d["sonar"]
        if ref_path:
            dr = np.load(ref_path)
            masks = compute_masks(depth_of(dr).astype(np.float32), dr["sonar"], ref_meta, self.thr, self.tol)
        else:
            masks = compute_masks(depth, sonar, meta, self.thr, self.tol)

        rgb_t = torch.from_numpy(np.ascontiguousarray(rgb)).permute(2, 0, 1).float() / 255.0
        depth_t = torch.from_numpy(depth)[None]
        valid = torch.from_numpy(masks["valid"])[None].float()
        in_beam = torch.from_numpy(masks["in_beam"])[None].float()
        supported = torch.from_numpy(masks["supported"])[None].float()
        # depth is resized with nearest-neighbour so open water (huge values) never bleeds into surfaces
        rgb_t = self._resize(rgb_t, "bilinear")
        depth_t = self._resize(depth_t, "nearest")[0]
        valid, in_beam, supported = (self._resize(m, "nearest")[0] > 0.5 for m in (valid, in_beam, supported))
        out = {"rgb": rgb_t, "depth": depth_t, "valid": valid, "in_beam": in_beam, "supported": supported,
               "run": os.path.basename(run.rstrip("/\\")), "idx": i}
        if self.with_sonar:
            out["sonar"] = torch.from_numpy(np.sqrt(np.clip(sonar, 0, None))).float()[None]

        if self.train:
            if random.random() < 0.5:                       # mirror the scene (and the sonar's azimuth columns)
                for k in ("rgb", "depth", "valid", "in_beam", "supported"):
                    out[k] = torch.flip(out[k], dims=[-1])
                if self.with_sonar:
                    out["sonar"] = torch.flip(out["sonar"], dims=[-1])
            gain = 1.0 + random.uniform(-0.15, 0.15)       # mild brightness / contrast jitter
            bias = random.uniform(-0.05, 0.05)
            out["rgb"] = (out["rgb"] * gain + bias).clamp(0, 1)
        return out
