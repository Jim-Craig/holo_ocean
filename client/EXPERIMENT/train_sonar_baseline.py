"""Sonar-only depth baseline (U-Net with an acoustic ResNet encoder), evaluated by region.

Train (splits are whole recording sessions, never random samples):
    python train_sonar_baseline.py --train Test_Pier Test_seabed --val Test_Kelp --test Test_mixed --epochs 20 --out baseline_sonar

Evaluate a trained model under another condition, with the evaluation regions taken from the clean reference run:
    python train_sonar_baseline.py --eval-only baseline_sonar/best.pt --test run_turbid@Test_Pier --out eval_sonar_turbid

The network predicts log-depth (metres) from the 2D imaging sonar array (range x azimuth).
Loss = L1 on log-depth over pixels with usable depth.
Results are reported separately for: all valid pixels, pixels inside the sonar beam, pixels with a real sonar echo,
in-beam pixels without an echo, pixels outside the beam, and by distance band.
"""
import argparse
import json
import math
import os
import random
import time
from collections import defaultdict

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
from torch.utils.data import DataLoader

from fusion_data import DepthSonarDataset


# ---------------------------------------------------------------- model
def block(cin, cout):
    return nn.Sequential(
        nn.Conv2d(cin, cout, 3, padding=1, bias=False), nn.BatchNorm2d(cout), nn.ReLU(inplace=True),
        nn.Conv2d(cout, cout, 3, padding=1, bias=False), nn.BatchNorm2d(cout), nn.ReLU(inplace=True))


class SonarUNetDepth(nn.Module):
    """Predicts a 2D depth map (H, W) from a 1-channel imaging sonar array (1, n_r, n_az)."""
    def __init__(self, encoder="resnet34", pretrained=True):
        super().__init__()
        assert encoder in ("resnet18", "resnet34"), "encoder must be resnet18 or resnet34"
        r = getattr(torchvision.models, encoder)(weights="DEFAULT" if pretrained else None)

        # Adapt first conv for 1-channel acoustic input
        self.stem_conv = nn.Conv2d(1, 64, kernel_size=7, stride=2, padding=3, bias=False)
        if pretrained:
            # Initialize 1-channel weights from the mean of RGB pre-trained filters
            with torch.no_grad():
                self.stem_conv.weight.copy_(r.conv1.weight.sum(dim=1, keepdim=True) / 3.0)

        self.stem = nn.Sequential(self.stem_conv, r.bn1, r.relu)             # 1/2, 64 ch
        self.pool, self.l1, self.l2, self.l3, self.l4 = r.maxpool, r.layer1, r.layer2, r.layer3, r.layer4

        # Decoder to upsample acoustic features into the (H, W) camera frame geometry
        self.d4 = block(512 + 256, 256)
        self.d3 = block(256 + 128, 128)
        self.d2 = block(128 + 64, 64)
        self.d1 = block(64 + 64, 64)
        self.d0 = block(64 + 1, 32)
        self.head = nn.Conv2d(32, 1, 1)
        nn.init.normal_(self.head.weight, std=1e-3)
        nn.init.constant_(self.head.bias, math.log(8.0))                     # prior initialization ~8m

    def forward(self, sonar, out_size=(240, 320)):
        # sonar: (B, 1, n_r, n_az)
        s0 = self.stem(sonar)
        s1 = self.l1(self.pool(s0))
        s2 = self.l2(s1)
        s3 = self.l3(s2)
        s4 = self.l4(s3)

        def up(a, skip):
            return torch.cat([F.interpolate(a, size=skip.shape[-2:], mode="bilinear", align_corners=False), skip], 1)

        y = self.d4(up(s4, s3))
        y = self.d3(up(y, s2))
        y = self.d2(up(y, s1))
        y = self.d1(up(y, s0))

        # Final projection to the target camera output resolution (H, W)
        y = F.interpolate(y, size=out_size, mode="bilinear", align_corners=False)
        sonar_resampled = F.interpolate(sonar, size=out_size, mode="bilinear", align_corners=False)
        y = self.d0(torch.cat([y, sonar_resampled], 1))
        return self.head(y)[:, 0]                                             # log-depth in metres, (B, H, W)


# ---------------------------------------------------------------- metrics
BANDS = [(0, 5), (5, 10), (10, 20), (20, 1e9)]


class Meter:
    def __init__(self):
        self.s = defaultdict(lambda: np.zeros(4))        # sum |p-g|/g, sum (p-g)^2, count(delta<1.25), n

    def update(self, name, pred, gt, mask):
        n = int(mask.sum())
        if n == 0:
            return
        p, g = pred[mask].double(), gt[mask].double()
        self.s[name] += np.array([(p - g).abs().div(g).sum().item(), ((p - g) ** 2).sum().item(),
                                  (torch.maximum(p / g, g / p) < 1.25).sum().item(), n])

    def table(self):
        return {k: {"n": int(v[3]), "absrel": v[0] / v[3], "rmse_m": math.sqrt(v[1] / v[3]), "delta1": v[2] / v[3]}
                for k, v in self.s.items()}


def update_regions(meter, pred_depth, batch, valid):
    gt = batch["depth"]
    ib, sup = batch["in_beam"] & valid, batch["supported"] & valid
    meter.update("all valid pixels", pred_depth, gt, valid)
    meter.update("in beam", pred_depth, gt, ib)
    meter.update("in beam, echo", pred_depth, gt, sup)
    meter.update("in beam, no echo", pred_depth, gt, ib & ~sup)
    meter.update("outside beam", pred_depth, gt, valid & ~batch["in_beam"])
    for lo, hi in BANDS:
        name = f"depth {lo:g}-{hi:g} m" if hi < 1e8 else f"depth > {lo:g} m"
        meter.update(name, pred_depth, gt, valid & (gt >= lo) & (gt < hi))


def print_table(title, tab):
    print(f"\n{title}")
    print(f"  {'region':<22}{'pixels':>12}{'AbsRel':>9}{'RMSE (m)':>10}{'delta<1.25':>12}")
    for k, v in tab.items():
        print(f"  {k:<22}{v['n']:>12,}{v['absrel']:>9.3f}{v['rmse_m']:>10.2f}{v['delta1']:>12.3f}")


@torch.no_grad()
def evaluate(model, loader, device, max_depth, out_size):
    model.eval()
    meter = Meter()
    for batch in loader:
        batch = {k: (v.to(device, non_blocking=True) if torch.is_tensor(v) else v) for k, v in batch.items()}
        with torch.autocast(device_type=device, dtype=torch.bfloat16, enabled=(device == "cuda")):
            out = model(batch["sonar"], out_size=tuple(out_size))
        pred = torch.exp(out.float().clamp(-3, 5))
        valid = batch["valid"] & (batch["depth"] <= max_depth)
        update_regions(meter, pred, batch, valid)
    return meter.table()


# ---------------------------------------------------------------- main
def parse():
    ap = argparse.ArgumentParser(description="Train and evaluate a sonar-only baseline for depth estimation")
    ap.add_argument("--train", nargs="*", default=[])
    ap.add_argument("--val", nargs="*", default=[])
    ap.add_argument("--test", nargs="*", default=[])
    ap.add_argument("--eval-only", default=None, help="checkpoint to evaluate instead of training")
    ap.add_argument("--out", default="baseline_sonar")
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--batch-size", type=int, default=16)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--size", type=int, nargs=2, default=[240, 320], metavar=("H", "W"))
    ap.add_argument("--encoder", default="resnet34", choices=["resnet18", "resnet34"])
    ap.add_argument("--no-pretrained", action="store_true")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--thr", type=float, default=0.02, help="sonar value that counts as an echo")
    ap.add_argument("--tol", type=int, default=2)
    ap.add_argument("--max-depth", type=float, default=40.0, help="ignore pixels deeper than this (m)")
    ap.add_argument("--max-per-run", type=int, default=None, help="use at most this many samples per run (quick tests)")
    ap.add_argument("--seed", type=int, default=0)
    return ap.parse_args()


def make_loader(specs, args, train):
    # with_sonar=True loads and returns batch["sonar"]
    ds = DepthSonarDataset(specs, size=args.size, with_sonar=True, thr=args.thr, tol=args.tol,
                           train=train, max_per_run=args.max_per_run)
    return DataLoader(ds, batch_size=args.batch_size, shuffle=train, num_workers=args.workers, drop_last=train,
                      pin_memory=torch.cuda.is_available(), persistent_workers=args.workers > 0)


def main():
    args = parse()
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    os.makedirs(args.out, exist_ok=True)
    print(f"device: {device}" + (f" ({torch.cuda.get_device_name(0)})" if device == "cuda" else ""))

    if args.eval_only:
        ck = torch.load(args.eval_only, map_location=device)
        model = SonarUNetDepth(ck["encoder"], pretrained=False).to(device)
        model.load_state_dict(ck["model"])
        args.size = ck.get("size", args.size)
        results = {}
        for name, specs in (("val", args.val), ("test", args.test)):
            if specs:
                results[name] = evaluate(model, make_loader(specs, args, False), device, args.max_depth, args.size)
                print_table(f"{name}: {' '.join(specs)}", results[name])
        json.dump(results, open(os.path.join(args.out, "results.json"), "w"), indent=2)
        return

    if not args.train:
        raise SystemExit("give --train runs (or --eval-only)")
    train_loader = make_loader(args.train, args, True)
    val_loader = make_loader(args.val, args, False) if args.val else None
    print(f"train samples: {len(train_loader.dataset)}   val: {len(val_loader.dataset) if val_loader else 0}")

    model = SonarUNetDepth(args.encoder, pretrained=not args.no_pretrained).to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    total_steps = max(1, args.epochs * len(train_loader))
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=total_steps)

    history, best = [], float("inf")
    for epoch in range(1, args.epochs + 1):
        model.train()
        t0, run_loss, n_batches = time.time(), 0.0, 0
        for batch in train_loader:
            batch = {k: (v.to(device, non_blocking=True) if torch.is_tensor(v) else v) for k, v in batch.items()}
            valid = batch["valid"] & (batch["depth"] <= args.max_depth)
            if valid.sum() == 0:
                continue
            with torch.autocast(device_type=device, dtype=torch.bfloat16, enabled=(device == "cuda")):
                out = model(batch["sonar"], out_size=tuple(args.size))
            tgt = torch.log(batch["depth"].clamp(min=0.3))
            loss = (out.float() - tgt).abs()[valid].mean()
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            sched.step()
            run_loss += loss.item(); n_batches += 1
        rec = {"epoch": epoch, "train_loss": run_loss / max(n_batches, 1), "seconds": time.time() - t0}
        if val_loader:
            tab = evaluate(model, val_loader, device, args.max_depth, args.size)
            rec["val_absrel"] = tab["all valid pixels"]["absrel"]
            rec["val_rmse_m"] = tab["all valid pixels"]["rmse_m"]
            score = rec["val_absrel"]
        else:
            score = rec["train_loss"]
        history.append(rec)
        print("  ".join(f"{k} {v:.4f}" if isinstance(v, float) else f"{k} {v}" for k, v in rec.items()))
        ck = {"model": model.state_dict(), "encoder": args.encoder, "size": list(args.size),
              "max_depth": args.max_depth, "epoch": epoch}
        torch.save(ck, os.path.join(args.out, "last.pt"))
        if score < best:
            best = score
            torch.save(ck, os.path.join(args.out, "best.pt"))
        json.dump(history, open(os.path.join(args.out, "history.json"), "w"), indent=2)

    ck = torch.load(os.path.join(args.out, "best.pt"), map_location=device)
    model.load_state_dict(ck["model"])
    print(f"\nbest checkpoint from epoch {ck['epoch']}")
    results = {}
    for name, specs in (("val", args.val), ("test", args.test)):
        if specs:
            results[name] = evaluate(model, make_loader(specs, args, False), device, args.max_depth, args.size)
            print_table(f"{name}: {' '.join(specs)}", results[name])
    json.dump(results, open(os.path.join(args.out, "results.json"), "w"), indent=2)
    print(f"\nsaved {args.out}/best.pt, last.pt, history.json, results.json")


if __name__ == "__main__":
    main()

