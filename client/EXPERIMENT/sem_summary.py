"""Summarise the semantic-camera labels in a recorded run.

usage:  python sem_summary.py run_YYYYMMDD_HHMMSS [n_examples]

Prints every distinct label colour found in the run (and how much of the image it covers), then saves
sem_summary.png with example samples: RGB | raw semantic image | RGB with a *candidate kelp mask* in red.
Candidate kelp mask = unlabelled (black) pixels that still have real depth (0.3-50 m). Check in the picture
whether the pier lattice ends up red too; if it does, the black label also covers the lattice and the rule is too broad.
"""
import sys, glob
import numpy as np
import matplotlib.pyplot as plt

run = sys.argv[1]
n_show = int(sys.argv[2]) if len(sys.argv) > 2 else 4
files = sorted(glob.glob(f"{run}/s*.npz"))
if not files:
    sys.exit(f"no samples found in {run}")

totals, first_seen, n_labels = {}, {}, []
for i, fn in enumerate(files):
    d = np.load(fn)
    if "sem" not in d.files:
        sys.exit(f"{fn} has no semantic image (record with WITH_SEMANTIC = True in rig.py)")
    px = d["sem"][..., :3].reshape(-1, 3)
    vals, cnt = np.unique(px, axis=0, return_counts=True)
    n_labels.append(len(vals))
    for v, c in zip(map(tuple, vals.tolist()), cnt):
        totals[v] = totals.get(v, 0) + int(c)
        first_seen.setdefault(v, i)

n_pix = sum(totals.values())
print(f"{len(files)} samples, {len(totals)} distinct label colours (values as saved, alpha ignored):")
for v, c in sorted(totals.items(), key=lambda kv: -kv[1]):
    print(f"   {v}   {c / n_pix:6.2%} of all pixels   first seen in sample #{first_seen[v]}")
print(f"samples containing more than 2 label colours: {sum(n > 2 for n in n_labels)} of {len(files)}")

# pick examples: the samples with the most distinct labels, otherwise evenly spaced
order = [int(i) for i in np.argsort(n_labels)[::-1]]
picks = order[:n_show] if max(n_labels) > 2 else [int(i) for i in np.linspace(0, len(files) - 1, min(n_show, len(files))).round()]
picks = sorted(set(picks))

fig, axs = plt.subplots(len(picks), 3, figsize=(15, 3.8 * len(picks)), squeeze=False)
for r, i in enumerate(picks):
    d = np.load(files[i])
    rgb, sem, depth = d["rgb"], d["sem"][..., :3], d["depth"]
    mask = (sem.sum(-1) == 0) & (depth > 0.3) & (depth < 50)
    ov = rgb.astype(float)
    ov[mask] = 0.4 * ov[mask] + 0.6 * np.array([255, 0, 0])
    axs[r, 0].imshow(rgb); axs[r, 0].set_title(f"sample #{i}: RGB")
    axs[r, 1].imshow(sem); axs[r, 1].set_title(f"semantic raw ({n_labels[i]} colours)")
    axs[r, 2].imshow(ov.astype(np.uint8)); axs[r, 2].set_title(f"candidate kelp mask in red ({mask.mean():.1%} of pixels)")
    for a in axs[r]:
        a.axis("off")
plt.tight_layout()
out = f"{run}/sem_summary.png"
fig.savefig(out, dpi=100)
print("saved", out)
plt.show()