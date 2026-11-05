"""Record aligned camera, sonar, and depth data for multi-modal sensor fusion.

Drives the AUV along a predefined survey path (or manual control) and saves
synchronised frames of:
  - RGB camera image    (H x W x 3, uint8)
  - Imaging sonar       (RangeBins x AzimuthBins, float32)
  - Depth camera map    (H x W, float32, per-pixel depth)
  - Vehicle depth       (scalar float32, metres below surface, positive down)
  - Full 4x4 world pose (for trajectory reconstruction)
  - Vehicle velocity    (3-vector float64)

Usage
-----
  python record_data.py --mode auto                       # auto survey at 3.5m depth
  python record_data.py --mode auto --depth 3.0           # auto survey at 3.0m depth
  python record_data.py --mode auto --depth 4.0 --laps 2  # 2 laps, starting at 4.0m
  python record_data.py --mode auto --depth 12.0          # default pier harbor depth
  python record_data.py --mode auto --no-display          # headless recording (fast)
  python record_data.py --mode manual --depth 3.5         # manual WASD control at 3.5m

Controls (manual mode — click the matplotlib window for keyboard focus)
  W/S   forward / back          A/D   turn left / right
  Q/E   strafe left / right     R/F   up / down
  SPACE hold position           X     quit
  L     toggle continuous recording
  P     save one snapshot now
"""

import os, sys, json, time, argparse
import numpy as np
import matplotlib.pyplot as plt
import holoocean
import rig


# ========================== CLI ==========================
ap = argparse.ArgumentParser(description="Record camera + sonar + depth data from HoloOcean")
ap.add_argument("note", nargs="?", default="", help="free-text note stored in meta.json")
ap.add_argument("--mode", choices=["manual", "auto"], default="auto",
                help="'auto' = survey path (default), 'manual' = WASD fly")
ap.add_argument("--depth", type=float, default=14.5,
                help="vehicle spawn depth in metres below surface (default: 3.5)")
ap.add_argument("--sonar-range", type=float, default=None,
                help="sonar max range in metres (default: 30.0m if depth <= 6m, else 20.0m)")
ap.add_argument("--laps", type=int, default=1,
                help="how many survey loops to perform in auto mode")
ap.add_argument("--depth-step", type=float, default=0.0,
                help="metres to increase depth by on each successive lap (default: 0.0)")
ap.add_argument("--survey-length", type=float, default=12.0,
                help="length of each survey lane in metres (default: 12.0)")
ap.add_argument("--survey-width", type=float, default=3.5,
                help="lane separation in metres (default: 3.5)")
ap.add_argument("--save-interval", type=float, default=0.5,
                help="minimum distance (m) between saved frames (default: 0.5)")
ap.add_argument("--save-interval-deg", type=float, default=10.0,
                help="minimum yaw change (deg) between saved frames (default: 10.0)")
ap.add_argument("--auto-save-hz", type=float, default=2.0,
                help="max frame save rate (Hz) in auto mode (default: 2.0)")
ap.add_argument("--warmup-ticks", type=int, default=180,
                help="ticks to settle / build octree before recording (default: 180)")
ap.add_argument("--no-display", action="store_true",
                help="skip the live matplotlib window (headless recording)")
args = ap.parse_args()


# ========================== sonar range configuration ==========================
# In PierHarbor, the seabed is at z ≈ -19.2m.
# At depth 12m (z = -12), altitude is 7.2m -> slant range is ~11-20m (within 20m).
# At depth 3-4m (z = -3.5), altitude is 15.7m -> slant range to seabed is 24.4m.
# Therefore, when operating near the surface (depth <= 6m), RangeMax must be at least 30m
# so the acoustic beam reaches the seabed and structures.
if args.sonar_range is not None:
    rig.SONAR["RangeMax"] = float(args.sonar_range)
elif args.depth <= 6.0:
    rig.SONAR["RangeMax"] = 30.0  # extended range for shallow depths to reach seabed
else:
    rig.SONAR["RangeMax"] = 20.0


# ========================== scenario setup ==========================
scenario = holoocean.packagemanager.get_scenario(rig.SCENARIO)
scenario["package_name"] = "Ocean"
agent = rig.apply(scenario)

# Configure start depth: surface is z=0, depth d is z=-abs(d)
desired_z = -abs(args.depth)
if rig.START_LOCATION is not None:
    agent["location"] = list(rig.START_LOCATION)
    agent["location"][2] = desired_z
else:
    # Use PierHarbor dock coordinates with the requested depth
    agent["location"] = [486.0, -632.0, desired_z]

if rig.START_ROTATION is not None:
    agent["rotation"] = list(rig.START_ROTATION)
else:
    agent["rotation"] = [0.0, 0.0, 135.0]


# ========================== constants ==========================
DT = 1.0 / rig.TICKS_PER_SEC
SPEED, TURN = 1.0, 45.0
LEASH, YAW_LEASH = 1.8, 45.0
R_MIN = rig.SONAR["RangeMin"]
R_MAX = rig.SONAR["RangeMax"]
AZ_DEG = rig.SONAR["Azimuth"]
N_R, N_AZ = rig.SONAR["RangeBins"], rig.SONAR["AzimuthBins"]
FLIP_AZIMUTH = True
REQUIRED_SENSORS = ["FrontCamera", "DepthCam", "ImagingSonar", "PoseSensor"]


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


# ========================== survey waypoints ==========================
def make_survey_waypoints(start_xyz, start_yaw_deg, length=12.0, width=3.5, laps=1, depth_step=0.0):
    """Generate lawnmower survey waypoints with alternating lanes.

    AUV moves along initial heading, turns right to adjacent lane, runs back,
    turns left to next lane, etc.
    """
    waypoints = []
    x, y, z = start_xyz
    yaw = start_yaw_deg

    for lap in range(laps):
        lap_z = z - (lap * abs(depth_step))

        # Lane 1: forward along heading
        h1 = np.radians(yaw)
        x += length * np.cos(h1)
        y += length * np.sin(h1)
        waypoints.append(np.array([x, y, lap_z, 0.0, 0.0, yaw]))

        # Turn 90 deg right & cross-track
        yaw = wrap(yaw - 90.0)
        h2 = np.radians(yaw)
        x += width * np.cos(h2)
        y += width * np.sin(h2)
        waypoints.append(np.array([x, y, lap_z, 0.0, 0.0, yaw]))

        # Lane 2: reverse track
        yaw = wrap(yaw - 90.0)
        h3 = np.radians(yaw)
        x += length * np.cos(h3)
        y += length * np.sin(h3)
        waypoints.append(np.array([x, y, lap_z, 0.0, 0.0, yaw]))

        # Turn 90 deg left & cross-track
        yaw = wrap(yaw + 90.0)
        h4 = np.radians(yaw)
        x += width * np.cos(h4)
        y += width * np.sin(h4)
        waypoints.append(np.array([x, y, lap_z, 0.0, 0.0, yaw]))

        # Turn 90 deg left & forward
        yaw = wrap(yaw + 90.0)
        h5 = np.radians(yaw)
        x += length * np.cos(h5)
        y += length * np.sin(h5)
        waypoints.append(np.array([x, y, lap_z, 0.0, 0.0, yaw]))

    return waypoints


# ========================== output directory ==========================
RUN = time.strftime("data_%Y%m%d_%H%M%S")
os.makedirs(RUN, exist_ok=True)
os.makedirs(f"{RUN}/rgb", exist_ok=True)
os.makedirs(f"{RUN}/sonar", exist_ok=True)


# ========================== sonar fan lookup (for display) ==========================
def fan_map(size=400):
    half = np.deg2rad(AZ_DEG) / 2
    X, Y = np.meshgrid(
        np.linspace(-R_MAX * np.sin(half), R_MAX * np.sin(half), size),
        np.linspace(R_MAX, 0, size),
    )
    R, TH = np.hypot(X, Y), np.arctan2(X, Y)
    ri = (R - R_MIN) / (R_MAX - R_MIN) * (N_R - 1)
    ai = (TH + half) / (2 * half) * (N_AZ - 1)
    valid = (ri >= 0) & (ri <= N_R - 1) & (ai >= 0) & (ai <= N_AZ - 1)
    ri = ri.round().astype(int).clip(0, N_R - 1)
    ai = ai.round().astype(int).clip(0, N_AZ - 1)
    if FLIP_AZIMUTH:
        ai = N_AZ - 1 - ai
    return ri, ai, valid, half

fan_ri, fan_ai, fan_valid, fan_half = fan_map()


# ========================== data containers ==========================
frames = []
trajectory = {"t": [], "pose": [], "vel": [], "target": []}
latest = {}
arrived = {}
n_saved = 0
tick = 0


def extract_depth_scalar(pose_matrix):
    """Vehicle depth below the surface in metres (positive down)."""
    return float(-pose_matrix[2, 3])


def save_frame():
    """Save a synchronised camera + sonar + depth frame to disk."""
    global n_saved
    missing = [k for k in REQUIRED_SENSORS if k not in latest]
    if missing:
        return False

    idx = n_saved
    pose = np.asarray(latest["PoseSensor"], dtype=np.float64).copy()
    depth_m = extract_depth_scalar(pose)

    # RGB: HoloOcean returns BGRA -> RGB
    rgb = latest["FrontCamera"][..., 2::-1].copy()
    depth_img = latest["DepthCam"]["depth"].astype(np.float32).copy()
    sonar = np.asarray(latest["ImagingSonar"], dtype=np.float32).copy()
    vel = np.asarray(latest.get("VelocitySensor", np.full(3, np.nan)), dtype=np.float64).copy()

    # Save per-modality files
    np.save(f"{RUN}/rgb/frame_{idx:06d}.npy", rgb)
    np.save(f"{RUN}/sonar/frame_{idx:06d}.npy", sonar)

    # Frame summary
    valid_d = depth_img[depth_img < 50]
    frames.append({
        "index": idx,
        "t": tick * DT,
        "depth_m": depth_m,
        "pose": pose.tolist(),
        "velocity": vel.tolist(),
        "depth_image_mean": float(np.nanmean(valid_d)) if len(valid_d) else float("nan"),
    })

    # Combined bundle (.npz)
    np.savez_compressed(
        f"{RUN}/frame_{idx:06d}.npz",
        rgb=rgb,
        depth_image=depth_img,
        sonar=sonar,
        pose=pose,
        vehicle_depth=np.float32(depth_m),
        velocity=vel,
        t=np.float64(tick * DT),
    )
    n_saved += 1
    return True


def save_all():
    """Save trajectory and index files."""
    if trajectory["t"]:
        np.savez_compressed(
            f"{RUN}/trajectory.npz",
            t=np.asarray(trajectory["t"], dtype=np.float64),
            pose=np.asarray(trajectory["pose"], dtype=np.float64),
            vel=np.asarray(trajectory["vel"], dtype=np.float64),
            target=np.asarray(trajectory["target"], dtype=np.float64),
        )
        print(f"Saved trajectory ({len(trajectory['t'])} ticks) -> {RUN}/trajectory.npz")

    if frames:
        with open(f"{RUN}/frames.json", "w") as f:
            json.dump(frames, f, indent=2)
        with open(f"{RUN}/frames.csv", "w") as f:
            f.write("index,t,depth_m,x,y,z,depth_img_mean\n")
            for fr in frames:
                p = fr["pose"]
                f.write(f"{fr['index']},{fr['t']:.4f},{fr['depth_m']:.4f},"
                        f"{p[0][3]:.4f},{p[1][3]:.4f},{p[2][3]:.4f},"
                        f"{fr['depth_image_mean']:.4f}\n")
        print(f"Saved {len(frames)} frames index -> {RUN}/frames.csv")


# ========================== meta.json ==========================
meta_out = rig.meta()
meta_out["note"] = args.note
meta_out["mode"] = args.mode
meta_out["target_start_depth_m"] = args.depth
meta_out["recording"] = {
    "frame_files": "frame_NNNNNN.npz (rgb, depth_image, sonar, pose, vehicle_depth, velocity, t)",
    "rgb_dir": "rgb/frame_NNNNNN.npy",
    "sonar_dir": "sonar/frame_NNNNNN.npy",
    "frames_csv": "frames.csv",
    "trajectory": "trajectory.npz",
}
json.dump(meta_out, open(f"{RUN}/meta.json", "w"), indent=2)


# ========================== display setup ==========================
show = not args.no_display

if show:
    for k in list(plt.rcParams):
        if k.startswith("keymap."):
            plt.rcParams[k] = []
    held = set()
    rec = {"on": True, "snap": False}

    def on_press(e):
        if not e.key:
            return
        k = e.key.lower()
        if k not in held:
            if k == "l":
                rec["on"] = not rec["on"]
            if k == "p":
                rec["snap"] = True
        held.add(k)

    def on_release(e):
        if e.key:
            held.discard(e.key.lower())

    plt.ion()
    fig, ax = plt.subplots(1, 3, figsize=(17, 5.5))
    fig.canvas.mpl_connect("key_press_event", on_press)
    fig.canvas.mpl_connect("key_release_event", on_release)
    cam_im = ax[0].imshow(np.zeros((rig.H, rig.W, 3), np.uint8))
    ax[0].axis("off")
    dep_im = ax[1].imshow(np.zeros((rig.H, rig.W)), cmap="viridis", vmin=0, vmax=30)
    ax[1].axis("off"); ax[1].set_title("Depth (m)")
    son_im = ax[2].imshow(
        np.zeros(fan_ri.shape), cmap="gray", vmin=0, vmax=0.7,
        extent=[-R_MAX * np.sin(fan_half), R_MAX * np.sin(fan_half), 0, R_MAX],
    )
    ax[2].set_title(f"Imaging sonar (fan, 0-{R_MAX:.0f} m)")
else:
    held = set()
    rec = {"on": True, "snap": False}
    fig = None


# ========================== motion threshold ==========================
last_save_pos = None
last_save_yaw = None

def moved_enough(pos, yaw):
    if last_save_pos is None:
        return True
    return (np.linalg.norm(pos - last_save_pos) >= args.save_interval
            or abs(wrap(yaw - last_save_yaw)) >= args.save_interval_deg)


# ========================== MAIN LOOP ==========================
print("=" * 70)
print(f"Mode: {args.mode} | Target depth: {args.depth:.1f} m (z = {desired_z:.1f})")
print(f"Scenario: {rig.SCENARIO} | Saving to: {RUN}/")
print(f"Camera: {rig.W}x{rig.H} @ {rig.TILT_DEG}° tilt | Sonar: {N_R}x{N_AZ} bins, {R_MIN:.1f}-{R_MAX:.1f} m range")
print(f"Sonar RangeMax set to {R_MAX:.1f}m to ensure acoustic coverage at {args.depth:.1f}m depth.")
if args.mode == "manual":
    print("W/S fwd/back | A/D turn | Q/E strafe | R/F up/down | SPACE hold | L rec toggle | P snap | X quit")
else:
    print(f"Auto survey: {args.laps} lap(s), lane length={args.survey_length}m, width={args.survey_width}m")
print("=" * 70)

with holoocean.make(scenario_cfg=scenario, show_viewport=show) as env:
    env.set_control_scheme("auv0", 1)  # PD controller
    s = env.tick()
    T = s["PoseSensor"]
    start_xyz = T[:3, 3].copy()
    start_yaw = float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))
    target = np.array([*start_xyz, 0.0, 0.0, start_yaw])

    actual_start_depth = extract_depth_scalar(T)
    meta_out["start_pose"] = {
        "xyz": start_xyz.tolist(),
        "yaw_deg": start_yaw,
        "initial_depth_m": actual_start_depth,
    }
    json.dump(meta_out, open(f"{RUN}/meta.json", "w"), indent=2)

    # ---- warm-up ----
    print(f"Warming up ({args.warmup_ticks} ticks, holding depth ~{actual_start_depth:.1f}m)...")
    sonar_max = 0.0
    for _ in range(args.warmup_ticks):
        env.act("auv0", target)
        s = env.tick()
        latest.update(s)
        if "ImagingSonar" in s:
            sonar_max = max(sonar_max, float(np.max(s["ImagingSonar"])))
    print(f"Warm-up complete. Sonar max return: {sonar_max:.4f}" +
          (" (OK, acoustic returns active)" if sonar_max > 0.02 else " (WARNING: no returns within RangeMax)"))

    # ---- auto-mode waypoints ----
    if args.mode == "auto":
        # Re-check actual position after warm-up settle
        T = s["PoseSensor"]
        settled_xyz = T[:3, 3].copy()
        settled_yaw = float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))
        waypoints = make_survey_waypoints(
            settled_xyz, settled_yaw,
            length=args.survey_length, width=args.survey_width,
            laps=args.laps, depth_step=args.depth_step,
        )
        wp_idx = 0
        auto_save_min_dt = 1.0 / max(args.auto_save_hz, 0.1)
        last_auto_save_t = -auto_save_min_dt
        print(f"Generated {len(waypoints)} survey waypoints.")

    # ---- main loop ----
    running = True
    while running:
        if show and (not plt.fignum_exists(fig.number) or "x" in held):
            break

        T = s["PoseSensor"]
        cur = T[:3, 3].copy()
        cur_yaw = float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))
        cur_depth = extract_depth_scalar(T)

        # ---- target computation ----
        if args.mode == "manual":
            fwd  = (("w" in held) - ("s" in held)) * SPEED * DT
            side = (("q" in held) - ("e" in held)) * SPEED * DT
            up   = (("r" in held) - ("f" in held)) * SPEED * DT
            dyaw = (("a" in held) - ("d" in held)) * TURN * DT
            h = np.radians(target[5])
            target[0] += fwd * np.cos(h) - side * np.sin(h)
            target[1] += fwd * np.sin(h) + side * np.cos(h)
            target[2] += up
            target[5] += dyaw

            if " " in held:
                target[:3], target[5] = cur, cur_yaw
            off = target[:3] - cur
            n = np.linalg.norm(off)
            if n > LEASH:
                target[:3] = cur + off * (LEASH / n)
            target[5] = wrap(cur_yaw + np.clip(wrap(target[5] - cur_yaw), -YAW_LEASH, YAW_LEASH))

        elif args.mode == "auto":
            if wp_idx < len(waypoints):
                wp = waypoints[wp_idx]
                delta = wp[:3] - cur
                dist = np.linalg.norm(delta)

                # Check if waypoint reached
                if dist < 0.8:
                    print(f"Reached waypoint {wp_idx + 1}/{len(waypoints)} (dist={dist:.2f}m, depth={cur_depth:.2f}m)")
                    wp_idx += 1
                    if wp_idx >= len(waypoints):
                        print("All survey waypoints completed!")
                        save_frame()
                        running = False
                        continue
                    wp = waypoints[wp_idx]
                    delta = wp[:3] - cur
                    dist = np.linalg.norm(delta)

                # Place controller target LEASH ahead along the vector to waypoint
                lead = min(LEASH, dist)
                direction = delta / max(dist, 1e-6)
                target[:3] = cur + direction * lead
                target[5] = wp[5]
            else:
                running = False
                continue

        env.act("auv0", target)
        s = env.tick()
        tick += 1

        # ---- update bookkeeping ----
        latest.update(s)
        latest["t"] = tick * DT
        pose = np.asarray(s["PoseSensor"], dtype=np.float64).copy()
        vel = np.asarray(s.get("VelocitySensor", np.full(3, np.nan)), dtype=np.float64).copy()

        trajectory["t"].append(tick * DT)
        trajectory["pose"].append(pose)
        trajectory["vel"].append(vel)
        trajectory["target"].append(np.asarray(target, dtype=np.float64).copy())

        for k in ("FrontCamera", "DepthCam", "ImagingSonar"):
            if k in s:
                arrived[k] = tick

        # ---- decide when to record frame ----
        should_save = False
        t_now = tick * DT

        if args.mode == "manual":
            if rec["snap"]:
                should_save = True
                rec["snap"] = False
            elif rec["on"] and "ImagingSonar" in s and all(k in latest for k in REQUIRED_SENSORS):
                if moved_enough(cur, cur_yaw):
                    should_save = True

        elif args.mode == "auto":
            # Save when a sonar frame arrives AND (moved enough OR minimum time elapsed)
            if "ImagingSonar" in s and all(k in latest for k in REQUIRED_SENSORS):
                if moved_enough(cur, cur_yaw) or (t_now - last_auto_save_t >= auto_save_min_dt):
                    should_save = True

        if should_save:
            if save_frame():
                last_save_pos = cur.copy()
                last_save_yaw = cur_yaw
                last_auto_save_t = t_now
                if n_saved % 10 == 0 or n_saved == 1:
                    print(f"  [Frame {n_saved:4d}] t={t_now:6.1f}s | depth={cur_depth:.2f}m | "
                          f"pos=({cur[0]:.1f}, {cur[1]:.1f}, {cur[2]:.1f}) | yaw={cur_yaw:5.1f}° | "
                          f"vel={np.linalg.norm(vel):.2f}m/s")

        # ---- live display ----
        if show:
            dirty = False
            if "FrontCamera" in s:
                cam_im.set_data(s["FrontCamera"][..., 2::-1])
                dirty = True
            if "DepthCam" in s:
                dep_im.set_data(np.ma.masked_greater(s["DepthCam"]["depth"], 50))
                dirty = True
            if "ImagingSonar" in s:
                son_im.set_data(np.where(fan_valid, np.sqrt(s["ImagingSonar"][fan_ri, fan_ai]), 0))
                dirty = True
            if dirty:
                v_mag = np.linalg.norm(vel)
                wp_txt = f"WP {wp_idx+1}/{len(waypoints)} " if args.mode == "auto" and wp_idx < len(waypoints) else ""
                ax[0].set_title(
                    f"{'[REC] ' if rec['on'] else ''}frames={n_saved}  depth={cur_depth:.2f}m\n"
                    f"{wp_txt}vel={v_mag:.2f}m/s  yaw={cur_yaw:.0f}°"
                )
                plt.pause(0.001)
            else:
                fig.canvas.flush_events()

# ========================== finalise ==========================
save_all()
print("=" * 70)
print(f"Done! {n_saved} synchronized frames recorded in {RUN}/")
print(f"Vehicle operated at depth: {args.depth:.1f} m")
print(f"Data files available for sensor fusion training:")
print(f"  - Combined:    {RUN}/frame_000000.npz")
print(f"  - RGB images:  {RUN}/rgb/frame_*.npy")
print(f"  - Sonar maps:  {RUN}/sonar/frame_*.npy")
print(f"  - Frame index: {RUN}/frames.csv")
print(f"  - Full path:   {RUN}/trajectory.npz")
print("=" * 70)
