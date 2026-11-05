"""Record multi-modal sensor fusion data for specialized underwater survey sessions
with environmental variations (turbidity, scattering, absorption, illumination, particles).

Sessions
--------
  1. pier     : Pier close-up - Keeps the pier centered in view at 3-4 m altitude, sweeps both sides.
  2. seabed   : Seabed only   - Away from the pier, 2-5 m altitude, multi-directional grid.
  3. kelp     : Kelp canopy   - Navigates through the kelp/seaweed vegetation along the seabed.
  4. mixed    : Mixed         - Transitions between pier columns, kelp bed, and open seabed.
  5. manual   : Manual WASD   - Free flight exploration.

Environmental Variations
------------------------
  --turbidity     : 'clean', 'moderate', 'turbid', 'murky' (controls underwater fog density/scattering)
  --lighting      : 'noon', 'morning', 'dusk', 'night' (controls sun angle, caustics, and AUV flashlights)
  --water-color   : 'coastal' (greenish), 'harbor' (sediment/brown), 'deep_blue', 'clear' (absorption)
  --weather       : 'sunny', 'cloudy', 'rainy' (ambient lighting & surface disturbance)
  --particles     : Adds physical optical attenuation and suspended marine snow flocs to camera frames

Usage Examples
--------------
  python record_sessions.py --session pier
  python record_sessions.py --session seabed --altitude 3.0 --turbidity moderate
  python record_sessions.py --session kelp --lighting morning --particles
  python record_sessions.py --session mixed --turbidity turbid --weather cloudy
  python record_sessions.py --session manual --lighting night
  python record_sessions.py --session pier --no-display   # fast headless recording
"""

import os, sys, json, time, argparse
import numpy as np
import matplotlib.pyplot as plt
import holoocean
import rig


# ==============================================================================
# CLI Arguments
# ==============================================================================
ap = argparse.ArgumentParser(description="Record specialized underwater survey sessions with environmental variations")
ap.add_argument("note", nargs="?", default="", help="Optional free-text note stored in meta.json")

# Session selection
ap.add_argument("--session", choices=["pier", "seabed", "kelp", "mixed", "manual"], default="pier",
                help="Survey session to execute (default: pier)")
ap.add_argument("--altitude", type=float, default=3.5,
                help="Target altitude above seabed in metres (default: 3.5m; typical 2.5-4.5m)")
ap.add_argument("--laps", type=int, default=1,
                help="Number of times to repeat the session route (default: 1)")

# Environmental Variations
ap.add_argument("--turbidity", choices=["clean", "moderate", "turbid", "murky"], default="clean",
                help="Water turbidity / scattering intensity (default: clean)")
ap.add_argument("--lighting", choices=["noon", "morning", "dusk", "night"], default="noon",
                help="Time of day / illumination condition (default: noon)")
ap.add_argument("--water-color", choices=["coastal", "harbor", "deep_blue", "clear"], default="coastal",
                help="Water body spectral color / light absorption (default: coastal)")
ap.add_argument("--weather", choices=["sunny", "cloudy", "rainy"], default="sunny",
                help="Weather condition (default: sunny)")
ap.add_argument("--particles", action="store_true",
                help="Simulate suspended marine snow particles and optical light attenuation on camera")

# Sensor & Recording Settings
ap.add_argument("--sonar-range", type=float, default=30.0,
                help="Sonar RangeMax in metres (default: 30.0m to guarantee returns at 3-4m altitude)")
ap.add_argument("--save-interval", type=float, default=0.5,
                help="Minimum vehicle travel distance (m) between saved frames (default: 0.5)")
ap.add_argument("--save-interval-deg", type=float, default=10.0,
                help="Minimum yaw change (deg) between saved frames (default: 10.0)")
ap.add_argument("--auto-save-hz", type=float, default=2.0,
                help="Max frame save rate in Hz for automated sessions (default: 2.0)")
ap.add_argument("--warmup-ticks", type=int, default=180,
                help="Ticks to settle / build octree before recording (default: 180)")
ap.add_argument("--no-display", action="store_true",
                help="Run headless without Matplotlib live display")
args = ap.parse_args()


# ==============================================================================
# Environmental Presets
# ==============================================================================
TURBIDITY_PRESETS = {
    "clean":    {"density": 0.2, "depth": 6.0, "color": (0.35, 0.55, 0.85)},
    "moderate": {"density": 1.0, "depth": 3.0, "color": (0.25, 0.55, 0.50)},
    "turbid":   {"density": 2.2, "depth": 1.5, "color": (0.20, 0.45, 0.35)},
    "murky":    {"density": 3.5, "depth": 0.8, "color": (0.30, 0.40, 0.25)},
}

LIGHTING_PRESETS = {
    "noon":    {"hour": 12.0, "flashlight": False},
    "morning": {"hour": 8.5,  "flashlight": False},
    "dusk":    {"hour": 18.0, "flashlight": False},
    "night":   {"hour": 22.0, "flashlight": True},
}

WATER_COLOR_PRESETS = {
    "coastal":   (0.08, 0.35, 0.30),
    "harbor":    (0.25, 0.35, 0.20),
    "deep_blue": (0.02, 0.15, 0.45),
    "clear":     (0.05, 0.25, 0.50),
}

WEATHER_PRESETS = {
    "sunny":  0,
    "cloudy": 1,
    "rainy":  2,
}


def apply_environmental_conditions(env):
    """Apply visual and physical environmental conditions to the HoloOcean world."""
    turb = TURBIDITY_PRESETS[args.turbidity]
    light = LIGHTING_PRESETS[args.lighting]
    wcolor = WATER_COLOR_PRESETS[args.water_color]
    weather_code = WEATHER_PRESETS[args.weather]

    # Weather
    env.change_weather(weather_code)
    if args.weather == "rainy":
        env.set_rain_parameters(0, 300, -1000, 3000)

    # Time of Day / Illumination
    env.change_time_of_day(light["hour"])

    # Water Fog (Turbidity & Scattering)
    env.water_fog(
        turb["density"],
        fogDepth=turb["depth"],
        color_R=turb["color"][0],
        color_G=turb["color"][1],
        color_B=turb["color"][2],
    )

    # Water Body Spectral Color (Absorption)
    env.water_color(wcolor[0], wcolor[1], wcolor[2])

    # AUV Flashlights (Active illumination in dark / night conditions)
    if light["flashlight"]:
        env.turn_on_flashlight("flashlight1", intensity=6000, beam_width=55)
        env.turn_on_flashlight("flashlight2", intensity=6000, beam_width=55)
    else:
        try:
            env.turn_off_flashlight("flashlight1")
            env.turn_off_flashlight("flashlight2")
        except Exception:
            pass


# ==============================================================================
# Suspended Particle & Optical Attenuation Model (Optional Post-Processing)
# ==============================================================================
def apply_marine_optical_effects(rgb, depth_img, turbidity_level="clean"):
    """Simulate physical light absorption and suspended marine snow particles."""
    if not args.particles:
        return rgb

    h, w, _ = rgb.shape
    d = np.clip(depth_img, 0.1, 35.0)

    # Wavelength-dependent attenuation coefficients (Red absorbs faster than Blue/Green)
    beta = {
        "clean":    np.array([0.08, 0.03, 0.02]),
        "moderate": np.array([0.15, 0.06, 0.04]),
        "turbid":   np.array([0.25, 0.10, 0.08]),
        "murky":    np.array([0.35, 0.16, 0.12]),
    }[turbidity_level]

    # Direct transmission: J * exp(-beta * d)
    transmission = np.exp(-d[..., None] * beta[None, None, :])
    water_ambient = np.array(TURBIDITY_PRESETS[turbidity_level]["color"]) * 255.0
    degraded = rgb.astype(np.float32) * transmission + water_ambient * (1.0 - transmission)

    # Suspended marine snow particles
    n_particles = {"clean": 120, "moderate": 400, "turbid": 900, "murky": 1500}[turbidity_level]
    py = np.random.randint(0, h, size=n_particles)
    px = np.random.randint(0, w, size=n_particles)
    p_sizes = np.random.randint(1, 4, size=n_particles)
    p_intensities = np.random.uniform(80, 220, size=n_particles)

    for y, x, s, val in zip(py, px, p_sizes, p_intensities):
        y0, y1 = max(0, y - s), min(h, y + s + 1)
        x0, x1 = max(0, x - s), min(w, x + s + 1)
        degraded[y0:y1, x0:x1] += val * 0.45

    return np.clip(degraded, 0, 255).astype(np.uint8)


# ==============================================================================
# Scenario & Rig Setup
# ==============================================================================
rig.SONAR["RangeMax"] = float(args.sonar_range)
scenario = holoocean.packagemanager.get_scenario(rig.SCENARIO)
scenario["package_name"] = "Ocean"
agent = rig.apply(scenario)

# In PierHarbor, seabed near the pier dock sits at z ≈ -19.1m.
# Desired altitude h means z = -19.1 + h.
# For altitude = 3.5m, initial z ≈ -15.6m.
SEABED_DATUM_Z = -19.1
desired_start_z = SEABED_DATUM_Z + args.altitude

# Starting positions for each session
SESSION_START_POSES = {
    "pier":   ([486.0, -632.0, desired_start_z], 135.0),
    "seabed": ([486.0, -665.0, desired_start_z], 90.0),
    "kelp":   ([495.0, -642.0, desired_start_z], 150.0),
    "mixed":  ([486.0, -632.0, desired_start_z], 135.0),
    "manual": ([486.0, -632.0, desired_start_z], 135.0),
}

start_pos, start_yaw = SESSION_START_POSES[args.session]
agent["location"] = list(start_pos)
agent["rotation"] = [0.0, 0.0, start_yaw]

# Constants
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


# ==============================================================================
# Session Trajectory Generators
# ==============================================================================
def make_pier_closeup_waypoints(start_xyz, alt_z, laps=1):
    """Session 1: Pier close-up.

    Keeps the pier in the middle of the camera and sonar view at 3-4m altitude.
    Traverses Side A (East face), rounds the pier end, and sweeps Side B (West face).
    """
    waypoints = []
    x0, y0, _ = start_xyz
    z = alt_z

    for _ in range(laps):
        # --- Side A: East face of pier, looking West-Southwest at the columns ---
        waypoints.append(np.array([484.0, -630.0, z, 0.0, 0.0, 135.0]))
        waypoints.append(np.array([480.0, -626.0, z, 0.0, 0.0, 135.0]))
        waypoints.append(np.array([476.0, -622.0, z, 0.0, 0.0, 135.0]))

        # --- Round the pier end ---
        waypoints.append(np.array([473.0, -621.0, z, 0.0, 0.0, 45.0]))
        waypoints.append(np.array([475.0, -618.0, z, 0.0, 0.0, 0.0]))
        waypoints.append(np.array([479.0, -617.0, z, 0.0, 0.0, -45.0]))

        # --- Side B: West face of pier, looking East-Northeast back along columns ---
        waypoints.append(np.array([482.0, -620.0, z, 0.0, 0.0, -45.0]))
        waypoints.append(np.array([486.0, -624.0, z, 0.0, 0.0, -45.0]))
        waypoints.append(np.array([490.0, -628.0, z, 0.0, 0.0, -45.0]))

        # --- Round back to starting face ---
        waypoints.append(np.array([489.0, -632.0, z, 0.0, 0.0, 135.0]))

    return waypoints


def make_seabed_only_waypoints(start_xyz, alt_z, laps=1):
    """Session 2: Seabed only.

    Away from the pier (y <= -660m), 2-5m altitude, multi-directional grid.
    Observes varying bathymetry, sediment textures, and slopes without dock obstacles.
    """
    waypoints = []
    z = alt_z

    for lap in range(laps):
        # Multi-directional grid over open seabed
        # Direction 1: South-East track
        waypoints.append(np.array([486.0, -665.0, z, 0.0, 0.0, 90.0]))
        waypoints.append(np.array([486.0, -678.0, z, 0.0, 0.0, 90.0]))

        # Direction 2: Cross-track East
        waypoints.append(np.array([494.0, -678.0, z + 0.5, 0.0, 0.0, 0.0]))

        # Direction 3: North-West reverse track
        waypoints.append(np.array([494.0, -665.0, z + 0.5, 0.0, 0.0, -90.0]))

        # Direction 4: Diagonal survey line
        waypoints.append(np.array([502.0, -675.0, z - 0.5, 0.0, 0.0, 45.0]))
        waypoints.append(np.array([490.0, -665.0, z, 0.0, 0.0, -135.0]))

    return waypoints


def make_kelp_waypoints(start_xyz, alt_z, laps=1):
    """Session 3: Kelp.

    Moves through the dense seaweed/kelp beds along the underwater slopes.
    Operates at 2.5-3.5m altitude through organic canopy and anchorage zones.
    """
    waypoints = []
    z = alt_z

    for _ in range(laps):
        # Navigates the kelp canopy corridor
        waypoints.append(np.array([495.0, -642.0, z, 0.0, 0.0, 150.0]))
        waypoints.append(np.array([498.0, -636.0, z - 0.4, 0.0, 0.0, 120.0]))
        waypoints.append(np.array([503.0, -631.0, z - 0.4, 0.0, 0.0, 90.0]))
        waypoints.append(np.array([507.0, -635.0, z, 0.0, 0.0, -30.0]))
        waypoints.append(np.array([503.0, -642.0, z + 0.3, 0.0, 0.0, -90.0]))
        waypoints.append(np.array([496.0, -645.0, z, 0.0, 0.0, -150.0]))

    return waypoints


def make_mixed_waypoints(start_xyz, alt_z, laps=1):
    """Session 4: Mixed.

    Smoothly transitions between all three conditions:
    Pier columns -> Kelp vegetation -> Open seabed -> Transition back.
    """
    waypoints = []
    z = alt_z

    for _ in range(laps):
        # Stage 1: Pier column sweep
        waypoints.append(np.array([486.0, -632.0, z, 0.0, 0.0, 135.0]))
        waypoints.append(np.array([480.0, -626.0, z, 0.0, 0.0, 135.0]))

        # Stage 2: Turn and transition into the Kelp bed
        waypoints.append(np.array([490.0, -630.0, z, 0.0, 0.0, 45.0]))
        waypoints.append(np.array([498.0, -635.0, z - 0.4, 0.0, 0.0, 90.0]))
        waypoints.append(np.array([504.0, -642.0, z - 0.4, 0.0, 0.0, 120.0]))

        # Stage 3: Transition out into Open Seabed
        waypoints.append(np.array([500.0, -655.0, z, 0.0, 0.0, 90.0]))
        waypoints.append(np.array([490.0, -670.0, z + 0.5, 0.0, 0.0, 90.0]))

        # Stage 4: Return circuit back to pier boundary
        waypoints.append(np.array([482.0, -655.0, z, 0.0, 0.0, -90.0]))
        waypoints.append(np.array([486.0, -635.0, z, 0.0, 0.0, -45.0]))

    return waypoints


# ==============================================================================
# Sonar Fan Map (Matplotlib Visualization)
# ==============================================================================
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


# ==============================================================================
# Output Directories & Containers
# ==============================================================================
RUN = time.strftime(f"session_{args.session}_%Y%m%d_%H%M%S")
os.makedirs(RUN, exist_ok=True)
os.makedirs(f"{RUN}/rgb", exist_ok=True)
os.makedirs(f"{RUN}/sonar", exist_ok=True)

frames = []
trajectory = {"t": [], "pose": [], "vel": [], "target": []}
latest = {}
arrived = {}
n_saved = 0
tick = 0


def extract_depth_and_altitude(pose_matrix, dvl_reading, depth_img):
    """Calculate depth below surface and altitude above seabed."""
    vehicle_depth = float(-pose_matrix[2, 3])

    altitude = float("nan")
    # Priority 1: DVL range beams
    if dvl_reading is not None and len(dvl_reading) >= 7:
        beams = dvl_reading[3:7]
        valid_b = beams[~np.isnan(beams)]
        if len(valid_b):
            altitude = float(np.mean(valid_b) * np.cos(np.radians(22.5)))

    # Priority 2: Inferred from seabed datum
    if np.isnan(altitude):
        altitude = float(pose_matrix[2, 3] - SEABED_DATUM_Z)

    return vehicle_depth, altitude


def save_frame():
    """Save a synchronised multi-modal data frame."""
    global n_saved
    missing = [k for k in REQUIRED_SENSORS if k not in latest]
    if missing:
        return False

    idx = n_saved
    pose = np.asarray(latest["PoseSensor"], dtype=np.float64).copy()
    depth_img = latest["DepthCam"]["depth"].astype(np.float32).copy()
    sonar = np.asarray(latest["ImagingSonar"], dtype=np.float32).copy()
    vel = np.asarray(latest.get("VelocitySensor", np.full(3, np.nan)), dtype=np.float64).copy()
    dvl = latest.get("DVLSensor")

    vehicle_depth, altitude = extract_depth_and_altitude(pose, dvl, depth_img)

    # RGB: BGRA -> RGB + optional physical particle/attenuation effects
    rgb_raw = latest["FrontCamera"][..., 2::-1].copy()
    rgb = apply_marine_optical_effects(rgb_raw, depth_img, args.turbidity)

    # Save per-modality arrays
    np.save(f"{RUN}/rgb/frame_{idx:06d}.npy", rgb)
    np.save(f"{RUN}/sonar/frame_{idx:06d}.npy", sonar)

    valid_d = depth_img[depth_img < 50]
    frames.append({
        "index": idx,
        "t": tick * DT,
        "session": args.session,
        "depth_m": vehicle_depth,
        "altitude_m": altitude,
        "pose": pose.tolist(),
        "velocity": vel.tolist(),
        "depth_image_mean": float(np.nanmean(valid_d)) if len(valid_d) else float("nan"),
        "env_turbidity": args.turbidity,
        "env_lighting": args.lighting,
        "env_weather": args.weather,
    })

    # Combined bundle (.npz)
    np.savez_compressed(
        f"{RUN}/frame_{idx:06d}.npz",
        rgb=rgb,
        depth_image=depth_img,
        sonar=sonar,
        pose=pose,
        vehicle_depth=np.float32(vehicle_depth),
        altitude=np.float32(altitude),
        velocity=vel,
        t=np.float64(tick * DT),
        env_state={
            "session": args.session,
            "turbidity": args.turbidity,
            "lighting": args.lighting,
            "water_color": args.water_color,
            "weather": args.weather,
            "particles": args.particles,
        },
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
            f.write("index,t,session,depth_m,altitude_m,x,y,z,turbidity,lighting\n")
            for fr in frames:
                p = fr["pose"]
                f.write(f"{fr['index']},{fr['t']:.4f},{fr['session']},"
                        f"{fr['depth_m']:.4f},{fr['altitude_m']:.4f},"
                        f"{p[0][3]:.4f},{p[1][3]:.4f},{p[2][3]:.4f},"
                        f"{fr['env_turbidity']},{fr['env_lighting']}\n")
        print(f"Saved {len(frames)} frames index -> {RUN}/frames.csv")


# Save meta.json
meta_out = rig.meta()
meta_out["note"] = args.note
meta_out["session"] = args.session
meta_out["target_altitude_m"] = args.altitude
meta_out["environment"] = {
    "turbidity": args.turbidity,
    "lighting": args.lighting,
    "water_color": args.water_color,
    "weather": args.weather,
    "particles": args.particles,
}
json.dump(meta_out, open(f"{RUN}/meta.json", "w"), indent=2)


# ==============================================================================
# Live Display Setup
# ==============================================================================
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
            if k == "l": rec["on"] = not rec["on"]
            if k == "p": rec["snap"] = True
        held.add(k)

    def on_release(e):
        if e.key: held.discard(e.key.lower())

    plt.ion()
    fig, ax = plt.subplots(1, 3, figsize=(17, 5.5))
    fig.canvas.mpl_connect("key_press_event", on_press)
    fig.canvas.mpl_connect("key_release_event", on_release)
    cam_im = ax[0].imshow(np.zeros((rig.H, rig.W, 3), np.uint8))
    ax[0].axis("off")
    dep_im = ax[1].imshow(np.zeros((rig.H, rig.W)), cmap="viridis", vmin=0, vmax=30)
    ax[1].axis("off"); ax[1].set_title("Depth Map (m)")
    son_im = ax[2].imshow(
        np.zeros(fan_ri.shape), cmap="gray", vmin=0, vmax=0.7,
        extent=[-R_MAX * np.sin(fan_half), R_MAX * np.sin(fan_half), 0, R_MAX],
    )
    ax[2].set_title(f"Imaging Sonar (fan, 0-{R_MAX:.0f} m)")
else:
    held = set()
    rec = {"on": True, "snap": False}
    fig = None


last_save_pos = None
last_save_yaw = None

def moved_enough(pos, yaw):
    if last_save_pos is None:
        return True
    return (np.linalg.norm(pos - last_save_pos) >= args.save_interval
            or abs(wrap(yaw - last_save_yaw)) >= args.save_interval_deg)


# ==============================================================================
# MAIN EXECUTION LOOP
# ==============================================================================
print("=" * 75)
print(f"SESSION       : {args.session.upper()}")
print(f"Altitude      : {args.altitude:.1f} m (start z = {desired_start_z:.1f} m)")
print(f"Environment   : Turbidity={args.turbidity} | Lighting={args.lighting} | Weather={args.weather}")
print(f"Sonar Range   : {R_MAX:.1f} m | Particles={args.particles}")
print(f"Output Folder : {RUN}/")
print("=" * 75)

with holoocean.make(scenario_cfg=scenario, show_viewport=show) as env:
    env.set_control_scheme("auv0", 1)  # PD Controller
    apply_environmental_conditions(env)

    s = env.tick()
    T = s["PoseSensor"]
    start_xyz = T[:3, 3].copy()
    start_yaw_deg = float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))
    target = np.array([*start_xyz, 0.0, 0.0, start_yaw_deg])

    # Warm-up ticks
    print(f"Warming up ({args.warmup_ticks} ticks, settling environment and building sonar octree)...")
    sonar_max = 0.0
    for _ in range(args.warmup_ticks):
        env.act("auv0", target)
        s = env.tick()
        latest.update(s)
        if "ImagingSonar" in s:
            sonar_max = max(sonar_max, float(np.max(s["ImagingSonar"])))
    print(f"Warm-up complete. Sonar max return: {sonar_max:.4f}" +
          (" (OK, acoustic returns active)" if sonar_max > 0.02 else " (WARNING: low acoustic returns)"))

    # Waypoint generation based on selected session
    settled_xyz = s["PoseSensor"][:3, 3].copy()
    settled_alt_z = settled_xyz[2]

    if args.session == "pier":
        waypoints = make_pier_closeup_waypoints(settled_xyz, settled_alt_z, laps=args.laps)
        print(f"Pier close-up session: Generated {len(waypoints)} waypoints covering both sides of pier.")
    elif args.session == "seabed":
        waypoints = make_seabed_only_waypoints(settled_xyz, settled_alt_z, laps=args.laps)
        print(f"Seabed session: Generated {len(waypoints)} waypoints across open bathymetry.")
    elif args.session == "kelp":
        waypoints = make_kelp_waypoints(settled_xyz, settled_alt_z, laps=args.laps)
        print(f"Kelp session: Generated {len(waypoints)} waypoints traversing seaweed canopy.")
    elif args.session == "mixed":
        waypoints = make_mixed_waypoints(settled_xyz, settled_alt_z, laps=args.laps)
        print(f"Mixed session: Generated {len(waypoints)} waypoints transitioning pier -> kelp -> seabed.")
    else:
        waypoints = []
        print("Manual mode: Use WASD/QE/RF keys. L = toggle record, P = snap, X = quit.")

    wp_idx = 0
    auto_save_min_dt = 1.0 / max(args.auto_save_hz, 0.1)
    last_auto_save_t = -auto_save_min_dt
    running = True

    while running:
        if show and (not plt.fignum_exists(fig.number) or "x" in held):
            break

        T = s["PoseSensor"]
        cur = T[:3, 3].copy()
        cur_yaw = float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))
        cur_depth, cur_alt = extract_depth_and_altitude(T, s.get("DVLSensor"), latest.get("DepthCam", {}).get("depth"))

        # Target Navigation
        if args.session == "manual":
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

        else:
            # Automated Waypoint Controller
            if wp_idx < len(waypoints):
                wp = waypoints[wp_idx]
                delta = wp[:3] - cur
                dist = np.linalg.norm(delta)

                if dist < 0.85:
                    print(f"Reached waypoint {wp_idx + 1}/{len(waypoints)} "
                          f"(dist={dist:.2f}m, alt={cur_alt:.2f}m, depth={cur_depth:.2f}m)")
                    wp_idx += 1
                    if wp_idx >= len(waypoints):
                        print("Session trajectory complete!")
                        save_frame()
                        running = False
                        continue
                    wp = waypoints[wp_idx]
                    delta = wp[:3] - cur
                    dist = np.linalg.norm(delta)

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

        latest.update(s)
        latest["t"] = tick * DT
        pose = np.asarray(s["PoseSensor"], dtype=np.float64).copy()
        vel = np.asarray(s.get("VelocitySensor", np.full(3, np.nan)), dtype=np.float64).copy()

        trajectory["t"].append(tick * DT)
        trajectory["pose"].append(pose)
        trajectory["vel"].append(vel)
        trajectory["target"].append(np.asarray(target, dtype=np.float64).copy())

        # Decide Frame Recording
        should_save = False
        t_now = tick * DT

        if args.session == "manual":
            if rec["snap"]:
                should_save = True
                rec["snap"] = False
            elif rec["on"] and "ImagingSonar" in s and all(k in latest for k in REQUIRED_SENSORS):
                if moved_enough(cur, cur_yaw):
                    should_save = True
        else:
            if "ImagingSonar" in s and all(k in latest for k in REQUIRED_SENSORS):
                if moved_enough(cur, cur_yaw) or (t_now - last_auto_save_t >= auto_save_min_dt):
                    should_save = True

        if should_save:
            if save_frame():
                last_save_pos = cur.copy()
                last_save_yaw = cur_yaw
                last_auto_save_t = t_now
                if n_saved % 10 == 0 or n_saved == 1:
                    print(f"  [{args.session.upper()} Frame {n_saved:4d}] t={t_now:6.1f}s | "
                          f"depth={cur_depth:.2f}m | alt={cur_alt:.2f}m | yaw={cur_yaw:5.1f}° | "
                          f"vel={np.linalg.norm(vel):.2f}m/s")

        # Live Display
        if show:
            dirty = False
            if "FrontCamera" in s:
                raw_rgb = s["FrontCamera"][..., 2::-1]
                disp_rgb = apply_marine_optical_effects(raw_rgb, latest["DepthCam"]["depth"], args.turbidity) if args.particles else raw_rgb
                cam_im.set_data(disp_rgb)
                dirty = True
            if "DepthCam" in s:
                dep_im.set_data(np.ma.masked_greater(s["DepthCam"]["depth"], 50))
                dirty = True
            if "ImagingSonar" in s:
                son_im.set_data(np.where(fan_valid, np.sqrt(s["ImagingSonar"][fan_ri, fan_ai]), 0))
                dirty = True
            if dirty:
                v_mag = np.linalg.norm(vel)
                wp_txt = f"WP {wp_idx+1}/{len(waypoints)} " if waypoints and wp_idx < len(waypoints) else ""
                ax[0].set_title(
                    f"[{args.session.upper()}] frames={n_saved}  depth={cur_depth:.1f}m  alt={cur_alt:.1f}m\n"
                    f"{wp_txt}vel={v_mag:.2f}m/s  yaw={cur_yaw:.0f}°"
                )
                plt.pause(0.001)
            else:
                fig.canvas.flush_events()

# ==============================================================================
# Finalize & Summary
# ==============================================================================
save_all()
print("=" * 75)
print(f"DONE! {n_saved} multi-modal frames recorded in {RUN}/")
print(f"Session       : {args.session.upper()}")
print(f"Environment   : Turbidity={args.turbidity}, Lighting={args.lighting}, Weather={args.weather}")
print(f"Output files  :")
print(f"  - Frame bundles: {RUN}/frame_*.npz")
print(f"  - RGB images:    {RUN}/rgb/frame_*.npy")
print(f"  - Sonar arrays:  {RUN}/sonar/frame_*.npy")
print(f"  - Tabular index: {RUN}/frames.csv")
print(f"  - Trajectory:    {RUN}/trajectory.npz")
print("=" * 75)

