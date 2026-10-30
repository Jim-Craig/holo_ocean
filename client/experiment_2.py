import os, json, time
import numpy as np
import matplotlib.pyplot as plt
import holoocean

# ---------------- scenario ----------------
scenario = holoocean.packagemanager.get_scenario("PierHarbor-Hovering")
scenario["package_name"] = "Ocean"
scenario["ticks_per_sec"] = 60
agent = scenario["agents"][0]
agent["sensors"] = [sd for sd in agent["sensors"]
                    if sd.get("sensor_name") not in ("FrontCamera", "DepthCam", "ImagingSonar")]
TILT = 40                                   # degrees below horizontal, same for camera and sonar
CAM_POSE = {"location": [1.0, 0, 0.3], "rotation": [0, TILT, 0]}
agent["sensors"] += [
    {"sensor_name": "FrontCamera", "sensor_type": "RGBCamera", "Hz": 10, **CAM_POSE,
     "configuration": {"CaptureWidth": 640, "CaptureHeight": 480}},
    {"sensor_name": "DepthCam", "sensor_type": "DepthCamera", "Hz": 10, **CAM_POSE,
     "configuration": {"CaptureWidth": 640, "CaptureHeight": 480, "FovAngle": 90}},
    {"sensor_name": "ImagingSonar", "sensor_type": "ImagingSonar", "Hz": 10,
     "socket": "SonarSocket", "rotation": [0, TILT, 0],
     "configuration": {"RangeBins": 512, "AzimuthBins": 512, "RangeMin": 0.5, "RangeMax": 20,
                       "InitOctreeRange": 30, "Azimuth": 120, "Elevation": 20,
                       "ViewRegion": False}},          # no ViewOctree key: default is off
]

# ---------------- settings ----------------
SPEED, TURN = 1.0, 45.0        # m/s and deg/s of target motion
LEASH, YAW_LEASH = 1.5, 45.0   # how far the target may run ahead of the vehicle (m, deg)
NEAR_AT_ROW0 = True            # set False if the sonar fan looks flipped front-to-back
R_MIN, R_MAX, AZ_DEG = 0.5, 20.0, 120.0
DEPTH_SHOW_MAX = 30.0          # depth panel colour range (m); pixels beyond 50 m are masked
DT = 1 / 60

# ---------------- sonar fan lookup ----------------
def fan_map(n_r, n_az, size=400):
    half = np.deg2rad(AZ_DEG) / 2
    X, Y = np.meshgrid(np.linspace(-R_MAX*np.sin(half), R_MAX*np.sin(half), size),
                       np.linspace(R_MAX, 0, size))
    R, TH = np.hypot(X, Y), np.arctan2(X, Y)
    ri = (R - R_MIN) / (R_MAX - R_MIN) * (n_r - 1)
    ai = (TH + half) / (2 * half) * (n_az - 1)
    valid = (ri >= 0) & (ri <= n_r - 1) & (ai >= 0) & (ai <= n_az - 1)
    return ri.round().astype(int).clip(0, n_r-1), ai.round().astype(int).clip(0, n_az-1), valid, half

ri, ai, valid, half = fan_map(512, 512)

# ---------------- recording setup ----------------
RUN = time.strftime("run_%Y%m%d_%H%M%S")
os.makedirs(RUN, exist_ok=True)
json.dump({"tilt_deg": TILT, "cam_pose": CAM_POSE, "fov_deg": 90, "image_size": [640, 480],
           "sonar": {"socket": "SonarSocket", "pitch_deg": TILT, "r_min": R_MIN, "r_max": R_MAX,
                     "azimuth_deg": AZ_DEG, "elevation_deg": 20, "bins": [512, 512]},
           "ticks_per_sec": 60, "depth_units": "m", "depth_model": "planar (assumed)"},
          open(f"{RUN}/meta.json", "w"), indent=2)

rec = {"on": False, "snap": False}
latest, arrived = {}, {}
n_saved, tick, n_sonar = 0, 0, 0

def save_sample():
    global n_saved
    np.savez_compressed(
        f"{RUN}/s{n_saved:05d}.npz",
        rgb=latest["FrontCamera"][..., 2::-1],                      # BGRA -> RGB
        depth=latest["DepthCam"]["depth"].astype(np.float32),
        sonar=latest["ImagingSonar"], pose=latest["PoseSensor"], vel=latest["VelocitySensor"],
        t=latest["t"],
        arrival_ticks=[arrived[k] for k in ("FrontCamera", "DepthCam", "ImagingSonar")])
    if n_saved == 0:
        print("arrival ticks (cam, depth, sonar):",
              [arrived[k] for k in ("FrontCamera", "DepthCam", "ImagingSonar")])
    n_saved += 1

# ---------------- display + keyboard ----------------
for k in list(plt.rcParams):                  # stop matplotlib's default hotkeys (s = save, q = quit, ...)
    if k.startswith("keymap."):
        plt.rcParams[k] = []
held = set()

def on_press(e):
    if not e.key:
        return
    k = e.key.lower()
    if k not in held:                          # ignore key auto-repeat for the one-shot keys
        if k == "l": rec["on"] = not rec["on"]    # toggle continuous recording (2 Hz)
        if k == "p": rec["snap"] = True           # save one sample
    held.add(k)

def on_release(e):
    if e.key:
        held.discard(e.key.lower())

plt.ion()
fig, ax = plt.subplots(1, 3, figsize=(17, 5.5))
fig.canvas.mpl_connect("key_press_event", on_press)
fig.canvas.mpl_connect("key_release_event", on_release)
cam_im = ax[0].imshow(np.zeros((480, 640, 3), np.uint8)); ax[0].axis("off")
dep_im = ax[1].imshow(np.zeros((480, 640)), cmap="viridis", vmin=0, vmax=DEPTH_SHOW_MAX)
ax[1].axis("off"); ax[1].set_title("Depth (m)")
son_im = ax[2].imshow(np.zeros(ri.shape), cmap="gray", vmin=0, vmax=0.5,
                      extent=[-R_MAX*np.sin(half), R_MAX*np.sin(half), 0, R_MAX])
ax[2].set_title("Imaging sonar (fan, metres)")

print("W/S forward/back | A/D turn left/right | Q/E strafe left/right | R/F up/down | SPACE hold position")
print("L toggle recording (2 Hz) | P save one sample | X quit")
print("Click the matplotlib window so it has keyboard focus.")
print("Sensors in scenario:", [sd.get("sensor_name", sd["sensor_type"]) for sd in agent["sensors"]])
print("ticks_per_sec:", scenario["ticks_per_sec"], "| saving to:", RUN)

def wrap(a):
    return (a + 180) % 360 - 180

# ---------------- main ----------------
with holoocean.make(scenario_cfg=scenario) as env:
    env.set_control_scheme("auv0", 1)                 # PD controller: action = [x, y, z, roll, pitch, yaw]
    s = env.tick()
    T = s["PoseSensor"]
    target = np.r_[T[:3, 3], 0.0, 0.0, np.degrees(np.arctan2(T[1, 0], T[0, 0]))]
    for _ in range(300):                              # settle (and let the octree build) while holding the start pose
        env.act("auv0", target)
        s = env.tick()

    while plt.fignum_exists(fig.number) and "x" not in held:
        T = s["PoseSensor"]
        cur, cur_yaw = T[:3, 3], np.degrees(np.arctan2(T[1, 0], T[0, 0]))

        # keys -> target pose, moved in the vehicle's heading frame
        fwd  = (("w" in held) - ("s" in held)) * SPEED * DT
        side = (("q" in held) - ("e" in held)) * SPEED * DT     # + = left
        up   = (("r" in held) - ("f" in held)) * SPEED * DT
        dyaw = (("a" in held) - ("d" in held)) * TURN * DT      # + = counter-clockwise
        h = np.radians(target[5])
        target[0] += fwd*np.cos(h) - side*np.sin(h)
        target[1] += fwd*np.sin(h) + side*np.cos(h)
        target[2] += up
        target[5] += dyaw

        if " " in held:                                # hold position
            target[:3], target[5] = cur, cur_yaw
        off = target[:3] - cur                         # leash: the target can't run away from the vehicle
        n = np.linalg.norm(off)
        if n > LEASH:
            target[:3] = cur + off * (LEASH / n)
        target[5] = wrap(cur_yaw + np.clip(wrap(target[5] - cur_yaw), -YAW_LEASH, YAW_LEASH))

        env.act("auv0", target)
        s = env.tick()

        # ---- bookkeeping + recording (a sample is saved whenever a sonar frame arrives) ----
        tick += 1
        latest.update(s)
        for k in ("FrontCamera", "DepthCam", "ImagingSonar"):
            if k in s:
                arrived[k] = tick
        if "ImagingSonar" in s and all(k in latest for k in ("FrontCamera", "DepthCam")):
            n_sonar += 1
            if rec["snap"] or (rec["on"] and n_sonar % 5 == 0):
                save_sample()
                rec["snap"] = False

        # ---- display ----
        dirty = False
        if "FrontCamera" in s:
            cam_im.set_data(s["FrontCamera"][..., 2::-1]); dirty = True
        if "DepthCam" in s:
            dep_im.set_data(np.ma.masked_greater(s["DepthCam"]["depth"], 50)); dirty = True
        if "ImagingSonar" in s:
            son = s["ImagingSonar"] if NEAR_AT_ROW0 else s["ImagingSonar"][::-1]
            son_im.set_data(np.where(valid, son[ri, ai], 0)); dirty = True
        if dirty:
            p = s["PoseSensor"][:3, 3]
            ax[0].set_title(f"{'[REC] ' if rec['on'] else ''}saved={n_saved}  "
                            f"x={p[0]:.1f} y={p[1]:.1f} z={p[2]:.1f} yaw={cur_yaw:.0f}°")
            plt.pause(0.001)
        else:
            fig.canvas.flush_events()

print(f"Done. {n_saved} samples saved in {RUN}/")