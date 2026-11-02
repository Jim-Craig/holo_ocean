import os, json, time
import numpy as np
import matplotlib.pyplot as plt
import holoocean
import rig                                   # sensor rig lives in rig.py

# ---------------- scenario ----------------
scenario = holoocean.packagemanager.get_scenario("PierHarbor-Hovering")
scenario["package_name"] = "Ocean"
agent = rig.apply(scenario)

# ---------------- settings ----------------
SPEED, TURN = 1.0, 45.0           # m/s and deg/s of target motion
LEASH, YAW_LEASH = 1.5, 45.0      # how far the target may run ahead of the vehicle (m, deg)
SAVE_EVERY_M, SAVE_EVERY_DEG = 0.5, 10.0     # while recording, save after moving or turning this much
START_RECORDING = False
FLIP_AZIMUTH = False              # set True if the sonar fan looks mirrored relative to the camera
R_MIN, R_MAX, AZ_DEG = rig.SONAR["RangeMin"], rig.SONAR["RangeMax"], rig.SONAR["Azimuth"]
N_R, N_AZ = rig.SONAR["RangeBins"], rig.SONAR["AzimuthBins"]
DT = 1 / rig.TICKS_PER_SEC
REQUIRED = ["FrontCamera", "DepthCam"] + (["SemCam"] if rig.WITH_SEMANTIC else [])

def wrap(a):
    return (a + 180) % 360 - 180

# ---------------- sonar fan lookup (row 0 = near range, confirmed) ----------------
def fan_map(size=400):
    half = np.deg2rad(AZ_DEG) / 2
    X, Y = np.meshgrid(np.linspace(-R_MAX*np.sin(half), R_MAX*np.sin(half), size),
                       np.linspace(R_MAX, 0, size))
    R, TH = np.hypot(X, Y), np.arctan2(X, Y)
    ri = (R - R_MIN) / (R_MAX - R_MIN) * (N_R - 1)
    ai = (TH + half) / (2 * half) * (N_AZ - 1)
    valid = (ri >= 0) & (ri <= N_R - 1) & (ai >= 0) & (ai <= N_AZ - 1)
    ri, ai = ri.round().astype(int).clip(0, N_R - 1), ai.round().astype(int).clip(0, N_AZ - 1)
    if FLIP_AZIMUTH:
        ai = N_AZ - 1 - ai
    return ri, ai, valid, half

ri, ai, valid, half = fan_map()

# ---------------- recording setup ----------------
RUN = time.strftime("run_%Y%m%d_%H%M%S")
os.makedirs(RUN, exist_ok=True)
json.dump(rig.meta(), open(f"{RUN}/meta.json", "w"), indent=2)

rec = {"on": START_RECORDING, "snap": False}
latest, arrived = {}, {}
last_saved = {"pos": None, "yaw": None}
n_saved, tick = 0, 0
sem_printed = False

def moved_enough(pos, yaw):
    if last_saved["pos"] is None:
        return True
    return (np.linalg.norm(pos - last_saved["pos"]) >= SAVE_EVERY_M
            or abs(wrap(yaw - last_saved["yaw"])) >= SAVE_EVERY_DEG)

def save_sample():
    global n_saved
    np.savez_compressed(
        f"{RUN}/s{n_saved:05d}.npz",
        rgb=latest["FrontCamera"][..., 2::-1],                      # BGRA -> RGB
        depth=latest["DepthCam"]["depth"].astype(np.float32),
        sonar=latest["ImagingSonar"], pose=latest["PoseSensor"], vel=latest["VelocitySensor"],
        dvl=latest.get("DVLSensor", np.full(7, np.nan)),
        **({"sem": latest["SemCam"]} if rig.WITH_SEMANTIC else {}),
        t=latest["t"],
        arrival_ticks=[arrived[k] for k in ("FrontCamera", "DepthCam", "ImagingSonar")])
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
        if k == "l": rec["on"] = not rec["on"]    # toggle recording (saves after SAVE_EVERY_M / SAVE_EVERY_DEG)
        if k == "p": rec["snap"] = True           # save one sample now
    held.add(k)

def on_release(e):
    if e.key:
        held.discard(e.key.lower())

plt.ion()
fig, ax = plt.subplots(1, 3, figsize=(17, 5.5))
fig.canvas.mpl_connect("key_press_event", on_press)
fig.canvas.mpl_connect("key_release_event", on_release)
cam_im = ax[0].imshow(np.zeros((rig.H, rig.W, 3), np.uint8)); ax[0].axis("off")
dep_im = ax[1].imshow(np.zeros((rig.H, rig.W)), cmap="viridis", vmin=0, vmax=30)
ax[1].axis("off"); ax[1].set_title("Depth (m)")
son_im = ax[2].imshow(np.zeros(ri.shape), cmap="gray", vmin=0, vmax=0.7,
                      extent=[-R_MAX*np.sin(half), R_MAX*np.sin(half), 0, R_MAX])
ax[2].set_title("Imaging sonar (fan, metres)")

print("W/S forward/back | A/D turn left/right | Q/E strafe left/right | R/F up/down | SPACE hold position")
print("L toggle recording | P save one sample | X quit.   Click the matplotlib window for keyboard focus.")
print("Sensors:", [sd.get("sensor_name", sd["sensor_type"]) for sd in agent["sensors"]])
print(f"Camera/sonar tilt {rig.TILT_DEG:.0f} deg. Seabed return band (slant range at fan centre) by altitude:")
for h in (1.5, 2, 3, 4, 6):
    near, far = rig.seabed_band(h)
    print(f"   altitude {h:>3} m -> {near:5.1f} .. {far:5.1f} m" + ("   (beyond RangeMax)" if far > R_MAX else ""))
print("Saving to:", RUN)

# ---------------- main ----------------
with holoocean.make(scenario_cfg=scenario) as env:
    env.set_control_scheme("auv0", 1)                 # PD controller: action = [x, y, z, roll, pitch, yaw]
    s = env.tick()
    T = s["PoseSensor"]
    target = np.r_[T[:3, 3], 0.0, 0.0, np.degrees(np.arctan2(T[1, 0], T[0, 0]))]
    sonar_max = 0.0
    for _ in range(300):                              # settle (and let the octree build) holding the start pose
        env.act("auv0", target)
        s = env.tick()
        if "ImagingSonar" in s:
            sonar_max = max(sonar_max, float(np.max(s["ImagingSonar"])))
    print(f"sonar max during warm-up: {sonar_max:.3f}")
    if sonar_max == 0:
        print("  WARNING: the sonar returned nothing. Possible causes: still building the octree (fly around),"
              " altitude outside the band above, or set rig.USE_SONAR_SOCKET = True.")

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

        # ---- bookkeeping + recording (checked whenever a sonar frame arrives) ----
        tick += 1
        latest.update(s)
        for k in ("FrontCamera", "DepthCam", "ImagingSonar", "SemCam"):
            if k in s:
                arrived[k] = tick
        if rig.WITH_SEMANTIC and "SemCam" in s and not sem_printed:
            sem = np.asarray(s["SemCam"])
            flat = sem.reshape(-1, sem.shape[-1]) if sem.ndim == 3 else sem.reshape(-1, 1)
            uniq = np.unique(flat, axis=0)
            print(f"SemCam: shape {sem.shape}, dtype {sem.dtype}, {len(uniq)} distinct values; first few: {uniq[:8].tolist()}")
            sem_printed = True
        if "ImagingSonar" in s and all(k in latest for k in REQUIRED):
            Tm = s["PoseSensor"]
            pos, yaw_now = Tm[:3, 3].copy(), np.degrees(np.arctan2(Tm[1, 0], Tm[0, 0]))
            if rec["snap"] or (rec["on"] and moved_enough(pos, yaw_now)):
                save_sample()
                last_saved.update(pos=pos, yaw=yaw_now)
                rec["snap"] = False

        # ---- display ----
        dirty = False
        if "FrontCamera" in s:
            cam_im.set_data(s["FrontCamera"][..., 2::-1]); dirty = True
        if "DepthCam" in s:
            dep_im.set_data(np.ma.masked_greater(s["DepthCam"]["depth"], 50)); dirty = True
        if "ImagingSonar" in s:
            son_im.set_data(np.where(valid, np.sqrt(s["ImagingSonar"][ri, ai]), 0)); dirty = True
        if dirty:
            p = s["PoseSensor"][:3, 3]
            dv = latest.get("DVLSensor")
            dvl_txt = (f"  alt~{np.nanmean(dv[3:7]) * np.cos(np.radians(22.5)):.1f}m (DVL)"
                       if dv is not None and len(dv) == 7 else "")
            ax[0].set_title(f"{'[REC] ' if rec['on'] else ''}saved={n_saved}  "
                            f"x={p[0]:.1f} y={p[1]:.1f} z={p[2]:.1f} yaw={cur_yaw:.0f}°{dvl_txt}")
            plt.pause(0.001)
        else:
            fig.canvas.flush_events()

print(f"Done. {n_saved} samples saved in {RUN}/")