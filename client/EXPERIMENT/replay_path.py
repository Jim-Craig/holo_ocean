"""Replay a fixed path so different conditions are recorded from exactly the same viewpoints.

The path is the sequence of vehicle poses saved in a reference run (s00000.npz, s00001.npz, ...). The vehicle is
teleported to each pose in turn and held there while the sensors capture, so the viewpoints do not depend on physics,
thrusters or flying skill. Replayed sample k always corresponds to reference sample k.

usage:
    python replay_path.py run_REFERENCE "clean replay"                          # same settings: a consistency check
    python replay_path.py run_REFERENCE "sonar noise" --sonar-extra shipped      # HoloOcean's shipped sonar noise
    python replay_path.py run_REFERENCE "heavy noise" --sonar-extra '{"AddSigma": 0.3, "MultSigma": 0.3}'
    python replay_path.py session_pier_REFERENCE "turbid dusk" --turbidity turbid --lighting dusk
    python replay_path.py session_pier_REFERENCE "murky night noisy" --turbidity murky --lighting night --sonar-extra heavy
The environment options (--turbidity --lighting --water-color --weather) use exactly the presets of record_sessions.py.
Without any of them (and with a reference from experiment_3.py) the environment is left untouched.
options: --every N (use every N-th pose)  --max N  --settle TICKS  --warmup TICKS  --fast  --no-viewport  --force

rig.py must match the reference run (tilt, sonar beam, ...). The script checks this and refuses otherwise.
"""
import argparse
import json
import math
import os
import time

import numpy as np

import holoocean
import rig
from run_io import depth_of, list_samples

SENSORS = ("FrontCamera", "DepthCam", "ImagingSonar")
MIN_TICKS = 8                      # ticks to hold a pose before a capture counts
SHIPPED_NOISE = {"AzimuthStreaks": -1, "ScaleNoise": True, "AddSigma": 0.15, "MultSigma": 0.2,
                 "RangeSigma": 0.1, "MultiPath": True}

# Environment presets: identical to record_sessions.py so a replay matches what that script would have recorded.
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
WEATHER_PRESETS = {"sunny": 0, "cloudy": 1, "rainy": 2}
HEAVY_NOISE = {k: (v * 2 if isinstance(v, float) else v) for k, v in SHIPPED_NOISE.items()}   # twice the shipped noise


def apply_environment(env, turbidity, lighting, water_color, weather):
    """Same calls, in the same order, as apply_environmental_conditions() in record_sessions.py."""
    turb, light = TURBIDITY_PRESETS[turbidity], LIGHTING_PRESETS[lighting]
    wcolor = WATER_COLOR_PRESETS[water_color]
    env.change_weather(WEATHER_PRESETS[weather])
    if weather == "rainy":
        env.set_rain_parameters(0, 300, -1000, 3000)
    env.change_time_of_day(light["hour"])
    env.water_fog(turb["density"], fogDepth=turb["depth"], color_R=turb["color"][0],
                  color_G=turb["color"][1], color_B=turb["color"][2])
    env.water_color(wcolor[0], wcolor[1], wcolor[2])
    if light["flashlight"]:
        env.turn_on_flashlight("flashlight1", intensity=6000, beam_width=55)
        env.turn_on_flashlight("flashlight2", intensity=6000, beam_width=55)
    else:
        try:
            env.turn_off_flashlight("flashlight1")
            env.turn_off_flashlight("flashlight2")
        except Exception:
            pass


def wrap(a):
    return (a + 180) % 360 - 180


def yaw_of(R):
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def rz(yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def nested(d, keys):
    for k in keys:
        d = d[k]
    return d


def rig_mismatches(ref, now):
    keys = [("tilt_deg",), ("fov_deg",), ("image_size",), ("camera", "location"), ("camera", "rotation"),
            ("sonar", "r_min"), ("sonar", "r_max"), ("sonar", "azimuth_deg"), ("sonar", "elevation_deg"),
            ("sonar", "bins"), ("sonar", "pitch_deg"),
            ("sonar_minus_camera", "translation_m"), ("sonar_minus_camera", "rotation_deg")]
    bad = []
    for k in keys:
        try:
            a, b = nested(ref, k), nested(now, k)
        except KeyError:
            continue
        if not np.allclose(np.asarray(a, float), np.asarray(b, float)):
            bad.append(f"{'.'.join(k)}: reference {a} vs rig.py {b}")
    return bad


def place(agent, loc, yaw):
    loc, rot = np.asarray(loc, float), np.array([0.0, 0.0, float(yaw)])
    try:
        agent.set_physics_state(loc, rot, np.zeros(3), np.zeros(3))      # also zeroes any residual velocity
    except AttributeError:
        agent.teleport(loc, rot)


def parse():
    ap = argparse.ArgumentParser()
    ap.add_argument("reference", help="reference run folder (its sample poses are the path)")
    ap.add_argument("note", nargs="?", default="", help="label for this condition")
    ap.add_argument("--sonar-extra", default=None,
                    help="'shipped', 'heavy' (twice shipped), 'none' or a JSON dict of extra sonar settings")
    ap.add_argument("--turbidity", choices=list(TURBIDITY_PRESETS), default=None)
    ap.add_argument("--lighting", choices=list(LIGHTING_PRESETS), default=None)
    ap.add_argument("--water-color", choices=list(WATER_COLOR_PRESETS), default=None)
    ap.add_argument("--weather", choices=list(WEATHER_PRESETS), default=None)
    ap.add_argument("--warmup", type=int, default=120, help="ticks to let the environment settle before the first capture")
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--max", type=int, default=None)
    ap.add_argument("--settle", type=int, default=18, help="ticks to hold each pose (at least 14)")
    ap.add_argument("--tol-pos", type=float, default=0.03, help="accepted position error (m)")
    ap.add_argument("--tol-yaw", type=float, default=1.0, help="accepted yaw error (deg)")
    ap.add_argument("--fast", action="store_true", help="try to run faster than real time")
    ap.add_argument("--no-viewport", action="store_true")
    ap.add_argument("--force", action="store_true", help="continue even if rig.py differs from the reference run")
    return ap.parse_args()


def main():
    args = parse()
    if args.settle < 14:
        raise SystemExit("--settle must be at least 14 ticks")
    ref_meta = json.load(open(os.path.join(args.reference, "meta.json")))
    ref_files = list_samples(args.reference)
    sel = list(range(0, len(ref_files), max(1, args.every)))
    if args.max:
        sel = sel[:args.max]
    poses = [np.load(ref_files[i])["pose"] for i in sel]
    targets = [(T[:3, 3].copy(), yaw_of(T[:3, :3])) for T in poses]
    tilts = [math.degrees(math.acos(max(-1.0, min(1.0, -T[2, 2])))) for T in poses]
    if max(tilts) > 2.0:
        print(f"WARNING: some reference poses are tilted up to {max(tilts):.1f} deg; the replay keeps the vehicle level.")

    if args.sonar_extra is not None:
        presets_ = {"shipped": SHIPPED_NOISE, "heavy": HEAVY_NOISE, "none": {}}
        rig.SONAR_EXTRA = dict(presets_[args.sonar_extra] if args.sonar_extra in presets_ else json.loads(args.sonar_extra))
    # the reference may have been recorded with another sonar range (record_sessions.py defaults to 30 m): follow it
    rig.SONAR["RangeMin"] = float(ref_meta["sonar"]["r_min"])
    rig.SONAR["RangeMax"] = float(ref_meta["sonar"]["r_max"])
    problems = rig_mismatches(ref_meta, rig.meta())
    if problems:
        print("rig.py does not match the reference run:\n  " + "\n  ".join(problems))
        if not args.force:
            raise SystemExit("fix rig.py (or use --force) so the sensors are identical to the reference run")

    scenario = holoocean.packagemanager.get_scenario(ref_meta.get("scenario", rig.SCENARIO))
    scenario["package_name"] = "Ocean"
    agent_def = rig.apply(scenario)
    agent_name = agent_def["agent_name"]

    run = time.strftime("run_%Y%m%d_%H%M%S") + "_replay"
    os.makedirs(run, exist_ok=True)
    meta_out = rig.meta()
    meta_out["note"] = args.note
    meta_out["reference_run"] = args.reference
    env_flags = [args.turbidity, args.lighting, args.water_color, args.weather]
    use_env = any(f is not None for f in env_flags) or "environment" in ref_meta
    env_settings = {"turbidity": args.turbidity or "clean", "lighting": args.lighting or "noon",
                    "water_color": args.water_color or "coastal", "weather": args.weather or "sunny", "particles": False}
    meta_out["environment"] = env_settings if use_env else None
    meta_out["condition"] = {"sonar_extra": rig.SONAR_EXTRA, "environment": meta_out["environment"]}
    json.dump(meta_out, open(os.path.join(run, "meta.json"), "w"), indent=2)
    print(f"replaying {len(targets)} poses from {args.reference} -> {run}   (note: {args.note or '-'})")

    make_kwargs = {}
    if args.no_viewport:
        make_kwargs["show_viewport"] = False
    if args.fast:
        make_kwargs["frames_per_sec"] = False
    try:
        cm = holoocean.make(scenario_cfg=scenario, **make_kwargs)
    except TypeError:
        print("this HoloOcean version does not accept --fast / --no-viewport; starting normally")
        cm = holoocean.make(scenario_cfg=scenario)

    with cm as env:
        agent = env.agents[agent_name]
        if use_env:
            apply_environment(env, env_settings["turbidity"], env_settings["lighting"],
                              env_settings["water_color"], env_settings["weather"])
            print("environment:", env_settings)
        for _ in range(args.warmup):                      # let lighting / fog / the sonar octree settle
            place(agent, targets[0][0], targets[0][1])
            env.tick()

        def capture(loc, yaw):
            """Hold the vehicle at (loc, yaw); return the last tick on which every rig sensor reported."""
            got = None
            for t in range(args.settle):
                place(agent, loc, yaw)
                s = env.tick()
                if t >= MIN_TICKS and all(k in s for k in SENSORS):
                    got = s
            extra = 0
            while got is None and extra < 12:
                place(agent, loc, yaw)
                s = env.tick()
                extra += 1
                if all(k in s for k in SENSORS):
                    got = s
            return got

        # PoseSensor reports the IMU socket, not the agent origin: measure that fixed offset once.
        p0, yaw0 = targets[0]
        s = capture(p0, yaw0)
        if s is None:
            raise SystemExit("sensors never reported during calibration")
        meas = s["PoseSensor"]
        yaw_err = wrap(yaw_of(meas[:3, :3]) - yaw0)
        if abs(yaw_err) > args.tol_yaw:
            raise SystemExit(f"after teleporting to yaw {yaw0:.1f} deg the pose sensor reads "
                             f"{yaw_of(meas[:3, :3]):.1f} deg: rotation conventions differ from what this script assumes")
        offset_body = rz(-yaw0) @ (meas[:3, 3] - p0)
        print(f"pose sensor offset from the agent origin (heading frame): {np.round(offset_body, 3)} m")

        pos_errs, yaw_errs, depth_med, depth_p95, bad, missing = [], [], [], [], 0, 0
        t_start = time.time()
        for n, (p_t, yaw_t) in enumerate(targets):
            loc = p_t - rz(yaw_t) @ offset_body
            s, ok = None, False
            for attempt in range(3):
                s = capture(loc, yaw_t)
                if s is None:
                    continue
                T = s["PoseSensor"]
                pos_err = float(np.linalg.norm(T[:3, 3] - p_t))
                yaw_e = abs(wrap(yaw_of(T[:3, :3]) - yaw_t))
                if pos_err <= args.tol_pos and yaw_e <= args.tol_yaw:
                    ok = True
                    break
            if s is None:
                missing += 1
                print(f"  sample {n}: sensors never reported, skipped")
                continue
            bad += 0 if ok else 1
            pos_errs.append(pos_err); yaw_errs.append(yaw_e)

            ref_depth = depth_of(np.load(ref_files[sel[n]]))
            cur_depth = s["DepthCam"]["depth"]
            both = (ref_depth > 0.3) & (ref_depth < 50) & (cur_depth > 0.3) & (cur_depth < 50)
            if both.any():
                diff = np.abs(ref_depth - cur_depth)[both]
                depth_med.append(float(np.median(diff))); depth_p95.append(float(np.percentile(diff, 95)))

            np.savez_compressed(
                os.path.join(run, f"s{n:05d}.npz"),
                rgb=s["FrontCamera"][..., 2::-1], depth=s["DepthCam"]["depth"].astype(np.float32),
                sonar=s["ImagingSonar"], pose=s["PoseSensor"],
                vel=np.full(3, np.nan), dvl=np.full(7, np.nan),             # meaningless while teleporting
                t=s["t"], arrival_ticks=np.array([-1, -1, -1]), ref_index=sel[n], pose_err=[pos_err, yaw_e])
            if (n + 1) % 25 == 0 or n + 1 == len(targets):
                eta = (time.time() - t_start) / (n + 1) * (len(targets) - n - 1)
                print(f"  {n + 1}/{len(targets)}   max position error so far {max(pos_errs) * 100:.1f} cm   ETA {eta / 60:.1f} min")

    summary = {"samples": len(pos_errs), "outside_tolerance": bad, "missing": missing,
               "pos_err_m_max": max(pos_errs) if pos_errs else None, "pos_err_m_mean": float(np.mean(pos_errs)) if pos_errs else None,
               "yaw_err_deg_max": max(yaw_errs) if yaw_errs else None,
               "depth_diff_median_m": float(np.median(depth_med)) if depth_med else None,
               "depth_diff_p95_m": float(np.median(depth_p95)) if depth_p95 else None,
               "pose_sensor_offset_m": np.round(offset_body, 4).tolist()}
    meta_out["replay"] = summary
    json.dump(meta_out, open(os.path.join(run, "meta.json"), "w"), indent=2)
    print("\nreplay summary:")
    for k, v in summary.items():
        print(f"  {k}: {v}")
    print("\nGood signs: position error of a few cm, yaw error under a degree, and a depth difference near zero")
    print("(the depth map should not change with sonar noise or weather; a clear difference means the viewpoint differs).")
    print(f"saved {run}/")


if __name__ == "__main__":
    main()
