import holoocean, numpy as np

W, H, FOV = 640, 480, 90
f = (W / 2) / np.tan(np.radians(FOV / 2)); cx, cy = W / 2, H / 2

scenario = holoocean.packagemanager.get_scenario("PierHarbor-Hovering")
scenario["package_name"] = "Ocean"
scenario["ticks_per_sec"] = 60
scenario["agents"][0]["sensors"].append(
    {"sensor_name": "DepthCam", "sensor_type": "DepthCamera", "Hz": 10,
     "location": [1.0, 0, 0.3], "rotation": [0, 0, 0],          # level camera for this test
     "configuration": {"CaptureWidth": W, "CaptureHeight": H, "FovAngle": FOV}})

with holoocean.make(scenario_cfg=scenario) as env:
    for _ in range(300): s = env.tick()
    T = s["PoseSensor"]; p = T[:3, 3]
    fwd = np.array([T[0, 0], T[1, 0], 0.0]); fwd /= np.linalg.norm(fwd)    # horizontal heading
    yaw = np.degrees(np.arctan2(fwd[1], fwd[0]))
    # thin wall, about 8 m ahead. If `scale` must be a single number, pass a scalar and tell me what happens.
    env.spawn_prop("box", location=(p + 8 * fwd).tolist(), rotation=[0, 0, yaw],
                   scale=[0.2, 40, 40], material="white")
    dep = None
    for _ in range(300):
        s = env.tick()
        if "DepthCam" in s: dep = s["DepthCam"]["depth"]

v, u = np.mgrid[0:H, 0:W]
xn, yn = (u - cx) / f, (v - cy) / f

def plane_rms(z):
    m = np.zeros((H, W), bool); m[:200] = True            # upper rows only: the wall, not the seabed
    m &= (dep < 50) & (dep > 1)
    P = np.stack([xn * z, yn * z, z], -1)[m]
    sv = np.linalg.svd(P - P.mean(0), compute_uv=False)
    return sv[-1] / np.sqrt(len(P))                        # RMS distance to the best-fit plane (m)

print("depth at left / centre / right (row 100):", dep[100, 5], dep[100, 320], dep[100, 635])
print("plane-fit RMS if depth is planar :", plane_rms(dep))
print("plane-fit RMS if depth is radial :", plane_rms(dep / np.sqrt(1 + xn**2 + yn**2)))