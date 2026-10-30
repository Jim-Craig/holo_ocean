import holoocean
import numpy as np
import matplotlib.pyplot as plt

scenario = holoocean.packagemanager.get_scenario("PierHarbor-Hovering")
scenario["package_name"] = "Ocean"
scenario["ticks_per_sec"] = 60   # clean multiple of 10 Hz and 20 Hz (DVL)

agent = scenario["agents"][0]

agent["sensors"] = [s for s in agent["sensors"] if s.get("sensor_name") != "FrontCamera"]
agent["sensors"].append({
    "sensor_name": "FrontCamera",
    "sensor_type": "RGBCamera",
    "location": [1.0, 0, 0.3],      # in front of and slightly above the frame; try 0.5 / 1.0 / 1.5
    "rotation": [0, 15, 0],
    "Hz": 10,
    "configuration": {"CaptureWidth": 640, "CaptureHeight": 480}
})

agent["sensors"].append({
    "sensor_name": "ImagingSonar",
    "sensor_type": "ImagingSonar",
    "socket": "SonarSocket",
    "Hz": 10,
    "rotation": [0, 20, 0],        # tilt toward the seabed; flip the sign if it points up
    "configuration": {
        "RangeBins": 512, "AzimuthBins": 512,
        "RangeMin": 0.5, "RangeMax": 20,
        "InitOctreeRange": 30,
        "Azimuth": 120, "Elevation": 20,
        "ViewRegion": False,        # draws the sonar's field of view in the Unreal window
        "ViewOctree": -1            # shows octree cells, -1 turns it off
    }
})
print("Sensors in scenario:", [s.get("sensor_name", s["sensor_type"]) for s in agent["sensors"]])
print("ticks_per_sec:", scenario["ticks_per_sec"])

with holoocean.make(scenario_cfg=scenario) as env:
    latest = {}
    sonar_frames = []

    for i in range(3000):
        env.act("auv0", np.array([0, 0, 0, 0, 10, 10, 10, 10], dtype=np.float32))
        state = env.tick()
        latest.update(state)

        if "ImagingSonar" in state and i > 120:      # skip octree/startup frames
            sonar_frames.append(state["ImagingSonar"].copy())

        # stop once both sensors have reported and we have a few post-warmup sonar frames
        if "FrontCamera" in latest and len(sonar_frames) >= 3:
            break

    print("tick:", i, "| keys seen:", sorted(latest.keys()))
    print("sonar maxes:", [float(f.max()) for f in sonar_frames])

    camera = latest["FrontCamera"][..., :3]
    sonar = sonar_frames[-1]

    cam = latest["FrontCamera"][..., :3]
    print("cam mean/max:", cam.mean(), cam.max(), "| frac > 10:", (cam > 10).mean())
    plt.imshow(np.clip(cam.astype(float) * 4, 0, 255).astype(np.uint8))  # crude brighten

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].imshow(camera); axes[0].set_title("RGB Camera"); axes[0].axis("off")
    im = axes[1].imshow(sonar, cmap="gray", vmin=0, vmax=max(sonar.max(), 1e-6))
    axes[1].set_title("Imaging Sonar"); axes[1].axis("off")
    plt.colorbar(im, ax=axes[1], fraction=0.046, pad=0.04)
    plt.tight_layout(); plt.show()