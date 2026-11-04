"""Camera / depth camera / imaging sonar rig for the HoloOcean experiments.

Edit values HERE only; experiment_3.py reads everything from this file and writes it to each run's meta.json.

Frames used below
  agent frame (sensor `location`):  x forward, y left, z up, metres
  rotation:                         [roll, pitch, yaw] in degrees; positive pitch tilts DOWN (confirmed from your data)
"""
import numpy as np

SCENARIO = "PierHarbor-Hovering"        # which world/start pose to load; the rig below replaces its sensors
TICKS_PER_SEC = 60                    # 10 Hz sensors then all land on the same tick (confirmed)

# ---- cameras (RGB and depth are identical so their pixels line up) ----
W, H, FOV_DEG, HZ = 640, 480, 90, 10
TILT_DEG = 20.0                       # pitch below horizontal, shared by camera, depth camera and sonar
CAM_LOC = [1.0, 0.0, 0.3]

# ---- sonar ----
SONAR_OFFSET = [0.0, 0.0, 0.0]        # sonar position relative to the camera. Zero = identity extrinsics.
SONAR_ROT_OFFSET = [0.0, 0.0, 0.0]    # extra [roll, pitch, yaw] relative to the camera
USE_SONAR_SOCKET = False              # set True if the sonar is blank with an explicit location
SONAR = dict(RangeBins=512, AzimuthBins=512, RangeMin=0.5, RangeMax=20.0, Azimuth=90.0, Elevation=40.0)
# SONAR_EXTRA = {"AzimuthStreaks": -1, "ScaleNoise": True, "AddSigma": 0.15,
#                "MultSigma": 0.2, "RangeSigma": 0.1, "MultiPath": True}   # the noise settings from the shipped scenarios
SONAR_EXTRA = {}

# ---- optional start pose (None = scenario default) ----
START_LOCATION = None                 # e.g. [x, y, z] in world metres
START_ROTATION = None                 # e.g. [0, 0, yaw_deg]

WITH_SEMANTIC = False                 # add a SemanticSegmentationCamera at the camera pose (WIP sensor; output format undocumented)

NAMES = ("FrontCamera", "DepthCam", "ImagingSonar", "SemCam")


def _sonar_pose():
    """Return the pose of the sonar, either as a socket or a location and rotation."""
    rot = [SONAR_ROT_OFFSET[0], TILT_DEG + SONAR_ROT_OFFSET[1], SONAR_ROT_OFFSET[2]]
    if USE_SONAR_SOCKET:
        return {"socket": "SonarSocket", "rotation": rot}
    return {"location": [c + o for c, o in zip(CAM_LOC, SONAR_OFFSET)], "rotation": rot}


def sensors():
    cam = {"location": list(CAM_LOC), "rotation": [0.0, TILT_DEG, 0.0]}
    size = {"CaptureWidth": W, "CaptureHeight": H}
    sonar_cfg = {**SONAR, "InitOctreeRange": SONAR["RangeMax"] + 10, "ViewRegion": False, **SONAR_EXTRA}
    out = [
        {"sensor_name": "FrontCamera", "sensor_type": "RGBCamera", "Hz": HZ, **cam,
         "configuration": dict(size)},
        {"sensor_name": "DepthCam", "sensor_type": "DepthCamera", "Hz": HZ, **cam,
         "configuration": {**size, "FovAngle": FOV_DEG}},
        {"sensor_name": "ImagingSonar", "sensor_type": "ImagingSonar", "Hz": HZ, **_sonar_pose(),
         "configuration": sonar_cfg},
    ]
    if WITH_SEMANTIC:       # per the docs this camera only takes CaptureWidth/Height (no FOV setting)
        out.append({"sensor_name": "SemCam", "sensor_type": "SemanticSegmentationCamera", "Hz": HZ, **cam,
                    "configuration": dict(size)})
    return out


def apply(scenario):
    """Replace the rig sensors in a HoloOcean scenario dict; returns the agent dict."""
    agent = scenario["agents"][0]
    agent["sensors"] = [s for s in agent["sensors"] if s.get("sensor_name") not in NAMES]
    agent["sensors"] += sensors()
    scenario["ticks_per_sec"] = TICKS_PER_SEC
    if START_LOCATION is not None:
        agent["location"] = list(START_LOCATION)
    if START_ROTATION is not None:
        agent["rotation"] = list(START_ROTATION)
    return agent


def seabed_band(h, tilt=None, elev=None):
    """Slant range (near, far) in metres at which a flat seabed h metres below the sonar crosses the wedge centre."""
    tilt = TILT_DEG if tilt is None else tilt
    elev = SONAR["Elevation"] if elev is None else elev
    hi, lo = min(tilt + elev / 2, 90.0), tilt - elev / 2
    near = h / np.sin(np.radians(hi))
    far = h / np.sin(np.radians(lo)) if lo > 0 else float("inf")
    return near, far


def meta():
    pose = _sonar_pose()
    return {
        "scenario": SCENARIO, "tilt_deg": TILT_DEG, "fov_deg": FOV_DEG, "image_size": [W, H], "ticks_per_sec": TICKS_PER_SEC, "semantic_camera": WITH_SEMANTIC,
        "camera": {"location": list(CAM_LOC), "rotation": [0.0, TILT_DEG, 0.0]},
        "depth_camera": {"location": list(CAM_LOC), "rotation": [0.0, TILT_DEG, 0.0], "fov_deg": FOV_DEG},
        "sonar": {**pose, "pitch_deg": TILT_DEG + SONAR_ROT_OFFSET[1],
                  "r_min": SONAR["RangeMin"], "r_max": SONAR["RangeMax"],
                  "azimuth_deg": SONAR["Azimuth"], "elevation_deg": SONAR["Elevation"],
                  "bins": [SONAR["RangeBins"], SONAR["AzimuthBins"]], "extra": SONAR_EXTRA},
        "sonar_minus_camera": {"translation_m": list(SONAR_OFFSET), "rotation_deg": list(SONAR_ROT_OFFSET)},
        "conventions": {
            "agent_frame": "x forward, y left, z up",
            "pose_sensor": "4x4 world pose; rotation columns are forward, right, down (NED-like)",
            "rgb": "saved as RGB (converted from the sensor's BGRA)",
            "depth": "metres, assumed planar (z along optical axis); >50 m = invalid (open water ~725 m)",
            "sonar": "(range_bin, azimuth_bin); row 0 = RangeMin (near); column 0 = rightmost (azimuth increases to the left); azimuth direction from check_extrinsics.py on 3 samples, to be reconfirmed",
        },
    }