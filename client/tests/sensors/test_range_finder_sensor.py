import holoocean
import uuid
import pytest

from tests.utils.equality import almost_equal


@pytest.fixture(scope="module")
def cfg1(vehicle_type="SphereAgent"):
    sphere_config = {
        "name": "test_range_finder_sensor",
        "world": "TestWorld",
        "main_agent": "sphere0",
        "frames_per_sec": False,
        "agents": [
            {
                "agent_name": "sphere0",
                "agent_type": vehicle_type,
                "sensors": [
                    {
                        "sensor_type": "RangeFinderSensor",
                        "configuration": {"LaserMaxDistance": 1, "LaserCount": 12},
                    }
                ],
                "control_scheme": 0,
                "location": [0.95, -1.75, 0.5],
            }
        ],
    }
    yield sphere_config


@pytest.mark.parametrize(
    "cfg1",
    ["SphereAgent", "HoveringAUV", "TorpedoAUV", "SurfaceVessel", "BlueROV2"],
    indirect=True,
)
def test_range_finder_sensor_max(cfg1):
    """Make sure the range sensor set max distance correctly."""
    binary_path = holoocean.packagemanager.get_binary_path_for_package("TestWorlds")

    with holoocean.environments.HoloOceanEnvironment(
        scenario=cfg1,
        binary_path=binary_path,
        show_viewport=False,
        verbose=True,
        uuid=str(uuid.uuid4()),
    ) as env:
        state = env.tick(10)

        actual = state["RangeFinderSensor"]

        expected_count = cfg1["agents"][0]["sensors"][0]["configuration"]["LaserCount"]
        assert (
            len(actual) == expected_count
        ), "Sensed range size did not match the expected size!"

        expected_dist = cfg1["agents"][0]["sensors"][0]["configuration"][
            "LaserMaxDistance"
        ]
        assert all(x > 0 for x in actual), "Sensed range includes 0!"
        assert all(
            x <= expected_dist for x in actual
        ), "Sensed range includes value greater than 1!"


@pytest.fixture(scope="module")
def cfg2(vehicle_type="UavAgent"):
    uav_config = {
        "name": "test_range_finder_sensor",
        "world": "TestWorld",
        "main_agent": "uav0",
        "frames_per_sec": False,
        "agents": [
            {
                "agent_name": "uav0",
                "agent_type": vehicle_type,
                "sensors": [
                    {
                        "sensor_type": "RangeFinderSensor",
                        "configuration": {"LaserAngle": -90, "LaserMaxDistance": 15},
                    }
                ],
                "control_scheme": 0,
                "location": [0, 0, 10],
            }
        ],
    }
    yield uav_config


@pytest.mark.parametrize(
    "cfg2",
    ["SphereAgent", "HoveringAUV", "TorpedoAUV", "SurfaceVessel", "BlueROV2"],
    indirect=True,
)
def test_range_finder_sensor_falling(cfg2):
    """Makes sure that the range sensor updates as the UAV falls, and after it comes to a rest."""

    binary_path = holoocean.packagemanager.get_binary_path_for_package("TestWorlds")

    with holoocean.environments.HoloOceanEnvironment(
        scenario=cfg2,
        binary_path=binary_path,
        show_viewport=False,
        verbose=True,
        uuid=str(uuid.uuid4()),
    ) as env:
        last_range = env.tick()["RangeFinderSensor"][0]

        for _ in range(10):
            new_range = env.tick(4)["RangeFinderSensor"][0]
            assert new_range < last_range, "UAV's range sensor did not detect falling!"
            last_range = new_range

        # Give the UAV time to bounce and settle
        env.tick(80)

        # Make sure it is stationary now
        last_range = env.tick()["RangeFinderSensor"][0]
        new_range = env.tick()["RangeFinderSensor"][0]

        assert almost_equal(last_range, new_range), "The UAV did not seem to settle!"
