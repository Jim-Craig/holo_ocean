import holoocean
import uuid
import pytest
import numpy as np


@pytest.fixture(scope="module")
def env(vehicle_type="HoveringAUV"):
    scenario = {
        "name": "test_gps_sensor",
        "world": "TestWorld",
        "main_agent": "vehicle",
        "frames_per_sec": False,
        "agents": [
            {
                "agent_name": "vehicle",
                "agent_type": vehicle_type,
                "sensors": [{"sensor_type": "GPSSensor", "configuration": {}}],
                "control_scheme": 0,
                "location": [0, 0, 0],
            }
        ],
    }
    binary_path = holoocean.packagemanager.get_binary_path_for_package("TestWorlds")
    with holoocean.environments.HoloOceanEnvironment(
        scenario=scenario,
        binary_path=binary_path,
        show_viewport=False,
        verbose=True,
        uuid=str(uuid.uuid4()),
        ticks_per_sec=30,
    ) as env:
        yield env


@pytest.mark.parametrize(
    "agent_type",
    ["HoveringAUV", "TorpedoAUV", "BlueROV2"],
)
def test_setting_depth(env, agent_type):
    """Make sure if it's above the depth we receive data, and if below we don't"""
    env._scenario["agents"][0]["agent_type"] = agent_type
    depth = np.random.rand() * 10 + 1
    env._scenario["agents"][0]["sensors"][0]["configuration"]["Depth"] = depth

    # Test above surface
    env._scenario["agents"][0]["location"] = [0, 0, 1]
    env.reset()
    state = env.tick()
    assert "GPSSensor" in state

    # Test below surface, above cutoff
    env._scenario["agents"][0]["location"] = [0, 0, -1 * depth + 1]
    env.reset()
    state = env.tick()
    assert "GPSSensor" in state

    # Test below surface, below cutoff
    env._scenario["agents"][0]["location"] = [0, 0, -1 * depth - 1]
    env.reset()
    state = env.tick()
    assert "GPSSensor" not in state


@pytest.mark.parametrize(
    "agent_type",
    ["HoveringAUV", "TorpedoAUV", "BlueROV2"],
)
def test_random_depth(env, agent_type):
    """Make sure the depth is changing according to the noise we put in"""
    env._scenario["agents"][0]["agent_type"] = agent_type
    num_ticks = 1000
    depth = np.random.rand() * 10
    env._scenario["agents"][0]["sensors"][0]["configuration"]["Depth"] = depth
    env._scenario["agents"][0]["sensors"][0]["configuration"]["DepthSigma"] = 1
    env._scenario["agents"][0]["location"] = [0, 0, -1 * depth]

    env.reset()

    count = 0
    for _ in range(num_ticks):
        state = env.tick()
        if "GPSSensor" in state:
            count += 1

    assert 0 < count
    assert count < num_ticks


@pytest.mark.parametrize(
    "agent_type",
    ["HoveringAUV", "TorpedoAUV", "BlueROV2"],
)
def test_tide(env, agent_type):
    env._scenario["agents"][0]["agent_type"] = agent_type
    env._scenario["agents"][0]["sensors"][0]["configuration"]["DepthSigma"] = 0
    depth = np.random.rand() * 10 + 1
    env._scenario["agents"][0]["sensors"][0]["configuration"]["Depth"] = depth
    env._scenario["agents"][0]["sensors"].append(
        {"sensor_type": "DepthSensor", "configuration": {}}
    )

    # Test that depth accounts for tide: location before tide is good, after tide is below cutoff
    env._scenario["agents"][0]["location"] = [0, 0, -1 * depth + 1]
    env.reset()
    env.tide(2, True)
    state = env.tick()
    assert "GPSSensor" not in state
