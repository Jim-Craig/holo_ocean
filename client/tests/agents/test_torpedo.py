import holoocean
import uuid
import pytest
import numpy as np
from holoocean.fossen_dynamics.dynamics import *
from holoocean.fossen_dynamics.torpedo import *


@pytest.fixture(scope="module")
def env():
    scenario = {
        "name": "hovering_dynamics",
        "world": "TestWorld",
        "frames_per_sec": False,
        "main_agent": "auv0",
        "agents": [
            {
                "agent_name": "auv0",
                "agent_type": "TorpedoAUV",
                "sensors": [
                    {
                        "sensor_type": "DynamicsSensor",
                        "configuration": {"UseCOM": True, "UseRPY": False},
                    },
                    {
                        "sensor_type": "DynamicsSensor",
                        "sensor_name": "DynamicsSensorRPY",
                        "configuration": {"UseCOM": True, "UseRPY": True},
                    },
                ],
                "control_scheme": 1,
                "location": [0, 0, -10],
                "rotation": [0, 0, 0],
                "dynamics": {
                    "mass": 16,
                    "length": 1.6,
                    "rho": 1026,
                    "diam": 0.19,
                    "r_bg": [0, 0, 0.02],
                    "r_bb": [0, 0, 0],
                    "r44": 0.3,
                    "Cd": 0.42,
                    "T_surge": 20,
                    "T_sway": 20,
                    "zeta_roll": 0.3,
                    "zeta_pitch": 0.8,
                    "T_yaw": 1,
                    "K_nomoto": 5.0 / 20.0,
                },
                "actuator": {
                    "fin_area": 0.00665,
                    "deltaMax_fin_deg": 15,
                    "nMax": 1525,
                    "T_delta": 0.1,
                    "T_n": 0.1,
                    "CL_delta_r": 0.5,
                    "CL_delta_s": 0.7,
                },
                "autopilot": {
                    "depth": {
                        "wn_d_z": 0.2,
                        "Kp_z": 0.08,
                        "T_z": 100,
                        "Kp_theta": 4.0,
                        "Kd_theta": 2.3,
                        "Ki_theta": 0.3,
                        "K_w": 5.0,
                    },
                    "heading": {
                        "wn_d": 1.2,
                        "zeta_d": 0.8,
                        "r_max": 0.9,
                        "lam": 0.1,
                        "phi_b": 0.1,
                        "K_d": 0.5,
                        "K_sigma": 0.05,
                    },
                },
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


def test_manual_dynamics(env):
    """Test to make sure it goes to the linear and angular acceleration we set"""
    des = [1, 2, 3, 0.1, 0.2, 0.3]

    env.reset()

    state = env.step(des, 20)

    accel = state["DynamicsSensor"][0:3]
    ang_accel = state["DynamicsSensor"][9:12]

    assert np.allclose(
        des[:3], accel
    ), "Manual dynamics didn't hit the correct linear acceleration"
    assert np.allclose(
        des[3:], ang_accel
    ), "Manual dynamics didn't hit the correct angular acceleration"


def test_fossen_dynamics(env):
    """Test to make sure it goes to the linear and angular acceleration we set"""
    env.reset()
    scenario = env._scenario

    numSteps = 200

    vehicle = fourFinDep(scenario, "auv0", "manualControl")

    period = 1.0 / 30.0
    torpedo_dynamics = FossenDynamics(vehicle)  # ,period)

    accel = np.array(np.zeros(6), float)

    u_control = np.array([0.087, 0.087, 800])  # [RudderAngle, SternAngle,Thruster]
    vehicle.set_control_mode("manualControl")
    torpedo_dynamics.set_u_control_rad(u_control)

    for i in range(numSteps):
        state = env.step(accel)
        accel = torpedo_dynamics.update(
            state
        )  # Calculate accelerations to be applied to HoloOcean agent

    pitch_heading = state["DynamicsSensorRPY"][
        16:18
    ]  # PULL OUT PITCH and heading in radians

    # Assert that pitch and heading are less than 0
    assert (
        pitch_heading[0] < 0
    ), f"Pitch should be less than 0, but got {pitch_heading[0]}"
    assert (
        pitch_heading[1] < 0
    ), f"Heading should be less than 0, but got {pitch_heading[1]}"


def test_fossen_autopilot(env):
    """Test to make sure it goes to the linear and angular acceleration we set"""
    env.reset()
    scenario = env._scenario

    vehicle = fourFinDep(scenario, "auv0", "manualControl")
    period = 1.0 / 30.0
    torpedo_dynamics = FossenDynamics(vehicle)  # ,period)

    numSteps = 800
    accel = np.array(np.zeros(6), float)  # HoloOcean parameter input
    depth = 15
    heading = 50
    vehicle.set_goal(
        depth, heading, 1300
    )  # Changes depth (positive depth), heading, thruster RPM goals for controller
    vehicle.set_control_mode(
        "depthHeadingAutopilot"
    )  # In this mode PID controller calculates control commands (u_control)
    for i in range(numSteps):
        state = env.step(accel)
        accel = torpedo_dynamics.update(state)

    depth_actual = -state["DynamicsSensor"][8]  # [x, y, z] #PULL OUT Depth and heading
    heading_actual = -state["DynamicsSensorRPY"][17]

    depth_error = abs(depth - depth_actual)
    heading_error = abs(heading - heading_actual)

    assert np.allclose(
        depth, depth_actual, atol=1.0
    ), f"Autopilot depth not achieved within 1 meter (Depth error was: {depth_error})"
    assert np.allclose(
        heading, heading_actual, atol=10.0
    ), f"Autopilot heading not acheived within 10 degrees (Heading error was {heading_error})"


# TODO: Test to make sure fins saturate at max deflection
