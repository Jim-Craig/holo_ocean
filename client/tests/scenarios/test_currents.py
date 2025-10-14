import holoocean
import numpy as np

def test_currents():
    scenario = {
        "name": "test_currents",
        "package_name": "TestWorlds",
        "world": "TestWorld",
        "main_agent": "auv0",
        "current": {
            "vehicle_debugging": False,
        },
        "ticks_per_sec": 60,
        "agents": [
            {
                "agent_name": "auv0",
                "agent_type": "HoveringAUV",
                "sensors": [
                    {
                        "sensor_type": "DynamicsSensor",
                        "configuration": {
                            "UseCOM": True,
                            "UseRPY": False,
                        }
                    }

                ],
                "control_scheme": 0,
                "location": [18.5, -25.5, -10]
            }
        ]
    }

    with holoocean.make(
        scenario_cfg=scenario, frames_per_sec=False, show_viewport=False, verbose=True
    ) as env:
        data = env.tick()
        dynamics_data = data["DynamicsSensor"]
        acceleration = dynamics_data[:3]
        # print(acceleration)

        assert np.allclose(acceleration, [0, 0, 0]), "Null current failed"
        
        env.set_ocean_currents("auv0", [1, 0, 0])
        data = env.tick()
        dynamics_data = data["DynamicsSensor"]
        acceleration = dynamics_data[:3]
        # print(acceleration)

        assert np.allclose(acceleration, [0.06320976, 0, 0]), "X current failed"

        env.set_ocean_currents("auv0", [0, 1, 0])
        data = env.tick()
        dynamics_data = data["DynamicsSensor"]
        acceleration = dynamics_data[:3]
        # print(acceleration)
    
        assert np.allclose(acceleration, [-0.00112009, 0.06320979, 0]), "Y current failed"
        
        env.set_ocean_currents("auv0", [0, 0, 1])
        data = env.tick()
        dynamics_data = data["DynamicsSensor"]
        acceleration = dynamics_data[:3]
        # print(acceleration)

        assert np.allclose(acceleration, [-0.00110024, -0.00112009, 0.06320982]), "Z current failed"

    scenario = {
        "name": "test_currents",
        "package_name": "TestWorlds",
        "world": "TestWorld",
        "main_agent": "auv0",
        "current": {
            "vehicle_debugging": False,
        },
        "ticks_per_sec": 60,
        "agents": [
            {
                "agent_name": "auv0",
                "agent_type": "SurfaceVessel",
                "sensors": [
                    {
                        "sensor_type": "DynamicsSensor",
                        "configuration": {
                            "UseCOM": True,
                            "UseRPY": False,
                        }
                    }
                ],
                "control_scheme": 0,
                "location": [50, -50, .1]
            }
        ]
    }

    with holoocean.make(
        scenario_cfg=scenario, frames_per_sec=False, show_viewport=False, verbose=True
    ) as env:
        for _ in range(500):
          env.tick()

        for _ in range(10):
            env.set_ocean_currents("auv0", [1, 0, 0])
            data = env.tick()
            dynamics_data = data["DynamicsSensor"]
            acceleration = dynamics_data[:3]

        # print(acceleration)
        assert np.allclose(acceleration[:2], [0.00199408, 0], atol=5e-5), "Surface Vessel X Current Failed"
        
        for _ in range(10):
            data = env.tick()
            env.set_ocean_currents("auv0", [0, 1, 0])
            dynamics_data = data["DynamicsSensor"]
            acceleration = dynamics_data[:3]
    
        # print(acceleration)
        assert np.allclose(acceleration[:2], [-0.00090362, 0.00209909], atol=5e-5), "Surface Vessel Y Current Failed"
        
