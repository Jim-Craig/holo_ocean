import numpy as np
from holoocean.fossen_dynamics.control import integralSMC
from holoocean.fossen_dynamics.helper_functions import crossFlowDrag, forceLiftDrag, Hmtrx, m2c, gvect, ssa, velocityTransform

"""
torpedo.py:  

   Class for the torpedo-shaped autonomous underwater vehicle (AUV), 
   which is controlled using fins at the back and a propeller. The 
   default parameters match the REMUS 100 vehicle.               

References: 
    
    B. Allen, W. S. Vorus and T. Prestero, "Propulsion system performance 
         enhancements on REMUS AUVs," OCEANS 2000 MTS/IEEE Conference and 
         Exhibition. Conference Proceedings, 2000, pp. 1869-1873 vol.3, 
         doi: 10.1109/OCEANS.2000.882209.    
    T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion 
         Control. 2nd. Edition, Wiley. URL: www.fossen.biz/wiley            

Author:     Thor I. Fossen
Modified:   Braden Meyers & Carter Noh
"""


class TAUV:
    """
    Parent class of the torpedo-shaped vehicle. General parameters and 
    calculations for all torpedo vehicles. Actuator parameters and calculations
    are implemented in subclasses.

    :param dict scenario: scenario dictionary for holoocean 
    :param str vehicle_name: name of vehicle to initialize that matches agent in scenario dictionary
    :param str controlSystem: autopilot method for controlling the actuators
    :param float r_z: desired depth (m), positive downwards
    :param float r_psi: desired yaw angle (deg)
    :param float r_rpm: desired propeller revolution (rpm)
    :param float r_rpm: desired surge speed (m/s)
    :param float V_current: current speed (m/s)
    :param float beta_current: current direction (deg)
    """
    def __init__(
        self,
        scenario=None,
        vehicle_name=None,
        controlSystem="stepInput",
        r_z=0,                      # goal depth
        r_psi=0,                    # goal heading
        r_rpm=0,                    # goal propeller rpm
        r_surge=0,                  # goal surge speed
        V_current=0,                # current speed
        beta_current=0,             # current direction
    ):

        # Define constants
        self.D2R = np.pi / 180
        self.g = 9.81                        # acceleration of gravity (m/s^2)
        self.dimU = len(self.controls)

        # Initialize the vehicle
        self.configure_from_scenario(scenario, vehicle_name)
        self.set_control_mode(controlSystem)
        self.set_goal(r_z, r_psi,r_rpm,r_surge)
        self.V_c = V_current
        self.beta_c = beta_current * self.D2R

        # Initialize the AUV model
        self.u_actual = np.array([0, 0, 0, 0], float)    # control input vector
        self.nu = np.array([0, 0, 0, 0, 0, 0], float)  # velocity vector in body frame
        self.eta = np.array([0, 0, 0, 0, 0, 0], float) # position vector in global frame
        
        # Use input parameters to calculate other vehicle paramaters
        self.calculate_additional_parameters()

        self.init_depth = False  #Set the LP filter inital state to current depth when false

    def configure_from_scenario(self, scenario, vehicle_name):
        """
        Dynamics Parameters:

        - Environment Parameters

        :param float rho:        Density of water in kg/m^3
        :param float sampleTime: Sample time for the dynamics manager (s)

        - Vehicle Physical Parameters 

        :param float mass:      Mass of vehicle in kilograms
        :param float length:    Length of vehicle in meters
        :param float diam:      Diameter of vehicle in m
        :param float r_bg:      Center of gravity of the vehicle (x, y, z) in body frame x forward, y right, z down
        :param float r_bb:      Center of boyancy of the vehicle (x, y, z) in body frame x forward, y right, z down
        :param float area_fraction: relates vehicle effective area to length and width. pi/4 for a spheroid

        - Low-Speed Linear Damping Matrix Parameters:

        :param float T_surge:   Time constant in surge (s)
        :param float T_sway:    Time constant in sway (s)
        :param float T_yaw:     Time constant in yaw (s)
        :param float zeta_roll: Relative damping ratio in roll
        :param float zeta_ptich: Relative damping ratio in pitch
        :param float K_nomoto: 

        - Other Damping Parameters

        :param float r44:       Added moment of inertia in roll: A44 = r44 * Ix   
        :param float Cd:        Coefficient of drag
        :param float e:         Oswald efficiency factor for vehicle drag

        Autopilot Paramters:
            
        - Depth

        :param float wn_d_z:    Damped natural frequency for low pass filter for depth commands
        :param float Kp_z:      Portional gain for depth controller
        :param float T_z:
        :param float Kp_theta:  Porportional gain for pitch angle for depth controller
        :param float Ki_theta:  Integral gain for pitch angle for depth controller
        :param float Kd_theta:  Derivative gain for pitch angle for depth controller
        :param float K_w: 
        :param float theta_max_deg: Max output of pitch controller inner loop

        - Heading

        :param float wn_d:      Damped natural frequency of input commands for low pass filter
        :param zeta_d:          Damping coefficient 
        :param r_max:           (?) Maximum yaw rate
        :param lam:             
        :param phi_b:           
        :param K_d:             (?) Derivative gain
        :param K_sigma:         (?) SMC switching gain

        - Surge

        :param kp_surge:        Porportional gain for surge
        :param ki_surge:        Integral gain for surge
        :param kd_surge:        Derivative gain for surge
    
        Actuator parameters: 

        - Fins
        
        :param fin_area:        Surface area of one side of a fin 
        :param x_fin:           Positive x distance from center of mass to actuation distance
        :param z_fin:           Positive Z distance from center of mass to fin center of pressure
        :param CL_delta_r:      Coefficient of lift for rudder 
        :param CL_delta_s:      Coefficient of lift for stern 
        :param deltaMax_fin_deg: Max deflection of the fin (degrees)
        :param fin_speed_max:   Max angular speed of the fin (rad/s)
        :param T_delta:         Time constant for fin actuation. (s)

        - Propellor

        :param D_prop:          Propeller diameter
        :param t_prop:          Propeller pitch
        :param KT_0:            Thrust coefficient at zero rpm
        :param KQ_0:            Torque coefficient at zero rpm
        :param KT_max:          Max thrust coefficient
        :param KQ_max:          Max torque coefficient
        :param w:               Wake fraction number
        :param Ja_max:          Max advance ratio
        :param nMax:            Max rpm of the thruster
        :param T_n:             Time constant for thruster actuation. (s)
        """

        self._scenario = scenario
        self.agent_name = vehicle_name
        

        ##### Initialize Default Vehicle parameters (REMUS100) #####
        self.dynamics_parameters ={
            # Environment parameters:
            "rho":          1026,   # Density of water in kg/m^3
            "sampleTime":   1/50,   # Length of timestep (s)

            # Vehicle physical parameters:
            "mass":         16,     # Mass of vehicle in kg
            "length":       1.6,    # Length of vehicle in m
            "diam":         0.19,   # Diameter of vehicle in m
            "r_bg": [0, 0, 0.02],   # Center of gravity of the vehicle (x, y, z) in body frame x forward, y right, z down
            "r_bb": [0, 0, 0],      # Center of boyancy of the vehicle (x, y, z) in body frame x forward, y right, z down
            "area_fraction": 0.7,   # relates vehicle effective area to length and width. pi/4 for a spheroid
            
            # Low-speed linear damping matrix parameters:
            "T_surge":      20,     # Surge time constant (s)
            "T_sway":       20,     # Sway time constant (s)
            "zeta_roll":    0.3,    # Roll damping ratio
            "zeta_pitch":   0.8,    # Pitch damping ratio
            "T_yaw":        1,      # Yaw time constant (s)
            "K_nomoto":     0.25,   # Nomoto gain

            # Other damping parameters:
            "r44":          0.3,    # Added moment of inertia in roll: A44 = r44 * Ix
            "Cd":           0.42,   # Coefficient of drag
            "e":            0.7,    # Oswald efficiency factor for vehicle drag
        }

        self.autopilot_parameters = {
            'depth': {
                'wn_d_z':   0.2,    # Damped natural frequency for low pass filter for depth commands
                'Kp_z':     0.1,    # Proportional gain for depth controller
                'T_z':      100,    # Time constant for depth controller
                'Kp_theta': 5.0,    # Proportional gain for pitch angle for depth controller
                'Kd_theta': 2.0,    # Derivative gain for pitch angle for depth controller
                'Ki_theta': 0.3,    # Integral gain for pitch angle for depth controller
                'K_w':      5.0,    # Optional heave velocity feedback gain
                'theta_max_deg': 30, # Max output of pitch controller inner loop
            },
            'heading': {
                'wn_d':     1.2,    # Damped natural frequency of input commands for low pass filter
                'zeta_d':   0.8,    # Damping coefficient
                'r_max':    0.9,    # (?) Maximum yaw rate
                'lam':      0.1,    # 
                'phi_b':    0.1,    # 
                'K_d':      0.5,    # (?) Derivative gain
                'K_sigma':  0.05,   # (?) SMC switching gain
            },
            'surge':{
                'kp_surge': 400.0,  # Proportional gain for surge
                'ki_surge': 50.0,   # Integral gain for surge
                'kd_surge': 30.0,   # Derivative gain for surge
            }
        }

        self.actuator_parameters = {
            # Fins: 
            "fin_area":     0.00697, # Surface area of one side of a fin
            "x_fin":       -0.8,    # Positive X distance from center of mass to actuation distance
            "z_fin":        0.07,   # Positive Z distance from center of mass to center of pressure
            "CL_delta_r":   0.5,    # Coefficient of lift for rudder
            "CL_delta_s":   0.7,    # Coefficient of lift for stern (elevators)
            "deltaMax_fin_deg": 20, # Max deflection of the fin (degrees)
            "fin_speed_max": 0.5,   # Max angular speed of the fin (rad/s)
            "T_delta":      0.1,    # Time constant for fin actuation. (s)

            # Propellor:
            "D_prop":       0.14,   # Propeller diameter
            "t_prop":       0.1,    # Propeller pitch
            "KT_0":         0.4566, # Thrust coefficient at zero rpm
            "KQ_0":         0.0700, # Torque coefficient at zero rpm
            "KT_max":       0.1798, # Max thrust coefficient
            "KQ_max":       0.0312, # Max torque coefficient
            "w":            0.056,  # wake fraction number
            "Ja_max":       0.6632, # Max advance ratio
            "nMax":         2000,   # Max rpm of the thruster
            "T_n":          0.1,    # Time constant for thruster actuation. (s)
        }   

        ##### Overwrite Parametes from Scenario #####
        if scenario is not None:
            if vehicle_name is None:
                raise ValueError("Vehicle name must be provided if a scenario is specified.")

            # Find the correct agent dictionary by agent_name
            agent_dict = None
            for agent in scenario.get('agents', []):
                if agent.get('agent_name') == vehicle_name:
                    agent_dict = agent
                    break

            if agent_dict is None:
                raise ValueError(f"No agent with name {vehicle_name} found in the scenario.")

            # Set vehicle parameters from the agent's 'dynamics' if it exists
            dynamics = agent_dict.get('dynamics')
            if dynamics is not None:
                self.set_vehicle_parameters(dynamics)
            else:
                self.set_vehicle_parameters(self.dynamics_parameters)
            
            # Set autopilot parameters from the agent's 'autopilot' if it exists
            autopilot_parameters = agent_dict.get('autopilot')
            if autopilot_parameters is not None:
                self.set_autopilot_parameters(autopilot_parameters)
            else:
                self.set_autopilot_parameters(self.autopilot_parameters)

            # Set actuator parameters from the agent's 'actuator' if it exists
            actuator = agent_dict.get('actuator')
            if actuator is not None:
                self.set_actuator_parameters(actuator)
            else:
                self.set_actuator_parameters(self.actuator_parameters)
        
        else:
            self.set_vehicle_parameters(self.dynamics_parameters)
            self.set_autopilot_parameters(self.autopilot_parameters)
            self.set_actuator_parameters(self.actuator_parameters)
            # this isn't the cleanest way to do this..?

    def set_vehicle_parameters(self, dynamics):
        """
        Set vehicle dynamics parameters. If not provided, will default to previous value
        """
        self.dynamics_parameters.update(dynamics)

        # Environment parameters:
        self.rho = self.dynamics_parameters.get('rho')
        self.sampleTime = self.dynamics_parameters.get('sampleTime')

        # Vehicle physical parameters:
        self.m = self.dynamics_parameters.get('mass')
        self.L = self.dynamics_parameters.get('length')
        self.diam = self.dynamics_parameters.get('diam')
        self.r_bg = np.array(self.dynamics_parameters.get('r_bg'))
        self.r_bb = np.array(self.dynamics_parameters.get('r_bb'))
        self.area_fraction = self.dynamics_parameters.get('area_fraction')

        # Low-speed linear damping matrix parameters:
        self.T_surge = self.dynamics_parameters.get('T_surge')
        self.T_sway = self.dynamics_parameters.get('T_sway')
        self.zeta_roll = self.dynamics_parameters.get('zeta_roll')
        self.zeta_pitch = self.dynamics_parameters.get('zeta_pitch')
        self.T_yaw = self.dynamics_parameters.get('T_yaw')
        self.K_nomoto = self.dynamics_parameters.get('K_nomoto')

        # Other damping parameters:
        self.r44 = self.dynamics_parameters.get('r44')
        self.Cd = self.dynamics_parameters.get('Cd')
        self.e = self.dynamics_parameters.get('e')
        

        # Use input parameters to calculate other vehicle paramaters
        self.calculate_additional_parameters()
      
    def calculate_additional_parameters(self):
        """
        After updating the vehicle parameters, calculations will be run 
        to update other variables related to these parameters.
        """

        #### VEHICLE GEOMETRY ####
        self.a = self.L/2
        self.b = self.diam/2

        ###### MASS MATRIX #######
        ### Rigid-body mass matrix expressed in CO ###
        # Assumes a spheroid body
        # m = 4/3 * np.pi * self.rho * a * b**2   # mass of spheriod
        Ix = (2/5) * self.m * self.b**2           # moment of inertia
        Iy = (1/5) * self.m * (self.a**2 + self.b**2)
        Iz = Iy
        MRB_CG = np.diag([self.m, self.m, self.m, Ix, Iy, Iz ]) # MRB expressed in the CG     
        H_rg = Hmtrx(self.r_bg)
        self.MRB = H_rg.T @ MRB_CG @ H_rg  # MRB expressed in the CO
        ### Added Mass Matrix
        MA_44 = self.r44 * Ix # added moment of inertia in roll: A44 = r44 * Ix
        # Lamb's k-factors
        e = np.sqrt( 1-(self.b/self.a)**2 )
        alpha_0 = ( 2 * (1-e**2)/pow(e,3) ) * ( 0.5 * np.log( (1+e)/(1-e) ) - e )  
        beta_0  = 1/(e**2) - (1-e**2) / (2*pow(e,3)) * np.log( (1+e)/(1-e) )
        k1 = alpha_0 / (2 - alpha_0)
        k2 = beta_0  / (2 - beta_0)
        k_prime = pow(e,4) * (beta_0-alpha_0) / ((2-e**2) * ( 2*e**2 - (2-e**2) * (beta_0-alpha_0) ) )   
        self.MA = np.diag([self.m*k1, self.m*k2, self.m*k2, MA_44, k_prime*Iy, k_prime*Iy ])
        ### Mass matrix including added mass
        self.M = self.MRB + self.MA
        self.Minv = np.linalg.inv(self.M)

        ##### GRAVITY VECTOR ######
        # Weight and buoyancy
        self.W = self.m * self.g
        self.B = self.W

        ##### HYDROYNAMICS ##### (Fossen 2021, Section 8.4.2) 
        # Parasitic drag: F_drag = 0.5 * rho * CD_0 * S
        self.S = self.area_fraction * self.L * self.diam  # Effective surface area for drag
        self.CD_0 = self.Cd * np.pi * self.b**2 / self.S  # Parasitic drag coefficient,i.e. drag at zero lift and alpha = 0
        # Natural frequencies in roll and pitch
        self.w_roll =  np.sqrt(self.W * (self.r_bg[2]-self.r_bb[2]) / self.M[3][3])
        self.w_pitch = np.sqrt(self.W * (self.r_bg[2]-self.r_bb[2]) / self.M[4][4])
        
        ###### DAMPING ######
        # Low-speed linear damping matrix parameters
        self.T_heave = self.T_sway  # Equal for for a cylinder-shaped AUV
        # Feed forward gains (Nomoto gain parameters)
        self.T_nomoto = self.T_yaw 

    def set_autopilot_parameters(self, autopilot):
        """
        Set depth and heading parameters from a configuration dictionary.

        :param cfg: Dictionary containing 'depth' and 'heading' sections with their respective parameters.
        """
        # Update depth parameters
        if 'depth' in autopilot:
            self.autopilot_parameters['depth'].update(autopilot['depth'])
        
        # Update heading parameters
        if 'heading' in autopilot:
            self.autopilot_parameters['heading'].update(autopilot['heading'])

        # Update heading parameters
        if 'surge' in autopilot:
            self.autopilot_parameters['surge'].update(autopilot['surge'])
        
        depth_cfg = self.autopilot_parameters.get('depth', {})
        heading_cfg = self.autopilot_parameters.get('heading', {})
        surge_cfg = self.autopilot_parameters.get('surge', {})

        #### Surge Parameters
        self.surge_control = False
        self.kp_surge = surge_cfg.get('kp_surge')
        self.ki_surge = surge_cfg.get('ki_surge')
        self.kd_surge = surge_cfg.get('kd_surge')
        self.u_int = 0   # surge error integral state
        self._last_error = 0

        #### Set depth parameters
        self.wn_d_z = depth_cfg.get('wn_d_z')   # desired natural frequency, reference mode
        self.Kp_z = depth_cfg.get('Kp_z')         # heave proportional gain, outer loop
        self.T_z = depth_cfg.get('T_z')            # heave integral gain, outer loop
        self.Kp_theta = depth_cfg.get('Kp_theta')    # pitch PID controller 
        self.Kd_theta = depth_cfg.get('Kd_theta')
        self.Ki_theta = depth_cfg.get('Ki_theta')
        self.K_w = depth_cfg.get('K_w')               # optional heave velocity feedback gain
        self.theta_max = np.deg2rad(depth_cfg.get('theta_max_deg'))               # optional heave velocity feedback gain
        

        # Heading autopilot (Equation 16.479 in Fossen 2021)
        # sigma = r-r_d + 2*lambda*ssa(psi-psi_d) + lambda^2 * integral(ssa(psi-psi_d))
        # delta = (T_nomoto * r_r_dot + r_r - K_d * sigma 
        #       - K_sigma * (sigma/phi_b)) / K_nomoto
        ##### heading parameters
        self.wn_d = heading_cfg.get('wn_d')      # desired natural frequency
        self.zeta_d = heading_cfg.get('zeta_d')    # desired realtive damping ratio
        self.r_max = heading_cfg.get('r_max')   # maximum yaw rate
        self.lam = heading_cfg.get('lam')
        self.phi_b = heading_cfg.get('phi_b')   # boundary layer thickness
        self.K_d = heading_cfg.get('K_d')         # PID gain
        self.K_sigma = heading_cfg.get('K_sigma') # SMC switching gain

        self.z_int = 0      # heave position integral state
        self.z_d = 0        # desired position, LP filter initial state
        self.theta_int = 0  # pitch angle integral state
        self.psi_d = 0      # desired heading from control loop
        self.r_d = 0        # desired yaw rate from control loop
        self.a_d = 0
        self.e_psi_int = 0  # yaw angle error integral state
        self.prev_pitch = 0
        self.prev_yaw = 0

    def set_actuator_parameters(self, actuator_parameters):
        """
        Set fin area limits, time constants, and lift coefficients for control surfaces. 
        Placeholder funtion, to be overridden by subclasses.
        """
        self.actuator_parameters.update(actuator_parameters)
        
        # Fin parameters
        self.S_fin = self.actuator_parameters.get('fin_area')
        self.x_fin = self.actuator_parameters.get('x_fin')
        self.z_fin = self.actuator_parameters.get('z_fin')
        self.CL_delta_r = self.actuator_parameters.get('CL_delta_r')
        self.CL_delta_s = self.actuator_parameters.get('CL_delta_s')
        self.deltaMax = np.radians(self.actuator_parameters.get('deltaMax_fin_deg'))
        self.fin_speed_max = self.actuator_parameters.get('fin_speed_max')
        self.T_delta = self.actuator_parameters.get('T_delta')
        
        # Propeller parameters
        self.D_prop = self.actuator_parameters.get('D_prop')
        self.t_prop = self.actuator_parameters.get('t_prop')
        self.KT_0 = self.actuator_parameters.get('KT_0')
        self.KQ_0 = self.actuator_parameters.get('KQ_0')
        self.KT_max = self.actuator_parameters.get('KT_max')
        self.KQ_max = self.actuator_parameters.get('KQ_max')
        self.w = self.actuator_parameters.get('w')
        self.Ja_max = self.actuator_parameters.get('Ja_max')       
        self.nMax = self.actuator_parameters.get('nMax')
        self.T_n = self.actuator_parameters.get('T_n')

    def set_control_mode(self, controlSystem, init_depth=False):
        """
        Sets the control mode for the vehicle.

        :param str controlSystem: The control system to use. Possible values are:
        
        - ``"depthHeadingAutopilot"``: Depth and heading autopilots.
        - ``"manualControl"``: Manual input control with set_u_control().
        - ``"stepInput"``: Step inputs for stern planes, rudder, and propeller
        - Any other value: controlSystem is set to "stepInput".

        :param bool init_depth: Whether to initialize depth (default is False).

        :returns: None
        """
        if controlSystem == "depthHeadingAutopilot":
            self.controlDescription = "Depth and heading autopilots"
            self.init_depth = init_depth
            self.z_int = 0
            self.e_psi_int = 0 
            self.u_int = 0
            print("Warning: Setting control mode resets controller so be careful to set control mode only when necessary")
        elif controlSystem == 'manualControl':
            self.controlDescription = 'Manual input control with set_u_control()'
        else:
            self.controlDescription = "Step inputs for stern planes, rudder and propeller"
            controlSystem = "stepInput"
        self.controlMode = controlSystem
        print(self.controlDescription)

    def set_goal(self, depth=None, heading=None, rpm=None, surge=None):
        """
        Set the goals for the autopilot.

        :param float depth: Desired depth (m), positive downwards.
        :param float heading: Desired yaw angle (deg). (-180 to 180)
        :param float rpm: Desired propeller revolution (rpm).
        :param float surge: Desired body frame x velocity (m/s).

        :returns: None
        """
        if rpm is not None:
            self.ref_n = rpm
            self.surge_control = False
            if rpm < 0.0 or rpm > self.nMax:
                raise ValueError(f"The RPM value should be in the interval 0-{self.nMax}")
        if heading is not None:
            self.ref_psi = heading
            if abs(heading) > 180.0:
                raise ValueError(f"The heading command value should be on the interval -180 to 180")
        if depth is not None:
            self.ref_z = depth
            if depth < 0.0:
                raise ValueError(f"The depth command value should be >= 0 (m)")
        if surge is not None:
            self.ref_u = surge
            self.surge_control = True

    def set_surge_goal(self, surge):
        """
        Set the surge goals for the autopilot.

        :param float depth: Desired surge (m/s), positive forward in body frame.

        :returns: None
        """
        #TODO add caps? negative values? and max surge?
        self.ref_u = surge
        self.surge_control = True

    def set_heading_goal(self, heading):
        """
        Set the heading goals for the autopilot.

        :param float depth: Desired heading (deg), -180 to 180 in NED frame

        :returns: None
        """
        

        self.ref_psi = heading
        if abs(heading) > 180.0:
            raise ValueError(f"The heading command value should be on the interval -180 to 180")

    def set_depth_goal(self, depth):
        """
        Set the depth goals for the autopilot.

        :param float depth: Desired depth (m), positive downward in world frame.

        :returns: None
        """

        self.ref_z = depth

        if depth < 0.0:
            raise ValueError(f"The depth command value should be >= 0 (m)")

    def set_rpm_goal(self, rpm):
        """
        Set the rpm goals for the autopilot.

        :param float depth: Desired rpm for thruster

        :returns: None
        """

        self.ref_n = rpm
        self.surge_control = False

        if rpm < 0.0 or rpm > self.nMax:
            raise ValueError(f"The RPM value should be in the interval 0-{self.nMax}")

    def surgeAutopilot(self, nu):
        #TODO: Check that this is working and grabbing the right variables

        u = nu[0]                # surge velocity
        # TODO: get nu_dot from linear acceleration IMU - gravity
        # udot = nu_dot[0]            # surge acceleration

        setpoint = self.ref_u
        error = setpoint - u
        derivative = (error - self._last_error) / self.sampleTime

        n = self.kp_surge * error + self.ki_surge * self.u_int + self.kd_surge * derivative

        self.u_int += self.sampleTime * (error)

        # Saturate to max RPM
        if n > self.nMax:
            n = self.nMax   
        # elif n < -self.nMax:
        #     n = -self.nMax
        
        return n

    ### DYNAMICS ###
    def dynamics(self, eta, nu, u_control):
        """
        Calculates vehicle and actuator accelerations based on the current state and control inputs.
        Acelerations are integrated in the step function to update the state.

        Inputs:
        :param array-like eta:       Current state/pose of the vehicle in the world frame.
        :param array-like nu:        Current velocity of the vehicle in the body frame.
        :param array-like u_control: Commanded control surface position.

        Outputs:
        :param array-like nu_dot:       Acceleration of the vehicle in the body frame.
        :param array-like u_actual_dot: Acceleration of the control surfaces.

        :returns: Two arrays: nu_dot, u_actual_dot.
        """
        self.eta = eta
        self.nu = nu

        ### Velocity of Vehicle and Current ###
        u_c = self.V_c * np.cos(self.beta_c - eta[5])  # current surge velocity
        v_c = self.V_c * np.sin(self.beta_c - eta[5])  # current sway velocity

        nu_c = np.array([u_c, v_c, 0, 0, 0, 0], float) # current velocity 
        Dnu_c = np.array([nu[5]*v_c, -nu[5]*u_c, 0, 0, 0, 0],float) # derivative
        nu_r = nu - nu_c                               # relative velocity between vehicle and current  
        alpha = np.arctan2( nu_r[2], nu_r[0] )         # angle of attack 
        U = np.sqrt(nu[0]**2 + nu[1]**2 + nu[2]**2)  # vehicle speed
        U_r = np.sqrt(nu_r[0]**2 + nu_r[1]**2 + nu_r[2]**2)  # relative speed

        ### Coriolis Matrix ###
        # Rigid-body/added mass Coriolis/centripetal matrices expressed in the CO
        CRB = m2c(self.MRB, nu_r)
        CA  = m2c(self.MA, nu_r)
        # CA-terms in roll, pitch and yaw can destabilize the model if quadratic
        # rotational damping is missing. These terms are assumed to be zero. 
        CA[4][0] = 0     # Quadratic velocity terms due to pitching
        CA[0][4] = 0  
        CA[2][4] = 0
        CA[5][0] = 0     # Munk moment in yaw
        CA[5][1] = 0
        CA[1][5] = 0
        C = CRB + CA

        ### Dissipative Matrix ###
        D = np.diag([
            self.M[0][0] / self.T_surge,
            self.M[1][1] / self.T_sway,
            self.M[2][2] / self.T_heave,
            self.M[3][3] * 2 * self.zeta_roll  * self.w_roll,
            self.M[4][4] * 2 * self.zeta_pitch * self.w_pitch,
            self.M[5][5] / self.T_yaw
            ])
        # Linear surge and sway damping
        D[0][0] = D[0][0] * np.exp(-3*U_r) # vanish at high speed where quadratic
        D[1][1] = D[1][1] * np.exp(-3*U_r) # drag and lift forces dominates

        ### Hydrodynamic Forces ###
        tau_liftdrag = forceLiftDrag(self.b, self.S, self.CD_0, alpha, U_r, self.rho, self.e) #b, S, CD_0, alpha, U_r, rho, e=0.7
        tau_crossflow = crossFlowDrag(self.L, self.diam, self.diam, nu_r)

        ### Restoring Forces ###
        g = gvect(self.W, self.B, eta[4], eta[3], self.r_bg, self.r_bb)

        ### Actuator Forces ###
        tau_actuators = self.actuator_forces(nu_r, U)


        ############### COMPUTE DYNAMICS #################
        # Vehicle accelerations
        tau_sum = tau_actuators + tau_liftdrag + tau_crossflow - np.matmul(C + D, nu_r) - g
        nu_dot = Dnu_c + np.matmul(self.Minv, tau_sum) # Acceleration from forces plus ocean current acceleration

        # Actuator accelerations
        u_actual_dot = self.actuator_dynamics(u_control) 

        self.move_actuators(u_actual_dot)
        self.saturate_actuators()

        return nu_dot
    
    def actuator_dynamics(self, u_control):
        """
        Vehicle-specific calculations for dynamics of the actuators (fins and thruster).

        Note: For Torpedo Vehicles, positive fin deflection will pitch the vehicle up and yaw to the starboard side.

        :param array-like self.u_actual: Current control surface position.
        :param array-like nu_r: Reference velocity of the vehicle in the body frame.

        :returns: array of actuator velocities/accelerations
        """

        # Actuator speed = (Actuator command - current actuator position) / time constant
        # The time constant is different for the fins and the thruster.
        u_dot = (u_control - self.u_actual) / np.concatenate((np.full(len(u_control)-1, self.T_delta), [self.T_n]))
        u_dot[:-1] = np.clip(u_dot[:-1], -self.fin_speed_max, self.fin_speed_max) # saturate fin speeds
        u_dot[-1] = np.clip(u_dot[-1], -self.nMax, self.nMax) # saturate thruster speed
        return u_dot

    def move_actuators(self, u_actual_dot, method='rk4'):
        
        u_actual = self.u_actual.copy()
        timestep = self.sampleTime # Time step for integration

        if method == 'euler':
            # Euler integration of actuator dynamics
            u_actual += timestep * u_actual_dot
        elif method == 'rk3':
            # RK3 integration of actuator dynamics
            k1 = u_actual_dot.copy()
            tempState = u_actual + timestep / 2 * k1
            next_u_actual_dot = self.actuator_dynamics(tempState)
            k2 = next_u_actual_dot.copy()
            tempState = u_actual + timestep / 2 * k2
            next_u_actual_dot = self.actuator_dynamics(tempState)
            k3 = next_u_actual_dot.copy()
            u_actual += timestep / 6 * (k1 + 4*k2 + k3)
        elif method == 'rk4':
            # RK4 integration of actuator dynamics
            k1 = u_actual_dot.copy()
            tempState = u_actual + timestep / 2 * k1
            next_u_actual_dot = self.actuator_dynamics(tempState)
            k2 = next_u_actual_dot.copy()
            tempState = u_actual + timestep / 2 * k2
            next_u_actual_dot = self.actuator_dynamics(tempState)
            k3 = next_u_actual_dot.copy()
            tempState = u_actual + timestep * k3
            next_u_actual_dot = self.actuator_dynamics(tempState)
            k4 = next_u_actual_dot.copy()
            u_actual += timestep / 6 * (k1 + 2*k2 + 2*k3 + k4)
        else:
            raise ValueError("method: {} not found, please use euler, rk3 or rk4".format(method))
        
        self.u_actual = u_actual.copy()

    def saturate_actuators(self):
        # Saturate fins
        self.u_actual[:-1] = np.clip(self.u_actual[:-1], -self.deltaMax, self.deltaMax)
        # Saturate thruster value  
        self.u_actual[-1] = np.clip(self.u_actual[-1], -self.nMax, self.nMax)
    
    def propellor_forces(self, U):
        n = self.u_actual[-1]
        n_rps = n / 60                  # propeller revolution (rps) 
        Va = (1-self.w) * U             # advance speed (m/s)  
        Ja = Va / (n_rps * self.D_prop) # advance ratio (dimensionless)      
        # Propeller thrust and propeller-induced roll moment      
        if n_rps > 0:   # forward thrust
            KT = self.KT_0 + (self.KT_max-self.KT_0)/self.Ja_max * Ja # Linear approximation for positive Ja values
            KQ = self.KQ_0 + (self.KQ_max-self.KQ_0)/self.Ja_max * Ja # Linear approximation for positive Ja values
        else:    # reverse thrust (braking)
            KT = self.KT_0
            KQ = self.KQ_0
        X_prop = self.rho * pow(self.D_prop, 4) * KT * abs(n_rps) * n_rps
        K_prop = self.rho * pow(self.D_prop, 5) * KQ * abs(n_rps) * n_rps

        # NOTE: Divide by 10 is from matching experimental results on the REMUS100. This may need to be adjusted. 
        return np.array([(1-self.t_prop)*X_prop, 0, 0, K_prop/10, 0, 0], float) # thrust and roll moment from propeller

    def actuator_forces(self, nu_r, U):
        tau_prop = self.propellor_forces(U) # propeller forces
        tau_fins = self.fin_forces(nu_r) # fin forces
        return tau_prop + tau_fins # sum of forces from propeller and fins

    ### The following three functions are used for standalone simulation outside of HoloOcean. They are included here only for reference. ###
    def update(self, command, method='euler'):
        # Calcuate the new velocities nu and position eta and the new control positions. 

        nu = self.nu.copy() # Vehicle velocity in body frame, array of size 6
        eta = self.eta.copy() # Vehicle position in global frame, array of size 6
        u_actual = self.u_actual.copy() # Vehicle control surface positions, array of size 6
        prior = np.concatenate((eta, nu, u_actual)) # Vehicle state, array of size 18
        timestep = self.sampleTime # Time step for integration

        if method == 'euler':
            next_nu_dot, next_u_actual_dot = self.dynamics(eta, nu, command)
            statedot = np.concatenate((nu, next_nu_dot, next_u_actual_dot))
            newState = self.stateEulerStep(prior, statedot, timestep)
            self.stateUpdate(newState)
        elif method == 'rk4':
            # rk4 has k1 = from prior, normal timestep to get statedot
            # k2 = prior + halfstep with k1, evaluated w half step
            # k3 = prior + halfstep with k2, evaluated with half step
            # k4 = prior + full step with k3, evaluated with full step
            # final is prior + timestep / 6 * (k1 + 2k2 + 2k3 + k4)
            next_nu_dot, next_u_actual_dot = self.dynamics(eta, nu, command)
            k1 = np.concatenate((nu, next_nu_dot, next_u_actual_dot))
            tempState = self.stateEulerStep(prior, k1, timestep/2)
            self.stateUpdate(tempState)
            next_nu_dot, next_u_actual_dot = self.dynamics(eta, nu, command)
            k2 = np.concatenate((nu, next_nu_dot, next_u_actual_dot))
            tempState = self.stateEulerStep(prior, k2, timestep/2)
            self.stateUpdate(tempState)
            next_nu_dot, next_u_actual_dot = self.dynamics(eta, nu, command)
            k3 = np.concatenate((nu, next_nu_dot, next_u_actual_dot))
            tempState = self.stateEulerStep(prior, k3, timestep)
            self.stateUpdate(tempState)
            next_nu_dot, next_u_actual_dot = self.dynamics(eta, nu, command)
            k4 = np.concatenate((nu, next_nu_dot, next_u_actual_dot))
            sumStateDot = (k1 + 2*k2 + 2*k3 + k4)/6
            final_state = self.stateEulerStep(prior, sumStateDot, eta, nu)
            self.stateUpdate(final_state)
        elif method == 'rk3':
            # rk3 has k1 = from prior, normal time step statedot
            # k2 = prior+halfstep with k1, evaluated with half step
            # k3 = prior+ halfstep with (k1+k2)/2, evaluated w half step
            # final is prior + normal timestep / 6 * (k1+4k2+k3)
            next_nu_dot, next_u_actual_dot = self.dynamics(eta, nu, command)
            k1 = np.concatenate((nu, next_nu_dot, next_u_actual_dot))
            tempState = self.stateEulerStep(prior, k1, timestep/2)
            self.stateUpdate(tempState)
            next_nu_dot, next_u_actual_dot = self.dynamics(eta, nu, command)
            k2 = np.concatenate((nu, next_nu_dot, next_u_actual_dot))
            tempState = self.stateEulerStep(prior, (k1+k2)/2, timestep/2)
            self.stateUpdate(tempState)
            next_nu_dot, next_u_actual_dot = self.dynamics(eta, nu, command)
            k3 = np.concatenate((nu, next_nu_dot, next_u_actual_dot))
            sumStateDot = (k1+4*k2+k3)/6
            final_state = self.stateEulerStep(prior, sumStateDot, timestep)
            self.stateUpdate(final_state)
        else:
            raise ValueError("method: {} not found, please use euler, rk3 or rk4".format(method))
    
    def stateUpdate(self, state):
        self.eta = state[:6].copy()
        self.nu = state[6:12].copy()
        self.u_actual = self.saturate_actuators(state[12:]).copy()
    
    def stateEulerStep(self, state, state_dot, timeStep):
        new_state = np.zeros_like(state)
        state_dot_transformed = state_dot
        state_dot_transformed[:6] = velocityTransform(state[:6],state_dot[0:6])
        new_state = state + timeStep * state_dot_transformed
        return new_state


    ################ Functions implemented in subclasses below: #################
    def fin_forces(self, nu_r, U):
        """
        Vehicle-specific calculations for forces of the actuators (fins and thruster).
        Placeholder funtion, to be overridden by subclasses.

        NOTE: 
        - For rudder, positive command turns the rudder CCW. Yaws vehicle right, negative roll.
        - For left stern, positive command turns the stern CCW. Pitches vehicle up in coordination, negative roll. 
        - For right stern, positive command turns the stern CW.  Pitches vehicle up in coordination, positive roll.

        :param array-like self.u_actual: Current control surface position.
        :param array-like nu_r: Reference velocity of the vehicle in the body frame.
        :param float U: forward speed of the vehicle

        :returns: (6,) numpy array of forces and moments
        """
        pass

    def stepInput(self, t):
        """
        Generates a pre-defined step input to the actuators for testing purposes.
        Placeholder funtion, to be overridden by subclasses.

        :param float t: Time parameter.

        :returns: The control input u_control.
        """
        pass 
    
    def depthHeadingAutopilot(self, eta, nu, imu=True):
        """
        Simultaneously control the heading and depth of the AUV using control laws of PID type.
        Propeller rpm is given as a step command.

        :param array-like eta:        State/pose of the vehicle in the world frame. (RPY - Euler angle order zyx in radians)
        :param array-like nu:         Velocity of the vehicle in the body frame.

        :returns: The control input u_control.
        """
        z = eta[2]                  # heave position (depth)
        theta = eta[4]              # pitch angle (Radians)
        psi = eta[5]                # yaw angle   (Radians)
        w = nu[2]                   # heave velocity

        if imu:
            q = nu[4]               # pitch rate
            r = nu[5]               # yaw rate
        else:
            q = (psi - self.prev_pitch) / self.sampleTime
            r = (theta - self.prev_yaw) / self.sampleTime
            self.prev_pitch = theta
            self.prev_yaw = psi

        e_psi = psi - self.psi_d    # yaw angle tracking error
        e_r   = r - self.r_d        # yaw rate tracking error
        z_ref = self.ref_z          # heave position (depth) setpoint
        psi_ref = self.ref_psi * self.D2R   # yaw angle setpoint
        
        # If surge command is 0 then control loop should not run 
        if self.ref_n > 0 or self.ref_u > 0:
            #######################################################################
            # Propeller command
            #######################################################################
            if self.surge_control:
                n = self.surgeAutopilot(nu)
            else:
                n = self.ref_n 
            
            #######################################################################            
            # Depth autopilot (succesive loop closure)
            #######################################################################
            # LP filtered desired depth command 
            if not self.init_depth:
                self.z_d = z    # On initialization of the autopilot the commanded depth is set to the current depth
                self.init_depth = True
            self.z_d  = np.exp(-self.sampleTime * self.wn_d_z) * self.z_d \
                + (1 - np.exp(-self.sampleTime * self.wn_d_z)) * z_ref  
                
            # PI controller    
            theta_d = self.Kp_z * ((z - self.z_d) + (1/self.T_z) * self.z_int)

            if abs(theta_d) > self.theta_max:
                theta_d = np.sign(theta_d) * self.theta_max

            delta_s = -self.Kp_theta * ssa(theta - theta_d) - self.Kd_theta * q \
                - self.Ki_theta * self.theta_int - self.K_w * w

            # Euler's integration method (k+1)
            self.z_int     += self.sampleTime * (z - self.z_d)
            self.theta_int += self.sampleTime * ssa(theta - theta_d)

            #######################################################################
            # Heading autopilot (SMC controller)
            #######################################################################
            
            wn_d = self.wn_d            # reference model natural frequency
            zeta_d = self.zeta_d        # reference model relative damping factor


            # Integral SMC with 3rd-order reference model
            [delta_r, self.e_psi_int, self.psi_d, self.r_d, self.a_d] = \
                integralSMC( 
                    self.e_psi_int, 
                    e_psi, e_r, 
                    self.psi_d, 
                    self.r_d, 
                    self.a_d, 
                    self.T_nomoto, 
                    self.K_nomoto, 
                    wn_d, 
                    zeta_d, 
                    self.K_d, 
                    self.K_sigma, 
                    self.lam,
                    self.phi_b,
                    psi_ref, 
                    self.r_max, 
                    self.sampleTime 
                    )
                    
            # Euler's integration method (k+1)
            self.e_psi_int += self.sampleTime * ssa(psi - self.psi_d)
            
            
            u_control = np.array([delta_r, delta_s, n], float)

        else:
            u_control = np.array([0.0, 0.0, 0.0], float)

        return u_control

    
    



class fourFinDep(TAUV):
    """
    Torpedo Vehicle with four fins where two fins move together on same plane (Rudder, Stern)
    """
    def __init__(self, scenario=None, vehicle_name=None, controlSystem="stepInput", r_z=0, r_psi=0, r_rpm=0, V_current=0, beta_current=0):
        self.controls = [
            "Tail rudder (deg)",
            "Stern plane (deg)",
            "Propeller revolution (rpm)"
            ]

        super().__init__(scenario, vehicle_name, controlSystem, r_z, r_psi, r_rpm, V_current, beta_current)
        self.u_actual = np.array([0, 0, 0], float)  # control input vector
        self.A_fin = 2 * self.S_fin # Effective fin area (m^2)

    def fin_forces(self, nu_r):
        """
        Vehicle-specific calculations for forces of the actuators (fins and thruster).
        
        :param array-like self.u_actual: Current control surface position.
        :param array-like nu_r: Reference velocity of the vehicle in the body frame.
        :param float U: forward speed of the vehicle

        :returns: (6,) numpy array of forces and moments
        """
    
        delta_r = self.u_actual[0] # Actual tail rudder (rad)
        delta_s = self.u_actual[1] # Actual stern plane (rad)

        ################### FIN CALCULATIONS #########################
        # Relative speed for rudder and stern
        U_rr = np.sqrt(nu_r[0]**2 + nu_r[1]**2)
        U_rs = np.sqrt(nu_r[0]**2 + nu_r[2]**2)

        # Rudder & Stern Drag
        X_r = -0.5 * self.rho * U_rr**2 * self.A_fin * self.CL_delta_r * delta_r**2 
        X_s = -0.5 * self.rho * U_rs**2 * self.A_fin * self.CL_delta_s * delta_s**2 

        # Rudder & Stern Lift
        Y_r = -0.5 * self.rho * U_rr**2 * self.A_fin * self.CL_delta_r * delta_r     # Rudder sway force (Positive deflection yaws vehicle to starboard)
        Z_s =  0.5 * self.rho * U_rs**2 * self.A_fin * self.CL_delta_s * delta_s     # Stern-plane heave force (Postive deflection pitches vehicle up)

        ################# Add Forces and Moments #################
        fx =  X_r + X_s
        fy =  Y_r
        fz =  Z_s
        Mx =  0                 # Extra roll moment cancelled by paired fins
        My = -self.x_fin * fz   # -1 comes from the cross product of x with z                           
        Mz =  self.x_fin * fy 

        return np.array([fx, fy, fz, Mx, My, Mz], float)

    def stepInput(self, t):
        """
        A pre-defined step input to the vehicle controls for testing purposes.

        Returns:
            list:
                The control input u_control as a list: [delta_r, delta_s, n], where:
                
                - delta_r (float): Rudder angle (rad)
                - delta_s (float): Stern plane angle (rad)
                - n (float): Propeller revolution (rpm)
        """
        delta_r =  15 * self.D2R      # rudder angle (rad)
        delta_s =  0 * self.D2R      # stern angle (rad)
        n = 1525                     # propeller revolution (rpm)
        
        if t > 100:
            delta_r = 0
            
        if t > 50:
            delta_s = 0     

        u_control = np.array([ delta_r, delta_s, n], float)

        return u_control
    
    #TODO: Seperate control loop into 3 different functions
    def depthHeadingAutopilot(self, eta, nu, imu=True):
        """
        Returns:
            list:
                The control input u_control as a list: [delta_r, delta_s, n], where:
                
                - delta_r (float): Rudder angle (rad)
                - delta_s (float): Stern plane angle (rad)
                - n (float): Propeller revolution (rpm)
        """
        return super().depthHeadingAutopilot(eta, nu, imu)
    

class fourFinInd(TAUV):
    """
    Torpedo vehicle with four independetly controlled fins (Rudder Top, Rudder Bottom, Stern left, Stern Right)
    """

    def __init__(self, scenario=None, vehicle_name=None, controlSystem="stepInput", r_z=0, r_psi=0, r_rpm=0, V_current=0, beta_current=0):
        self.controls = [
            "Top Tail rudder (deg)",
            "Bottom Tail rudder (deg)",
            "Left Stern (deg)",
            "Right Stern  (deg)",
            "Propeller revolution (rpm)"
            ]
        
        super().__init__(scenario, vehicle_name, controlSystem, r_z, r_psi, r_rpm, V_current, beta_current)
        self.u_actual = np.array([0, 0, 0, 0, 0], float)  # control input vector

    def fin_forces(self, nu_r):
        """
        Vehicle-specific calculations for forces of the actuators (fins and thruster).
        The implementation below is for a three-fin vehicle. 
        NOTE: 
        - For rudder, positive command turns the rudder CCW. Yaws vehicle starboard, negative roll.
        - For left stern, positive command turns the stern CCW. Pitches vehicle up in coordination, negative roll. 
        - For right stern, positive command turns the stern CW. Pitches vehicle up in coordination, positive roll.

        :param array-like nu_r: Reference velocity of the vehicle in the body frame.
        :param float U: forward speed of the vehicle

        :returns: (6,) numpy array of forces and moments
        """

        delta_rt = self.u_actual[0] # actual tail rudder top (rad)
        delta_rb = self.u_actual[1] # actual tail rudder bottom (rad)
        delta_sl = self.u_actual[2] # actual stern plane left (rad)
        delta_sr = self.u_actual[3] # actual stern plane right (rad)

        ################### FIN CALCULATIONS #########################
        # Relative speeds for each fin   
        U_rr = np.sqrt(nu_r[0]**2 + nu_r[1]**2)
        U_rs = np.sqrt(nu_r[0]**2 + nu_r[2]**2)

        # Rudder and stern drag (Always in negative direction regardless of fin deflection sign)
        X_rt = -0.5 * self.rho * U_rr**2 * self.S_fin * self.CL_delta_r * delta_rt**2
        X_rb = -0.5 * self.rho * U_rr**2 * self.S_fin * self.CL_delta_r * delta_rb**2
        X_st = -0.5 * self.rho * U_rs**2 * self.S_fin * self.CL_delta_s * delta_sl**2
        X_sb = -0.5 * self.rho * U_rs**2 * self.S_fin * self.CL_delta_s * delta_sr**2

        # Rudder sway force (Positive deflection causes negative force in Y body frame NED, yaw to starboard)
        Y_rt = -0.5 * self.rho * U_rr**2 * self.S_fin * self.CL_delta_r * delta_rt
        Y_rb = -0.5 * self.rho * U_rr**2 * self.S_fin * self.CL_delta_r * delta_rb

        # Stern-plane heave force (Postive deflection causes positve force in Z body frame NED, pitches up)
        Z_sl = 0.5 * self.rho * U_rs**2 * self.S_fin * self.CL_delta_s * delta_sl
        Z_sr = 0.5 * self.rho * U_rs**2 * self.S_fin * self.CL_delta_s * delta_sr

        ################# Add Forces and Moments #################
        fx = X_rt + X_rb + X_st + X_sb
        fy = Y_rt + Y_rb
        fz = Z_sl + Z_sr
        Mx =  self.z_fin * (Y_rb - Y_rt + Z_sr - Z_sl) # Rolling moment from the fins
        My = -self.x_fin * (Z_sr + Z_sl) # -1 comes from the cross product of x with z                           
        Mz =  self.x_fin * (Y_rt + Y_rb)

        tau = np.array([fx, fy, fz, Mx, My, Mz], float)
        return tau

    def stepInput(self, t):
        """
        A pre-defined step input to the vehicle controls for testing purposes.

        Returns:
            list:
                The control input u_control as a list: [delta_rt, delta_rb, delta_sl, delta_sr, n], where:
                
                - delta_rt: Rudder top angle (rad).
                - delta_rb: Rudder bottom angle (rad).
                - delta_sl: Stern left angle (rad).
                - delta_sr: Stern right angle (rad).
                - n: Propeller revolution (rpm).
        """
        delta_rt =  5 * self.D2R      # rudder angle (rad)
        delta_rb =  5 * self.D2R      # rudder angle (rad)
        delta_sl = -5 * self.D2R      # stern angle (rad)
        delta_sr = -5 * self.D2R      # stern angle (rad)
        n = 1525                     # propeller revolution (rpm)
        
        if t > 100:
            delta_rt = 0
            delta_rb = 0
           
        if t > 50:
            delta_sl = 0     
            delta_sr = 0     
        u_control = np.array([delta_rt, delta_rb, delta_sl, delta_sr, n], float)

        return u_control
    
    def depthHeadingAutopilot(self, eta, nu, imu=True):
        """
        Returns:
            list:
                The control input u_control as a list: [delta_rt, delta_rb, delta_sl, delta_sr, n], where:

                - delta_rt: Rudder top angle (rad).
                - delta_rb: Rudder bottom angle (rad).
                - delta_sl: Stern left angle (rad).
                - delta_sr: Stern right angle (rad).
                - n: Propeller revolution (rpm).
        """
        # Treat the fins as dependant
        control = super().depthHeadingAutopilot(eta, nu, imu)
        delta_rt = delta_rb = control[0]
        delta_sl = delta_sr = control[1]
        n = control[2]

        u_control = np.array([delta_rt, delta_rb, delta_sl, delta_sr, n], float)
        return u_control


class threeFinInd(TAUV):
    """
    Torpedo vehicle with four independently controlled fins (Rudder Top, Rudder Bottom, Stern left, Stern Right)
    """

    def __init__(self, scenario=None, vehicle_name=None, controlSystem="stepInput", r_z=0, r_psi=0, r_rpm=0, V_current=0, beta_current=0):
        self.controls = [
            "Tail rudder (rad)",
            "Left stern (rad)",
            "Right stern (rad)",
            "Propeller revolution (rpm)"
            ]
        
        super().__init__(scenario, vehicle_name, controlSystem, r_z, r_psi, r_rpm, V_current, beta_current)
        self.u_actual = np.array([0, 0, 0, 0], float)  # control input vector

    def fin_forces(self, nu_r):
        """
        Vehicle-specific calculations for forces of the actuators (fins and thruster).
        The implementation below is for a three-fin vehicle. 
        NOTE: 
        - For rudder, positive command turns the rudder CCW. Yaws vehicle starboard, negative roll.
        - For left stern, positive command turns the stern CCW. Pitches vehicle up in coordination, negative roll. 
        - For right stern, positive command turns the stern CW. Pitches vehicle up in coordination, positive roll.

        :param array-like nu_r: Reference velocity of the vehicle in the body frame.
        :param float U: forward speed of the vehicle

        :returns: (6,) numpy array of forces and moments
        """

        delta_r = self.u_actual[0]       # actual tail rudder (rad)
        delta_sr = self.u_actual[1]      # actual right stern (rad)
        delta_sl = self.u_actual[2]      # actual left stern (rad)

        ################### FIN CALCULATIONS #########################
        # Relative speeds for each fin   
        U_rr = np.sqrt(nu_r[0]**2 + nu_r[1]**2)
        U_rs = np.sqrt(nu_r[0]**2 + (nu_r[1] * np.sin(self.D2R * 30))**2 + (nu_r[2] * np.sin(self.D2R * 60))**2)

        # Lift forces on the stern fins on right and left both positive; set direction below
        fl_sr = 0.5 * self.rho * U_rs**2 * self.S_fin * self.CL_delta_s * delta_sr
        fl_sl = 0.5 * self.rho * U_rs**2 * self.S_fin * self.CL_delta_s * delta_sl

        # Rudder and stern drag [TODO: these don't look right?]
        X_r = -0.5 * self.rho * U_rr**2 * self.S_fin * self.CL_delta_r * delta_r**2
        X_sr = -0.5 * self.rho * U_rs**2 * self.S_fin * self.CL_delta_s * delta_sr**2
        X_sl = -0.5 * self.rho * U_rs**2 * self.S_fin * self.CL_delta_s * delta_sl**2

        # Rudder and stern sway force (Positive deflection -> negative Y force -> positive Z moment (yaw right))
        Y_r = -0.5 * self.rho * U_rr**2 * self.S_fin * self.CL_delta_r * delta_r
        Y_sr = -fl_sr * np.sin(30 * self.D2R)
        Y_sl = fl_sl * np.sin(30 * self.D2R)  

        # stern heave force  (positive z force)
        Z_sr = fl_sr * np.sin(60 * self.D2R)     
        Z_sl = fl_sl * np.sin(60 * self.D2R)

        ################# Add Forces and Moments #################
        fx = X_r + X_sr + X_sl
        fy = Y_r + Y_sr + Y_sl
        fz = Z_sr + Z_sl
        Mx = self.z_fin * (fl_sr - fl_sl - Y_r)  # Rolling moment from the fins
        My = -self.x_fin * fz # -1 comes from the cross product of x with z                           
        Mz =  self.x_fin * (Y_r + Y_sr + Y_sl)

        tau = np.array([fx, fy, fz, Mx, My, Mz], float)
        return tau

    def stepInput(self, t):
        """
        A pre-defined step input to the vehicle controls for testing purposes.

        Returns:
            list: [delta_r, delta_rb, delta_sl, delta_sr, n], where:

            - delta_r: Rudder top angle (rad).
            - delta_re: right stern angle (rad).
            - delta_le: left stern angle (rad).
            - n: Propeller revolution (rpm).
        """
        delta_r =  5 * self.D2R      # rudder angle (rad)
        delta_re = 0 * self.D2R      # right stern angle (rad)
        delta_le = 0 * self.D2R      # left stern angle (rad)  
        n = 1000 #1525                    # propeller revolution (rpm)
        
        if t > 50:
            delta_r = 0
            
        if t > 25:
            delta_re = 0     
            delta_le = 0     

        u_control = np.array([ delta_r, delta_re, delta_le, n], float)

        return u_control
    
    def depthHeadingAutopilot(self, eta, nu, imu=True):
        """
        Returns:
            list: [delta_r, delta_rb, delta_sl, delta_sr, n], where:
            
            - delta_r: Rudder top angle (rad).
            - delta_re: right stern angle (rad).
            - delta_le: left stern angle (rad).
            - n: Propeller revolution (rpm).
        """
        control = super().depthHeadingAutopilot(eta, nu, imu)
        delta_r = control[0]
        delta_re = delta_le = control[1]
        n = control[2]

        u_control = np.array([delta_r, delta_re, delta_le, n], float)
        return u_control