from enum import Enum
import gymnasium as gym
from gymnasium import spaces
import pygame
import numpy as np
import time
import pybullet as p
import pybullet_data
import math
import importlib.resources


SIM_F = 60.0
TIME_STEP = 1.0/SIM_F
OBSERVATION_TYPE=np.float32
ACTION_TYPE = np.float32


def calculate_ddt(theta, r, w, phi, nu, R, alpha):
    """Given the current state variables theta, r and network parameters
    w, phi, nu, R, alpha, calculate the time derivatives of theta and r."""
    intrinsic_term = 2 * np.pi * nu
    phase_diff = theta[np.newaxis, :] - theta[:, np.newaxis]
    coupling_term = (r * w * np.sin(phase_diff - phi)).sum(axis=1)
    dtheta_dt = intrinsic_term + coupling_term
    dr_dt = alpha * (R - r)
    return dtheta_dt, dr_dt

class CPGNetwork:
    def __init__(
        self,
        timestep,
        intrinsic_freqs,
        intrinsic_amps,
        coupling_weights,
        phase_biases,
        convergence_coefs,
        init_phases=None,
        init_magnitudes=None,
        seed=0,
    ) -> None:
        """Initialize a CPG network consisting of N oscillators.

        Parameters
        ----------
        timestep : float
            The timestep of the simulation.
        intrinsic_freqs : np.ndarray
            The intrinsic frequencies of the oscillators, shape (N,).
        intrinsic_amps : np.ndarray
            The intrinsic amplitude of the oscillators, shape (N,).
        coupling_weights : np.ndarray
            The coupling weights between the oscillators, shape (N, N).
        phase_biases : np.ndarray
            The phase biases between the oscillators, shape (N, N).
        convergence_coefs : np.ndarray
            Coefficients describing the rate of convergence to oscillator
            intrinsic amplitudes, shape (N,).
        init_phases : np.ndarray, optional
            Initial phases of the oscillators, shape (N,). The phases are
            randomly initialized if not provided.
        init_magnitudes : np.ndarray, optional
            Initial magnitudes of the oscillators, shape (N,). The
            magnitudes are randomly initialized if not provided.
        seed : int, optional
            The random seed to use for initializing the phases and
            magnitudes.
        """
        self.timestep = timestep
        self.num_cpgs = intrinsic_freqs.size
        self.intrinsic_freqs = intrinsic_freqs
        self.intrinsic_amps = intrinsic_amps
        self.coupling_weights = coupling_weights
        self.phase_biases = phase_biases
        self.convergence_coefs = convergence_coefs
        self.random_state = np.random.RandomState(seed)

        self.reset(init_phases, init_magnitudes)

        # Check if the parameters have the right shape
        assert intrinsic_freqs.shape == (self.num_cpgs,)
        assert coupling_weights.shape == (self.num_cpgs, self.num_cpgs)
        assert phase_biases.shape == (self.num_cpgs, self.num_cpgs)
        assert convergence_coefs.shape == (self.num_cpgs,)
        assert self.curr_phases.shape == (self.num_cpgs,)
        assert self.curr_magnitudes.shape == (self.num_cpgs,)

    def set_coupling_weights(self, coupling_weights):
        self.coupling_weights = coupling_weights * 40

    def set_phase_biases(self, phase_biases):
        self.phase_biases = phase_biases * 2 * np.pi

    def step(self):
        """Integrate the ODEs using Euler's method."""
        dtheta_dt, dr_dt = calculate_ddt(
            theta=self.curr_phases,
            r=self.curr_magnitudes,
            w=self.coupling_weights,
            phi=self.phase_biases,
            nu=self.intrinsic_freqs,
            R=self.intrinsic_amps,
            alpha=self.convergence_coefs,
        )
        self.curr_phases += dtheta_dt * self.timestep
        self.curr_phases %= 2*np.pi
        self.curr_magnitudes += dr_dt * self.timestep

        return self.curr_phases, self.curr_magnitudes

    def reset(self, init_phases=None, init_magnitudes=None):
        """Reset the phases and magnitudes of the oscillators.
        High magnitudes and unfortunate phases might cause physics error
        """
        if init_phases is None:
            self.curr_phases = self.random_state.random(self.num_cpgs) * 2 * np.pi
        else:
            self.curr_phases = init_phases

        if init_magnitudes is None:
            self.curr_magnitudes = np.zeros(self.num_cpgs)
        else:
            self.curr_magnitudes = init_magnitudes

class SpidyEnvV3(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array", None]}

    def __init__(self, render_mode=None):
        super().__init__()
        

        self.observation_space = spaces.Box(low=0.,high= 1., shape=(2,12,12), dtype=OBSERVATION_TYPE)
        
        self.action_space = spaces.Box(low=0.,high= 1., shape=(2,12,12), dtype=ACTION_TYPE)
        

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

        self._distance = 0.0

        # CPG
        num_cpg = 12
        intrinsic_freqs = np.ones(num_cpg)
        amp_x = 0.04
        amp_z = 0.02
        intrinsic_amps = np.array([amp_x,amp_z,amp_x,amp_z,amp_x,amp_z,amp_x,amp_z,amp_x,amp_z,amp_x,amp_z])
        coupling_weights = np.array(
        [
            [0,1,1,0,1,0,0,0,0,0,0,0],
            [1,0,0,1,0,1,0,0,0,0,0,0],
            [1,0,0,1,1,0,1,0,0,0,0,0],
            [0,1,1,0,0,1,0,1,0,0,0,0],
            [1,0,1,0,0,1,1,0,1,0,0,0],
            [0,1,0,1,1,0,0,1,0,1,0,0],
            [0,0,1,0,1,0,0,1,1,0,1,0],
            [0,0,0,1,0,1,1,0,0,1,0,1],
            [0,0,0,0,1,0,1,0,0,1,1,0],
            [0,0,0,0,0,1,0,1,1,0,0,1],
            [0,0,0,0,0,0,1,0,1,1,0,1],
            [0,0,0,0,0,0,0,1,0,0,1,0],
        ])*40
        xz = 270
        mir = 180
        phase_biases = np.deg2rad(
            np.array(
                [
                    [0,xz,mir,0,0,0,0,0,0,0,0,0],
                    [xz,0,0,mir,0,0,0,0,0,0,0,0],
                    [mir,0,0,xz,mir,0,0,0,0,0,0,0],
                    [0,mir,xz,0,0,mir,0,0,0,0,0,0],
                    [0,0,mir,0,0,xz,mir,0,0,0,0,0],
                    [0,0,0,mir,xz,0,0,mir,0,0,0,0],
                    [0,0,0,0,mir,0,0,xz,mir,0,0,0],
                    [0,0,0,0,0,mir,xz,0,0,mir,0,0],
                    [0,0,0,0,0,0,mir,0,0,xz,mir,0],
                    [0,0,0,0,0,0,0,mir,xz,0,0,mir],
                    [0,0,0,0,0,0,0,0,mir,0,0,xz],
                    [0,0,0,0,0,0,0,0,0,mir,xz,0],
                ]
            ))
        
        convergence_coefs = np.ones(num_cpg)
        self.cpg_network = CPGNetwork(
            timestep=TIME_STEP,
            intrinsic_freqs=intrinsic_freqs,
            intrinsic_amps=intrinsic_amps,
            coupling_weights=coupling_weights,
            phase_biases=phase_biases,
            convergence_coefs=convergence_coefs,
        )
        

        # PyBullet
        p.connect(p.GUI if self.render_mode == "human" else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)  # Use step simulation for better control
        p.setTimeStep(TIME_STEP)  # Set a higher time step for smoother simulation
        p.setPhysicsEngineParameter(numSolverIterations=50) 
        camera_target_position = [0, 0, 0] # Position to look at
        camera_distance = 1 # Distance from the target
        camera_yaw = 165 # Yaw angle
        camera_pitch = -40 # Pitch angle

        # Calculate the camera position
        camera_position = [
        camera_target_position[0] + camera_distance * math.sin(math.radians(camera_yaw)) * math.cos(math.radians(camera_pitch)),
        camera_target_position[1] + camera_distance * math.sin(math.radians(camera_pitch)),
        camera_target_position[2] + camera_distance * math.cos(math.radians(camera_yaw)) * math.cos(math.radians(camera_pitch))
        ]
        # Set the camera
        p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=camera_target_position)

        # Load the plane and the mill URDF model
        plane_id = p.loadURDF("plane.urdf")
        urdf_path = importlib.resources.files('babybot01_env.data') / 'hexapod.urdf'
        self._robot_id = p.loadURDF(str(urdf_path), useFixedBase=0, basePosition=[0, 0, 0.2], baseOrientation=[0, 0, 0, 1],)

        # debug
        #self._button_id = p.addUserDebugParameter("value",-1.0,1.0,0.0)

        # Joints info
        joint_index = {}
        self._revolute_joint_index = {}
        self.end_effector_index = {}
        numJoints = p.getNumJoints(self._robot_id)
        for i in range(numJoints):
            jointInfo = p.getJointInfo(self._robot_id, i)
            key = jointInfo[1].decode('utf-8')
            joint_index[key] = i
            if key[0] == 'u' or key[0] == 'v' or key[0] == 'w':
                self._revolute_joint_index[key] = i
            elif key[0] == 'x':
                self.end_effector_index[key] = i
        self._num_revolute_joint = len(self._revolute_joint_index)

        # Change dynamics
        for idx in self._revolute_joint_index.values():
            p.changeDynamics(
                bodyUniqueId=self._robot_id,
                linkIndex=idx,
                jointDamping=0.1             # Damping specifically for joints
            )
        for idx in self.end_effector_index.values():
            p.changeDynamics(
                bodyUniqueId=self._robot_id,
                linkIndex=idx,
                lateralFriction=1,         # Lateral friction coefficient
                contactStiffness=10.0,     # Stiffness of contact constraints
                contactDamping=0.5,          # Damping of contact constraints
            )

        self.end_effector_world_position = [0,0,0,0,0,0]
        self.last_ep_mean_rew =0
        

    
    def _get_obs(self):

        return [self.cpg_network.coupling_weights / 40, self.cpg_network.phase_biases / (2*np.pi)]
    
    def _get_info(self):
        return {
            "distance": self._distance,
            "curr_phases": self.cpg_network.curr_phases,
            "curr_magnitudes": self.cpg_network.curr_magnitudes,
            "coupling_weights": self.cpg_network.coupling_weights,
            "phase_biases": self.cpg_network.phase_biases
        }
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        p.resetBasePositionAndOrientation(self._robot_id,[0,0,0.2],[0,0,0,1])
        for i in self._revolute_joint_index.values():
            p.resetJointState(self._robot_id, i, 0)

        observation = self._get_obs()
        info = self._get_info()
        self._steps = 0

        self.cum_rew = 0
        #print(f"max episode steps: {self.spec.max_episode_steps}")

        return observation, info

    def step(self, action):

        # Action: modify CPG network parameters
        coupling_weights = action[0]
        phase_biases = action[1]
        self.cpg_network.set_coupling_weights(coupling_weights)
        self.cpg_network.set_phase_biases(phase_biases)
        
        # Compute CPG values
        phases, magnitudes = self.cpg_network.step()
        values = np.sin(phases)

        # Compute XZ pos according to base position and CPG 
        x1 = 0.16
        y2 = 0.24
        y1 = 0.16
        z1 = -0.07
        base_positions = [[x1,y1,z1],[0,y2,z1],[-x1,y1,z1],[-x1,-y1,z1],[0,-y2,z1],[x1,-y1,z1]]
        for leg_i in range(12 //2):
            cpg_i = leg_i *2
            base_positions[leg_i][0] += values[cpg_i] * magnitudes[cpg_i]
            base_positions[leg_i][2] += values[cpg_i+1] * magnitudes[cpg_i+1]

        # Get robot position and orientation
        robot_state = p.getLinkState(self._robot_id,0)
        robot_position = list(robot_state[0])
        robot_orientation = list(robot_state[1])

        # Transform end effector position according to robot pos and orientation
        for i in range(6):
            self.end_effector_world_position[i], end_effector_world_orientation = p.multiplyTransforms(robot_position,robot_orientation, base_positions[i], [0,0,0,1])


        # Compute angle with IK
        angles = p.calculateInverseKinematics2(self._robot_id, self.end_effector_index.values(), self.end_effector_world_position)
    
        # Set motor angle
        p.setJointMotorControlArray(self._robot_id, self._revolute_joint_index.values(), p.POSITION_CONTROL, angles, forces=[5]*self._num_revolute_joint, positionGains=[0.5]*self._num_revolute_joint)
        

        # Step simulation
        p.stepSimulation()
        if self.render_mode == 'human':
            time.sleep(TIME_STEP)

        # Update camera
        cam = p.getDebugVisualizerCamera()
        p.resetDebugVisualizerCamera(
        cameraDistance=cam[10],
        cameraYaw=cam[8],
        cameraPitch=cam[9],
        cameraTargetPosition=robot_position    )


        # Get observation
        observation = self._get_obs()

        # Calculate reward
        robot_state = p.getLinkState(self._robot_id,0)
        robot_position = list(robot_state[0])
        self._distance = np.sqrt(robot_position[0]**2 + robot_position[1]**2)
        reward = self._distance
        self.cum_rew += reward
        
        # Calculate terminated or truncated
        terminated = False
        if self._steps >= 990:
            terminated = True
            self.last_ep_mean_rew = self.cum_rew / self._steps


        truncated = False
        
        reward_info = {"distance_reward":reward, "last_ep_mean_rew": self.last_ep_mean_rew}
        info = {**self._get_info(), **reward_info}

        self._steps += 1
        return observation, reward, terminated, truncated, info
    
    def close(self):
        p.disconnect()
        




