#!/usr/bin/env python3

import rclpy
import torch
import numpy as np
import io
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer


class SpotFullbodyController(Node):
    """Fullbody controller for Spot quadruped robot."""

    def __init__(self):
        """Initialize the Spot fullbody controller node."""
        super().__init__('spot_fullbody_controller')

        # Declare and set parameters
        self.declare_parameter('publish_period_ms', 5)
        self.declare_parameter('policy_path', 'policy/spot_policy.pt')
        self.set_parameters(
            [rclpy.parameter.Parameter(
                'use_sim_time', 
                rclpy.Parameter.Type.BOOL, 
                True
            )]
        )

        self._logger = self.get_logger()
        
        # Configure QoS profile for simulation
        sim_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_ALL,
        )

        # Create subscription for velocity commands
        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            qos_profile=10)

        # Create publisher for joint commands
        self._joint_publisher = self.create_publisher(
            JointState,
            '/joint_command',
            qos_profile=sim_qos_profile)

        # Setup synchronized subscribers for IMU and joint state data
        self._imu_sub_filter = Subscriber(
            self,
            Imu,
            '/imu',
            qos_profile=sim_qos_profile,
        )
        self._joint_states_sub_filter = Subscriber(
            self,
            JointState,
            '/joint_states',
            qos_profile=sim_qos_profile,
        )
        queue_size = 10
        subscribers = [self._joint_states_sub_filter, self._imu_sub_filter]

        # Time synchronizer to ensure joint state and IMU data are processed together
        # self.sync = ApproximateTimeSynchronizer(
        #     [self._joint_states_sub_filter, self._imu_sub_filter],
        #     queue_size=20,      # a little bigger buffer
        #     slop=0.2,           # 200 ms tolerance
        #     allow_headerless=True
        # )
        # self.sync.registerCallback(self._tick)
        self._imu_sub_filter.registerCallback(self._imu_cache)
        self._joint_states_sub_filter.registerCallback(self._joint_cache)
        self._latest_joint = None
        self._latest_imu = None

        
        # Load neural network policy
        self.policy_path = self.get_parameter('policy_path').value
        self.load_policy()
        self._logger.info(f"✅ Loaded TorchScript policy: {self.policy_path}")
        try:
            example_input = torch.zeros(1, 48)
            test_out = self.policy(example_input)
            self._logger.info(f"Test output shape: {test_out.shape}, values: {test_out}")
        except Exception as e:
            self._logger.error(f"Policy test inference failed: {e}")
        # Initialize state variables
        self._joint_state = JointState()
        self._joint_command = JointState()
        self._cmd_vel = Twist()
        self._imu = Imu()
        self._action_scale = 0.5  # Scale factor for policy output
        self._previous_action = np.zeros(12, dtype=np.float32)
        self._policy_counter = 0
        self._decimation = 4  # Run policy every 4 ticks to reduce computation
        self._last_tick_time = self.get_clock().now().nanoseconds * 1e-9
        self._lin_vel_b = np.zeros(3, dtype=np.float32)  # Linear velocity in body frame
        self._dt = 0.0  # Time delta between ticks
        self.action = np.zeros(12, dtype=np.float32)  # ensure defined before first publish
        self.timer = self.create_timer(0.01, self._step) 
        # Default joint positions representing the nominal stance
        self.default_pos = np.array([
            0.1,   # fl_hx
           -0.1,   # fr_hx
            0.1,   # hl_hx
           -0.1,   # hr_hx
            0.9,   # fl_hy
            0.9,   # fr_hy
            1.1,   # hl_hy
            1.1,   # hr_hy
           -1.5,   # fl_kn
           -1.5,   # fr_kn
           -1.5,   # hl_kn
           -1.5    # hr_kn
        ], dtype=np.float32)

        # Joint names in the order expected by the policy
        self.joint_names = [
            'fl_hx', 'fr_hx', 'hl_hx', 'hr_hx',
            'fl_hy', 'fr_hy', 'hl_hy', 'hr_hy',
            'fl_kn', 'fr_kn', 'hl_kn', 'hr_kn'
        ]

        self._logger.info("Initializing SpotFullbodyController")

    def _cmd_vel_callback(self, msg):
        self._cmd_vel = msg
    def _imu_cache(self, msg):
            self._latest_imu = msg
            

    def _joint_cache(self, msg):
            self._latest_joint = msg
            

    def _step(self):
        if self._latest_joint is None or self._latest_imu is None:
            return
        if abs(self._cmd_vel.linear.x) < 1e-4 and abs(self._cmd_vel.angular.z) < 1e-4:
            return
        self._tick(self._latest_joint, self._latest_imu)
        
    def _tick(self, joint_state: JointState, imu: Imu):
        self._logger.info(
            f"✅ Sync hit | joint_stamp={joint_state.header.stamp.sec}.{joint_state.header.stamp.nanosec} "
            f"| imu_stamp={imu.header.stamp.sec}.{imu.header.stamp.nanosec}"
        )

        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._last_tick_time:
            self._logger.error(f'{self._get_stamp_prefix()} Time jumped backwards. Resetting.')
        
        self._dt = (now - self._last_tick_time)
        if self._dt <= 0.0:
            self._dt = 1e-4
        self._last_tick_time = now
        self._logger.info("✅ Received synchronized IMU + JointState")
        
        self.forward(joint_state, imu)

        self._joint_command.header.stamp = self.get_clock().now().to_msg()
        self._joint_command.name = self.joint_names
        
        action_pos = self.default_pos + self.action * self._action_scale
        self._joint_command.position = action_pos.tolist()
        self._joint_command.velocity = [0.0] * len(self.joint_names)
        self._joint_command.effort = [0.0] * len(self.joint_names)
        self._joint_publisher.publish(self._joint_command)

    def _compute_observation(self, joint_state: JointState, imu: Imu):
        quat_I = imu.orientation
        quat_array = np.array([quat_I.w, quat_I.x, quat_I.y, quat_I.z], dtype=np.float64)
        R_BI = self.quat_to_rot_matrix(quat_array).T

        lin_acc_b = np.array([
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z
        ], dtype=np.float32)
        self._lin_vel_b = self._lin_vel_b + lin_acc_b * float(self._dt)
        
        ang_vel_b = np.array([
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z
        ], dtype=np.float32)
        gravity_b = (R_BI @ np.array([0.0, 0.0, -1.0], dtype=np.float64)).astype(np.float32)

        obs = np.zeros(48, dtype=np.float32)  # 3+3+3+3 + 12 + 12 + 12

        obs[:3] = self._lin_vel_b
        obs[3:6] = ang_vel_b
        obs[6:9] = gravity_b
        obs[9:12] = np.array(
            [self._cmd_vel.linear.x, self._cmd_vel.linear.y, self._cmd_vel.angular.z],
            dtype=np.float32
        )

        current_joint_pos = np.zeros(12, dtype=np.float32)
        current_joint_vel = np.zeros(12, dtype=np.float32)

        # Safely map joints (velocity may be empty in some publishers)
        for i, name in enumerate(self.joint_names):
            if name in joint_state.name:
                idx = joint_state.name.index(name)
                if joint_state.position and len(joint_state.position) > idx:
                    current_joint_pos[i] = float(joint_state.position[idx])
                if joint_state.velocity and len(joint_state.velocity) > idx:
                    current_joint_vel[i] = float(joint_state.velocity[idx])
                else:
                    current_joint_vel[i] = 0.0  # fallback

        obs[12:24] = current_joint_pos - self.default_pos
        obs[24:36] = current_joint_vel
        obs[36:48] = self._previous_action

        # ensure finite
        np.nan_to_num(obs, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        return obs

    def _compute_action(self, obs):
        self._logger.info(f"🧠 Policy forward() — obs shape: {obs.shape}")
        with torch.no_grad():
            obs_t = torch.from_numpy(obs).view(1, -1).float()
            self._logger.info(f"Input tensor shape: {obs_t.shape}")
            output = self.policy(obs_t)
            self._logger.info(f"Policy raw output: {output}")

            # Make sure inference is on CPU for .numpy()
            out = self.policy(obs_t)
            if out.is_cuda:
                out = out.cpu()
            action = out.detach().view(-1).numpy()
            # Shape guard with a log (won't change logic if 12)
            if action.shape[0] != 12:
                self._logger.error(f"Policy output has shape {action.shape}, expected 12. Clipping/truncating to 12.")
                action = action[:12] if action.shape[0] > 12 else np.pad(action, (0, 12 - action.shape[0]))
        return action.astype(np.float32, copy=False)

    def forward(self, joint_state: JointState, imu: Imu):
        obs = self._compute_observation(joint_state, imu)
        if self._policy_counter % self._decimation == 0:
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()
            # Log first few actions to verify non-zero output
            if self._policy_counter < 12:
                self._logger.info(f"action norm={np.linalg.norm(self.action):.4f}, "
                                  f"min={self.action.min():.4f}, max={self.action.max():.4f}")
        self._policy_counter += 1

    def quat_to_rot_matrix(self, quat: np.ndarray) -> np.ndarray:
        q = np.array(quat, dtype=np.float64, copy=True)
        nq = np.dot(q, q)
        if nq < 1e-10:
            return np.identity(3)
        q *= np.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array(
            (
                (1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0]),
                (q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0]),
                (q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2]),
            ),
            dtype=np.float64,
        )

    def load_policy(self):
        with open(self.policy_path, 'rb') as f:
            buffer = io.BytesIO(f.read())
        self.policy = torch.jit.load(buffer)

    def _get_stamp_prefix(self) -> str:
        now = time.time()
        now_ros = self.get_clock().now().nanoseconds / 1e9
        return f'[{now}][{now_ros}]'

    def header_time_in_seconds(self, header) -> float:
        return header.stamp.sec + header.stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = SpotFullbodyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
