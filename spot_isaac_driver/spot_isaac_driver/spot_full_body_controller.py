#!/usr/bin/env python3
"""
Spot Fullbody Controller Node
-----------------------------
ROS 2 node that runs a trained RL policy (TorchScript)
to command the 12 joints of Spot:
[ fl_hx, fr_hx, hl_hx, hr_hx, fl_hy, fr_hy, hl_hy, hr_hy, fl_kn, fr_kn, hl_kn, hr_kn ]
"""

import rclpy
import torch
import numpy as np
import io
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from message_filters import Subscriber, TimeSynchronizer


class SpotFullbodyController(Node):
    """Fullbody controller for Spot quadruped robot."""

    def __init__(self):
        """Initialize the Spot fullbody controller node."""
        super().__init__('spot_fullbody_controller')

        # Parameters
        self.declare_parameter('publish_period_ms', 5)
        self.declare_parameter('policy_path', 'policy/flat_terrain.pt')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True
            )
        ])

        self._logger = self.get_logger()

        # QoS for simulation topics
        sim_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_ALL,
        )

        # Subscriptions
        self._cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )

        # Publisher
        self._joint_publisher = self.create_publisher(
            JointState, '/joint_command', qos_profile=sim_qos_profile
        )

        # IMU + Joint sync
        self._imu_sub_filter = Subscriber(self, Imu, '/imu', qos_profile=sim_qos_profile)
        self._joint_states_sub_filter = Subscriber(self, JointState, '/joint_states', qos_profile=sim_qos_profile)
        self.sync = TimeSynchronizer([self._joint_states_sub_filter, self._imu_sub_filter], 10)
        self.sync.registerCallback(self._tick)

        # Load neural network policy
        self.policy_path = self.get_parameter('policy_path').value
        self.load_policy()

        # State
        self._joint_state = JointState()
        self._joint_command = JointState()
        self._cmd_vel = Twist()
        self._imu = Imu()
        self._action_scale = 0.20
        self._previous_action = np.zeros(12)
        self._policy_counter = 0
        self._decimation = 10
        self._last_tick_time = self.get_clock().now().nanoseconds * 1e-9
        self._lin_vel_b = np.zeros(3)
        self._dt = 0.0

        # Default stance (nominal)
        self.default_pos = np.array([
            0.1,  -0.1,  0.1,  -0.1,   # hx joints
            0.9,   0.9,  1.1,   1.1,   # hy joints
           -1.5,  -1.5, -1.5,  -1.5    # kn joints
        ])

        # Joint names
        self.joint_names = [
            'fl_hx', 'fr_hx', 'hl_hx', 'hr_hx',
            'fl_hy', 'fr_hy', 'hl_hy', 'hr_hy',
            'fl_kn', 'fr_kn', 'hl_kn', 'hr_kn'
        ]

        self._logger.info("Initializing SpotFullbodyController")

    def _cmd_vel_callback(self, msg):
        self._cmd_vel = msg

    def _tick(self, joint_state: JointState, imu: Imu):
        """Process synchronized joint state and IMU data to generate robot commands.
        
        This method is called whenever new joint state and IMU data are available.
        It computes the policy's action and publishes the resulting joint
        commands.
        
        Args:
            joint_state: Current joint positions and velocities
            imu: Current IMU data (orientation, angular velocity, acceleration)
        """
        # Reset if time jumped backwards (most likely due to sim time reset)
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._last_tick_time:
            self._logger.error(
                f'{self._get_stamp_prefix()} Time jumped backwards. Resetting.'
            )
        
        # Calculate time delta since last tick
        self._dt = (now - self._last_tick_time)
        self._last_tick_time = now

        # Run the control policy
        self.forward(joint_state, imu)

        # Prepare and publish the joint command message
        self._joint_command.header.stamp = self.get_clock().now().to_msg()
        self._joint_command.name = self.joint_names
        
        # Compute final joint positions by adding scaled actions to default positions
        action_pos = self.default_pos + self.action * self._action_scale
        self._joint_command.position = action_pos.tolist()
        self._joint_command.velocity = np.zeros(len(self.joint_names)).tolist()
        self._joint_command.effort = np.zeros(len(self.joint_names)).tolist()
        self._joint_publisher.publish(self._joint_command)


    def _compute_observation(self, joint_state: JointState, imu: Imu):
        quat_I = imu.orientation
        quat_array = np.array([quat_I.w, quat_I.x, quat_I.y, quat_I.z])
        R_BI = self.quat_to_rot_matrix(quat_array).T

        lin_acc_b = np.array([
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z
        ])
        # 🔹 Disable integration (reduces drift)
        self._lin_vel_b = lin_acc_b * self._dt + self._lin_vel_b

        ang_vel_b = np.array([
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z
        ])
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        obs = np.zeros(48)
        obs[:3] = self._lin_vel_b
        obs[3:6] = ang_vel_b
        obs[6:9] = gravity_b
        obs[9:12] = np.array([
            self._cmd_vel.linear.x,
            self._cmd_vel.linear.y,
            self._cmd_vel.angular.z
        ])

        current_joint_pos = np.zeros(12)
        current_joint_vel = np.zeros(12)

        for i, name in enumerate(self.joint_names):
            if name in joint_state.name:
                idx = joint_state.name.index(name)
                current_joint_pos[i] = joint_state.position[idx]
                current_joint_vel[i] = joint_state.velocity[idx]

        obs[12:24] = current_joint_pos - self.default_pos
        obs[24:36] = current_joint_vel
        obs[36:48] = self._previous_action
        return obs

    def _compute_action(self, obs):
        """Run the neural network policy to compute an action from the observation.
        
        Args:
            obs: Observation vector containing robot state information
            
        Returns:
            np.ndarray: Action vector containing joint position adjustments
        """
        # Run inference with the PyTorch policy
        with torch.no_grad():
            obs = torch.from_numpy(obs).view(1, -1).float()
          
            action = self.policy(obs).detach().view(-1).numpy()
        return action


    def forward(self, joint_state: JointState, imu: Imu):
        obs = self._compute_observation(joint_state, imu)
        if self._policy_counter % self._decimation == 0:
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()
        self._policy_counter += 1

    def quat_to_rot_matrix(self, quat: np.ndarray) -> np.ndarray:
        q = np.array(quat, dtype=np.float64, copy=True)
        nq = np.dot(q, q)
        if nq < 1e-10:
            return np.identity(3)
        q *= np.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array((
            (1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0]),
            (q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0]),
            (q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2]),
        ), dtype=np.float64)

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