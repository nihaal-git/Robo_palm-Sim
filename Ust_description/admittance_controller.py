import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32, Float32MultiArray
from std_msgs.msg import Float64MultiArray
import numpy as np

class JointAdmitController(Node):

    def __init__(self):
        super().__init__("joint_admittance_controller_node")
        self._tau_ext_sub = self.create_subscription(
            Float64MultiArray, "/assist_torque_cmd", self.tau_ext_callback, 10
        )

        # EMG subscriber (std_msgs/Float32, same as test_data_sender.py)
        self.emg_value = 0.0
        self._emg_sub = self.create_subscription(
            Float32, "/emg", self.emg_callback, 10
        )

        # Torque (Nm) added to every joint when EMG = 1.0
        self.declare_parameter("emg_gain", 0.5)
        self.emg_gain = self.get_parameter("emg_gain").value

        # Debug publisher: the computed desired joint positions
        self._debug_pub = self.create_publisher(
            Float64MultiArray, "/admittance/debug_output", 10
        )

        # =========================================================
        # Joint names
        # =========================================================
        self.joint_names = [
            "pinky_mcp_joint",
            "ring_mcp_joint",
            "middle_mcp_joint",
            "index_mcp_joint",
            "thumb_mcp_joint",
        ]

        self.num_joints = len(self.joint_names)

        # =========================================================
        # Joint state subscriber
        # =========================================================

        self._joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            10
        )

        # =========================================================
        # Trajectory command publisher
        # =========================================================

        self._joint_command_pub = self.create_publisher(
            JointTrajectory,
            "/mcp_trajectory_controller/joint_trajectory",
            10
        )

        # =========================================================
        # Control loop
        # =========================================================

        self.timer_period = 0.01  # 100 Hz

        self._timer = self.create_timer(
            self.timer_period,
            self.control_loop
        )

        # =========================================================
        # Admittance parameters
        #
        # M*theta_ddot + B*theta_dot + K*theta = tau_ext
        # =========================================================

        self.M = np.array([
            1.0,  # pinky
            1.0,  # ring
            1.0,  # middle
            1.0,  # index
            1.0,  # thumb
        ])

        self.B = np.array([
            0.5,  # pinky
            0.5,  # ring
            0.5,  # middle
            0.5,  # index
            0.5,  # thumb
        ])

        self.K = np.array([
            1.0,  # pinky
            1.0,  # ring
            1.0,  # middle
            1.0,  # index
            1.0,  # thumb
        ])

        # =========================================================
        # Virtual admittance states
        # =========================================================

        self.theta_des = np.zeros(self.num_joints)

        self.theta_dot_des = np.zeros(self.num_joints)

        self.theta_ddot_des = np.zeros(self.num_joints)

        # =========================================================
        # Limits
        # =========================================================

        self.theta_ddot_des_max = np.array([
            5.0,
            5.0,
            5.0,
            5.0,
            5.0,
        ])

        self.theta_dot_des_max = np.array([
            1.0,
            1.0,
            1.0,
            1.0,
            1.0,
        ])

        # Joint position limits from Ust.urdf, shrunk by a margin.
        # A joint driven exactly onto its lower limit locks there in
        # Gazebo, and an unreachable target makes the trajectory
        # controller give up and hold ALL joints where they are.
        position_margin = 0.02

        self.theta_des_min = np.array([
            -0.7,   # pinky
            -0.7,   # ring
            -0.7,   # middle
            -0.7,   # index
            -0.35,  # thumb
        ]) + position_margin

        self.theta_des_max = np.array([
            1.0,    # pinky
            1.0,    # ring
            1.0,    # middle
            1.0,    # index
            0.65,   # thumb
        ]) - position_margin

        # =========================================================
        # External torque
        #
        # For initial testing, give every joint a small constant
        # torque.
        # =========================================================

        self.tau_ext = np.array([
            0.1,  # pinky
            0.1,  # ring
            0.1,  # middle
            0.1,  # index
            0.1,  # thumb
        ])

        # =========================================================
        # Actual joint feedback
        # =========================================================

        self.joint_position = np.zeros(self.num_joints)
        self.tau_assist = np.zeros(self.num_joints)
        self.joint_velocity = np.zeros(self.num_joints)

        self.joint_state_received = False

    def tau_ext_callback(self, msg):
        data = np.array(msg.data, dtype=float)
        if data.shape[0] != self.num_joints:
            self.get_logger().warn(
                f"Expected {self.num_joints} torques, got {data.shape[0]}"
            )
            return
        self.tau_assist = data

    def emg_callback(self, msg):
        self.emg_value = float(msg.data)

    # =============================================================
    # Joint state callback
    # =============================================================

    def joint_states_callback(self, msg):

        # ---------------------------------------------------------
        # Find each joint by name
        # ---------------------------------------------------------

        for i, joint_name in enumerate(self.joint_names):

            if joint_name not in msg.name:
                continue

            index = msg.name.index(joint_name)

            if index < len(msg.position):
                self.joint_position[i] = msg.position[index]

            if index < len(msg.velocity):
                self.joint_velocity[i] = msg.velocity[index]

        # ---------------------------------------------------------
        # Initialize virtual system from actual robot state
        # ---------------------------------------------------------

        if not self.joint_state_received:

            self.theta_des = self.joint_position.copy()

            self.theta_dot_des = self.joint_velocity.copy()

            self.joint_state_received = True

            self.get_logger().info(
                "Initialised admittance controller from "
                "actual joint positions."
            )

    # =============================================================
    # Main control loop
    # =============================================================

    def control_loop(self):

        # Don't run until we receive joint states

        if not self.joint_state_received:
            return

        # =========================================================
        # Admittance equation
        #
        # M*theta_ddot + B*theta_dot + K*theta = tau_ext
        #
        # theta_ddot =
        # (tau_ext - B*theta_dot - K*theta) / M
        # =========================================================

        damping_force = self.B * self.theta_dot_des

        spring_force = self.K * self.theta_des

        # EMG torque (same on every joint) + torque from /assist_torque_cmd
        self.tau_ext = self.tau_assist + self.emg_gain * self.emg_value

        tau_net = (
            self.tau_ext
            - damping_force
            - spring_force
        )

        # ---------------------------------------------------------
        # Calculate acceleration
        # ---------------------------------------------------------

        self.theta_ddot_des = tau_net / self.M

        # ---------------------------------------------------------
        # Acceleration limits
        # ---------------------------------------------------------

        self.theta_ddot_des = np.clip(
            self.theta_ddot_des,
            -self.theta_ddot_des_max,
            self.theta_ddot_des_max
        )

        # =========================================================
        # Integrate acceleration -> velocity
        # =========================================================

        self.theta_dot_des += (
            self.theta_ddot_des *
            self.timer_period
        )

        # ---------------------------------------------------------
        # Velocity limits
        # ---------------------------------------------------------

        self.theta_dot_des = np.clip(
            self.theta_dot_des,
            -self.theta_dot_des_max,
            self.theta_dot_des_max
        )

        # =========================================================
        # Integrate velocity -> position
        # =========================================================

        self.theta_des += (
            self.theta_dot_des *
            self.timer_period
        )

        # ---------------------------------------------------------
        # Position limits (stop the virtual joint at the limit)
        # ---------------------------------------------------------

        at_limit = (
            (self.theta_des < self.theta_des_min) |
            (self.theta_des > self.theta_des_max)
        )

        self.theta_des = np.clip(
            self.theta_des,
            self.theta_des_min,
            self.theta_des_max
        )

        self.theta_dot_des[at_limit] = 0.0

        # =========================================================
        # Create JointTrajectory message
        # =========================================================

        msg = JointTrajectory()

        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        # ---------------------------------------------------------
        # Desired positions
        # ---------------------------------------------------------

        point.positions = self.theta_des.tolist()

        # ---------------------------------------------------------
        # Desired velocities
        # ---------------------------------------------------------

        point.velocities = self.theta_dot_des.tolist()

        # ---------------------------------------------------------
        # Time from start
        # ---------------------------------------------------------

        point.time_from_start.sec = 0

        point.time_from_start.nanosec = int(
            self.timer_period * 1e9
        )

        msg.points.append(point)

        # =========================================================
        # Publish
        # =========================================================

        self._joint_command_pub.publish(msg)

        # =========================================================
        # Debug output
        # =========================================================

        debug_msg = Float64MultiArray()
        debug_msg.data = self.theta_des.tolist()
        self._debug_pub.publish(debug_msg)

        self.get_logger().info(
            f"EMG={self.emg_value:.3f} | "
            f"theta_des={np.round(self.theta_des, 3).tolist()}",
            throttle_duration_sec=1.0
        )


# ================================================================
# Main
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = JointAdmitController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()