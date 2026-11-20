"""
Adaptive human force scaling via admittance control.

Implementation of Hamad, Aydin & Basdogan, "Adaptive Human Force Scaling
via Admittance Control for Physical Human-Robot Interaction",
IEEE Trans. Haptics 14(4), 2021, applied per joint to the 5 MCP joints.

Signal chain (paper Fig. 2), one independent loop per joint:

    tau_H --> H(s) --> G_H(t) --> Y(s) = Ka / (tau_a s + 1) --> v_ref
                         ^
                         |  adaptation mechanism (paper Fig. 4a)
    (v, f, f_dot) --> fuzzy K_HII --> integrate --> sigmoid --> G_H

    v_ref is integrated to a position and sent, with v_ref as feed-forward
    velocity, to mcp_trajectory_controller (the robot motion controller).

"Human force" f_H is the joint torque from /assist_torque_cmd plus
emg_gain * /emg, as in admittance_controller.py, times closing_direction:
both inputs are positive grip strengths, and on this hand a negative MCP
angle closes the finger.

With use_external_intention (default), K_HII is read from /intention
instead of the fuzzy estimator; the rest of the chain is unchanged.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ================================================================
# Mamdani fuzzy intention estimator (paper Sec. III-A, Eq. 4)
# ================================================================

# Output universe for centroid defuzzification
_Y = np.linspace(-1.0, 1.0, 201)


def _tri_memberships(x):
    """
    Three normalized triangular membership functions on [-1, 1].

    Returns (negative, intermediate, positive) degrees, each the same
    shape as x. Inputs outside [-1, 1] saturate in the outer sets.
    """
    x = np.clip(x, -1.0, 1.0)
    neg = np.clip(-x, 0.0, 1.0)
    zero = 1.0 - np.abs(x)
    pos = np.clip(x, 0.0, 1.0)
    return neg, zero, pos


_OUT_NEG, _OUT_ZERO, _OUT_POS = _tri_memberships(_Y)


def human_intention_index(v_fdot, v_f):
    """
    Human Intention Index K_HII in [-1, 1] for each joint.

    v_fdot, v_f: normalized products v*f_dot and v*f (arrays).

    Rules (Eq. 4):
        v*f_dot > 0 and v*f > 0  -> accelerate
        v*f_dot > 0 and v*f < 0  -> decelerate
        v*f_dot < 0              -> decelerate
        v*f_dot ~ 0              -> no change
    v*f_dot > 0 with v*f ~ 0 is not covered by the paper and is mapped
    to "no change".
    """
    a_neg, a_zero, a_pos = _tri_memberships(v_fdot)
    b_neg, b_zero, b_pos = _tri_memberships(v_f)

    # Rule firing strengths (AND = min), aggregated per output set (OR = max)
    w_acc = np.minimum(a_pos, b_pos)
    w_dec = np.maximum(np.minimum(a_pos, b_neg), a_neg)
    w_none = np.maximum(np.minimum(a_pos, b_zero), a_zero)

    # Clip output sets (implication = min), aggregate (max), shape (n, len(_Y))
    agg = np.maximum.reduce([
        np.minimum(w_dec[:, None], _OUT_NEG[None, :]),
        np.minimum(w_none[:, None], _OUT_ZERO[None, :]),
        np.minimum(w_acc[:, None], _OUT_POS[None, :]),
    ])

    area = agg.sum(axis=1)
    centroid = (agg * _Y[None, :]).sum(axis=1)

    return np.divide(
        centroid, area, out=np.zeros_like(centroid), where=area > 1e-12
    )


class AdaptiveAdmittanceController(Node):

    def __init__(self):
        super().__init__("adaptive_admittance_controller_node")

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
        # Parameters
        #
        # Paper values (UR5 + 26 kg cart, N and m/s) are noted in
        # brackets. Ka and the intention scales are retuned for
        # finger joints (Nm and rad/s).
        # =========================================================

        # Must stay below the mcp_trajectory_controller update rate (100 Hz).
        # The JTC starts every new trajectory from the measured joint state;
        # if a new one arrives every JTC cycle it only ever samples that start
        # point, so it never corrects the error and the joints just coast.
        #
        # The loop is clocked by the /joint_states header stamps (simulation
        # time, one message per JTC cycle), not by a ROS timer. A wall-clock
        # timer (use_sim_time:=false, the `ros2 run` default) drifts against
        # the JTC cycle: commands arrive 0, 1 or 2 per cycle, tracking
        # chatters, and a joint can overshoot onto its URDF limit, where
        # Gazebo (DART) pins it for good. Clocking on the joint states keeps
        # exactly one command every rate period of sim time either way.
        self.declare_parameter("rate_hz", 50.0)             # [125]

        # Admittance Y(s) = Ka / (tau_a s + 1)   (Eq. 3)
        self.declare_parameter("Ka", 2.0)                   # rad/s/Nm [0.0167 m/s/N]
        self.declare_parameter("tau_a", 0.1)                # s [0.054]

        # Not in the paper: virtual spring back to the rest pose. Without it
        # the admittance is a pure integrator and a force that never goes
        # negative (EMG, assist torque) drives theta_des to the upper limit
        # and never releases. Steady state: theta = theta_rest + f / stiffness.
        self.declare_parameter("stiffness", 1.0)            # Nm/rad

        # Force scaling gain limits (Eq. 7)
        self.declare_parameter("G_nom", 1.0)                # [1.0]
        self.declare_parameter("G_max", 2.68)               # [2.68]
        self.declare_parameter("G_min", 0.33)               # [0.33]

        # Intention integration constant (Eq. 5)
        self.declare_parameter("alpha", 20.0)               # [20]

        # Force filter H(s) = (hf_gain s + pole) / (s + pole)
        self.declare_parameter("filter_pole", 31.75)        # rad/s [31.75]
        self.declare_parameter("filter_hf_gain", 0.1122)    # [0.1122]

        # Not in the paper: extra low-pass on the force derivative
        # (finite differences of a sampled noisy torque are very noisy).
        # <= 0 disables it.
        self.declare_parameter("fdot_cutoff_hz", 5.0)

        # Normalization of fuzzy inputs: products at or above these
        # magnitudes are fully "positive"/"negative".
        self.declare_parameter("v_fdot_scale", 0.1)         # (rad/s)*(Nm/s)
        self.declare_parameter("v_f_scale", 0.1)            # (rad/s)*Nm

        # Not in the paper: anti-windup bound on the integrated index.
        # sigmoid(+/-6) maps to +/-0.995, so this barely changes the gain
        # range but stops the integral running away during long holds.
        self.declare_parameter("intention_integral_limit", 6.0)

        # Take K_HII from /intention (Float64MultiArray, values in [-1, 1],
        # 1 value for all joints or 1 per joint) instead of the fuzzy
        # estimator. K_HII falls back to 0 if no message for the timeout.
        self.declare_parameter("use_external_intention", True)
        self.declare_parameter("intention_timeout", 0.5)    # s

        # Torque (Nm) added to every joint when EMG = 1.0
        self.declare_parameter("emg_gain", 0.5)

        # Sign of the MCP angle change that closes the finger (-1.0 or 1.0).
        # On this hand a negative angle closes it (towards the URDF lower
        # limit), so a positive grip input must drive the joints negative.
        # 1.0 restores the old behaviour (grip input opens the hand).
        self.declare_parameter("closing_direction", -1.0)

        # Safety limits on the reference. The position limits are the URDF
        # limits shrunk by 0.02 rad: a joint driven exactly onto its URDF
        # lower limit locks there in Gazebo and ignores later commands.
        self.declare_parameter("v_ref_max", 1.0)            # rad/s
        self.declare_parameter(
            "position_lower", [-0.68, -0.68, -0.68, -0.68, -0.33]
        )
        self.declare_parameter(
            "position_upper", [0.98, 0.98, 0.98, 0.98, 0.63]
        )

        # Soft-limit braking: the reference may approach a position limit no
        # faster than limit_braking_gain * (distance to the limit), so it
        # glides into the limit instead of stopping dead on it (a dead stop
        # at speed overshoots in the JTC and can hit the URDF hard stop).
        # 1/s; <= 0 disables it.
        self.declare_parameter("limit_braking_gain", 5.0)

        # A measured position this far past position_lower/upper means the
        # joint has reached its URDF limit, where Gazebo locks it (it ignores
        # every later velocity command). Only used to report it.
        self.declare_parameter("hard_limit_margin", 0.015)  # rad

        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self.dt = 1.0 / p("rate_hz")
        self.Ka = p("Ka")
        self.tau_a = p("tau_a")
        self.stiffness = p("stiffness")
        self.G_nom = p("G_nom")
        self.G_max = p("G_max")
        self.G_min = p("G_min")
        self.alpha = p("alpha")
        self.v_fdot_scale = p("v_fdot_scale")
        self.v_f_scale = p("v_f_scale")
        self.K_m_limit = p("intention_integral_limit")
        self.use_external_intention = p("use_external_intention")
        self.intention_timeout = p("intention_timeout")
        self.emg_gain = p("emg_gain")
        self.closing_direction = p("closing_direction")
        self.v_ref_max = p("v_ref_max")
        self.pos_lower = np.array(p("position_lower"), dtype=float)
        self.pos_upper = np.array(p("position_upper"), dtype=float)
        self.limit_braking_gain = p("limit_braking_gain")
        self.hard_limit_margin = p("hard_limit_margin")

        if not self.G_min <= self.G_nom <= self.G_max:
            raise ValueError("Require G_min <= G_nom <= G_max")
        if self.closing_direction not in (-1.0, 1.0):
            raise ValueError("closing_direction must be -1.0 or 1.0")

        # ---------------------------------------------------------
        # Discretizations
        # ---------------------------------------------------------

        # Y(s): exact zero-order-hold discretization of the first-order lag,
        # stable for any tau_a > 0 and dt.
        self.y_pole = np.exp(-self.dt / self.tau_a)

        # H(s): Tustin. y[k] = h_b0 u[k] + h_b1 u[k-1] - h_a1 y[k-1]
        pole = p("filter_pole")
        hf = p("filter_hf_gain")
        c = 2.0 / self.dt
        self.h_b0 = (hf * c + pole) / (c + pole)
        self.h_b1 = (pole - hf * c) / (c + pole)
        self.h_a1 = (pole - c) / (c + pole)

        # f_dot low-pass: y[k] = y[k-1] + beta (u[k] - y[k-1])
        fc = p("fdot_cutoff_hz")
        self.fdot_beta = (
            1.0 - np.exp(-2.0 * np.pi * fc * self.dt) if fc > 0.0 else 1.0
        )

        # =========================================================
        # Subscribers / publishers
        # =========================================================

        self.tau_assist = np.zeros(self.num_joints)
        self._tau_ext_sub = self.create_subscription(
            Float64MultiArray, "/assist_torque_cmd", self.tau_ext_callback, 10
        )

        self.emg_value = 0.0
        self._emg_sub = self.create_subscription(
            Float32, "/emg", self.emg_callback, 10
        )

        self.intention_value = np.zeros(self.num_joints)
        self.intention_stamp = None
        self._intention_sub = self.create_subscription(
            Float64MultiArray, "/intention", self.intention_callback, 10
        )

        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        self._joint_command_pub = self.create_publisher(
            JointTrajectory, "/mcp_trajectory_controller/joint_trajectory", 10
        )

        # Debug topics, one value per joint (pinky, ring, middle, index, thumb)
        self._debug_pubs = {
            name: self.create_publisher(
                Float64MultiArray, f"/adaptive_admittance/{name}", 10
            )
            for name in (
                "force_filtered",   # H(s) * f_H
                "force_rate",       # d/dt of the filtered force
                "intention",        # K_HII
                "intention_bounded",  # bar K^m_HII
                "gain",             # G_H
                "v_ref",            # admittance output
                "theta_des",        # integrated v_ref sent to the JTC
            )
        }

        # =========================================================
        # State
        # =========================================================

        n = self.num_joints

        self.f_raw_prev = np.zeros(n)
        self.f_filt = np.zeros(n)
        self.f_filt_prev = np.zeros(n)
        self.f_dot = np.zeros(n)

        self.K_hii = np.zeros(n)
        self.K_m = np.zeros(n)
        self.K_m_bar = np.zeros(n)
        self.G_H = np.full(n, self.G_nom)

        self.v_ref = np.zeros(n)
        self.theta_des = np.zeros(n)
        self.theta_rest = np.zeros(n)

        self.joint_position = np.zeros(n)
        self.joint_velocity = np.zeros(n)
        self.joint_state_received = False

        # Stamp (s) of the joint state the last control step ran on
        self.last_step_stamp = None
        self.pinned = np.zeros(n, dtype=bool)

        self.get_logger().info(
            f"Adaptive admittance: Ka={self.Ka}, tau_a={self.tau_a}, "
            f"G_H in [{self.G_min}, {self.G_max}] (nominal {self.G_nom}), "
            f"alpha={self.alpha}, dt={self.dt:.4f}s, "
            f"closing_direction={self.closing_direction:+.0f}"
        )

    # =============================================================
    # Callbacks
    # =============================================================

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

    def intention_callback(self, msg):
        data = np.array(msg.data, dtype=float)
        if data.shape[0] == 1:
            data = np.full(self.num_joints, data[0])
        elif data.shape[0] != self.num_joints:
            self.get_logger().warn(
                f"Expected 1 or {self.num_joints} intention values, "
                f"got {data.shape[0]}"
            )
            return
        self.intention_value = np.clip(data, -1.0, 1.0)
        self.intention_stamp = self.get_clock().now()

    def joint_states_callback(self, msg):
        for i, joint_name in enumerate(self.joint_names):
            if joint_name not in msg.name:
                continue
            index = msg.name.index(joint_name)
            if index < len(msg.position):
                self.joint_position[i] = msg.position[index]
            if index < len(msg.velocity):
                self.joint_velocity[i] = msg.velocity[index]

        if not self.joint_state_received:
            self.theta_des = np.clip(
                self.joint_position, self.pos_lower, self.pos_upper
            )
            self.theta_rest = self.theta_des.copy()
            self.joint_state_received = True
            self.get_logger().info(
                "Initialised adaptive admittance controller from "
                "actual joint positions."
            )

        # Clock the control loop on the joint state stamps (see rate_hz).
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp_ns == 0:
            stamp_ns = self.get_clock().now().nanoseconds
        if self.last_step_stamp is None or stamp_ns < self.last_step_stamp:
            # First message, or the simulation was reset
            self.last_step_stamp = stamp_ns
            return
        if stamp_ns - self.last_step_stamp >= int(self.dt * 1e9) - 1000:
            self.last_step_stamp = stamp_ns
            self.control_loop()

    # =============================================================
    # Main control loop
    # =============================================================

    def control_loop(self):
        if not self.joint_state_received:
            return

        # ---------------------------------------------------------
        # Measured human force f_H, filtered by H(s)
        # ---------------------------------------------------------

        # Grip inputs are positive; closing_direction turns them into a
        # joint torque that closes the finger. The fuzzy estimator only sees
        # the products v*f and v*f_dot, which this sign flip leaves unchanged.
        f_raw = self.closing_direction * (
            self.tau_assist + self.emg_gain * self.emg_value
        )

        self.f_filt = (
            self.h_b0 * f_raw
            + self.h_b1 * self.f_raw_prev
            - self.h_a1 * self.f_filt
        )
        self.f_raw_prev = f_raw

        f_dot_raw = (self.f_filt - self.f_filt_prev) / self.dt
        self.f_filt_prev = self.f_filt.copy()
        self.f_dot += self.fdot_beta * (f_dot_raw - self.f_dot)

        # ---------------------------------------------------------
        # Adaptation mechanism (Eqs. 4-7)
        # ---------------------------------------------------------

        if self.use_external_intention:
            self.K_hii = self.external_intention()
        else:
            v = self.joint_velocity
            self.K_hii = human_intention_index(
                v * self.f_dot / self.v_fdot_scale,
                v * self.f_filt / self.v_f_scale,
            )

        # Eq. 5: K^m(t) = K^m(t-1) + K_HII(t) alpha dt
        self.K_m = np.clip(
            self.K_m + self.K_hii * self.alpha * self.dt,
            -self.K_m_limit,
            self.K_m_limit,
        )

        # Eq. 6: logistic bound to (-1, 1)
        self.K_m_bar = -1.0 + 2.0 / (1.0 + np.exp(-self.K_m))

        # Eq. 7: G_H = G_nom + I_H * bar K^m
        I_H = np.where(
            self.K_m_bar >= 0.0,
            self.G_max - self.G_nom,
            self.G_nom - self.G_min,
        )
        self.G_H = self.G_nom + I_H * self.K_m_bar

        # ---------------------------------------------------------
        # Admittance (Eq. 8): Y(s) = G_H Ka / (tau_a s + 1), driven by the
        # net force after the virtual spring toward the rest pose.
        # ---------------------------------------------------------

        G_E = self.G_H * self.Ka
        f_net = self.f_filt - self.stiffness * (self.theta_des - self.theta_rest)

        self.v_ref = (
            self.y_pole * self.v_ref
            + (1.0 - self.y_pole) * G_E * f_net
        )
        self.v_ref = np.clip(self.v_ref, -self.v_ref_max, self.v_ref_max)

        # Soft-limit braking: cap the speed toward each limit in proportion
        # to the remaining distance, so theta_des glides into the limit.
        if self.limit_braking_gain > 0.0:
            k = self.limit_braking_gain
            self.v_ref = np.clip(
                self.v_ref,
                -k * np.maximum(self.theta_des - self.pos_lower, 0.0),
                k * np.maximum(self.pos_upper - self.theta_des, 0.0),
            )

        # ---------------------------------------------------------
        # Integrate to a position reference, respecting joint limits.
        # At a limit, drop the velocity component that pushes into it
        # so the admittance state doesn't wind up against the stop.
        # ---------------------------------------------------------

        self.theta_des += self.v_ref * self.dt

        at_lower = self.theta_des <= self.pos_lower
        at_upper = self.theta_des >= self.pos_upper
        self.v_ref[(at_lower & (self.v_ref < 0.0))
                   | (at_upper & (self.v_ref > 0.0))] = 0.0
        self.theta_des = np.clip(self.theta_des, self.pos_lower, self.pos_upper)

        # ---------------------------------------------------------
        # Command the robot motion controller
        # ---------------------------------------------------------

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.theta_des.tolist()
        point.velocities = self.v_ref.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)
        msg.points.append(point)

        self._joint_command_pub.publish(msg)

        self.publish_debug()
        self.check_pinned_joints()

        self.get_logger().info(
            f"EMG={self.emg_value:.3f} | "
            f"G_H={np.round(self.G_H, 2).tolist()} | "
            f"theta_des={np.round(self.theta_des, 3).tolist()}",
            throttle_duration_sec=1.0,
        )

    def check_pinned_joints(self):
        """Report joints that have reached their URDF hard limit."""
        m = self.hard_limit_margin
        pinned = ((self.joint_position < self.pos_lower - m)
                  | (self.joint_position > self.pos_upper + m))
        for i in np.flatnonzero(pinned & ~self.pinned):
            self.get_logger().error(
                f"{self.joint_names[i]} is at its URDF limit "
                f"({self.joint_position[i]:+.3f} rad). Gazebo locks a "
                "velocity-controlled joint there and ignores later commands; "
                "restart the simulation to free it."
            )
        self.pinned = pinned

    def external_intention(self):
        """Latest /intention value, or 0 (no change) if it has gone stale."""
        if self.intention_stamp is None:
            return np.zeros(self.num_joints)
        age = (self.get_clock().now() - self.intention_stamp).nanoseconds / 1e9
        if age > self.intention_timeout:
            return np.zeros(self.num_joints)
        return self.intention_value.copy()

    def publish_debug(self):
        values = {
            "force_filtered": self.f_filt,
            "force_rate": self.f_dot,
            "intention": self.K_hii,
            "intention_bounded": self.K_m_bar,
            "gain": self.G_H,
            "v_ref": self.v_ref,
            "theta_des": self.theta_des,
        }
        for name, data in values.items():
            msg = Float64MultiArray()
            msg.data = data.tolist()
            self._debug_pubs[name].publish(msg)


# ================================================================
# Main
# ================================================================

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveAdmittanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
