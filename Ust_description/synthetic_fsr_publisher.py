#!/usr/bin/env python3
"""
synthetic_fsr_publisher.py

Publishes a FAKE (synthetic) FSR fingertip force signal on the topic
/fsr/fingertip_force, shaped to mimic a REALISTIC MECHANICAL RESPONSE
to the synthetic EMG grip signal -- i.e. force rises AFTER muscle
activation, not at the exact same instant, and releases slightly
more slowly than it engages.

This is separate from (and NOT the same as) two other force sources:
  1. The real FSR 402 hardware sensor (not wired into ROS2 yet)
  2. Gazebo's force-torque sensor plugin (simulated PHYSICS, not fake data)
This node is pure fabricated test data, used only to validate your
force_assist_controller's subscriber/combination logic before either
of those two real sources is ready to use.

Message type: std_msgs/msg/Float32
  - Value range: ~0.0 N (no contact) to ~5.0 N (firm fingertip grip
    force). Unlike EMG (normalized 0-1), this is expressed in Newtons,
    a real physical unit, since that's how FSR readings are typically
    interpreted once converted from raw resistance/voltage.
"""

import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SyntheticFSRPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_fsr_publisher')

        # ---- Tunable parameters ----
        self.baseline_force = 0.05      # near-zero force, no real contact
        self.peak_force = 4.5           # Newtons, firm grip
        self.baseline_duration = 2.0    # seconds at rest (matches EMG baseline)

        # Mechanical lag: force starts rising AFTER EMG would have started,
        # simulating the delay between muscle activation and actual
        # fingertip contact force building up.
        self.response_lag = 0.5         # seconds of delay before force starts rising

        self.ramp_up_duration = 1.5     # force builds faster once it starts
        self.hold_duration = 3.0        # matches EMG hold duration
        self.ramp_down_duration = 3.0   # SLOWER release than EMG (finger stays
                                         # pressed briefly even as intent relaxes)

        self.noise_amplitude = 0.08     # Newtons of jitter, mimics sensor noise
        self.publish_rate_hz = 50.0

        # ---- Phase boundaries (seconds, cumulative within one cycle) ----
        self.t_baseline_end = self.baseline_duration + self.response_lag
        self.t_rampup_end = self.t_baseline_end + self.ramp_up_duration
        self.t_hold_end = self.t_rampup_end + self.hold_duration
        self.t_rampdown_end = self.t_hold_end + self.ramp_down_duration
        self.cycle_duration = self.t_rampdown_end

        self.publisher_ = self.create_publisher(Float32, '/fsr/fingertip_force', 10)

        self.start_time = self.get_clock().now()
        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Synthetic FSR publisher started. '
            f'Cycle length: {self.cycle_duration:.1f}s '
            f'(baseline+lag {self.t_baseline_end:.1f}s -> ramp up {self.ramp_up_duration}s '
            f'-> hold {self.hold_duration}s -> ramp down {self.ramp_down_duration}s)'
        )

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        t = elapsed % self.cycle_duration

        value = self._compute_value_for_phase(t)

        value += random.uniform(-self.noise_amplitude, self.noise_amplitude)
        value = max(0.0, value)  # force can't go negative

        msg = Float32()
        msg.data = value
        self.publisher_.publish(msg)

    def _compute_value_for_phase(self, t: float) -> float:
        """Return the ideal (noise-free) force value for elapsed time t within one cycle."""
        if t < self.t_baseline_end:
            # Resting / lag phase -- no meaningful contact force yet
            return self.baseline_force

        elif t < self.t_rampup_end:
            progress = (t - self.t_baseline_end) / self.ramp_up_duration
            eased = 0.5 * (1 - math.cos(progress * math.pi))
            return self.baseline_force + eased * (self.peak_force - self.baseline_force)

        elif t < self.t_hold_end:
            return self.peak_force

        elif t < self.t_rampdown_end:
            progress = (t - self.t_hold_end) / self.ramp_down_duration
            eased = 0.5 * (1 - math.cos(progress * math.pi))
            return self.peak_force - eased * (self.peak_force - self.baseline_force)

        else:
            return self.baseline_force


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticFSRPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()