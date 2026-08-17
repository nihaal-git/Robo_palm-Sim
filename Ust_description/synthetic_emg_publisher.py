import math
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SyntheticEMGPublisher(Node):
    def __init__(self):
        super().__init__("synthetic_emg_publisher")
        self.baseline_level = 0.03
        self.peak_level = 0.85
        self.baseline_duration = 2.0
        self.ramp_up_duration = 2.0
        self.hold_duration = 3.0
        self.ramp_down_duration = 2.0
        self.noise_amplitude = 0.02
        self.publish_rate_hz = 50

        self.t_baseline_end = self.baseline_duration
        self.t_rampup_end = self.t_baseline_end + self.ramp_up_duration
        self.t_hold_end = self.t_rampup_end +  self.hold_duration
        self.t_rampdown_end = self.t_hold_end + self.ramp_down_duration
        self.cycle_duration = self.t_rampdown_end
        self.publisher = self.create_publisher(Float32, '/emg/forearm_activation', 10)
        self.start_time = self.get_clock().now()
        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Synthetic Emg publisher started.'
            f'Cycle length:{self.cycle_duration:.1f}s '
            f'(baseline {self.baseline_duration}s -> ramp down {self.ramp_down_duration}s)'
            f'-> hold {self.hold_duration}s -> ramp down {self.ramp_down_duration}s'
        )
    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        # Loop the pattern indefinitely
        t = elapsed % self.cycle_duration
 
        value = self._compute_value_for_phase(t)
 
        # Add small random noise, clamped so we never go negative or above 1.0
        value += random.uniform(-self.noise_amplitude, self.noise_amplitude)
        value = max(0.0, min(1.0, value))
 
        msg = Float32()
        msg.data = value
        self.publisher.publish(msg)
 
    def _compute_value_for_phase(self, t: float) -> float:
        """Return the ideal (noise-free) signal value for elapsed time t within one cycle."""
        if t < self.t_baseline_end:
            # Resting phase
            return self.baseline_level
 
        elif t < self.t_rampup_end:
            # Ramping up: use a smooth S-curve (cosine ease) instead of a straight
            # line, since real muscle activation doesn't turn on instantly/linearly.
            progress = (t - self.t_baseline_end) / self.ramp_up_duration  # 0 -> 1
            eased = 0.5 * (1 - math.cos(progress * math.pi))  # smooth 0 -> 1
            return self.baseline_level + eased * (self.peak_level - self.baseline_level)
 
        elif t < self.t_hold_end:
            # Holding near peak
            return self.peak_level
 
        elif t < self.t_rampdown_end:
            # Ramping down, same smooth easing
            progress = (t - self.t_hold_end) / self.ramp_down_duration  # 0 -> 1
            eased = 0.5 * (1 - math.cos(progress * math.pi))  # smooth 0 -> 1
            return self.peak_level - eased * (self.peak_level - self.baseline_level)
 
        else:
            return self.baseline_level
 
 
def main(args=None):
    rclpy.init(args=args)
    node = SyntheticEMGPublisher()
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
 