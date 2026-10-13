#!/usr/bin/env python3
"""
test_data_sender.py
-------------------
ONE script that sends fake ("synthetic") test data to the admittance
controller. It replaces the three earlier scripts.

What it publishes (both at the same time, in sync):
  1. EMG  -> muscle-activity level, 0.0 (relaxed) to 1.0 (strong squeeze)
  2. FSR  -> fingertip force in Newtons, one value per finger (5 values)

It plays a repeating "story" of a grip:
  rest -> grip_on -> hold -> squeeze (too hard!) -> release -> (repeat)

Run:
  python3 test_data_sender.py
Or with your own topic names:
  python3 test_data_sender.py --ros-args -p emg_topic:=/my_emg -p fsr_topic:=/my_fsr
"""

import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray

# Each phase: (name, duration_seconds, emg_start, emg_end, fsr_start_N, fsr_end_N)
# The values move in a straight line from "start" to "end" during the phase.
PHASES = [
    ("rest",     2.0, 0.05, 0.05, 0.0,  0.0),   # hand relaxed
    ("grip_on",  2.0, 0.10, 0.70, 0.0,  4.0),   # user starts gripping
    ("hold",     3.0, 0.70, 0.70, 4.0,  4.0),   # steady grip
    ("squeeze",  2.0, 0.70, 0.95, 4.0, 12.0),   # too hard -> controller should limit
    ("release",  2.0, 0.95, 0.05, 12.0, 0.0),   # let go
]

# How much of the total force each finger feels
# order: thumb, index, middle, ring, little  (my assumption - change if needed)
FINGER_SCALE = [1.0, 1.0, 0.9, 0.8, 0.6]

# The admittance controller expects torques in THIS joint order:
#   pinky, ring, middle, index, thumb
# which is the reverse of FINGER_SCALE / FSR order above.


class TestDataSender(Node):
    def __init__(self):
        super().__init__("test_data_sender")

        # Parameters = settings you can change without editing the code
        self.declare_parameter("emg_topic", "/emg")
        self.declare_parameter("fsr_topic", "/fsr")
        self.declare_parameter("rate_hz", 50.0)      # how many messages per second
        self.declare_parameter("torque_topic", "/assist_torque_cmd")  # read by admittance controller
        self.declare_parameter("max_torque", 0.5)    # assist torque (Nm) at full EMG
        self.declare_parameter("force_soft", 6.0)    # N: assist starts fading above this
        self.declare_parameter("force_limit", 10.0)  # N: assist is zero at/above this
        self.declare_parameter("noise", 0.02)        # 0.0 = perfectly clean signal

        emg_topic = self.get_parameter("emg_topic").value
        fsr_topic = self.get_parameter("fsr_topic").value
        rate_hz = self.get_parameter("rate_hz").value
        self.noise = self.get_parameter("noise").value
        torque_topic = self.get_parameter("torque_topic").value
        self.max_torque = self.get_parameter("max_torque").value
        self.force_soft = self.get_parameter("force_soft").value
        self.force_limit = self.get_parameter("force_limit").value

        # Publishers = the "mouths" that send data on a topic
        self.emg_pub = self.create_publisher(Float32, emg_topic, 10)
        self.fsr_pub = self.create_publisher(Float32MultiArray, fsr_topic, 10)

        self.torque_pub = self.create_publisher(Float64MultiArray, torque_topic, 10)

        self.phase_index = 0
        self.phase_start = self.get_clock().now()
        self.get_logger().info(f"Phase: {PHASES[0][0]}")

        # Timer = calls self.tick() again and again at the chosen rate
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(f"Sending EMG on {emg_topic}, FSR on {fsr_topic}, "
            f"assist torque on {torque_topic}")

    def tick(self):
        name, duration, e0, e1, f0, f1 = PHASES[self.phase_index]

        elapsed = (self.get_clock().now() - self.phase_start).nanoseconds / 1e9

        # Move to the next phase when this one is finished (loop at the end)
        if elapsed >= duration:
            self.phase_index = (self.phase_index + 1) % len(PHASES)
            self.phase_start = self.get_clock().now()
            self.get_logger().info(f"Phase: {PHASES[self.phase_index][0]}")
            return

        # progress goes from 0.0 to 1.0 through the phase
        progress = elapsed / duration
        emg = e0 + (e1 - e0) * progress
        total_force = f0 + (f1 - f0) * progress

        # Add a little random wobble so it looks like a real sensor
        emg = min(1.0, max(0.0, emg + random.gauss(0.0, self.noise)))

        fsr_values = []
        for scale in FINGER_SCALE:
            value = total_force * scale + random.gauss(0.0, self.noise * 5)
            fsr_values.append(max(0.0, value))

        emg_msg = Float32()
        emg_msg.data = float(emg)
        self.emg_pub.publish(emg_msg)

        fsr_msg = Float32MultiArray()
        fsr_msg.data = [float(v) for v in fsr_values]
        self.fsr_pub.publish(fsr_msg)

        # Turn EMG + FSR into the assist torque the admittance controller uses.
        # Torque follows the user's intent (EMG) and fades to zero as the
        # fingertip force approaches the safety limit.
        torques = []
        for force in fsr_values:
            fade = (self.force_limit - force) / (self.force_limit - self.force_soft)
            fade = min(1.0, max(0.0, fade))
            torques.append(self.max_torque * emg * fade)

        # FSR order is thumb..little; controller order is pinky..thumb
        torque_msg = Float64MultiArray()
        torque_msg.data = [float(t) for t in reversed(torques)]
        self.torque_pub.publish(torque_msg)


def main():
    rclpy.init()
    node = TestDataSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()