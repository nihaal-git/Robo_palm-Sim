#!/usr/bin/env python3
"""
force_assist_controller.py  (SKELETON -- no control logic yet)

This node is the future "brain" of the exoglove: it will eventually read
EMG (intent) and FSR (outcome/force feedback) together and decide how much
assist effort to command to the MCP joint servos.

Right now, this file ONLY wires up the two subscriptions and logs what it
receives. No decision-making logic exists yet -- that's the deliberate
next step, built on top of this once we've confirmed both signals are
flowing into the same node correctly.

Subscribes to:
  /emg/forearm_activation   (std_msgs/Float32)  -- 0.0 to 1.0, grip intent
  /fsr/fingertip_force      (std_msgs/Float32)  -- placeholder topic name,
                                                    update this once your
                                                    real FSR topic exists

Publishes: nothing yet.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ForceAssistController(Node):
    def __init__(self):
        super().__init__('force_assist_controller')

        # ---- Stored "latest known value" for each sensor ----
        # These start as None so we can tell the difference between
        # "no data received yet" and "received a real value of 0.0".
        self.latest_emg_value = None
        self.latest_fsr_value = None

        # ---- Subscription 1: EMG (intent signal) ----
        self.emg_subscription = self.create_subscription(
            Float32,                       # message type
            '/emg/forearm_activation',     # topic name to listen to
            self.emg_callback,             # function to call on each new message
            10                              # queue size
        )

        # ---- Subscription 2: FSR (outcome/force signal) ----
        # NOTE: '/fsr/fingertip_force' is a PLACEHOLDER topic name.
        # Update this to match whatever your real FSR-publishing node
        # (or synthetic FSR test node, if you build one later) actually uses.
        self.fsr_subscription = self.create_subscription(
            Float32,
            '/fsr/fingertip_force',
            self.fsr_callback,
            10
        )

        self.get_logger().info(
            'ForceAssistController skeleton started. '
            'Listening on /emg/forearm_activation and /fsr/fingertip_force. '
            'No control logic active yet -- just logging received values.'
        )

    def emg_callback(self, msg: Float32):
        """Called automatically every time a new EMG message arrives."""
        self.latest_emg_value = msg.data
        self.get_logger().info(f'[EMG] received: {self.latest_emg_value:.3f}')
        # TODO: once FSR is also flowing, this is where we'll eventually
        # call a combine/decide function instead of just logging.

    def fsr_callback(self, msg: Float32):
        """Called automatically every time a new FSR message arrives."""
        self.latest_fsr_value = msg.data
        self.get_logger().info(f'[FSR] received: {self.latest_fsr_value:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = ForceAssistController()
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