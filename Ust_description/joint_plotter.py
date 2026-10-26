#!/usr/bin/env python3
"""
joint_plotter.py
A ROS2 node that listens to /joint_states and plots
  (top graph)    position vs time
  (bottom graph) velocity vs time
for the joints you choose.

Run:
  ros2 run Ust_description joint_plotter
  ros2 run Ust_description joint_plotter --ros-args -p joints:="['joint_a','joint_b']"
  ros2 run Ust_description joint_plotter --ros-args -p use_sim_time:=true
"""

import os
import threading
import time
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import matplotlib

# If there is no display (headless machine), use a drawing backend that
# works without a window. We then save the graph as a PNG image instead.
HEADLESS = not os.environ.get("DISPLAY")
if HEADLESS:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (must come after matplotlib.use)


class JointPlotter(Node):
    """Collects joint data in the background. Plotting happens in main()."""

    def __init__(self):
        super().__init__("joint_plotter")

        # ---- Parameters (settings you can change without editing code) ----
        self.declare_parameter("topic", "/joint_states")
        self.declare_parameter("joints", [""])          # [""] means "all joints"
        self.declare_parameter("window_seconds", 10.0)  # how much history to show
        self.declare_parameter("save_path", "/tmp/joint_plot.png")

        topic = self.get_parameter("topic").value
        self.window = float(self.get_parameter("window_seconds").value)
        wanted = [j for j in self.get_parameter("joints").value if j]
        self.wanted = set(wanted)  # empty set = plot everything
        self.save_path = self.get_parameter("save_path").value

        # ---- Storage: for each joint, lists of time / position / velocity ----
        # deque(maxlen=N) automatically throws away the oldest data when full.
        maxlen = 20000
        self.t = defaultdict(lambda: deque(maxlen=maxlen))
        self.pos = defaultdict(lambda: deque(maxlen=maxlen))
        self.vel = defaultdict(lambda: deque(maxlen=maxlen))
        self.t0 = None  # time of the very first message, so graphs start at 0 s
        self.lock = threading.Lock()  # prevents reading and writing at the same moment

        # ---- Subscribe: "call self.callback every time a message arrives" ----
        self.create_subscription(JointState, topic, self.callback, 10)
        self.get_logger().info(f"Listening to {topic}. Headless mode: {HEADLESS}")

    def callback(self, msg: JointState):
        # Use the time written inside the message (works with sim time too).
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if now == 0.0:  # some publishers leave the stamp empty
            now = self.get_clock().now().nanoseconds * 1e-9

        with self.lock:
            if self.t0 is None:
                self.t0 = now
            t = now - self.t0

            for i, name in enumerate(msg.name):
                if self.wanted and name not in self.wanted:
                    continue
                if i >= len(msg.position):
                    continue

                p = msg.position[i]

                if i < len(msg.velocity):
                    v = msg.velocity[i]
                elif len(self.t[name]) > 0:
                    # No velocity given -> estimate it: change in position / change in time
                    dt = t - self.t[name][-1]
                    v = (p - self.pos[name][-1]) / dt if dt > 0 else 0.0
                else:
                    v = 0.0

                self.t[name].append(t)
                self.pos[name].append(p)
                self.vel[name].append(v)


def main():
    rclpy.init()
    node = JointPlotter()

    # rclpy.spin() blocks (waits forever), so run it in a background thread.
    # That leaves the main thread free to draw the graph.
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    ax_pos.set_ylabel("Position (rad)")
    ax_vel.set_ylabel("Velocity (rad/s)")
    ax_vel.set_xlabel("Time (s)")
    ax_pos.set_title("Joint position and velocity vs time")
    ax_pos.grid(True)
    ax_vel.grid(True)

    lines_pos, lines_vel = {}, {}
    last_save = 0.0

    try:
        while rclpy.ok():
            # Copy the data quickly, then release the lock before drawing.
            with node.lock:
                snapshot = {
                    n: (list(node.t[n]), list(node.pos[n]), list(node.vel[n]))
                    for n in node.t
                }

            latest_t = 0.0
            for name, (t, p, v) in snapshot.items():
                if not t:
                    continue
                if name not in lines_pos:  # first time we see this joint
                    (lines_pos[name],) = ax_pos.plot([], [], label=name)
                    (lines_vel[name],) = ax_vel.plot([], [], label=name)
                    ax_pos.legend(loc="upper right", fontsize="small")
                lines_pos[name].set_data(t, p)
                lines_vel[name].set_data(t, v)
                latest_t = max(latest_t, t[-1])

            # Show only the last `window` seconds
            ax_vel.set_xlim(max(0.0, latest_t - node.window), max(node.window, latest_t))
            for ax in (ax_pos, ax_vel):
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)

            if HEADLESS:
                # No window available -> save an image every 2 seconds
                if time.time() - last_save > 2.0:
                    fig.savefig(node.save_path, dpi=100)
                    last_save = time.time()
                time.sleep(0.2)
            else:
                plt.pause(0.1)  # draws the window and keeps it responsive
    except KeyboardInterrupt:
        pass
    finally:
        fig.savefig(node.save_path, dpi=100)  # final snapshot on exit
        node.get_logger().info(f"Saved final plot to {node.save_path}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()