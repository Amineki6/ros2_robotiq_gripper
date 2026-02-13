#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time

# --- Config ---
HISTORY = 60000  # Increased for longer history
WINDOW_DURATION = 120.0  # Show last 2 minutes

# --- Message Imports ---
try:
    from ur_msgs.msg import ToolDataMsg
except ImportError:
    from ur_dashboard_msgs.msg import ToolDataMsg

class GripperStateVisualizer(Node):
    def __init__(self):
        super().__init__('gripper_state_viz')

        # buffers to store history
        self.lock = threading.Lock()
        self.times = deque(maxlen=HISTORY)
        self.positions = deque(maxlen=HISTORY)
        self.velocities = deque(maxlen=HISTORY)
        self.efforts = deque(maxlen=HISTORY)
        
        self.start_time = time.time()
        self.latest_tool_current = 0.0

        # subscribe to joint states
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # subscribe to tool data for effort/current
        self.create_subscription(
            ToolDataMsg,
            '/left_arm/io_and_status_controller/tool_data',
            self.tool_callback,
            10
        )

        self.get_logger().info("Subscribed to /left_gripper/joint_states and /left_arm/io_and_status_controller/tool_data")

    def tool_callback(self, msg):
        with self.lock:
            # Convert to mA if needed, or keep as is. gripping_viz multiplied by 1000.
            # Assuming msg.tool_current is in Amps, we convert to mA.
            self.latest_tool_current = msg.tool_current * 1000.0

    def joint_callback(self, msg):
        with self.lock:
            # Timestamp relative to start
            t = time.time() - self.start_time
            
            # Using index 0 assuming it's the gripper joint
            pos = msg.position[0] if len(msg.position) > 0 else 0.0
            vel = msg.velocity[0] if len(msg.velocity) > 0 else 0.0
            
            # Use joint effort if available (now provided by driver), fallback to tool current
            eff = msg.effort[0] if len(msg.effort) > 0 else self.latest_tool_current
            
            self.times.append(t)
            self.positions.append(pos)
            self.velocities.append(vel)
            self.efforts.append(eff)

    def get_data(self):
        with self.lock:
            return list(self.times), list(self.positions), list(self.velocities), list(self.efforts)

def main():
    rclpy.init()
    node = GripperStateVisualizer()

    # Run ROS in a thread
    spinner_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner_thread.start()

    # --- Plot Setup ---
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fig.suptitle('Real-Time Gripper State')

    # Lines
    line_pos, = ax1.plot([], [], 'b-', label='Position', linewidth=1.5)
    line_vel, = ax2.plot([], [], 'g-', label='Velocity', linewidth=1.5)
    line_eff, = ax3.plot([], [], 'r-', label='Effort (mA)', linewidth=1.5)

    ax1.set_ylabel('Position')
    ax1.legend(loc='upper left')
    ax1.grid(True)

    ax2.set_ylabel('Velocity')
    ax2.legend(loc='upper left')
    ax2.grid(True)

    ax3.set_ylabel('Effort (mA)')
    ax3.set_xlabel('Time (s)')
    ax3.legend(loc='upper left')
    ax3.grid(True)

    def update(frame):
        times, positions, velocities, efforts = node.get_data()
        
        if not times:
            return line_pos, line_vel, line_eff

        line_pos.set_data(times, positions)
        line_vel.set_data(times, velocities)
        line_eff.set_data(times, efforts)

        # Dynamic X-Axis Windowing
        current_time = times[-1]
        x_min = max(0, current_time - WINDOW_DURATION)
        x_max = max(WINDOW_DURATION, current_time + 0.1)
        
        ax3.set_xlim(x_min, x_max)

        # Dynamic Y-Axis Scaling
        for ax, data in zip([ax1, ax2, ax3], [positions, velocities, efforts]):
            if data:
                ymin, ymax = min(data), max(data)
                margin = (ymax - ymin) * 0.1 if ymax != ymin else 1.0
                ax.set_ylim(ymin - margin, ymax + margin)

        return line_pos, line_vel, line_eff

    ani = FuncAnimation(fig, update, interval=50, blit=False)

    print("Plotting... Close the window to exit.")
    plt.show()

    rclpy.shutdown()
    spinner_thread.join()

if __name__ == '__main__':
    main()
