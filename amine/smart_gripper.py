#!/usr/bin/env python3
import sys
import os
import rclpy
import time
import threading
import serial.tools.list_ports  # Added for USB detection
from sensor_msgs.msg import JointState

# Add the parent directory to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# --- Custom Imports (Your Setup) ---
try:
    from motion_engine import GiskardMotionEngine
    from config import DualArmConfig
    from skills import GripperController
except ImportError:
    print("WARNING: Custom imports failed. Ensure you are in the correct workspace.")

# --- CONFIGURATION ---
POS_OPEN = 0.0  # Fully Open
POS_CLOSE = 0.35  # Fully Closed (Adjusted based on your previous snippet)
TOLERANCE = 0.05  # Stall detection tolerance


# --- HELPER: CHECK USB PORTS ---
def check_usb_ports():
    """
    Lists all available USB-Serial ports (FTDI/Robotiq).
    Does not open them (safe to run while ROS is active).
    """
    print("\n" + "=" * 50)
    print(" CHECKING CONNECTED USB DEVICES")
    print("=" * 50)

    ports = serial.tools.list_ports.comports()
    robotiq_candidates = []

    if not ports:
        print(" [!] No Serial ports found.")
    else:
        for p in ports:
            # Robotiq grippers typically use FTDI chips (VID:0403 PID:6001)
            # We print everything just in case.
            print(f" Found Port: {p.device}")
            print(f"    - Desc: {p.description}")
            print(f"    - HWID: {p.hwid}")

            if "0403:6001" in p.hwid or "FTDI" in p.description:
                robotiq_candidates.append(p.device)
                print("    -> [LIKELY ROBOTIQ / FTDI ADAPTER]")
            print("-" * 30)

    print(f" Summary: Found {len(robotiq_candidates)} likely gripper adapter(s).")
    print(" (Note: Cannot identify Left/Right specifically without stopping the ROS driver")
    print("  and blinking the devices, or checking your udev rules).")
    print("=" * 50 + "\n")


class GraspMonitor:
    def __init__(self, node):
        self.node = node
        self.current_pos = None
        self.current_vel = None
        self.lock = threading.Lock()

        # Subscribe to gripper states
        self.sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )

    def callback(self, msg):
        with self.lock:
            if len(msg.position) > 0:
                self.current_pos = msg.position[0]
                self.current_vel = msg.velocity[0] if len(msg.velocity) > 0 else 0.0

    def get_status(self):
        with self.lock:
            return self.current_pos, self.current_vel

    def wait_for_connection(self):
        print("Waiting for gripper ROS topic data...", end='', flush=True)
        while rclpy.ok():
            if self.current_pos is not None:
                print(" Connected!")
                return
            time.sleep(0.1)


def main():
    # 1. Run the USB Check first
    check_usb_ports()

    rclpy.init()
    node = rclpy.create_node('interactive_grasp_tester')

    # 2. Setup Controller & Monitor
    try:
        G = GripperController(node=node)
    except Exception as e:
        print(f"Controller Error: {e}")
        return

    monitor = GraspMonitor(node)

    # 3. Run ROS in background thread
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    # 4. Wait for data
    monitor.wait_for_connection()

    print("\n" + "=" * 50)
    print(" INTERACTIVE GRASP TESTER")
    print(" ------------------------")
    print(f" OPEN Value:  {POS_OPEN}")
    print(f" CLOSE Value: {POS_CLOSE}")
    print("=" * 50)
    print("COMMANDS:")
    print("  [Enter]   -> Test Grasp (Close & Check)")
    print("  'o'       -> Open Gripper")
    print("  'q'       -> Quit")
    print("=" * 50 + "\n")

    try:
        while True:
            # Get User Input
            user_input = input("Cmd > ").strip().lower()

            if user_input == 'q':
                break

            elif user_input == 'o':
                print(f"Opening to {POS_OPEN}...")
                G.command("left", POS_OPEN, effort=50.0)
                time.sleep(1.0)  # Wait for move

            else:
                # Default Action: GRASP TEST
                print(f"Closing to {POS_CLOSE} with detection...", end='', flush=True)

                # 1. Send Close Command
                G.command("left", POS_CLOSE, effort=50.0)

                # 2. Wait for Stall (Motion Detection Loop)
                start_time = time.time()
                time.sleep(0.5)  # Give it time to start moving

                while (time.time() - start_time) < 3.0:  # 3s Timeout
                    pos, vel = monitor.get_status()

                    # Check if velocity is near zero (stopped)
                    if abs(vel) < 0.005:
                        print(" Stopped.")

                        # 3. Analyze the Position
                        dist_from_closed = abs(POS_CLOSE - pos)

                        print("-" * 30)
                        print(f"Final Position: {pos:.4f}")

                        if dist_from_closed < TOLERANCE:
                            print("RESULT: [EMPTY] Grasp Failed.")
                            print("(Gripper closed fully)")
                        else:
                            print("RESULT: [OBJECT DETECTED] Success!")
                            print(f"(Blocked {dist_from_closed:.3f} before full close)")
                        print("-" * 30)
                        break

                    time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        rclpy.shutdown()
        spinner.join()


if __name__ == '__main__':
    main()