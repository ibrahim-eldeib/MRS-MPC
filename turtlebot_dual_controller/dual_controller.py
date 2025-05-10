#!/usr/bin/env python

import os
import select
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Platform-specific imports
if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# Velocity adjustment steps
LIN_VEL_STEP_SIZE = 0.01  # Linear velocity increment per key press
ANG_VEL_STEP_SIZE = 0.1   # Angular velocity increment per key press

# Control instructions for the user
msg = """
Control Your TurtleBots!
---------------------------
TurtleBot 1 (WASD):
   W: Increase linear velocity
   S: Stop
   A: Increase angular velocity (left)
   D: Increase angular velocity (right)
   X: Decrease linear velocity


TurtleBot 2 (Arrow Keys):
   ↑: Increase linear velocity
   ↓: Decrease linear velocity
   ←: Increase angular velocity (left)
   →: Increase angular velocity (right)
   P: Stop
SPACE key: Stop both robots
CTRL-C: Quit
"""

class DualTurtleBotController(Node):
    """
    A ROS2 Node for controlling two TurtleBots simultaneously using keyboard input.
    """

    def __init__(self):
        super().__init__('dual_turtlebot_keyboard_controller')

        # Publishers for TurtleBot command velocities
        self.pub_turtlebot1 = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.pub_turtlebot2 = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # Velocities for each TurtleBot
        self.tb1_linear = 0.0
        self.tb1_angular = 0.0
        self.tb2_linear = 0.0
        self.tb2_angular = 0.0

        # Terminal settings (for non-Windows systems)
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        """
        Reads a key press from the terminal.
        Handles escape sequences for arrow keys.
        """
        if os.name == 'nt':  # Windows platform
            return msvcrt.getch().decode('utf-8')
        tty.setraw(sys.stdin.fileno())  # Raw mode for immediate key reading
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':  # Start of an escape sequence (arrow keys)
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def update_turtlebot1_velocity(self, key):
        """
        Updates velocities for TurtleBot 1 based on WASD input.
        """
        if key == 'w':
            self.tb1_linear += LIN_VEL_STEP_SIZE
        elif key == 'x':  # Decelerate
            self.tb1_linear -= LIN_VEL_STEP_SIZE
        elif key == 'a':  # Turn left
            self.tb1_angular += ANG_VEL_STEP_SIZE
        elif key == 'd':  # Turn right
            self.tb1_angular -= ANG_VEL_STEP_SIZE
        elif key == 's':  # Stop TurtleBot 1
            self.tb1_linear = 0.0
            self.tb1_angular = 0.0

    def update_turtlebot2_velocity(self, key):
        """
        Updates velocities for TurtleBot 2 based on arrow key input.
        """
        if key == '\x1b[A':  # UP arrow
            self.tb2_linear += LIN_VEL_STEP_SIZE
        elif key == '\x1b[B':  # DOWN arrow
            self.tb2_linear -= LIN_VEL_STEP_SIZE
        elif key == '\x1b[D':  # LEFT arrow
            self.tb2_angular += ANG_VEL_STEP_SIZE
        elif key == '\x1b[C':  # RIGHT arrow
            self.tb2_angular -= ANG_VEL_STEP_SIZE
        elif key == 'p':  # Stop TurtleBot 2
            self.tb2_linear = 0.0
            self.tb2_angular = 0.0


    def stop_both_robots(self):
        """
        Stops both robots by resetting their velocities to zero.
        """
        self.tb1_linear = 0.0
        self.tb1_angular = 0.0
        self.tb2_linear = 0.0
        self.tb2_angular = 0.0

    def publish_velocities(self):
        """
        Publishes the current velocities to the respective topics for both TurtleBots.
        """
        # Publish TurtleBot 1 velocities
        twist_tb1 = Twist()
        twist_tb1.linear.x = -self.tb1_linear
        twist_tb1.angular.z = -self.tb1_angular
        self.pub_turtlebot1.publish(twist_tb1)

        # Publish TurtleBot 2 velocities
        twist_tb2 = Twist()
        twist_tb2.linear.x = self.tb2_linear
        twist_tb2.angular.z = self.tb2_angular
        self.pub_turtlebot2.publish(twist_tb2)

    def print_velocities(self):
        """
        Prints the current velocities for both TurtleBots to the terminal.
        """
        print(f"TurtleBot 1: linear={self.tb1_linear:.2f}, angular={self.tb1_angular:.2f}")
        print(f"TurtleBot 2: linear={self.tb2_linear:.2f}, angular={self.tb2_angular:.2f}")

def main():
    """
    The main function initializes the ROS2 node and handles the control loop.
    """
    rclpy.init()
    node = DualTurtleBotController()

    print(msg)

    try:
        while True:
            # Read a key press from the user
            key = node.get_key()

            # Update velocities based on key presses
            if key in ['w', 's', 'a', 'd','x']:
                node.update_turtlebot1_velocity(key)
            elif key in ['\x1b[A', '\x1b[B', '\x1b[D', '\x1b[C', 'p']:
                node.update_turtlebot2_velocity(key)
            elif key == ' ':  # Space key stops both robots
                node.stop_both_robots()
            elif key == '\x03':  # CTRL-C to exit
                break

            # Publish and display velocities
            node.publish_velocities()
            node.print_velocities()

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Stop both robots and reset terminal settings
        node.stop_both_robots()
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
