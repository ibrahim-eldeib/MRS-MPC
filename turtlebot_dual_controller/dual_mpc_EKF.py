#!/usr/bin/env python3
"""
Dual MPC Controller Node

This node controls two robots using Model Predictive Control (MPC) combined with 
a simple Kalman filter for each robot. It:
  - Subscribes to the pose topics (/robot1/pose and /robot2/pose) (as Pose2D messages)
    to receive robot pose measurements.
  - Uses two Kalman filters (one per robot) to fuse noisy sensor measurements.
  - Sets up a continuous dynamic model (differential-drive kinematics) for both robots.
  - Configures an MPC controller that minimizes a quadratic cost:
         J = (x - x_goal)^T Q (x - x_goal) + u^T R u,
    where the error includes a wrapped orientation error.
  - Publishes Twist commands to /robot1/cmd_vel and /robot2/cmd_vel.
  - Logs debug information about raw measurements, filtered states, errors, and control commands.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
import numpy as np
import do_mpc
import casadi as ca
from math import pi, sqrt
import time
import signal

def wrap_angle(angle):
    """
    Wrap an angle to the range [-pi, pi].

    Args:
        angle (float): Angle in radians.
    Returns:
        float: Wrapped angle.
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

class DualMPCController(Node):
    """
    DualMPCController controls two robots via MPC and Kalman filtering.
    
    Key tasks:
      - Receives Pose2D messages from both robots.
      - Uses separate Kalman filters to estimate each robot's state.
      - Sets up a combined dynamic model (with differential-drive kinematics) for both robots.
      - Configures an MPC controller with a quadratic cost to drive each robot toward its goal.
      - Publishes Twist commands to control the robots.
      - Logs debug information for monitoring and troubleshooting.
    """

    # --- Inner Kalman Filter class ---
    class SimpleKF:
        """
        SimpleKF implements a basic Kalman Filter for a 3D state [x, y, theta].
        
        Assumes:
          - A constant state model (F = Identity).
          - An identity measurement model (H = Identity).
        """
        def __init__(self, initial_state, P, Q, R):
            # Initialize state estimate and covariance matrices.
            self.x = initial_state  # Current state estimate (3x1 vector)
            self.P = P              # State covariance (3x3)
            self.Q = Q              # Process noise covariance (3x3)
            self.R = R              # Measurement noise covariance (3x3)
            self.F = np.eye(3)      # State transition matrix (identity)
            self.H = np.eye(3)      # Measurement matrix (identity)

        def predict(self):
            """
            Predict the next state and update the covariance.
            Returns:
                np.array: Predicted state.
            """
            self.x = self.F.dot(self.x)
            self.P = self.F.dot(self.P).dot(self.F.T) + self.Q
            return self.x

        def update(self, z):
            """
            Update the state estimate using the new measurement z.
            
            Args:
                z (np.array): Measurement vector [x, y, theta].
            Returns:
                np.array: Updated state estimate.
            """
            self.predict()  # Predict step
            y = z - self.H.dot(self.x)  # Measurement residual
            y[2] = wrap_angle(y[2])     # Wrap the angular difference
            S = self.H.dot(self.P).dot(self.H.T) + self.R  # Innovation covariance
            K = self.P.dot(self.H.T).dot(np.linalg.inv(S))  # Kalman gain
            self.x = self.x + K.dot(y)                      # Update state estimate
            self.x[2] = wrap_angle(self.x[2])               # Wrap the updated angle
            self.P = (np.eye(3) - K.dot(self.H)).dot(self.P)  # Update covariance
            return self.x
    # --- End of inner KF class ---

    def __init__(self):
        """
        Initializes the DualMPCController node.
        
        Sets up:
          - State vectors and flags for both robots.
          - Goal states for each robot.
          - Publishers for Twist commands.
          - Subscribers for Pose2D messages (robot poses).
          - Kalman filters for each robot.
          - The dynamic model and MPC controller.
          - A periodic control loop timer.
        """
        super().__init__('dual_mpc_controller')
        self.get_logger().info("Dual MPC Controller Node Starting...")

        # Initialize raw and filtered state vectors for both robots.
        self.robot1_state_raw = np.array([0.0, 0.0, 0.0])
        self.robot2_state_raw = np.array([0.0, 0.0, 0.0])
        self.robot1_state_filt = np.array([0.0, 0.0, 0.0])
        self.robot2_state_filt = np.array([0.0, 0.0, 0.0])
        self.pose1_received = False
        self.pose2_received = False

        # Set goal states (x, y, theta in radians) for each robot.
        self.robot1_goal = np.array([2.0, 1.0, 0])
        self.robot2_goal = np.array([2.0, 0.5, 0])

        # Set maximum speeds.
        self.max_velocity = 0.02
        self.max_angular_velocity = 0.05

        # Create publishers for robot command topics.
        self.pub_robot1 = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.pub_robot2 = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # --- Updated: Use Pose2D messages for both robot pose subscriptions ---
        self.sub_robot1 = self.create_subscription(Pose2D, '/robot1/pose', self.robot1_pose_callback, 10)
        self.sub_robot2 = self.create_subscription(Pose2D, '/robot2/pose', self.robot2_pose_callback, 10)

        # Create separate Kalman filter instances for each robot.
        self.kf_robot1 = self.SimpleKF(initial_state=np.array([0.0, 0.0, 0.0]),
                                       P=np.diag([1.0, 1.0, 0.1]),
                                       Q=np.diag([0.01, 0.01, 0.005]),
                                       R=np.diag([0.5, 0.5, 0.1]))
        self.kf_robot2 = self.SimpleKF(initial_state=np.array([0.0, 0.0, 0.0]),
                                       P=np.diag([1.0, 1.0, 0.1]),
                                       Q=np.diag([0.01, 0.01, 0.005]),
                                       R=np.diag([0.5, 0.5, 0.1]))

        # Set up the continuous dynamic model and MPC controller for both robots.
        self.model = self.setup_model()
        self.mpc = self.setup_mpc(self.model)

        # Create a periodic timer to run the control loop at 10 Hz.
        self.timer = self.create_timer(0.1, self.control_loop)

    def setup_model(self):
        """
        Configures the continuous dynamic model for both robots.
        
        Each robot is modeled with:
          - State: [x, y, theta].
          - Control inputs: [v, omega].
        Using differential-drive kinematics:
          - x_dot = v * cos(theta)
          - y_dot = v * sin(theta)
          - theta_dot = omega
        """
        model = do_mpc.model.Model('continuous')

        # Define state variables for Robot1.
        model.set_variable(var_type='_x', var_name='x1')
        model.set_variable(var_type='_x', var_name='y1')
        model.set_variable(var_type='_x', var_name='theta1')

        # Define state variables for Robot2.
        model.set_variable(var_type='_x', var_name='x2')
        model.set_variable(var_type='_x', var_name='y2')
        model.set_variable(var_type='_x', var_name='theta2')

        # Define control inputs for Robot1.
        model.set_variable(var_type='_u', var_name='v1')
        model.set_variable(var_type='_u', var_name='omega1')

        # Define control inputs for Robot2.
        model.set_variable(var_type='_u', var_name='v2')
        model.set_variable(var_type='_u', var_name='omega2')

        # Set up differential-drive kinematics for Robot1.
        model.set_rhs('x1', model.u['v1'] * ca.cos(model.x['theta1']))
        model.set_rhs('y1', model.u['v1'] * ca.sin(model.x['theta1']))
        model.set_rhs('theta1', model.u['omega1'])

        # Set up differential-drive kinematics for Robot2.
        model.set_rhs('x2', model.u['v2'] * ca.cos(model.x['theta2']))
        model.set_rhs('y2', model.u['v2'] * ca.sin(model.x['theta2']))
        model.set_rhs('theta2', model.u['omega2'])

        model.setup()
        self.get_logger().info("Model setup complete.")
        return model

    def setup_mpc(self, model):
        """
        Configures the MPC controller for both robots.
        
        The overall cost is the sum of the costs for both robots.
        For each robot:
          - State error: difference between current state and goal.
          - Orientation error is computed using wrapped differences.
          - Quadratic cost penalizes state error and control effort.
        """
        mpc = do_mpc.controller.MPC(model)
        setup_mpc = {
            'n_horizon': 20,    # Prediction horizon
            't_step': 0.1,      # Discretization time step
            'state_discretization': 'collocation',
            'collocation_type': 'radau',
            'collocation_deg': 3,
            'collocation_ni': 2,
            'store_full_solution': True,
            'nlpsol_opts': {'print_time': 0, 'ipopt': {'print_level': 0}}
        }
        mpc.set_param(**setup_mpc)

        # Extract state variables from the model.
        x1 = model.x['x1']
        y1 = model.x['y1']
        theta1 = model.x['theta1']
        x2 = model.x['x2']
        y2 = model.x['y2']
        theta2 = model.x['theta2']

        # Unpack goal states for each robot.
        goal1_x, goal1_y, goal1_theta = self.robot1_goal
        goal2_x, goal2_y, goal2_theta = self.robot2_goal

        # Compute wrapped orientation errors.
        theta_error1 = ca.atan2(ca.sin(theta1 - goal1_theta), ca.cos(theta1 - goal1_theta))
        theta_error2 = ca.atan2(ca.sin(theta2 - goal2_theta), ca.cos(theta2 - goal2_theta))

        # Construct error vectors for each robot.
        error1 = ca.vertcat(x1 - goal1_x, y1 - goal1_y, theta_error1)
        error2 = ca.vertcat(x2 - goal2_x, y2 - goal2_y, theta_error2)

        # Define weight matrices for state error (Q) and control effort (R).
        Q = ca.diag(ca.vertcat(5, 5, 0.01))
        R = ca.diag(ca.vertcat(0.2, 0.2))

        # Compute individual costs and sum them.
        cost1 = ca.mtimes([error1.T, Q, error1])
        cost2 = ca.mtimes([error2.T, Q, error2])
        total_cost = cost1 + cost2

        mpc.set_objective(mterm=total_cost, lterm=total_cost)
        mpc.set_rterm(v1=0.01, omega1=0.01, v2=0.01, omega2=0.01)

        # Set symmetric control bounds for both robots.
        mpc.bounds['lower', '_u', 'v1'] = -self.max_velocity
        mpc.bounds['upper', '_u', 'v1'] = self.max_velocity
        mpc.bounds['lower', '_u', 'omega1'] = -self.max_angular_velocity
        mpc.bounds['upper', '_u', 'omega1'] = self.max_angular_velocity

        mpc.bounds['lower', '_u', 'v2'] = -self.max_velocity
        mpc.bounds['upper', '_u', 'v2'] = self.max_velocity
        mpc.bounds['lower', '_u', 'omega2'] = -self.max_angular_velocity
        mpc.bounds['upper', '_u', 'omega2'] = self.max_angular_velocity

        mpc.setup()
        self.get_logger().info("MPC setup complete.")
        return mpc

    # --- Updated Pose Callbacks Using Pose2D ---
    def robot1_pose_callback(self, msg):
        """
        Callback for /robot1/pose.
        
        Reads a Pose2D message and updates the raw and filtered state
        for robot1 using the Kalman filter.
        """
        x = msg.x
        y = msg.y
        theta = msg.theta  # Already in radians.
        measurement = np.array([x, y, theta])
        self.robot1_state_raw = measurement
        filtered_state = self.kf_robot1.update(measurement)
        self.robot1_state_filt = filtered_state
        self.pose1_received = True
        try:
            self.get_logger().info(f"Robot1 raw: {measurement} | Filtered: {filtered_state}")
        except Exception:
            pass

    def robot2_pose_callback(self, msg):
        """
        Callback for /robot2/pose.
        
        Reads a Pose2D message and updates the raw and filtered state
        for robot2 using the Kalman filter.
        """
        x = msg.x
        y = msg.y
        theta = msg.theta  # Already in radians.
        measurement = np.array([x, y, theta])
        self.robot2_state_raw = measurement
        filtered_state = self.kf_robot2.update(measurement)
        self.robot2_state_filt = filtered_state
        self.pose2_received = True
        try:
            self.get_logger().info(f"Robot2 raw: {measurement} | Filtered: {filtered_state}")
        except Exception:
            pass
    # --- End of Updated Pose Callbacks ---

    def control_loop(self):
        """
        Main control loop (runs at 10 Hz):
          1. Checks if poses for both robots have been received.
          2. Computes the error between each robot's filtered state and its goal.
          3. Logs the goal, state, and error.
          4. Concatenates the filtered states of both robots to form the overall state vector.
          5. Updates the MPC state and computes the MPC step.
          6. Extracts the control commands and publishes Twist messages.
        """
        if not (self.pose1_received and self.pose2_received):
            try:
                self.get_logger().info("Waiting for both robot poses...")
            except Exception:
                pass
            return

        # Compute errors for each robot.
        error_robot1 = np.array([
            self.robot1_state_filt[0] - self.robot1_goal[0],
            self.robot1_state_filt[1] - self.robot1_goal[1],
            wrap_angle(self.robot1_state_filt[2] - self.robot1_goal[2])
        ])
        error_robot2 = np.array([
            self.robot2_state_filt[0] - self.robot2_goal[0],
            self.robot2_state_filt[1] - self.robot2_goal[1],
            wrap_angle(self.robot2_state_filt[2] - self.robot2_goal[2])
        ])
        try:
            self.get_logger().info(f"Robot1 goal: {self.robot1_goal}, State: {self.robot1_state_filt}, Error: {error_robot1}")
            self.get_logger().info(f"Robot2 goal: {self.robot2_goal}, State: {self.robot2_state_filt}, Error: {error_robot2}")
        except Exception:
            pass

        # Concatenate filtered states into a single state vector for the MPC.
        x0 = np.concatenate((self.robot1_state_filt, self.robot2_state_filt)).reshape(-1, 1)
        self.mpc.x0 = x0

        try:
            # Compute the control command using the MPC.
            u0 = self.mpc.make_step(x0)
        except Exception as e:
            try:
                self.get_logger().error(f"MPC error: {e}")
            except Exception:
                pass
            return

        # Extract individual control commands.
        v1_cmd = float(u0[0])
        omega1_cmd = float(u0[1])
        v2_cmd = float(u0[2])
        omega2_cmd = float(u0[3])
        try:
            self.get_logger().info(f"Robot1: cmd: v={v1_cmd:.2f}, omega={omega1_cmd:.2f}")
            self.get_logger().info(f"Robot2: cmd: v={v2_cmd:.2f}, omega={omega2_cmd:.2f}")
        except Exception:
            pass

        # Create and publish Twist commands for both robots.
        twist1 = Twist()
        twist1.linear.x = -v1_cmd  # Invert if necessary to match robot's frame.
        twist1.angular.z = -omega1_cmd
        self.pub_robot1.publish(twist1)

        twist2 = Twist()
        twist2.linear.x = -v2_cmd
        twist2.angular.z = -omega2_cmd
        self.pub_robot2.publish(twist2)

    def stop_motors(self):
        """
        Sends zero-velocity Twist commands to both robots to safely stop them.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        try:
            if rclpy.ok():
                self.pub_robot1.publish(twist)
                self.pub_robot2.publish(twist)
                try:
                    self.get_logger().info("Stopping motors: Published zero velocity commands.")
                except Exception:
                    pass
                time.sleep(0.1)
        except Exception as e:
            print(f"Warning: Failed to publish stop command: {e}")

def main(args=None):
    """
    Main function that initializes the ROS2 node, sets up a SIGINT handler for clean shutdown,
    spins the node, and performs cleanup after shutdown.
    """
    rclpy.init(args=args)
    node = DualMPCController()

    def sigint_handler(signum, frame):
        try:
            node.stop_motors()
            node.get_logger().info("Shutdown signal received, stopping motors...")
        except Exception:
            pass
        rclpy.shutdown()
    import signal
    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_motors()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
