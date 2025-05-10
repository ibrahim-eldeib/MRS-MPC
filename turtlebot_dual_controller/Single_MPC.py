#!/usr/bin/env python3
"""
This node implements a single-robot MPC controller with a Kalman filter.
It:
  - Subscribes to the /robot1/pose topic (PoseStamped) to receive robot pose measurements.
  - Uses a simple Kalman Filter to fuse the noisy measurements and obtain a filtered state.
  - Sets up an MPC (using do-mpc) that uses a quadratic cost (S-Q-R style) to drive the robot toward a goal.
  - Publishes control commands (Twist messages) to the /robot1/cmd_vel topic.
  - Plots the predicted trajectory from the MPC, the current filtered pose, and the goal pose.
  - Logs debug information such as raw measurements, filtered state, error, and computed control command.
  
The cost function is defined as:
    J = (x - x_goal)^T Q (x - x_goal) + u^T R u,
with the orientation error computed using a wrapped difference.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
import numpy as np
import do_mpc
import casadi as ca
from math import pi, sqrt
import time
import signal
import matplotlib.pyplot as plt

# Enable interactive mode for matplotlib so that plots update in real time.
plt.ion()

def wrap_angle(angle):
    """
    Wrap an angle to the range [-pi, pi].
    
    Args:
        angle (float): The angle to be wrapped (in radians).
    
    Returns:
        float: The wrapped angle.
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

class SingleMPCController(Node):
    """
    The SingleMPCController node implements a single-robot MPC controller.
    
    Key functionalities:
      - Subscribes to /robot1/pose for receiving PoseStamped messages.
      - Uses an inner SimpleKF (Kalman Filter) class to estimate the robot state.
      - Sets up a do-mpc model and controller with a quadratic cost (S-Q-R style).
      - Publishes Twist commands to /robot1/cmd_vel to control the robot.
      - Plots (and logs) relevant debug information including state, goal, error, and control command.
    """

    # --- Inner Kalman Filter class ---
    class SimpleKF:
        """
        A simple Kalman Filter for estimating a 3D state [x, y, theta] for a robot with a constant
        state model. It assumes that the state does not change drastically between measurements and uses
        a linear model for the update.
        """
        def __init__(self, initial_state, P, Q, R):
            # Initialize state estimate and covariance matrices.
            self.x = initial_state  # Current state estimate (numpy array with shape (3,))
            self.P = P              # State covariance matrix (3x3)
            self.Q = Q              # Process noise covariance matrix (3x3)
            self.R = R              # Measurement noise covariance matrix (3x3)
            self.F = np.eye(3)      # State transition matrix (assumed identity)
            self.H = np.eye(3)      # Measurement matrix (assumed identity)

        def predict(self):
            """
            Predict the next state and update the covariance.
            
            Returns:
                np.array: The predicted state.
            """
            self.x = self.F.dot(self.x)
            self.P = self.F.dot(self.P).dot(self.F.T) + self.Q
            return self.x

        def update(self, z):
            """
            Update the state estimate given a new measurement z.
            
            Args:
                z (np.array): The new measurement (numpy array with shape (3,))
            
            Returns:
                np.array: The updated state estimate.
            """
            # First, predict the state based on the model.
            self.predict()
            # Compute the measurement residual.
            y = z - self.H.dot(self.x)
            # Wrap the angle difference.
            y[2] = wrap_angle(y[2])
            # Compute the innovation covariance.
            S = self.H.dot(self.P).dot(self.H.T) + self.R
            # Compute the Kalman gain.
            K = self.P.dot(self.H.T).dot(np.linalg.inv(S))
            # Update the state estimate.
            self.x = self.x + K.dot(y)
            self.x[2] = wrap_angle(self.x[2])
            # Update the covariance estimate.
            self.P = (np.eye(3) - K.dot(self.H)).dot(self.P)
            return self.x
    # --- End of inner KF class ---

    def __init__(self):
        """
        Initializes the SingleMPCController node.
        Sets up the state, goal, MPC, Kalman filter, publishers, and subscribers.
        """
        super().__init__('single_mpc_controller')
        self.get_logger().info("Single MPC Controller Node Starting...")

        # Initialize state vectors.
        self.state_raw = np.array([0.0, 0.0, 0.0])   # Latest raw measurement from sensor.
        self.state_filt = np.array([0.0, 0.0, 0.0])  # Filtered state after Kalman update.
        self.pose_received = False  # Flag indicating if a pose has been received.

        # Set goal state (modify as needed). For example, to move in y, set a nonzero goal orientation.
        # If goal is [0, 2, 0], with initial state [0,0,0], the robot will not move in y since sin(0)=0.
        # Use a goal like [0, 2, pi/2] to force a rotation and movement in y.
        self.goal = np.array([1, 2, np.pi/2])


        # Define the MPC prediction horizon (number of steps).
        self.n_horizon = 30

        # Set the maximum allowed speeds.
        self.max_velocity = 0.07          # Maximum linear velocity.
        self.max_angular_velocity = 0.1   # Maximum angular velocity.

        # Create a publisher for the robot command (Twist messages).
        self.pub_cmd = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        # Create a subscriber for the robot pose (PoseStamped messages).
        self.sub_pose = self.create_subscription(Pose2D, '/robot1/pose', self.pose_callback, 10)

        # Instantiate the Kalman Filter with initial state and covariance matrices.
        self.kf = self.SimpleKF(
            initial_state=np.array([0.0, 0.0, 0.0]),
            P=np.diag([0.1, 0.1, 0.01]),   # Lower initial uncertainty
            Q=np.diag([0.01, 0.01, 0.001]),  # Low process noise
            R=np.diag([0.05, 0.05, 0.01])    # Reduced measurement noise
        )

        # Set up the MPC model and controller.
        self.model = self.setup_model()
        self.mpc = self.setup_mpc(self.model)
        self.mpc.set_initial_guess()  # Provide an initial guess for the solver to help convergence.

        # Create a timer to run the control loop at 10 Hz.
        self.timer = self.create_timer(0.1, self.control_loop)

    def setup_model(self):
        """
        Sets up the dynamic model for the robot.
        
        The model is continuous and defined with:
          - State variables: x1, y1, theta1.
          - Control inputs: v1 (linear velocity), omega1 (angular velocity).
          - Kinematic equations for differential drive.
        
        Returns:
            do_mpc.model.Model: The configured model.
        """
        model = do_mpc.model.Model('continuous')
        # Define state variables.
        model.set_variable(var_type='_x', var_name='x1')
        model.set_variable(var_type='_x', var_name='y1')
        model.set_variable(var_type='_x', var_name='theta1')
        # Define control inputs.
        model.set_variable(var_type='_u', var_name='v1')
        model.set_variable(var_type='_u', var_name='omega1')

        # Set the state dynamics (differential drive kinematics).
        model.set_rhs('x1', model.u['v1'] * ca.cos(model.x['theta1']))
        model.set_rhs('y1', model.u['v1'] * ca.sin(model.x['theta1']))
        model.set_rhs('theta1', model.u['omega1'])
        
        model.setup()
        self.get_logger().info("Model setup complete.")
        return model

    def setup_mpc(self, model):
        """
        Configures the MPC controller using a quadratic cost function.
        
        The cost function is:
            J = (x - x_goal)^T Q (x - x_goal) + u^T R u,
        where the orientation error is computed using wrapped differences.
        
        Returns:
            do_mpc.controller.MPC: The configured MPC controller.
        """
        mpc = do_mpc.controller.MPC(model)
        # Set MPC parameters.
        setup_mpc = {
            'n_horizon': self.n_horizon,          # Prediction horizon.
            't_step': 0.1,                        # Time step.
            'state_discretization': 'collocation',
            'collocation_type': 'radau',
            'collocation_deg': 3,
            'collocation_ni': 2,
            'store_full_solution': True,
            'nlpsol_opts': {'print_time': 0, 'ipopt': {'print_level': 0}}
        }
        mpc.set_param(**setup_mpc)
        
        # Define the cost function.
        # Extract the state variables.
        x = model.x['x1']
        y = model.x['y1']
        theta = model.x['theta1']
        # Goal values.
        goal_x, goal_y, goal_theta = self.goal
        # Compute the orientation error using atan2 to properly wrap the difference.
        theta_error = ca.atan2(ca.sin(theta - goal_theta), ca.cos(theta - goal_theta))
        # Construct the error vector.
        error = ca.vertcat(x - goal_x, y - goal_y, theta_error)
        # Define weight matrices Q (for state error) and R (for input effort).
        Q = ca.diag(ca.vertcat(2, 2, 0.2))
        R = ca.diag(ca.vertcat(0.1, 0.1))
        # Compute the cost.
        cost = ca.mtimes([error.T, Q, error])
        mpc.set_objective(mterm=cost, lterm=cost)
        mpc.set_rterm(v1=0.01, omega1=0.01)
        
        # Define control bounds.
        mpc.bounds['lower', '_u', 'v1'] = -self.max_velocity
        mpc.bounds['upper', '_u', 'v1'] = self.max_velocity
        mpc.bounds['lower', '_u', 'omega1'] = -self.max_angular_velocity
        mpc.bounds['upper', '_u', 'omega1'] = self.max_angular_velocity
        
        mpc.setup()
        self.get_logger().info("MPC setup complete.")
        return mpc

    def quaternion_to_yaw(self, orientation):
        """
        Converts a quaternion to a yaw angle.
        
        Args:
            orientation: A geometry_msgs.msg.Quaternion.
            
        Returns:
            float: Yaw angle in radians.
        """
        norm = sqrt(orientation.x**2 + orientation.y**2 + orientation.z**2 + orientation.w**2)
        qx = orientation.x / norm
        qy = orientation.y / norm
        qz = orientation.z / norm
        qw = orientation.w / norm
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy**2 + qz**2)
        return np.arctan2(siny_cosp, cosy_cosp)

    # def pose_callback(self, msg):
    #     """
    #     Callback function for incoming PoseStamped messages on /robot1/pose.
        
    #     It updates the raw state and filtered state using the Kalman filter,
    #     sets the pose_received flag to True, and logs the raw, filtered states,
    #     and the error with respect to the goal.
    #     """
    #     x = msg.pose.position.x
    #     y = msg.pose.position.y
    #     theta = self.quaternion_to_yaw(msg.pose.orientation)
    #     measurement = np.array([x, y, theta])
    #     self.state_raw = measurement
    #     filtered_state = self.kf.update(measurement)
    #     self.state_filt = filtered_state
    #     self.pose_received = True
    #     try:
    #         error = np.array([
    #             self.state_filt[0] - self.goal[0],
    #             self.state_filt[1] - self.goal[1],
    #             wrap_angle(self.state_filt[2] - self.goal[2])
    #         ])
    #         self.get_logger().info(f"Robot1: raw: {measurement}, filtered: {filtered_state}, error: {error}")
    #     except Exception:
    #         pass



    def pose_callback(self, msg):
        """
        Callback function for incoming Pose2D messages on /robot1/pose.
        
        It updates the raw state and filtered state using the Kalman filter,
        sets the pose_received flag to True, and logs the raw, filtered states,
        and the error with respect to the goal.
        """
        x = msg.x
        y = msg.y
        theta = msg.theta  # Theta is already in radians.
        measurement = np.array([x, y, theta])
        self.state_raw = measurement
        filtered_state = self.kf.update(measurement)
        self.state_filt = filtered_state
        self.pose_received = True
        try:
            error = np.array([
                self.state_filt[0] - self.goal[0],
                self.state_filt[1] - self.goal[1],
                wrap_angle(self.state_filt[2] - self.goal[2])
            ])
            self.get_logger().info(f"Robot1: raw: {measurement}, filtered: {filtered_state}, error: {error}")
        except Exception:
            pass

    def control_loop(self):
        """
        Main control loop, called periodically (10 Hz):
          1. Checks if a pose has been received.
          2. Computes the error between the current state and the goal.
          3. Updates the MPC state (x0) with the filtered state.
          4. Calls the MPC step to compute a new control command.
          5. Logs the current goal, state, error, and control command.
          6. Publishes the computed Twist command on /robot1/cmd_vel.
        """
        if not self.pose_received:
            try:
                self.get_logger().info("Waiting for pose...")
            except Exception:
                pass
            return

        # Compute the error between the filtered state and the goal.
        error = np.array([
            self.state_filt[0] - self.goal[0],
            self.state_filt[1] - self.goal[1],
            wrap_angle(self.state_filt[2] - self.goal[2])
        ])
        try:
            self.get_logger().info(f"POSE: {self.state_raw}")
            self.get_logger().info(f"Goal: {self.goal}, State: {self.state_filt}, Error: {error}")
        except Exception:
            pass

        # Set the current state for the MPC.
        x0 = self.state_filt.reshape(-1, 1)
        self.mpc.x0 = x0

        # Compute the MPC step (control command).
        try:
            u0 = self.mpc.make_step(x0)
        except Exception as e:
            try:
                self.get_logger().error(f"MPC error: {e}")
            except Exception:
                pass
            return

        # Extract the control command.
        v_cmd = float(u0[0])
        omega_cmd = float(u0[1])
        try:
            self.get_logger().info(f"Robot1: cmd: v={v_cmd:.2f}, omega={omega_cmd:.2f}")
        except Exception:
            pass

        # Create and publish the Twist command.
        twist = Twist()
        twist.linear.x = -v_cmd
        twist.angular.z = -omega_cmd
        self.pub_cmd.publish(twist)

    def stop_motors(self):
        """
        Sends a zero-velocity Twist command to safely stop the robot.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        try:
            if rclpy.ok():
                self.pub_cmd.publish(twist)
                try:
                    self.get_logger().info("Stopping motors: Published zero velocity commands.")
                except Exception:
                    pass
                time.sleep(0.1)
        except Exception as e:
            print(f"Warning: Failed to publish stop command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SingleMPCController()

    # Handle SIGINT (Ctrl+C) to safely shutdown.
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
