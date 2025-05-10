#!/usr/bin/env python3
"""
Dual MPC Controller Node for Two TurtleBots with Obstacle Avoidance

This node controls two TurtleBots. The aim is to drive the center of the two robots
(i.e. the average of their positions and orientations) to a target goal pose.
Static obstacles are taken into account via additional nonlinear constraints.

Key functionalities:
  - Subscribes to /TB3_1/odom and /TB3_2/odom for odometry updates.
  - Subscribes to /goal_pose (Pose2D) for the desired goal.
  - Subscribes to /obstacles (std_msgs/String) where a JSON list of obstacles is provided.
  - Uses raw odometry for state estimation.
  - Sets up a dual-robot dynamic model and an MPC controller with:
      * A cost function defined on the error between the center and the goal using stage (Q) 
        and terminal (S) cost weights and an explicit control cost (R).
      * A distance constraint between the robots.
      * Obstacle avoidance constraints for each obstacle.
  - Publishes Twist commands to /TB3_1/cmd_vel and /TB3_2/cmd_vel.
  - Cleanly stops the robots on shutdown.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import do_mpc
import casadi as ca
from math import pi, atan2
import json
import time
import signal
import math

def quaternion_to_yaw(q):
    """
    Converts a quaternion into a yaw angle.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_angle(angle):
    """
    Wrap an angle to the range [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

class DualMPCController(Node):
    """
    Dual MPC Controller for two TurtleBots with obstacle avoidance.
    Drives the center (average state) of the two TurtleBots to a desired goal pose.
    """
    def __init__(self):
        super().__init__('dual_mpc_controller')
        self.get_logger().info("Dual MPC Controller Node Starting for Two TurtleBots...")

        # ----------------------------
        # Default parameters and goal.
        # ----------------------------
        self.goal_pose = np.array([15.0, 5, pi/2])  # [x, y, theta] for the center goal
        self.obstacles = [(0,4.0,0.4)]  # List of obstacles: each is a tuple (obs_x, obs_y, obs_radius)
        self.goal_updated = False

        # ----------------------------
        # Subscribers and Publishers.
        # ----------------------------
        self.sub_robot1 = self.create_subscription(Odometry, '/TB3_1/odom', self.robot1_odom_callback, 10)
        self.sub_robot2 = self.create_subscription(Odometry, '/TB3_2/odom', self.robot2_odom_callback, 10)
        self.sub_goal   = self.create_subscription(Pose2D, '/goal_pose', self.goal_pose_callback, 10)
        self.sub_obstacles = self.create_subscription(String, '/obstacles', self.obstacles_callback, 10)

        self.pub_robot1 = self.create_publisher(Twist, '/TB3_1/cmd_vel', 10)
        self.pub_robot2 = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)

        # ----------------------------
        # Initialize robot states.
        # ----------------------------
        self.robot1_state = np.array([0.0, 0.0, 0.0])
        self.robot2_state = np.array([0.0, 0.0, 0.0])
        self.pose1_received = False
        self.pose2_received = False

        # ----------------------------
        # Parameters from ROS parameter server.
        # ----------------------------
        self.declare_parameter("max_velocity", 0.22)
        self.declare_parameter("max_angular_velocity", 0.22)
        self.declare_parameter("min_distance", 0.5)   # Minimum separation squared will be used.
        self.declare_parameter("max_distance", 1.0)   # Maximum separation.
        self.declare_parameter("margin", 0.2)           # Margin for obstacle avoidance.

        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
        self.min_distance = self.get_parameter("min_distance").get_parameter_value().double_value
        self.max_distance = self.get_parameter("max_distance").get_parameter_value().double_value
        self.margin = self.get_parameter("margin").get_parameter_value().double_value

        # ----------------------------
        # Setup MPC model and controller.
        # ----------------------------
        self.model = self.setup_model()
        self.mpc = self.setup_mpc(self.model)

        # ----------------------------
        # Setup control loop timer (10 Hz).
        # ----------------------------
        self.timer = self.create_timer(0.1, self.control_loop)

    # -------------------------------------------------------------------------
    # Callback Methods.
    # -------------------------------------------------------------------------
    def robot1_odom_callback(self, msg: Odometry):
        """
        Callback for /TB3_1/odom.
        Updates the state for TurtleBot 1.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.robot1_state = np.array([x, y, yaw])
        self.pose1_received = True
        self.get_logger().info(f"TB3_1 state: {self.robot1_state}")

    def robot2_odom_callback(self, msg: Odometry):
        """
        Callback for /TB3_2/odom.
        Updates the state for TurtleBot 2.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.robot2_state = np.array([x, y, yaw])
        self.pose2_received = True
        self.get_logger().info(f"TB3_2 state: {self.robot2_state}")

    def goal_pose_callback(self, msg: Pose2D):
        """
        Callback for /goal_pose.
        Updates the desired goal pose for the center.
        """
        self.goal_pose = np.array([msg.x, msg.y, msg.theta])
        self.goal_updated = True
        self.get_logger().info(f"Updated goal pose: {self.goal_pose}")

    def obstacles_callback(self, msg: String):
        """
        Callback for /obstacles.
        Expects a JSON string containing a list of obstacles.
        Each obstacle should be [obs_x, obs_y, obs_radius].
        """
        try:
            obstacles_list = json.loads(msg.data)
            self.obstacles = [(obs[0], obs[1], obs[2]) for obs in obstacles_list]
            self.get_logger().info(f"Updated obstacles: {self.obstacles}")
        except Exception as e:
            self.get_logger().error(f"Error parsing obstacles: {e}")

    # -------------------------------------------------------------------------
    # MPC Model and Controller Setup.
    # -------------------------------------------------------------------------
    def setup_model(self):
        """
        Sets up the continuous dynamic model for two TurtleBots.
        State: [x1, y1, theta1, x2, y2, theta2]
        Control: [v1, omega1, v2, omega2] with differential-drive kinematics.
        """
        model = do_mpc.model.Model('continuous')

        # Define states for TurtleBot 1.
        model.set_variable(var_type='_x', var_name='x1')
        model.set_variable(var_type='_x', var_name='y1')
        model.set_variable(var_type='_x', var_name='theta1')

        # Define states for TurtleBot 2.
        model.set_variable(var_type='_x', var_name='x2')
        model.set_variable(var_type='_x', var_name='y2')
        model.set_variable(var_type='_x', var_name='theta2')

        # Define controls for TurtleBot 1.
        model.set_variable(var_type='_u', var_name='v1')
        model.set_variable(var_type='_u', var_name='omega1')

        # Define controls for TurtleBot 2.
        model.set_variable(var_type='_u', var_name='v2')
        model.set_variable(var_type='_u', var_name='omega2')

        # Differential-drive kinematics for TurtleBot 1.
        model.set_rhs('x1', model.u['v1'] * ca.cos(model.x['theta1']))
        model.set_rhs('y1', model.u['v1'] * ca.sin(model.x['theta1']))
        model.set_rhs('theta1', model.u['omega1'])

        # Differential-drive kinematics for TurtleBot 2.
        model.set_rhs('x2', model.u['v2'] * ca.cos(model.x['theta2']))
        model.set_rhs('y2', model.u['v2'] * ca.sin(model.x['theta2']))
        model.set_rhs('theta2', model.u['omega2'])

        model.setup()
        self.get_logger().info("Dynamic model for dual TurtleBots setup complete.")
        return model

    def setup_mpc(self, model):
        """
        Configures the MPC controller for two TurtleBots.
        The objective is to drive the center of the robots to the goal.
        Cost matrices:
          Q: stage weight on state error.
          S: terminal weight on state error.
          R: explicit weight on control inputs.
        In addition, distance constraints and obstacle avoidance constraints are added.
        """
        mpc = do_mpc.controller.MPC(model)
        setup_mpc = {
            'n_horizon': 60,
            't_step': 0.1,
            'state_discretization': 'collocation',
            'collocation_type': 'radau',
            'collocation_deg': 3,
            'collocation_ni': 2,
            'store_full_solution': True,
            'nlpsol_opts': {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
        }
        
        mpc.set_param(**setup_mpc)

        # Compute the center of the two TurtleBots.
        x_center = (model.x['x1'] + model.x['x2']) / 2
        y_center = (model.x['y1'] + model.x['y2']) / 2
        # For theta, a simple average is used (be cautious with angle wrapping).
        theta_center = (model.x['theta1'] + model.x['theta2']) / 2

        # Define the error between the center and the goal.
        goal_x, goal_y, goal_theta = self.goal_pose
        error_x = x_center - goal_x
        error_y = y_center - goal_y
        error_theta = ca.atan2(ca.sin(theta_center - goal_theta), ca.cos(theta_center - goal_theta))
        error = ca.vertcat(error_x, error_y, error_theta)

        # Define cost matrices.
        Q = ca.diag(ca.vertcat(10, 10, 0.1))
        S = ca.diag(ca.vertcat(1, 1, 0.1))
        R = ca.diag(ca.vertcat(1e-2, 1e-2, 1e-2, 1e-2))

        # Control vector.
        u = ca.vertcat(model.u['v1'], model.u['omega1'], model.u['v2'], model.u['omega2'])
        control_cost = ca.mtimes([u.T, R, u])
        stage_cost = ca.mtimes([error.T, Q, error]) + control_cost
        terminal_cost = ca.mtimes([error.T, S, error])
        mpc.set_objective(lterm=stage_cost, mterm=terminal_cost)

        # Penalize rate-of-change of each input to smooth commands
        mpc.set_rterm(
            v1=1e-2,
            omega1=1e-2,
            v2=1e-2,
            omega2=1e-2
        )

        # Set control input bounds.
        mpc.bounds['lower', '_u', 'v1'] = -self.max_velocity
        mpc.bounds['upper', '_u', 'v1'] = self.max_velocity
        mpc.bounds['lower', '_u', 'omega1'] = -self.max_angular_velocity
        mpc.bounds['upper', '_u', 'omega1'] = self.max_angular_velocity
        mpc.bounds['lower', '_u', 'v2'] = -self.max_velocity
        mpc.bounds['upper', '_u', 'v2'] = self.max_velocity
        mpc.bounds['lower', '_u', 'omega2'] = -self.max_angular_velocity
        mpc.bounds['upper', '_u', 'omega2'] = self.max_angular_velocity



        # Optional: Distance constraint between the two robots.
        distance = ((model.x['x1'] - model.x['x2'])**2 + (model.x['y1'] - model.x['y2'])**2)
        mpc.set_nl_cons('distance_min', distance, ub=self.max_distance**2)
        mpc.set_nl_cons('distance_max', -distance, ub=-self.min_distance**2)

        # Add obstacle avoidance constraints (if obstacles are provided).
        if self.obstacles:
            self.define_obstacle_constraints(mpc, model)

        mpc.setup()
        self.get_logger().info("MPC controller setup complete for two TurtleBots.")
        
        mpc.x0 = np.zeros((model.n_x, 1))
        mpc.u0 = np.zeros((model.n_u, 1))
        mpc.set_initial_guess()
        
        return mpc

    def define_obstacle_constraints(self, mpc, model):
        """
        Adds nonlinear obstacle avoidance constraints.
        For each obstacle (obs_x, obs_y, obs_radius) in self.obstacles, the following constraints
        are imposed:
          - The squared distance from TurtleBot 1 to the obstacle must be >= (obstacle_radius + margin)^2.
          - The squared distance from TurtleBot 2 to the obstacle must be >= (obstacle_radius + margin)^2.
          - The squared distance from the center of the two robots to the obstacle must be >= (obstacle_radius + margin)^2.
        """
        for i, obstacle in enumerate(self.obstacles):
            obstacle_x, obstacle_y, obstacle_radius = obstacle
            margin = self.margin
            safety_sq = (obstacle_radius + margin)**2

            # Squared distances.
            obstacle_distance1 = ( (model.x['x1'] - obstacle_x)**2 + (model.x['y1'] - obstacle_y)**2 )
            obstacle_distance2 = ( (model.x['x2'] - obstacle_x)**2 + (model.x['y2'] - obstacle_y)**2 )
            object_x = (model.x['x1'] + model.x['x2']) / 2
            object_y = (model.x['y1'] + model.x['y2']) / 2
            object_distance = ( (object_x - obstacle_x)**2 + (object_y - obstacle_y)**2 )

            mpc.set_nl_cons(f'obstacle_avoidance1_{i}', -obstacle_distance1, ub=-safety_sq)
            mpc.set_nl_cons(f'obstacle_avoidance2_{i}', -obstacle_distance2, ub=-safety_sq)
            mpc.set_nl_cons(f'obstacle_avoidance_object_{i}', -object_distance, ub=-safety_sq)

    # -------------------------------------------------------------------------
    # Control Loop.
    # -------------------------------------------------------------------------
    def control_loop(self):
        """
        Main control loop running at 10 Hz.
        - If a new goal has been received, the MPC is reconfigured.
        - Waits until both robot odometry messages have been received.
        - Forms the state vector and computes the MPC step.
        - Publishes the computed control commands to each robot.
        """
        if self.goal_updated:
            self.get_logger().info("Reconfiguring MPC with updated goal...")
            self.mpc = self.setup_mpc(self.model)
            self.goal_updated = False

        if not (self.pose1_received and self.pose2_received):
            self.get_logger().info("Waiting for both robot poses...")
            return

        # Form the state vector: [x1, y1, theta1, x2, y2, theta2]
        x0 = np.concatenate((self.robot1_state, self.robot2_state)).reshape(-1, 1)
        self.mpc.x0 = x0

        try:
            u0 = self.mpc.make_step(x0)
        except Exception as e:
            self.get_logger().error(f"MPC error: {e}")
            return

        # Extract control commands.
        v1_cmd = float(u0[0])
        omega1_cmd = float(u0[1])
        v2_cmd = float(u0[2])
        omega2_cmd = float(u0[3])
        self.get_logger().info(
            f"Control Commands: TB3_1: v={v1_cmd:.2f}, omega={omega1_cmd:.2f} | "
            f"TB3_2: v={v2_cmd:.2f}, omega={omega2_cmd:.2f}"
        )

        # Publish Twist commands.
        twist1 = Twist()
        twist1.linear.x = v1_cmd
        twist1.angular.z = omega1_cmd
        self.pub_robot1.publish(twist1)

        twist2 = Twist()
        twist2.linear.x = v2_cmd
        twist2.angular.z = omega2_cmd
        self.pub_robot2.publish(twist2)

    # -------------------------------------------------------------------------
    # Shutdown.
    # -------------------------------------------------------------------------
    def stop_robots(self):
        """
        Publishes zero velocities to both robots to ensure safe stopping.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_robot1.publish(twist)
        self.pub_robot2.publish(twist)
        self.get_logger().info("Stopping robots: Published zero velocity commands.")
        time.sleep(0.1)

def main(args=None):
    """
    Main function that initializes the ROS2 node, sets up a SIGINT handler for a clean shutdown,
    spins the node, and then cleans up.
    """
    rclpy.init(args=args)
    node = DualMPCController()

    def sigint_handler(signum, frame):
        node.stop_robots()
        node.get_logger().info("Shutdown signal received, stopping robots...")
        rclpy.shutdown()

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robots()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# End of the dual_mpc_V3.py file.
