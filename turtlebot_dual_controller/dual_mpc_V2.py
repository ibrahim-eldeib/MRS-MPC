# #!/usr/bin/env python3
# """
# Dual MPC Controller Node with Dynamic Goal and Obstacle Updates

# This node controls two robots carrying a long object. The aim is to drive
# the center of the carried object (the average of the robots’ positions) to a target
# goal pose. The goal pose and obstacles are received via ROS2 topics.

# Key functionalities:
#   - Subscribes to /TB3_1/odom and /TB3_2/odom for robot odometry updates.
#   - Subscribes to /goal_pose (Pose2D) for the desired goal.
#   - Subscribes to /obstacles (std_msgs/String) for a JSON string of obstacles.
#   - Uses a simple Kalman filter per robot for state estimation.
#   - Sets up a differential-drive model and MPC (using do_mpc) with constraints:
#       * The cost is defined on the center of the two robots with stage (Q) and terminal (S) state-cost matrices.
#       * An input cost (R matrix) is also applied.
#       * A fixed-distance (rigid connection) constraint is enforced.
#       * Obstacle avoidance constraints keep both robots and the object center outside safety zones around obstacles.
#   - Updates the MPC setup when the goal or obstacles change.
#   - Publishes Twist commands to control the robots on /TB3_1/cmd_vel and /TB3_2/cmd_vel.
#   - Cleanly stops the motors on shutdown.
# """

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose2D, Twist
# from nav_msgs.msg import Odometry  # Using Odometry messages instead of Pose2D
# from std_msgs.msg import String
# import numpy as np
# import do_mpc
# import casadi as ca
# from math import pi, atan2
# import json
# import time
# import signal
# import math

# def quaternion_to_yaw(q):
#     """
#     Converts a quaternion into yaw angle.
#     """
#     # Standard conversion:
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)

# def wrap_angle(angle):
#     """
#     Wrap an angle to the range [-pi, pi].
#     """
#     return (angle + np.pi) % (2 * np.pi) - np.pi

# class DualMPCController(Node):
#     """
#     DualMPCController for two robots carrying a long object.
#     Goal pose and obstacles are updated via ROS2 topics.
#     """

#     class SimpleKF:
#         """
#         A simple Kalman filter for a 3D state [x, y, theta].
#         Assumes a constant state model (F = Identity) and an identity measurement model.
#         """
#         def __init__(self, initial_state, P, Q, R):
#             self.x = initial_state
#             self.P = P
#             self.Q = Q
#             self.R = R
#             self.F = np.eye(3)
#             self.H = np.eye(3)

#         def predict(self):
#             self.x = self.F.dot(self.x)
#             self.P = self.F.dot(self.P).dot(self.F.T) + self.Q
#             return self.x

#         def update(self, z):
#             # Perform the predict-update cycle.
#             self.predict()
#             y = z - self.H.dot(self.x)  # Measurement residual.
#             y[2] = wrap_angle(y[2])
#             S = self.H.dot(self.P).dot(self.H.T) + self.R
#             K = self.P.dot(self.H.T).dot(np.linalg.inv(S))
#             self.x = self.x + K.dot(y)
#             self.x[2] = wrap_angle(self.x[2])
#             self.P = (np.eye(3) - K.dot(self.H)).dot(self.P)
#             return self.x

#     def __init__(self):
#         super().__init__('dual_mpc_controller')
#         self.get_logger().info("Dual MPC Controller Node Starting...")

#         # ----------------------------
#         # Default values (can be overridden by topic messages)
#         # ----------------------------
#         self.goal_pose = np.array([0, -5.2, np.pi/2])  # [x, y, theta]
#         self.obstacles = []  # List of tuples: (obs_x, obs_y, obs_radius)

#         # Flags to indicate updated goal or obstacles.
#         self.goal_updated = False
#         self.obstacles_updated = False

#         # ----------------------------
#         # Setup Subscribers and Publishers
#         # ----------------------------
#         # Subscribe to odometry topics for TB3_1 and TB3_2.
#         self.sub_robot1 = self.create_subscription(Odometry, '/TB3_1/odom', self.robot1_odom_callback, 10)
#         self.sub_robot2 = self.create_subscription(Odometry, '/TB3_2/odom', self.robot2_odom_callback, 10)
#         # Goal pose subscriber.
#         self.sub_goal = self.create_subscription(Pose2D, '/goal_pose', self.goal_pose_callback, 10)
#         # Obstacles subscriber: expects a JSON string (e.g., "[[3.0,2.0,1.0], [5.0,0.0,0.5]]").
#         self.sub_obstacles = self.create_subscription(String, '/obstacles', self.obstacles_callback, 10)

#         # Command publishers. Now publishing to /TB3_1/cmd_vel and /TB3_2/cmd_vel.
#         self.pub_robot1 = self.create_publisher(Twist, '/TB3_1/cmd_vel', 10)
#         self.pub_robot2 = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)

#         # ----------------------------
#         # Initialize raw and filtered state for each robot.
#         # ----------------------------
#         self.robot1_state_raw = np.array([0.0, 0.0, 0.0])
#         self.robot2_state_raw = np.array([0.0, 0.0, 0.0])
#         self.robot1_state_filt = np.array([0.0, 0.0, 0.0])
#         self.robot2_state_filt = np.array([0.0, 0.0, 0.0])
#         self.pose1_received = False
#         self.pose2_received = False

#         # ----------------------------
#         # Create a Kalman filter instance for each robot.
#         # ----------------------------
#         self.kf_robot1 = self.SimpleKF(
#             initial_state=np.array([0.0, 0.0, 0.0]),
#             P=np.diag([1.0, 1.0, 0.1]),
#             Q=np.diag([0.01, 0.01, 0.005]),
#             R=np.diag([0.5, 0.5, 0.1])
#         )
#         self.kf_robot2 = self.SimpleKF(
#             initial_state=np.array([0.0, 0.0, 0.0]),
#             P=np.diag([1.0, 1.0, 0.1]),
#             Q=np.diag([0.01, 0.01, 0.005]),
#             R=np.diag([0.5, 0.5, 0.1])
#         )

#         # ----------------------------
#         # Get additional fixed parameters from the parameter server.
#         # These include maximum speeds and fixed distance between the robots.
#         # ----------------------------
#         self.declare_parameter("max_velocity", 1)
#         self.declare_parameter("max_angular_velocity", 0.5)
#         self.declare_parameter("min_distance", 0.1)
#         self.declare_parameter("max_distance", 1.2)
#         self.declare_parameter("margin", 0.2)  # Safety margin for obstacle avoidance.

#         self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
#         self.max_angular_velocity = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
#         self.min_distance = self.get_parameter("min_distance").get_parameter_value().double_value
#         self.max_distance = self.get_parameter("max_distance").get_parameter_value().double_value
#         self.margin = self.get_parameter("margin").get_parameter_value().double_value

#         # ----------------------------
#         # Setup dynamic model and MPC controller.
#         # ----------------------------
#         self.model = self.setup_model()
#         self.mpc = self.setup_mpc(self.model)

#         # ----------------------------
#         # Setup control loop timer (10 Hz).
#         # ----------------------------
#         self.timer = self.create_timer(0.1, self.control_loop)

#     # -------------------------------------------------------------------------
#     # ROS2 Callback Methods
#     # -------------------------------------------------------------------------
#     def robot1_odom_callback(self, msg: Odometry):
#         """
#         Callback for /TB3_1/odom.
#         Extracts pose information from the Odometry message and updates the state for TB3_1.
#         """
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#         yaw = quaternion_to_yaw(msg.pose.pose.orientation)
#         measurement = np.array([x, y, yaw])
#         self.robot1_state_raw = measurement
#         self.robot1_state_filt = self.kf_robot1.update(measurement)
#         self.pose1_received = True
#         self.get_logger().info(f"TB3_1 raw: {measurement} | Filtered: {self.robot1_state_filt}")

#     def robot2_odom_callback(self, msg: Odometry):
#         """
#         Callback for /TB3_2/odom.
#         Extracts pose information from the Odometry message and updates the state for TB3_2.
#         """
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#         yaw = quaternion_to_yaw(msg.pose.pose.orientation)
#         measurement = np.array([x, y, yaw])
#         self.robot2_state_raw = measurement
#         self.robot2_state_filt = self.kf_robot2.update(measurement)
#         self.pose2_received = True
#         self.get_logger().info(f"TB3_2 raw: {measurement} | Filtered: {self.robot2_state_filt}")

#     def goal_pose_callback(self, msg: Pose2D):
#         """
#         Callback for /goal_pose. Updates the goal pose.
#         """
#         self.goal_pose = np.array([msg.x, msg.y, msg.theta])
#         self.goal_updated = True
#         self.get_logger().info(f"Updated goal pose: {self.goal_pose}")

#     def obstacles_callback(self, msg: String):
#         """
#         Callback for /obstacles. Expects a JSON string containing a list of obstacles.
#         Each obstacle should be [obs_x, obs_y, obs_radius].
#         """
#         try:
#             obstacles_list = json.loads(msg.data)
#             self.obstacles = [(obs[0], obs[1], obs[2]) for obs in obstacles_list]
#             self.obstacles_updated = True
#             self.get_logger().info(f"Updated obstacles: {self.obstacles}")
#         except Exception as e:
#             self.get_logger().error(f"Error parsing obstacles: {e}")

#     # -------------------------------------------------------------------------
#     # MPC Setup Methods
#     # -------------------------------------------------------------------------
#     def setup_model(self):
#         """
#         Sets up the continuous dynamic model for both robots.
#         Uses differential-drive kinematics for each robot.
#         """
#         model = do_mpc.model.Model('continuous')

#         # State variables for TB3_1.
#         model.set_variable(var_type='_x', var_name='x1')
#         model.set_variable(var_type='_x', var_name='y1')
#         model.set_variable(var_type='_x', var_name='theta1')

#         # State variables for TB3_2.
#         model.set_variable(var_type='_x', var_name='x2')
#         model.set_variable(var_type='_x', var_name='y2')
#         model.set_variable(var_type='_x', var_name='theta2')

#         # Control inputs for TB3_1.
#         model.set_variable(var_type='_u', var_name='v1')
#         model.set_variable(var_type='_u', var_name='omega1')

#         # Control inputs for TB3_2.
#         model.set_variable(var_type='_u', var_name='v2')
#         model.set_variable(var_type='_u', var_name='omega2')

#         # Differential-drive kinematics for TB3_1.
#         model.set_rhs('x1', model.u['v1'] * ca.cos(model.x['theta1']))
#         model.set_rhs('y1', model.u['v1'] * ca.sin(model.x['theta1']))
#         model.set_rhs('theta1', model.u['omega1'])

#         # Differential-drive kinematics for TB3_2.
#         model.set_rhs('x2', model.u['v2'] * ca.cos(model.x['theta2']))
#         model.set_rhs('y2', model.u['v2'] * ca.sin(model.x['theta2']))
#         model.set_rhs('theta2', model.u['omega2'])

#         model.setup()
#         self.get_logger().info("Dynamic model setup complete.")
#         return model

#     def setup_mpc(self, model):
#         """
#         Configures the MPC controller.
#         The cost is defined on the center of the two robots.
#         Cost matrices Q, R, S are used:
#           - Q: weight on the state (center error) during each stage.
#           - S: terminal weight on the state error.
#           - R: weight on the control inputs.
#         Additional constraints include:
#           - A distance constraint ensuring the robots remain between a minimum and maximum separation.
#           - Obstacle avoidance constraints keeping both robots and the object center outside safety zones.
#         """
#         mpc = do_mpc.controller.MPC(model)
#         setup_mpc = {
#             'n_horizon': 30,
#             't_step': 0.1,
#             'state_discretization': 'collocation',
#             'collocation_type': 'radau',
#             'collocation_deg': 3,
#             'collocation_ni': 2,
#             'store_full_solution': True,
#             'nlpsol_opts': {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
#         }
#         mpc.set_param(**setup_mpc)

#         # Compute the center of the carried object.
#         x_center = (model.x['x1'] + model.x['x2']) / 2
#         y_center = (model.x['y1'] + model.x['y2']) / 2
#         theta_center = (model.x['theta1'] + model.x['theta2']) / 2

#         # # State error between the current center and the goal.
#         # goal_x, goal_y, goal_theta = self.goal_pose
#         # error_x = x_center - goal_x
#         # error_y = y_center - goal_y
#         # error_theta = ca.atan2(ca.sin(theta_center - goal_theta), ca.cos(theta_center - goal_theta))
#         # error = ca.vertcat(error_x, error_y, error_theta)

#         # # Define the cost matrices.
#         # Q = ca.diag(ca.vertcat(25, 25, 0.1))      # Stage cost weight on state error.
#         # S = ca.diag(ca.vertcat(25, 25, 0.1))      # Terminal cost weight on state error.
#         # R = ca.diag(ca.vertcat(0, 0, 0, 0))  # Explicit control cost matrix.

#         # # Control vector.
#         # u = ca.vertcat(model.u['v1'], model.u['omega1'], model.u['v2'], model.u['omega2'])
#         # control_cost = ca.mtimes([u.T, R, u])

#         # # Define stage and terminal cost.
#         # stage_cost = ca.mtimes([error.T, Q, error]) + control_cost
#         # terminal_cost = ca.mtimes([error.T, S, error])
#         # # Set the objective function.
#         # mpc.set_objective(lterm=stage_cost, mterm=terminal_cost)

#         # # Input cost: R is defined through the rterm penalties.
#         # # Here, we are assigning a cost for each input: [v1, omega1, v2, omega2].
#         # mpc.set_rterm(v1=1e-2, omega1=1e-2, v2=1e-2, omega2=1e-2)




#         # Define error between the current center and the goal pose.
#         goal_x, goal_y, goal_theta = self.goal_pose
#         error_x = x_center - goal_x
#         error_y = y_center - goal_y
#         error_theta = ca.atan2(ca.sin(theta_center - goal_theta), ca.cos(theta_center - goal_theta))
#         error = ca.vertcat(error_x, error_y, error_theta)

#         # Weight matrix for center error.
#         Q_center = ca.diag(ca.vertcat(1, 1, 0.1))
#         cost = ca.mtimes([error.T, Q_center, error])
#         mpc.set_objective(mterm=cost, lterm=cost)
#         mpc.set_rterm(v1=1e-2, omega1=1e-2, v2=1e-2, omega2=1e-2)

#         # Set control input bounds.
#         mpc.bounds['lower', '_u', 'v1'] = -self.max_velocity
#         mpc.bounds['upper', '_u', 'v1'] = self.max_velocity
#         mpc.bounds['lower', '_u', 'omega1'] = -self.max_angular_velocity
#         mpc.bounds['upper', '_u', 'omega1'] = self.max_angular_velocity
#         mpc.bounds['lower', '_u', 'v2'] = -self.max_velocity
#         mpc.bounds['upper', '_u', 'v2'] = self.max_velocity
#         mpc.bounds['lower', '_u', 'omega2'] = -self.max_angular_velocity
#         mpc.bounds['upper', '_u', 'omega2'] = self.max_angular_velocity

#         # Set control input bounds.
#         mpc.bounds['lower', '_u', 'v1'] = -self.max_velocity
#         mpc.bounds['upper', '_u', 'v1'] = self.max_velocity
#         mpc.bounds['lower', '_u', 'omega1'] = -self.max_angular_velocity
#         mpc.bounds['upper', '_u', 'omega1'] = self.max_angular_velocity
#         mpc.bounds['lower', '_u', 'v2'] = -self.max_velocity
#         mpc.bounds['upper', '_u', 'v2'] = self.max_velocity
#         mpc.bounds['lower', '_u', 'omega2'] = -self.max_angular_velocity
#         mpc.bounds['upper', '_u', 'omega2'] = self.max_angular_velocity

#         # # Distance constraint between the two robots.
#         # distance = ((model.x['x1'] - model.x['x2'])**2 + (model.x['y1'] - model.x['y2'])**2)
#         # mpc.set_nl_cons('distance_min', distance, ub=self.max_distance**2)
#         # mpc.set_nl_cons('distance_max', -distance, ub=-self.min_distance**2)

#         # Obstacle avoidance constraints.
#         if self.obstacles:
#             obstacle_x, obstacle_y = self.obstacles[0][:2]
#             margin = 0.1
#             obstacle_distance1 = ((model.x['x1'] - obstacle_x)**2 + (model.x['y1'] - obstacle_y)**2)
#             obstacle_distance2 = ((model.x['x2'] - obstacle_x)**2 + (model.x['y2'] - obstacle_y)**2)
#             object_x = (model.x['x1'] + model.x['x2']) / 2
#             object_y = (model.x['y1'] + model.x['y2']) / 2
#             object_distance = ((object_x - obstacle_x)**2 + (object_y - obstacle_y)**2)
#             safety_dist_sq = (0.2 + margin)**2
#             mpc.set_nl_cons('obstacle_avoidance1', -obstacle_distance1, ub=-safety_dist_sq)
#             mpc.set_nl_cons('obstacle_avoidance2', -obstacle_distance2, ub=-safety_dist_sq)
#             mpc.set_nl_cons('obstacle_avoidance_object', -object_distance, ub=-safety_dist_sq)

#         mpc.setup()
#         self.get_logger().info("MPC controller setup complete.")
#         return mpc

#     # -------------------------------------------------------------------------
#     # Main Control Loop
#     # -------------------------------------------------------------------------
#     def control_loop(self):
#         """
#         Main control loop running at 10 Hz.
#         - Reconfigures the MPC if a new goal message is received.
#         - Waits until both robot poses are received.
#         - Forms the state vector and computes the MPC step.
#         - Publishes control commands.
#         """
#         if self.goal_updated:
#             self.get_logger().info("Reconfiguring MPC with updated goal...")
#             self.mpc = self.setup_mpc(self.model)
#             self.goal_updated = False

#         if not (self.pose1_received and self.pose2_received):
#             self.get_logger().info("Waiting for both robot poses...")
#             return

#         # State vector: [x1, y1, theta1, x2, y2, theta2]
#         x0 = np.concatenate((self.robot1_state_filt, self.robot2_state_filt)).reshape(-1, 1)
#         self.mpc.x0 = x0

#         try:
#             # Compute control action using MPC.
#             u0 = self.mpc.make_step(x0)
#         except Exception as e:
#             self.get_logger().error(f"MPC error: {e}")
#             return

#         # Extract control commands.
#         v1_cmd = float(u0[0])
#         omega1_cmd = float(u0[1])
#         v2_cmd = float(u0[2])
#         omega2_cmd = float(u0[3])
#         self.get_logger().info(
#             f"Control Commands: TB3_1: v={v1_cmd:.2f}, omega={omega1_cmd:.2f} | "
#             f"TB3_2: v={v2_cmd:.2f}, omega={omega2_cmd:.2f}"
#         )

#         # Publish Twist commands.
#         twist1 = Twist()
#         twist1.linear.x = v1_cmd
#         twist1.angular.z = omega1_cmd
#         self.pub_robot1.publish(twist1)

#         twist2 = Twist()
#         twist2.linear.x = v2_cmd
#         twist2.angular.z = omega2_cmd
#         self.pub_robot2.publish(twist2)

#     # -------------------------------------------------------------------------
#     # Shutdown and Cleanup
#     # -------------------------------------------------------------------------
#     def stop_motors(self):
#         """
#         Publishes zero velocities to both robots to stop them safely.
#         """
#         twist = Twist()
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         if rclpy.ok():
#             self.pub_robot1.publish(twist)
#             self.pub_robot2.publish(twist)
#             self.get_logger().info("Stopping motors: Published zero velocity commands.")
#             time.sleep(0.1)

# def main(args=None):
#     """
#     Main function initializes the ROS2 node, sets up a SIGINT handler for a clean shutdown,
#     spins the node, and cleans up afterward.
#     """
#     rclpy.init(args=args)
#     node = DualMPCController()

#     def sigint_handler(signum, frame):
#         node.stop_motors()
#         node.get_logger().info("Shutdown signal received, stopping motors...")
#         rclpy.shutdown()

#     signal.signal(signal.SIGINT, sigint_handler)

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if rclpy.ok():
#             node.stop_motors()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# #!/usr/bin/env python3
# """
# MPC Controller Node for a Single TurtleBot

# This node controls a TurtleBot with a differential-drive model. The goal is to drive
# the robot to a target pose. The goal pose and obstacles are received via ROS2 topics.

# Key functionalities:
#   - Subscribes to /TB3_1/odom for the robot’s odometry.
#   - Subscribes to /goal_pose (Pose2D) for the desired goal.
#   - (Optionally) Subscribes to /obstacles (std_msgs/String) for obstacle information.
#   - Uses a single dynamic model with state [x, y, theta] and control inputs [v, omega].
#   - Implements an MPC with a stage cost (using Q and R matrices) and terminal cost (using S).
#   - Publishes Twist commands to control the robot on /TB3_1/cmd_vel.
#   - Cleanly stops the robot on shutdown.
# """

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose2D, Twist
# from nav_msgs.msg import Odometry  # Using Odometry messages
# from std_msgs.msg import String
# import numpy as np
# import do_mpc
# import casadi as ca
# from math import pi, atan2
# import json
# import time
# import signal
# import math

# def quaternion_to_yaw(q):
#     """
#     Converts a quaternion into a yaw angle.
#     """
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)

# def wrap_angle(angle):
#     """
#     Wrap an angle to the range [-pi, pi].
#     """
#     return (angle + np.pi) % (2 * np.pi) - np.pi

# class MPCController(Node):
#     """
#     MPCController for a single TurtleBot.
    
#     The robot is controlled by an MPC that minimizes the error
#     between its current state and a desired goal pose.
#     """
#     def __init__(self):
#         super().__init__('mpc_controller')
#         self.get_logger().info("MPC Controller Node Starting for Single TurtleBot...")

#         # ----------------------------
#         # Default values (can be overridden by topic messages)
#         # ----------------------------
#         self.goal_pose = np.array([10.0, 10.5, pi/2])  # [x, y, theta]
#         self.obstacles = []  # Optional: list of obstacles as tuples (obs_x, obs_y, obs_radius)

#         # Flag to indicate updated goal.
#         self.goal_updated = False

#         # ----------------------------
#         # Setup Subscribers and Publishers
#         # ----------------------------
#         # Subscribe to the odometry topic.
#         self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         # Goal pose subscriber.
#         self.sub_goal = self.create_subscription(Pose2D, '/goal_pose', self.goal_pose_callback, 10)
#         # Optional: Obstacles subscriber.
#         self.sub_obstacles = self.create_subscription(String, '/obstacles', self.obstacles_callback, 10)

#         # Publisher for velocity commands.
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

#         # ----------------------------
#         # Initialize state for the robot.
#         # ----------------------------
#         self.robot_state = np.array([0.0, 0.0, 0.0])
#         self.pose_received = False

#         # ----------------------------
#         # Get additional fixed parameters from the parameter server.
#         # ----------------------------
#         self.declare_parameter("max_velocity", 0.22)
#         self.declare_parameter("max_angular_velocity", 0.2)
#         self.declare_parameter("margin", 0.2)  # For obstacle avoidance if used.

#         self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
#         self.max_angular_velocity = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
#         self.margin = self.get_parameter("margin").get_parameter_value().double_value

#         # ----------------------------
#         # Setup dynamic model and MPC controller.
#         # ----------------------------
#         self.model = self.setup_model()
#         self.mpc = self.setup_mpc(self.model)

#         # ----------------------------
#         # Setup control loop timer (10 Hz).
#         # ----------------------------
#         self.timer = self.create_timer(0.1, self.control_loop)

#     # -------------------------------------------------------------------------
#     # ROS2 Callback Methods
#     # -------------------------------------------------------------------------
#     def odom_callback(self, msg: Odometry):
#         """
#         Callback for /TB3_1/odom.
#         Extracts pose information and updates the robot's state.
#         """
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#         yaw = quaternion_to_yaw(msg.pose.pose.orientation)
#         self.robot_state = np.array([x, y, yaw])
#         self.pose_received = True
#         self.get_logger().info(f"Odometry received: {self.robot_state}")

#     def goal_pose_callback(self, msg: Pose2D):
#         """
#         Callback for /goal_pose. Updates the goal pose.
#         """
#         self.goal_pose = np.array([msg.x, msg.y, msg.theta])
#         self.goal_updated = True
#         self.get_logger().info(f"Updated goal pose: {self.goal_pose}")

#     def obstacles_callback(self, msg: String):
#         """
#         Callback for /obstacles. Expects a JSON string containing a list of obstacles.
#         Each obstacle should be [obs_x, obs_y, obs_radius].
#         """
#         try:
#             obstacles_list = json.loads(msg.data)
#             self.obstacles = [(obs[0], obs[1], obs[2]) for obs in obstacles_list]
#             self.get_logger().info(f"Updated obstacles: {self.obstacles}")
#         except Exception as e:
#             self.get_logger().error(f"Error parsing obstacles: {e}")

#     # -------------------------------------------------------------------------
#     # MPC Setup Methods
#     # -------------------------------------------------------------------------
#     def setup_model(self):
#         """
#         Sets up the continuous dynamic model for the TurtleBot.
#         The model state is [x, y, theta] with control inputs [v, omega].
#         """
#         model = do_mpc.model.Model('continuous')

#         # Define state variables.
#         model.set_variable(var_type='_x', var_name='x')
#         model.set_variable(var_type='_x', var_name='y')
#         model.set_variable(var_type='_x', var_name='theta')

#         # Define control inputs.
#         model.set_variable(var_type='_u', var_name='v')
#         model.set_variable(var_type='_u', var_name='omega')

#         # Differential-drive kinematics.
#         model.set_rhs('x', model.u['v'] * ca.cos(model.x['theta']))
#         model.set_rhs('y', model.u['v'] * ca.sin(model.x['theta']))
#         model.set_rhs('theta', model.u['omega'])

#         model.setup()
#         self.get_logger().info("Dynamic model for single TurtleBot setup complete.")
#         return model

#     def setup_mpc(self, model):
#         """
#         Configures the MPC controller for the single robot.
#         The cost is defined based on the error between the current state and the goal.
#         Cost matrices Q (state), R (input), and S (terminal state) are used.
#         """
#         mpc = do_mpc.controller.MPC(model)
#         setup_mpc = {
#             'n_horizon': 30,
#             't_step': 0.1,
#             'state_discretization': 'collocation',
#             'collocation_type': 'radau',
#             'collocation_deg': 3,
#             'collocation_ni': 2,
#             'store_full_solution': True,
#             'nlpsol_opts': {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
#         }
#         mpc.set_param(**setup_mpc)

#         # Define error between current state and goal.
#         goal_x, goal_y, goal_theta = self.goal_pose
#         error_x = model.x['x'] - goal_x
#         error_y = model.x['y'] - goal_y
#         error_theta = ca.atan2(ca.sin(model.x['theta'] - goal_theta), ca.cos(model.x['theta'] - goal_theta))
#         error = ca.vertcat(error_x, error_y, error_theta)

#         # Define cost matrices.
#         Q = ca.diag(ca.vertcat(1, 1, 0.1))            # Stage cost weight on state error.
#         S = ca.diag(ca.vertcat(1, 1, 0.1))            # Terminal cost weight on state error.
#         R = ca.diag(ca.vertcat(1e-2, 1e-2))            # Control cost weight.

#         # Define control vector.
#         u = ca.vertcat(model.u['v'], model.u['omega'])
#         control_cost = ca.mtimes([u.T, R, u])

#         # Define stage and terminal cost.
#         stage_cost = ca.mtimes([error.T, Q, error]) + control_cost
#         terminal_cost = ca.mtimes([error.T, S, error])
#         mpc.set_objective(lterm=stage_cost, mterm=terminal_cost)

#         # Set control input bounds.
#         mpc.bounds['lower', '_u', 'v'] = -self.max_velocity
#         mpc.bounds['upper', '_u', 'v'] = self.max_velocity
#         mpc.bounds['lower', '_u', 'omega'] = -self.max_angular_velocity
#         mpc.bounds['upper', '_u', 'omega'] = self.max_angular_velocity

#         # (Optional) Obstacle avoidance constraints can be added here.

#         mpc.setup()
#         self.get_logger().info("MPC controller setup complete for single TurtleBot.")
#         return mpc

#     # -------------------------------------------------------------------------
#     # Main Control Loop
#     # -------------------------------------------------------------------------
#     def control_loop(self):
#         """
#         Main control loop running at 10 Hz.
#         - If a new goal message is received, the MPC is reconfigured.
#         - Waits until the robot pose is received.
#         - Computes the MPC step and publishes the control commands.
#         """
#         if self.goal_updated:
#             self.get_logger().info("Reconfiguring MPC with updated goal...")
#             self.mpc = self.setup_mpc(self.model)
#             self.goal_updated = False

#         if not self.pose_received:
#             self.get_logger().info("Waiting for odometry...")
#             return

#         # Form the state vector: [x, y, theta]
#         x0 = self.robot_state.reshape(-1, 1)
#         self.mpc.x0 = x0

#         try:
#             u0 = self.mpc.make_step(x0)
#         except Exception as e:
#             self.get_logger().error(f"MPC error: {e}")
#             return

#         # Extract control commands.
#         v_cmd = float(u0[0])
#         omega_cmd = float(u0[1])
#         self.get_logger().info(f"Control Commands: v = {v_cmd:.2f}, omega = {omega_cmd:.2f}")

#         # Publish Twist command.
#         twist = Twist()
#         twist.linear.x = v_cmd
#         twist.angular.z = omega_cmd
#         self.pub_cmd.publish(twist)

#     # -------------------------------------------------------------------------
#     # Shutdown and Cleanup
#     # -------------------------------------------------------------------------
#     def stop_robot(self):
#         """
#         Publishes zero velocities to stop the robot safely.
#         """
#         twist = Twist()
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         self.pub_cmd.publish(twist)
#         self.get_logger().info("Stopping robot: Published zero velocity command.")
#         time.sleep(0.1)

# def main(args=None):
#     """
#     Main function that initializes the ROS2 node, sets up a SIGINT handler for a clean shutdown,
#     spins the node, and cleans up afterward.
#     """
#     rclpy.init(args=args)
#     node = MPCController()

#     def sigint_handler(signum, frame):
#         node.stop_robot()
#         node.get_logger().info("Shutdown signal received, stopping robot...")
#         rclpy.shutdown()

#     signal.signal(signal.SIGINT, sigint_handler)

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if rclpy.ok():
#             node.stop_robot()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




#!/usr/bin/env python3
"""
Dual MPC Controller Node for Two TurtleBots

This node controls two TurtleBots. The aim is to drive the center
(i.e. the average of the robots' positions and orientations) to a target goal pose.
The goal pose is received via a ROS2 topic.

Key functionalities:
  - Subscribes to /TB3_1/odom and /TB3_2/odom for odometry updates.
  - Subscribes to /goal_pose (Pose2D) for the desired goal.
  - (Optionally) Subscribes to /obstacles (std_msgs/String) for obstacle information.
  - Uses raw odometry for state estimation (no filtering).
  - Sets up a dual-robot dynamic model and MPC (using do_mpc) with:
      * A cost that penalizes the error between the center of the robots and the goal using Q and S.
      * An explicit control cost using an R matrix.
      * (Optional) A distance constraint between the robots.
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
    Dual MPC Controller for two TurtleBots.
    The controller drives the average of the two robot states to a desired goal.
    """
    def __init__(self):
        super().__init__('dual_mpc_controller')
        self.get_logger().info("Dual MPC Controller Node Starting for Two TurtleBots...")

        # ----------------------------
        # Default values (can be overridden by topic messages)
        # ----------------------------
        self.goal_pose = np.array([10.0, 0, pi/2])  # [x, y, theta] for the center goal
        self.obstacles = [(5.0, 0 , 0.2)]  # Optional: list of obstacles as tuples (obs_x, obs_y, obs_radius)

        # Flag to indicate that a new goal has been received.
        self.goal_updated = False

        # ----------------------------
        # Setup Subscribers and Publishers
        # ----------------------------
        # Subscribe to odometry topics for both robots.
        self.sub_robot1 = self.create_subscription(Odometry, '/TB3_1/odom', self.robot1_odom_callback, 10)
        self.sub_robot2 = self.create_subscription(Odometry, '/TB3_2/odom', self.robot2_odom_callback, 10)
        # Goal pose subscriber.
        self.sub_goal = self.create_subscription(Pose2D, '/goal_pose', self.goal_pose_callback, 10)
        # Optional: Obstacles subscriber.
        self.sub_obstacles = self.create_subscription(String, '/obstacles', self.obstacles_callback, 10)

        # Publishers for velocity commands.
        self.pub_robot1 = self.create_publisher(Twist, '/TB3_1/cmd_vel', 10)
        self.pub_robot2 = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)

        # ----------------------------
        # Initialize state for each robot (raw odometry; no filtering).
        # ----------------------------
        self.robot1_state = np.array([0.0, 0.0, 0.0])
        self.robot2_state = np.array([0.0, 0.0, 0.0])
        self.pose1_received = False
        self.pose2_received = False

        # ----------------------------
        # Get additional fixed parameters.
        # ----------------------------
        self.declare_parameter("max_velocity", 0.2)
        self.declare_parameter("max_angular_velocity", 0.2)
        self.declare_parameter("min_distance", 0.1)
        self.declare_parameter("max_distance", 1.2)
        self.declare_parameter("margin", 0.2)  # For obstacle avoidance (if used)

        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
        self.min_distance = self.get_parameter("min_distance").get_parameter_value().double_value
        self.max_distance = self.get_parameter("max_distance").get_parameter_value().double_value
        self.margin = self.get_parameter("margin").get_parameter_value().double_value

        # ----------------------------
        # Setup dynamic model and MPC controller.
        # ----------------------------
        self.model = self.setup_model()
        self.mpc = self.setup_mpc(self.model)

        # ----------------------------
        # Setup control loop timer (10 Hz).
        # ----------------------------
        self.timer = self.create_timer(0.1, self.control_loop)

    # -------------------------------------------------------------------------
    # ROS2 Callback Methods
    # -------------------------------------------------------------------------
    def robot1_odom_callback(self, msg: Odometry):
        """
        Callback for /TB3_1/odom.
        Updates the state for the first TurtleBot.
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
        Updates the state for the second TurtleBot.
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
        Expects a JSON string representing a list of obstacles.
        Each obstacle should be [obs_x, obs_y, obs_radius].
        """
        try:
            obstacles_list = json.loads(msg.data)
            self.obstacles = [(obs[0], obs[1], obs[2]) for obs in obstacles_list]
            self.get_logger().info(f"Updated obstacles: {self.obstacles}")
        except Exception as e:
            self.get_logger().error(f"Error parsing obstacles: {e}")

    # -------------------------------------------------------------------------
    # MPC Setup Methods
    # -------------------------------------------------------------------------
    def setup_model(self):
        """
        Sets up the continuous dynamic model for the two TurtleBots.
        The state vector is: [x1, y1, theta1, x2, y2, theta2].
        The control vector is: [v1, omega1, v2, omega2].
        """
        model = do_mpc.model.Model('continuous')

        # State variables for TurtleBot 1.
        model.set_variable(var_type='_x', var_name='x1')
        model.set_variable(var_type='_x', var_name='y1')
        model.set_variable(var_type='_x', var_name='theta1')

        # State variables for TurtleBot 2.
        model.set_variable(var_type='_x', var_name='x2')
        model.set_variable(var_type='_x', var_name='y2')
        model.set_variable(var_type='_x', var_name='theta2')

        # Control inputs for TurtleBot 1.
        model.set_variable(var_type='_u', var_name='v1')
        model.set_variable(var_type='_u', var_name='omega1')

        # Control inputs for TurtleBot 2.
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
        Configures the MPC controller for the two TurtleBots.
        The objective is to drive the center (average) of the robots to a desired goal.
        Cost matrices:
            Q: weight on the state (center error) during each stage.
            S: terminal weight on the state error.
            R: explicit weight on the control inputs.
        A distance constraint (to maintain a desired separation) is optionally included.
        """
        mpc = do_mpc.controller.MPC(model)
        setup_mpc = {
            'n_horizon': 40,
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
        # Compute a simple average for theta (note: proper averaging of angles may require care).
        theta_center = (model.x['theta1'] + model.x['theta2']) / 2

        # Define error between the current center and the goal.
        goal_x, goal_y, goal_theta = self.goal_pose
        error_x = x_center - goal_x
        error_y = y_center - goal_y
        error_theta = ca.atan2(ca.sin(theta_center - goal_theta), ca.cos(theta_center - goal_theta))
        error = ca.vertcat(error_x, error_y, error_theta)

        # Define cost matrices.
        Q = ca.diag(ca.vertcat(1, 1, 0.1))            # Stage cost weight on state error.
        S = ca.diag(ca.vertcat(1, 1, 0.1))            # Terminal cost weight.
        R = ca.diag(ca.vertcat(1e-2, 1e-2, 1e-2, 1e-2))# Explicit control cost weight.

        # Define the control vector.
        u = ca.vertcat(model.u['v1'], model.u['omega1'], model.u['v2'], model.u['omega2'])
        control_cost = ca.mtimes([u.T, R, u])
        stage_cost = ca.mtimes([error.T, Q, error]) + control_cost
        terminal_cost = ca.mtimes([error.T, S, error])
        mpc.set_objective(lterm=stage_cost, mterm=terminal_cost)

        # Set control input bounds.
        mpc.bounds['lower', '_u', 'v1'] = -self.max_velocity
        mpc.bounds['upper', '_u', 'v1'] = self.max_velocity
        mpc.bounds['lower', '_u', 'omega1'] = -self.max_angular_velocity
        mpc.bounds['upper', '_u', 'omega1'] = self.max_angular_velocity
        mpc.bounds['lower', '_u', 'v2'] = -self.max_velocity
        mpc.bounds['upper', '_u', 'v2'] = self.max_velocity
        mpc.bounds['lower', '_u', 'omega2'] = -self.max_angular_velocity
        mpc.bounds['upper', '_u', 'omega2'] = self.max_angular_velocity

        # (Optional) Distance constraint between the two robots.
        # Ensures that the squared distance is between min_distance^2 and max_distance^2.
        distance = ((model.x['x1'] - model.x['x2'])**2 + (model.x['y1'] - model.x['y2'])**2)
        mpc.set_nl_cons('distance_min', distance, ub=self.max_distance**2)
        mpc.set_nl_cons('distance_max', -distance, ub=-self.min_distance**2)

        # (Optional) Obstacle avoidance constraints can be added here if needed.
        if self.obstacles:
            # Example: use the first obstacle in the list.
            obstacle_x, obstacle_y = self.obstacles[0][:2]
            margin = 0.1
            obstacle_distance1 = ((model.x['x1'] - obstacle_x)**2 + (model.x['y1'] - obstacle_y)**2)
            obstacle_distance2 = ((model.x['x2'] - obstacle_x)**2 + (model.x['y2'] - obstacle_y)**2)
            object_x = (model.x['x1'] + model.x['x2']) / 2
            object_y = (model.x['y1'] + model.x['y2']) / 2
            object_distance = ((object_x - obstacle_x)**2 + (object_y - obstacle_y)**2)
            safety_dist_sq = (0.2 + margin)**2
            mpc.set_nl_cons('obstacle_avoidance1', -obstacle_distance1, ub=-safety_dist_sq)
            mpc.set_nl_cons('obstacle_avoidance2', -obstacle_distance2, ub=-safety_dist_sq)
            mpc.set_nl_cons('obstacle_avoidance_object', -object_distance, ub=-safety_dist_sq)

        mpc.setup()
        self.get_logger().info("MPC controller setup complete for two TurtleBots.")
        return mpc

    # -------------------------------------------------------------------------
    # Main Control Loop
    # -------------------------------------------------------------------------
    def control_loop(self):
        """
        Main control loop running at 10 Hz.
        - If a new goal message is received, reconfigure the MPC.
        - Wait until both robot poses are received.
        - Form the state vector and compute the MPC step.
        - Publish the computed control commands to each robot.
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

        # Publish Twist commands for each robot.
        twist1 = Twist()
        twist1.linear.x = v1_cmd
        twist1.angular.z = omega1_cmd
        self.pub_robot1.publish(twist1)

        twist2 = Twist()
        twist2.linear.x = v2_cmd
        twist2.angular.z = omega2_cmd
        self.pub_robot2.publish(twist2)

    # -------------------------------------------------------------------------
    # Shutdown and Cleanup
    # -------------------------------------------------------------------------
    def stop_robots(self):
        """
        Publishes zero velocities to both robots to stop them safely.
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
    spins the node, and cleans up afterward.
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
