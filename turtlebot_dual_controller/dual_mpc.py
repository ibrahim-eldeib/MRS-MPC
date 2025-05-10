import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import do_mpc
import casadi as ca
from math import pi, sqrt

class DualMPCController(Node):
    def __init__(self):
        super().__init__('dual_mpc_controller')
        self.get_logger().info("Dual MPC Controller Node Starting...")

        # Initialize states and flags for both robots: [x, y, theta]
        self.robot1_state = np.array([0.0, 0.0, 0.0])
        self.robot2_state = np.array([0.0, 0.0, 0.0])
        self.pose1_received = False
        self.pose2_received = False

        # Set goal states for each robot (you can use the same or different goals)
        self.robot1_goal = np.array([0.48, 2.0, pi/2])
        self.robot2_goal = np.array([0.90, 2.0, pi/2])

        # Control bounds
        self.max_velocity = 0.08
        self.max_angular_velocity = 0.15

        # Create publishers for each robot's command topic
        self.pub_robot1 = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.pub_robot2 = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # Create subscriptions for each robot's pose topic
        self.sub_robot1 = self.create_subscription(PoseStamped, '/robot1/pose', self.robot1_pose_callback, 10)
        self.sub_robot2 = self.create_subscription(PoseStamped, '/robot2/pose', self.robot2_pose_callback, 10)

        # Set up the dual robot model and MPC controller
        self.model = self.setup_model()
        self.mpc = self.setup_mpc(self.model)

        # Create a timer for the control loop (running at 10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def setup_model(self):
        # Create a continuous-time model
        model = do_mpc.model.Model('continuous')

        # Robot1 state variables: x1, y1, theta1
        x1 = model.set_variable(var_type='_x', var_name='x1')
        y1 = model.set_variable(var_type='_x', var_name='y1')
        theta1 = model.set_variable(var_type='_x', var_name='theta1')

        # Robot2 state variables: x2, y2, theta2
        x2 = model.set_variable(var_type='_x', var_name='x2')
        y2 = model.set_variable(var_type='_x', var_name='y2')
        theta2 = model.set_variable(var_type='_x', var_name='theta2')

        # Control inputs for Robot1: linear velocity (v1) and angular velocity (omega1)
        v1 = model.set_variable(var_type='_u', var_name='v1')
        omega1 = model.set_variable(var_type='_u', var_name='omega1')

        # Control inputs for Robot2: linear velocity (v2) and angular velocity (omega2)
        v2 = model.set_variable(var_type='_u', var_name='v2')
        omega2 = model.set_variable(var_type='_u', var_name='omega2')

        # Dynamics: simple kinematic model (differential drive)
        model.set_rhs('x1', v1 * ca.cos(theta1))
        model.set_rhs('y1', v1 * ca.sin(theta1))
        model.set_rhs('theta1', omega1)

        model.set_rhs('x2', v2 * ca.cos(theta2))
        model.set_rhs('y2', v2 * ca.sin(theta2))
        model.set_rhs('theta2', omega2)

        model.setup()
        self.get_logger().info("Dual robot model setup complete.")
        return model

    def setup_mpc(self, model):
        mpc = do_mpc.controller.MPC(model)
        setup_mpc = {
            'n_horizon': 10,
            't_step': 0.1,
            'state_discretization': 'collocation',
            'collocation_type': 'radau',
            'collocation_deg': 3,
            'collocation_ni': 2,
            'store_full_solution': True,
            'nlpsol_opts': {'print_time': 0, 'ipopt': {'print_level': 0}}, 

        }
        mpc.set_param(**setup_mpc)

        # Define the objective: minimize the squared distance to each robot's goal (only x and y)
        x1 = model.x['x1']
        y1 = model.x['y1']
        x2 = model.x['x2']
        y2 = model.x['y2']
        goal1_x, goal1_y, _ = self.robot1_goal
        goal2_x, goal2_y, _ = self.robot2_goal

        cost = 3*(x1 - goal1_x)**2 + (y1 - goal1_y)**2 + (x2 - goal2_x)**2 + (y2 - goal2_y)**2
        mpc.set_objective(mterm=cost, lterm=cost)
        mpc.set_rterm(v1=0.1, omega1=0.1, v2=0.1, omega2=0.1)

        # Set control bounds for Robot1
        mpc.bounds['lower', '_u', 'v1'] = -self.max_velocity
        mpc.bounds['upper', '_u', 'v1'] = self.max_velocity
        mpc.bounds['lower', '_u', 'omega1'] = -self.max_angular_velocity
        mpc.bounds['upper', '_u', 'omega1'] = self.max_angular_velocity

        # Set control bounds for Robot2
        mpc.bounds['lower', '_u', 'v2'] = -self.max_velocity
        mpc.bounds['upper', '_u', 'v2'] = self.max_velocity
        mpc.bounds['lower', '_u', 'omega2'] = -self.max_angular_velocity
        mpc.bounds['upper', '_u', 'omega2'] = self.max_angular_velocity

        mpc.setup()
        self.get_logger().info("Dual MPC setup complete.")
        return mpc
    
    def quaternion_to_yaw(self, orientation):
        # Normalize the quaternion
        norm = sqrt(orientation.x**2 + orientation.y**2 + orientation.z**2 + orientation.w**2)
        qx = orientation.x / norm
        qy = orientation.y / norm
        qz = orientation.z / norm
        qw = orientation.w / norm

        # Convert to yaw using the standard formula
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy**2 + qz**2)
        return np.arctan2(siny_cosp, cosy_cosp)

    def robot1_pose_callback(self, msg):
        # Update robot1 state from PoseStamped
        x = msg.pose.position.x
        y = msg.pose.position.y
        # Convert quaternion to yaw (theta)
        theta = self.quaternion_to_yaw(msg.pose.orientation)
        self.robot1_state = np.array([x, y, theta])
        self.pose1_received = True
        self.get_logger().info(f"Robot1 pose: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")

    def robot2_pose_callback(self, msg):
        # Update robot2 state from PoseStamped
        x = msg.pose.position.x
        y = msg.pose.position.y
        # Convert quaternion to yaw (theta)
        theta = self.quaternion_to_yaw(msg.pose.orientation)
        self.robot2_state = np.array([x, y, theta])
        self.pose2_received = True
        self.get_logger().info(f"Robot2 pose: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")


    def control_loop(self):
        # Ensure both poses have been received
        if not (self.pose1_received and self.pose2_received):
            self.get_logger().info("Waiting for both robot poses...")
            return

        # Combine states: [x1, y1, theta1, x2, y2, theta2]
        x0 = np.concatenate((self.robot1_state, self.robot2_state)).reshape(-1, 1)
        self.mpc.x0 = x0

        try:
            # Compute control inputs: u0 = [v1, omega1, v2, omega2]
            u0 = self.mpc.make_step(x0)
        except Exception as e:
            self.get_logger().error(f"MPC error: {e}")
            return

        v1_cmd = float(u0[0])
        omega1_cmd = float(u0[1])
        v2_cmd = float(u0[2])
        omega2_cmd = float(u0[3])
        self.get_logger().info(f"Robot1 cmd: v={v1_cmd:.2f}, omega={omega1_cmd:.2f}")
        self.get_logger().info(f"Robot2 cmd: v={v2_cmd:.2f}, omega={omega2_cmd:.2f}")

        # Publish commands for Robot1
        twist1 = Twist()
        twist1.linear.x = v1_cmd
        twist1.angular.z = omega1_cmd
        self.pub_robot1.publish(twist1)

        # Publish commands for Robot2
        twist2 = Twist()
        twist2.linear.x = v2_cmd
        twist2.angular.z = omega2_cmd
        self.pub_robot2.publish(twist2)

    def stop_motors(self):
        """Publish zero commands to stop the robots if the ROS context is still valid."""
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
                import time
                time.sleep(0.1)
        except Exception as e:
            print(f"Warning: Failed to publish stop command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DualMPCController()

    # Install a SIGINT handler to catch Ctrl+C and stop motors before shutdown.
    import signal
    def sigint_handler(signum, frame):
        try:
            node.stop_motors()
            node.get_logger().info("Shutdown signal received, stopping motors...")
        except Exception:
            pass
        rclpy.shutdown()
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
