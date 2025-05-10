# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Pose2D
# from nav_msgs.msg import Odometry
# import numpy as np
# import casadi as ca
# from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
# import math

# def quaternion_to_yaw(q):
#     siny = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
#     return math.atan2(siny, cosy)

# class SingleRobotAcados(Node):
#     def __init__(self):
#         super().__init__('single_robot_acados')
#         # Default goal if none received yet:
#         self.goal = np.array([15.0, 15.0, 0.0])  # x, y, theta
#         self.x_meas = np.zeros(3)
#         self.odom_ready = False

#         # ROS subscribers & publishers
#         self.create_subscription(Odometry,  '/odom',      self.odom_cb, 10)
#         self.create_subscription(Pose2D,    '/goal_pose', self.goal_cb, 10)
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Build ACADOS solver
#         self.solver = self.setup_acados()
#         # “Warm up” first solve so it JIT-compiles before control loop starts
#         self.solver.set(0, "lbx", np.zeros(3))
#         self.solver.set(0, "ubx", np.zeros(3))
#         self.solver.solve()

#         # Control loop at 10 Hz
#         self.create_timer(0.1, self.control_loop)
#         self.get_logger().info("Single‐robot ACADOS MPC node started.")

#     def setup_acados(self):
#         from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
#         import casadi as ca
#         import numpy as np

#         # 1) Symbolic variables
#         # States: x, y, theta
#         x = ca.SX.sym('x', 3)
#         # Controls: v (linear), omega (angular)
#         u = ca.SX.sym('u', 2)

#         # 2) Continuous‐time dynamics (simple unicycle)
#         f_expl = ca.vertcat(
#             u[0] * ca.cos(x[2]),
#             u[0] * ca.sin(x[2]),
#             u[1]
#         )

#         # 3) Wrap into an AcadosModel
#         model = AcadosModel()
#         model.name = 'single_tb3'
#         model.x = x
#         model.u = u
#         model.f_expl_expr = f_expl

#         # 4) Create the OCP object
#         ocp = AcadosOcp()
#         ocp.model = model

#         # 5) Solver & horizon settings
#         N  = 30        # number of intervals
#         Ts = 0.1       # length of each interval (s)
#         ocp.dims.N = N
#         ocp.solver_options.tf            = N * Ts
#         ocp.solver_options.qp_solver     = 'FULL_CONDENSING_HPIPM'
#         ocp.solver_options.integrator_type  = 'ERK'
#         ocp.solver_options.hessian_approx   = 'GAUSS_NEWTON'
#         ocp.solver_options.nlp_solver_type  = 'SQP_RTI'

#         # 6) Cost: track (x,y,θ) to self.goal and penalize u
#         #    error = [x−gx, y−gy, wrap(θ−gθ)]
#         gx, gy, gth = self.goal
#         err = ca.vertcat(
#             x[0] - gx,
#             x[1] - gy,
#             ca.atan2(ca.sin(x[2] - gth), ca.cos(x[2] - gth))
#         )
#         # stage cost sees [err; u], terminal cost only err
#         y   = ca.vertcat(err,   u)
#         y_e = err

#         ocp.model.cost_y_expr   = y
#         ocp.model.cost_y_expr_e = y_e

#         ocp.cost.cost_type   = 'NONLINEAR_LS'
#         ocp.cost.cost_type_e = 'NONLINEAR_LS'

#         ocp.dims.ny   = y.size1()
#         ocp.dims.ny_e = y_e.size1()

#         # Weight matrices (numeric numpy arrays!)
#         ocp.cost.W    = np.diag([10.0, 10.0, 0.1, 1e-2, 1e-2])
#         ocp.cost.W_e  = np.diag([1.0, 1.0, 0.1])

#         # Zero‐reference
#         ocp.cost.yref   = np.zeros(ocp.dims.ny)
#         ocp.cost.yref_e = np.zeros(ocp.dims.ny_e)

#         # 7) Input bounds on v and ω
#         max_v = 0.22
#         max_w = 0.50
#         ocp.constraints.idxbu = np.array([0, 1])
#         ocp.constraints.lbu   = np.array([-max_v, -max_w])
#         ocp.constraints.ubu   = np.array([ max_v,  max_w])

#         # 8) *** Initial‐state bounds (stage 0) ***
#         #     Constrain x[0], x[1], x[2] to equal measured state at runtime
#         ocp.constraints.idxbx_0 = np.array([0, 1, 2])
#         ocp.constraints.lbx_0   = np.zeros(3)
#         ocp.constraints.ubx_0   = np.zeros(3)

#         # 9) Build and return the ACADOS solver
#         return AcadosOcpSolver(ocp, json_file='single_tb3_ocp.json')


#     def odom_cb(self, msg: Odometry):
#         px = msg.pose.pose.position.x
#         py = msg.pose.pose.position.y
#         yaw = quaternion_to_yaw(msg.pose.pose.orientation)
#         self.x_meas = np.array([px, py, yaw])
#         self.odom_ready = True

#     def goal_cb(self, msg: Pose2D):
#         self.goal = np.array([msg.x, msg.y, msg.theta])
#         self.get_logger().info(f"New goal: {self.goal}")

#     def control_loop(self):
#         if not self.odom_ready:
#             return

#         # 1) fix current state
#         self.solver.set(0, "lbx", self.x_meas)
#         self.solver.set(0, "ubx", self.x_meas)

#         # 2) solve OCP
#         status = self.solver.solve()
#         if status != 0:
#             self.get_logger().warn(f"ACADOS returned status {status}")

#         # 3) extract & publish first control
#         u0 = self.solver.get(0, "u")
#         twist = Twist()
#         twist.linear.x  = float(u0[0])
#         twist.angular.z = float(u0[1])
#         self.cmd_pub.publish(twist)

#     def destroy_node(self):
#         # stop robot on shutdown
#         twist = Twist()
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         self.cmd_pub.publish(twist)
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = SingleRobotAcados()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()












# #!/usr/bin/env python3
# """
# dual_robot_acados_soft_obstacles.py

# Dual‐TurtleBot MPC via ACADOS, with soft obstacle constraints:
#   - If an obstacle constraint cannot be met, a slack variable
#     absorbs the violation at a high cost, preventing QP failures.
# """

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Pose2D
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String
# import numpy as np
# import casadi as ca
# from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
# import json, math, signal

# def quaternion_to_yaw(q):
#     """Convert ROS quaternion to yaw angle."""
#     siny = 2.0*(q.w*q.z + q.x*q.y)
#     cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
#     return math.atan2(siny, cosy)

# class DualRobotAcadosSoft(Node):
#     def __init__(self):
#         super().__init__('dual_robot_acados_soft')
#         self.get_logger().info("Starting dual‐robot ACADOS MPC (soft obstacles)…")

#         # — Default goal & obstacles —
#         self.goal_pose = np.array([5.0, -5.0, math.pi/2])
#         # Each obstacle: (x, y, radius)
#         self.obstacles = [(-3.9,1.8,0.6), (-3.3,2.5,0.6), (-3.2,1.7,1.0)]
#         self.goal_updated = False

#         # — Robot state placeholders —
#         self.x1, self.x2 = np.zeros(3), np.zeros(3)
#         self.pose1_ready = self.pose2_ready = False

#         # — Parameters —
#         self.declare_parameter("max_velocity",         0.22)
#         self.declare_parameter("max_angular_velocity", 0.22)
#         self.declare_parameter("margin",               0.2)
#         # how heavily to penalize slack (violation) in the cost
#         self.declare_parameter("slack_weight",        1e4)

#         self.max_v      = self.get_parameter("max_velocity").value
#         self.max_w      = self.get_parameter("max_angular_velocity").value
#         self.margin     = self.get_parameter("margin").value
#         self.slack_w    = self.get_parameter("slack_weight").value

#         # — Subscriptions & publications —
#         self.create_subscription(Odometry, '/TB3_1/odom',  self.cb_robot1_odom, 10)
#         self.create_subscription(Odometry, '/TB3_2/odom',  self.cb_robot2_odom, 10)
#         self.create_subscription(Pose2D,   '/goal_pose',   self.cb_goal_pose,  10)
#         self.create_subscription(String,   '/obstacles',   self.cb_obstacles,  10)
#         self.pub1 = self.create_publisher(Twist, '/TB3_1/cmd_vel', 10)
#         self.pub2 = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)

#         # — Build & warm‐up solver —
#         self.solver = self.setup_acados()
#         zero6 = np.zeros(6)
#         # initialize lbx/ubx so codegen finishes happily
#         self.solver.set(0, "lbx", zero6)
#         self.solver.set(0, "ubx", zero6)
#         self.solver.solve()
#         self.last_u = np.zeros(4)

#         # — Control loop @ 10 Hz —
#         self.create_timer(0.1, self.control_loop)
#         signal.signal(signal.SIGINT, self.shutdown_handler)

#     def setup_acados(self):
#         """
#         Build ACADOS OCP with:
#           - 6 states: [x1,y1,θ1, x2,y2,θ2]
#           - 4 controls: [v1,ω1, v2,ω2]
#           - M slack controls for M obstacles × 2 robots
#           - Quadratic cost on midpoint→goal + control effort + slack penalty
#           - Soft obstacle constraints: h_i(x)+s_i ≥ 0, s_i ≥ 0
#         """
#         import numpy as np
#         import casadi as ca
#         from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

#         # 1) Unpack parameters
#         gx, gy, gth = self.goal_pose
#         obs_list    = self.obstacles
#         margin      = self.margin
#         slack_w     = self.slack_w

#         M = len(obs_list)
#         # two robots ⇒ 2 constraints per obstacle ⇒ 2M slacks
#         n_slack = 2 * M

#         # 2) Symbolic variables
#         x = ca.SX.sym('x', 6)
#         u = ca.SX.sym('u', 4 + n_slack)  # [v1,w1, v2,w2, s1…s2M]

#         # 3) Dynamics
#         x1,y1,th1, x2,y2,th2 = x[0],x[1],x[2], x[3],x[4],x[5]
#         v1,w1,v2,w2         = u[0],u[1],u[2],u[3]
#         xdot = ca.vertcat(
#             v1*ca.cos(th1),
#             v1*ca.sin(th1),
#             w1,
#             v2*ca.cos(th2),
#             v2*ca.sin(th2),
#             w2
#         )

#         # 4) ACADOS model
#         model = AcadosModel()
#         model.name        = 'dual_tb3_soft_obs'
#         model.x           = x
#         model.u           = u
#         model.f_expl_expr = xdot

#         # 5) OCP object
#         ocp = AcadosOcp()
#         ocp.model = model

#         # 6) Horizon & solver opts
#         N, Ts = 30, 0.1
#         ocp.dims.N                         = N
#         ocp.solver_options.tf              = N * Ts
#         ocp.solver_options.qp_solver       = 'FULL_CONDENSING_HPIPM'
#         ocp.solver_options.integrator_type = 'ERK'
#         ocp.solver_options.hessian_approx  = 'GAUSS_NEWTON'
#         ocp.solver_options.nlp_solver_type = 'SQP_RTI'
#         # robustness
#         ocp.solver_options.levenberg_marquardt = 1e-6
#         ocp.solver_options.qp_solver_tol_eq    = 1e-4
#         ocp.solver_options.qp_solver_tol_ineq = 1e-4
#         ocp.solver_options.qp_solver_iter_max = 30

#         # 7) Cost: midpoint error + controls + slack penalties
#         x_c  = (x1 + x2)/2
#         y_c  = (y1 + y2)/2
#         th_c = ca.atan2(
#                     ca.sin((th1+th2)/2 - gth),
#                     ca.cos((th1+th2)/2 - gth)
#                )
#         err    = ca.vertcat(x_c-gx, y_c-gy, th_c)
#         # stage cost uses: [err; v1,w1,v2,w2; slack]
#         y_expr = ca.vertcat(err, u[0:4], u[4:4+n_slack])
#         y_e    = err  # terminal cost only on error

#         ocp.model.cost_y_expr   = y_expr
#         ocp.model.cost_y_expr_e = y_e
#         ocp.cost.cost_type      = 'NONLINEAR_LS'
#         ocp.cost.cost_type_e    = 'NONLINEAR_LS'
#         ocp.dims.ny    = y_expr.size1()
#         ocp.dims.ny_e  = y_e.size1()

#         # Build weight matrix:
#         #   [10,10,0.1] on err, [1e-2]*4 on controls, [slack_w]*n_slack on slacks
#         W = np.diag( [10,10,0.1] +
#                      [1e-2]*4 +
#                      [slack_w]*n_slack )
#         ocp.cost.W    = W
#         ocp.cost.W_e  = np.diag([1,1,0.1])
#         ocp.cost.yref   = np.zeros(ocp.dims.ny)
#         ocp.cost.yref_e = np.zeros(ocp.dims.ny_e)

#         # 8) Input bounds: controls and slack ≥ 0
#         idxbu = np.arange(4+n_slack)
#         lbu   = np.array([-self.max_v, -self.max_w,
#                           -self.max_v, -self.max_w] +
#                          [0.0]*n_slack)
#         ubu   = np.array([ self.max_v,  self.max_w,
#                            self.max_v,  self.max_w] +
#                          [1e3]*n_slack)  # slack upper could be large
#         ocp.constraints.idxbu = idxbu
#         ocp.constraints.lbu   = lbu
#         ocp.constraints.ubu   = ubu

#         # 9) Pin initial state at stage 0
#         ocp.constraints.idxbx_0 = np.arange(6)
#         ocp.constraints.lbx_0   = np.zeros(6)
#         ocp.constraints.ubx_0   = np.zeros(6)

#         # 10) Soft obstacle constraints:
#         #     h_i(x) + s_i ≥ 0
#         h_exprs = []
#         for i, (ox,oy,orad) in enumerate(obs_list):
#             safe = (orad + margin)**2
#             # robot1
#             h1 = (x1-ox)**2 + (y1-oy)**2 - safe
#             # robot2
#             h2 = (x2-ox)**2 + (y2-oy)**2 - safe
#             # slack indices in u: s1 at u[4+2*i], s2 at u[5+2*i]
#             s1 = u[4 + 2*i]
#             s2 = u[4 + 2*i + 1]
#             h_exprs += [h1 + s1, h2 + s2]

#         h = ca.vertcat(*h_exprs)
#         ocp.model.con_h_expr = h
#         ocp.dims.nh = h.size1()
#         INF = 1e6
#         ocp.constraints.lh = np.zeros(ocp.dims.nh)       # ≥ 0
#         ocp.constraints.uh = INF * np.ones(ocp.dims.nh)   # no upper

#         # 11) Create solver (writes JSON, compiles C, makes .so)
#         return AcadosOcpSolver(ocp, json_file='dual_tb3_soft_ocp.json')

#     # — Callbacks —
#     def cb_robot1_odom(self, msg: Odometry):
#         self.x1 = np.array([msg.pose.pose.position.x,
#                             msg.pose.pose.position.y,
#                             quaternion_to_yaw(msg.pose.pose.orientation)])
#         self.pose1_ready = True

#     def cb_robot2_odom(self, msg: Odometry):
#         self.x2 = np.array([msg.pose.pose.position.x,
#                             msg.pose.pose.position.y,
#                             quaternion_to_yaw(msg.pose.pose.orientation)])
#         self.pose2_ready = True

#     def cb_goal_pose(self, msg: Pose2D):
#         self.goal_pose    = np.array([msg.x, msg.y, msg.theta])
#         self.goal_updated = True
#         self.get_logger().info(f"New goal: {self.goal_pose}")

#     def cb_obstacles(self, msg: String):
#         try:
#             lst = json.loads(msg.data)
#             self.obstacles = [(o[0], o[1], o[2]) for o in lst]
#             self.get_logger().info(f"Obstacles updated: {self.obstacles}")
#         except Exception as e:
#             self.get_logger().error(f"Obstacle parse error: {e}")

#     # — Control loop —
#     def control_loop(self):
#         # If goal changed, rebuild solver
#         if self.goal_updated:
#             self.solver = self.setup_acados()
#             zero6 = np.zeros(6)
#             self.solver.set(0, "lbx", zero6)
#             self.solver.set(0, "ubx", zero6)
#             self.solver.solve()
#             self.goal_updated = False

#         # Wait for both odom msgs
#         if not (self.pose1_ready and self.pose2_ready):
#             return

#         # Pin current state
#         x0 = np.hstack((self.x1, self.x2))
#         self.solver.set(0, "lbx", x0)
#         self.solver.set(0, "ubx", x0)

#         # Solve OCP
#         status = self.solver.solve()
#         if status != 0:
#             self.get_logger().warn(f"QP failed (status={status}), reusing last u")
#             u0 = self.last_u
#         else:
#             u0 = self.solver.get(0, "u")[:4]  # only the real controls
#             self.last_u = u0

#         # Publish TB3_1
#         t1 = Twist(); t1.linear.x = float(u0[0]); t1.angular.z = float(u0[1])
#         self.pub1.publish(t1)
#         # Publish TB3_2
#         t2 = Twist(); t2.linear.x = float(u0[2]); t2.angular.z = float(u0[3])
#         self.pub2.publish(t2)

#     def shutdown_handler(self, signum, frame):
#         """Stop robots and shutdown ROS cleanly."""
#         stop = Twist()
#         self.pub1.publish(stop)
#         self.pub2.publish(stop)
#         self.get_logger().info("Shutdown: robots stopped.")
#         rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = DualRobotAcadosSoft()
#     rclpy.spin(node)
#     node.destroy_node()

# if __name__ == "__main__":
#     main()






#!/usr/bin/env python3
"""
dual_robot_acados_soft_distance.py

ROS 2 node implementing MPC for two TurtleBots via ACADOS,
with both **soft obstacle** and **soft distance** constraints,
and explicit weight matrices Q, R, S, Ws.

Features:
  - States: x₁,y₁,θ₁, x₂,y₂,θ₂
  - Controls: v₁,ω₁, v₂,ω₂ plus slack variables for obstacles & distance
  - Soft obstacle avoidance: (‖pᵢ−p_obs‖² ≥ (r+margin)² − s_obs), s_obs ≥ 0
  - Soft inter‐robot distance: (min_d² ≤ ‖p₁−p₂‖² ≤ max_d²) ± s_dist ≥ 0
  - Stage cost: eᵀQe + uᵀRu + sᵀWₛs
  - Terminal cost: eᵀS e
  - SQP‐RTI + HPIPM with Levenberg–Marquardt and relaxed QP tolerances
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import casadi as ca
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import json, math, signal

def quaternion_to_yaw(q):
    """Convert ROS quaternion to yaw (radians)."""
    siny = 2.0*(q.w*q.z + q.x*q.y)
    cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(siny, cosy)

class DualRobotAcadosSoftDistance(Node):
    def __init__(self):
        super().__init__('dual_robot_acados_soft_distance')
        self.get_logger().info("Dual‐Robot ACADOS MPC (soft obs + soft dist)...")

        # ─── Default goal & obstacles ────────────────────────────
        self.goal_pose = np.array([15.0, -15.0, 0.0])  # x, y, theta
        # Each obstacle: (x, y, radius)
        self.obstacles = [(0,0,0.6), (3,-3.5,2.5), (3.2,-5.5,5.0), (9.3,-9.3,0.5), (9,-10,0.5)]
        self.goal_updated = False

        # ─── Distance & slack parameters ────────────────────────
        self.declare_parameter("min_distance",   0.5)
        self.declare_parameter("max_distance",   0.8)
        self.declare_parameter("margin",         0.1)
        self.declare_parameter("slack_weight", 1e4)

        self.min_d   = self.get_parameter("min_distance").value
        self.max_d   = self.get_parameter("max_distance").value
        self.margin  = self.get_parameter("margin").value
        self.slack_w = self.get_parameter("slack_weight").value

        # ─── Control limits ────────────────────────────────────
        self.declare_parameter("max_velocity",         0.22)
        self.declare_parameter("max_angular_velocity", 0.50)
        self.max_v = self.get_parameter("max_velocity").value
        self.max_w = self.get_parameter("max_angular_velocity").value

        # ─── Robot state placeholders ──────────────────────────
        self.x1 = np.zeros(3)
        self.x2 = np.zeros(3)
        self.pose1_ready = self.pose2_ready = False

        # ─── ROS I/O ────────────────────────────────────────────
        self.create_subscription(Odometry, '/TB3_1/odom',
                                 self.cb_robot1_odom, 10)
        self.create_subscription(Odometry, '/TB3_2/odom',
                                 self.cb_robot2_odom, 10)
        self.create_subscription(Pose2D,   '/goal_pose',
                                 self.cb_goal_pose,   10)
        self.create_subscription(String,   '/obstacles',
                                 self.cb_obstacles,   10)
        self.pub1 = self.create_publisher(Twist, '/TB3_1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)

        # ─── Build & warm‐up solver ────────────────────────────
        self.solver = self.setup_acados()
        zero6 = np.zeros(6)
        # pin x0 for codegen
        self.solver.set(0, "lbx", zero6)
        self.solver.set(0, "ubx", zero6)
        self.solver.solve()
        self.last_u = np.zeros(4)

        # ─── Control loop @10Hz & clean shutdown ───────────────
        self.create_timer(0.1, self.control_loop)
        signal.signal(signal.SIGINT, self.shutdown_handler)

    def setup_acados(self):
        """
        Build the ACADOS OCP solver for two TurtleBots with:
          - Soft obstacle‐avoidance constraints (one slack per robot per obstacle)
          - Soft inter‐robot distance constraints (two slacks: min & max)
          - Weight matrices Q, R, S, and W_s for slack penalties
          - SQP‐RTI + HPIPM with Levenberg–Marquardt damping and relaxed tolerances
        """
        import numpy as np
        import casadi as ca
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

        # 1) Unpack ROS parameters
        # ------------------------------------------------------------
        gx, gy, gth = self.goal_pose          # desired goal [x, y, θ]
        obs_list    = self.obstacles          # list of (ox, oy, radius)
        min_d       = self.min_d              # minimum allowed distance
        max_d       = self.max_d              # maximum allowed distance
        margin      = self.margin             # obstacle safety margin
        slack_w     = self.slack_w            # penalty weight for all slacks

        # 2) Determine number of slack variables
        # ------------------------------------------------------------
        M = len(obs_list)                     # number of obstacles
        n_slack_obs  = 2 * M                  # two slacks per obstacle (robot1 + robot2)
        n_slack_dist = 2                      # slack for distance min and max
        n_slack      = n_slack_obs + n_slack_dist

        # 3) Define symbolic states and controls
        # ------------------------------------------------------------
        # States: [x1, y1, θ1,  x2, y2, θ2]
        x = ca.SX.sym('x', 6)
        # Controls: [v1, ω1,  v2, ω2,  s_obs1, s_obs2, ..., s_dist_min, s_dist_max]
        u = ca.SX.sym('u', 4 + n_slack)

        # 4) Continuous‐time dynamics (uncoupled unicycles)
        # ------------------------------------------------------------
        x1, y1, th1, x2, y2, th2 = x[0], x[1], x[2], x[3], x[4], x[5]
        v1, w1, v2, w2           = u[0], u[1], u[2], u[3]
        xdot = ca.vertcat(
            v1 * ca.cos(th1),
            v1 * ca.sin(th1),
            w1,
            v2 * ca.cos(th2),
            v2 * ca.sin(th2),
            w2
        )

        # 5) Wrap into an ACADOS model
        # ------------------------------------------------------------
        model = AcadosModel()
        model.name        = 'dual_tb3_with_soft_constraints'
        model.x           = x
        model.u           = u
        model.f_expl_expr = xdot

        # 6) Create OCP and attach model
        # ------------------------------------------------------------
        ocp = AcadosOcp()
        ocp.model = model

        # 7) Solver options and horizon
        # ------------------------------------------------------------
        N, Ts = 30, 0.1
        ocp.dims.N                          = N
        ocp.solver_options.tf               = N * Ts
        ocp.solver_options.qp_solver        = 'FULL_CONDENSING_HPIPM'
        ocp.solver_options.integrator_type  = 'ERK'
        ocp.solver_options.hessian_approx   = 'GAUSS_NEWTON'
        ocp.solver_options.nlp_solver_type  = 'SQP_RTI'
        # Levenberg–Marquardt damping + relaxed tolerances
        ocp.solver_options.levenberg_marquardt = 1e-6
        ocp.solver_options.qp_solver_tol_eq    = 1e-4
        ocp.solver_options.qp_solver_tol_ineq  = 1e-4
        ocp.solver_options.qp_solver_iter_max  = 30

        # 8) Define weight matrices Q, R, S, W_s
        # ------------------------------------------------------------
        # Stage cost on midpoint error
        Q = np.diag([10.0, 10.0, 1.1])
        # Control cost on [v1, ω1, v2, ω2]
        R = np.diag([1, 1, 1, 1])
        # Terminal cost on midpoint error
        S = np.diag([1.0, 1.0, 0.1])
        # Slack penalty
        W_s = slack_w * np.eye(n_slack)

        # 9) Stage and terminal cost expressions
        # ------------------------------------------------------------
        # Midpoint state and orientation error
        x_c  = (x1 + x2) / 2
        y_c  = (y1 + y2) / 2
        th_c = ca.atan2(ca.sin((th1 + th2)/2 - gth),
                       ca.cos((th1 + th2)/2 - gth))
        e    = ca.vertcat(x_c - gx, y_c - gy, th_c)        # 3×1 error

        # Build y = [e; v1; ω1; v2; ω2; slacks]
        y   = ca.vertcat(e, u[0:4], u[4:4 + n_slack])
        y_e = e                                           # terminal error

        ocp.model.cost_y_expr   = y
        ocp.model.cost_y_expr_e = y_e
        ocp.cost.cost_type      = 'NONLINEAR_LS'
        ocp.cost.cost_type_e    = 'NONLINEAR_LS'
        ocp.dims.ny   = y.size1()
        ocp.dims.ny_e = y_e.size1()

        # Block‐diagonal full weight matrix W = diag(Q, R, W_s)
        W_full = np.block([
            [Q,                np.zeros((3, 4)),          np.zeros((3, n_slack))],
            [np.zeros((4, 3)), R,                         np.zeros((4, n_slack))],
            [np.zeros((n_slack, 3)), np.zeros((n_slack, 4)), W_s            ]
        ])
        ocp.cost.W    = W_full
        ocp.cost.W_e  = S
        ocp.cost.yref   = np.zeros(ocp.dims.ny)
        ocp.cost.yref_e = np.zeros(ocp.dims.ny_e)

        # 10) Input bounds (controls + slacks)
        # ------------------------------------------------------------
        idxbu = np.arange(4 + n_slack)
        lbu   = np.hstack(([-self.max_v, -self.max_w,
                             -self.max_v, -self.max_w],
                           np.zeros(n_slack)))         # slacks ≥ 0
        ubu   = np.hstack(([ self.max_v,  self.max_w,
                              self.max_v,  self.max_w],
                           1e3*np.ones(n_slack)))      # slack upper bound
        ocp.constraints.idxbu = idxbu
        ocp.constraints.lbu   = lbu
        ocp.constraints.ubu   = ubu

        # 11) Pin initial state at stage 0
        # ------------------------------------------------------------
        ocp.constraints.idxbx_0 = np.arange(6)
        ocp.constraints.lbx_0   = np.zeros(6)
        ocp.constraints.ubx_0   = np.zeros(6)

        # 12) Build soft‐constraint list h(x,u)
        # ------------------------------------------------------------
        h_list = []

        # 12a) Soft distance constraints
        #    - s_min at u[4 + 2*M]
        #    - s_max at u[4 + 2*M + 1]
        s_min = u[4 + n_slack_obs]
        s_max = u[4 + n_slack_obs + 1]
        d2    = (x1 - x2)**2 + (y1 - y2)**2
        # d² - min_d² + s_min ≥ 0
        h_list.append(d2 - min_d**2 + s_min)
        # max_d² - d² + s_max ≥ 0
        h_list.append(max_d**2 - d2 + s_max)

        # 12b) Soft obstacle constraints
        for i, (ox, oy, orad) in enumerate(obs_list):
            safe = (orad + margin)**2
            s1 = u[4 + 2*i]       # slack for robot1 vs obstacle i
            s2 = u[4 + 2*i + 1]   # slack for robot2 vs obstacle i
            d21 = (x1 - ox)**2 + (y1 - oy)**2
            d22 = (x2 - ox)**2 + (y2 - oy)**2
            h_list.append(d21 - safe + s1)
            h_list.append(d22 - safe + s2)

        # Concatenate & set one‐sided bounds (≥ 0)
        h = ca.vertcat(*h_list)
        ocp.model.con_h_expr = h
        ocp.dims.nh = h.size1()
        INF = 1e6
        ocp.constraints.lh = np.zeros(ocp.dims.nh)
        ocp.constraints.uh = INF * np.ones(ocp.dims.nh)

        # 13) Create and return the ACADOS solver
        # ------------------------------------------------------------
        return AcadosOcpSolver(ocp, json_file='dual_tb3_dist_soft_ocp.json')


    # ─── Callbacks ─────────────────────────────────────────────────
    def cb_robot1_odom(self, msg: Odometry):
        self.x1 = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_to_yaw(msg.pose.pose.orientation)
        ])
        self.pose1_ready = True

    def cb_robot2_odom(self, msg: Odometry):
        self.x2 = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternion_to_yaw(msg.pose.pose.orientation)
        ])
        self.pose2_ready = True

    def cb_goal_pose(self, msg: Pose2D):
        self.goal_pose    = np.array([msg.x, msg.y, msg.theta])
        self.goal_updated = True
        self.get_logger().info(f"New goal: {self.goal_pose}")

    def cb_obstacles(self, msg: String):
        try:
            lst = json.loads(msg.data)
            self.obstacles = [(o[0],o[1],o[2]) for o in lst]
            self.get_logger().info(f"Obstacles: {self.obstacles}")
        except Exception as e:
            self.get_logger().error(f"Obstacle parse error: {e}")

    # ─── Control loop ───────────────────────────────────────────────
    def control_loop(self):
        # If goal changed, rebuild solver
        if self.goal_updated:
            self.solver = self.setup_acados()
            zero6 = np.zeros(6)
            self.solver.set(0, "lbx", zero6)
            self.solver.set(0, "ubx", zero6)
            self.solver.solve()
            self.goal_updated = False

        # Wait for odometry
        if not (self.pose1_ready and self.pose2_ready):
            return

        # Pin current state
        x0 = np.hstack((self.x1, self.x2))
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)

        # Solve
        status = self.solver.solve()
        if status != 0:
            self.get_logger().warn(f"QP failed (status={status}), reusing last u")
            u0 = self.last_u
        else:
            sol = self.solver.get(0, "u")
            u0  = sol[0:4]
            self.last_u = u0

        # Publish commands
        t1 = Twist(); t1.linear.x = float(u0[0]); t1.angular.z = float(u0[1])
        self.pub1.publish(t1)
        t2 = Twist(); t2.linear.x = float(u0[2]); t2.angular.z = float(u0[3])
        self.pub2.publish(t2)

    def shutdown_handler(self, signum, frame):
        """Stop robots on shutdown."""
        stop = Twist()
        self.pub1.publish(stop)
        self.pub2.publish(stop)
        self.get_logger().info("Shutdown: robots stopped.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DualRobotAcadosSoftDistance()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()
