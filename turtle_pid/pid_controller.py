import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class PidController(Node):
    """Drives turtlesim's turtle to a goal using two PID loops."""

    def __init__(self):
        super().__init__('pid_controller')

        # --- Parameters (changeable without editing code) ---
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)
        self.declare_parameter('kp_linear', 1.0)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.1)
        self.declare_parameter('kp_angular', 4.0)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.2)
        self.declare_parameter('max_linear', 2.0)
        self.declare_parameter('max_angular', 3.0)
        self.declare_parameter('tolerance', 0.1)
        self.declare_parameter('world_min', 0.0)
        self.declare_parameter('world_max', 11.08)
        self.declare_parameter('safety_margin', 0.5)
        self.declare_parameter('turn_threshold', 0.3)   

        # --- Controller state ---
        self.pose = None
        self.dt = 0.05                      # control loop period: 20 Hz
        self.prev_dist_error = None
        self.prev_angle_error = None
        self.int_dist = 0.0
        self.int_angle = 0.0
        self.goal_reached = False

        # --- ROS connections ---
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('PID controller started')

    def param(self, name):
        return self.get_parameter(name).value

    def pose_callback(self, msg):
        """Called every time a new pose arrives."""
        self.pose = msg

    def goal_is_safe(self, gx, gy):
        """True if the goal lies inside the safe area."""
        low = self.param('world_min') + self.param('safety_margin')
        high = self.param('world_max') - self.param('safety_margin')
        return low <= gx <= high and low <= gy <= high

    def heading_into_wall(self):
        """True if the turtle is near a wall and facing toward it."""
        low = self.param('world_min') + self.param('safety_margin')
        high = self.param('world_max') - self.param('safety_margin')
        x, y = self.pose.x, self.pose.y
        dir_x = math.cos(self.pose.theta)   # >0 means facing right
        dir_y = math.sin(self.pose.theta)   # >0 means facing up
        return ((x <= low and dir_x < 0) or (x >= high and dir_x > 0)
                or (y <= low and dir_y < 0) or (y >= high and dir_y > 0))

    def control_loop(self):
        """Called 20 times per second by the timer."""
        if self.pose is None:
            return  # no position received yet

        # 0. Safety: refuse goals outside the safe area
        gx, gy = self.param('goal_x'), self.param('goal_y')
        if not self.goal_is_safe(gx, gy):
            self.get_logger().warn(
                f'Goal ({gx}, {gy}) is outside the safe area, not moving',
                throttle_duration_sec=2.0)
            self.cmd_pub.publish(Twist())  # stop
            return

        # 1. Errors
        dx = gx - self.pose.x
        dy = gy - self.pose.y
        dist_error = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.pose.theta
        # wrap to [-pi, pi] so the turtle turns the short way
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()

        # 2. Goal reached? Stop.
        if dist_error < self.param('tolerance'):
            if not self.goal_reached:
                self.get_logger().info('Goal reached!')
                self.goal_reached = True
            self.cmd_pub.publish(cmd)  # all zeros = stop
            return
        self.goal_reached = False

        # 3. PID terms
        self.int_dist += dist_error * self.dt
        self.int_angle += angle_error * self.dt

        d_dist = 0.0 if self.prev_dist_error is None else \
            (dist_error - self.prev_dist_error) / self.dt
        d_angle = 0.0 if self.prev_angle_error is None else \
            (angle_error - self.prev_angle_error) / self.dt

        self.prev_dist_error = dist_error
        self.prev_angle_error = angle_error

        linear = (self.param('kp_linear') * dist_error
                  + self.param('ki_linear') * self.int_dist
                  + self.param('kd_linear') * d_dist)
        angular = (self.param('kp_angular') * angle_error
                   + self.param('ki_angular') * self.int_angle
                   + self.param('kd_angular') * d_angle)

        # 4. Turn in place until roughly facing the goal, then drive
        if abs(angle_error) > self.param('turn_threshold'):
            linear = 0.0
        else:
            linear *= math.cos(angle_error)

        # 5. Clamp to safe speeds
        max_lin = self.param('max_linear')
        max_ang = self.param('max_angular')
        cmd.linear.x = max(-max_lin, min(max_lin, linear))
        cmd.angular.z = max(-max_ang, min(max_ang, angular))

        # 6. Safety: never drive forward into a wall
        if self.heading_into_wall() and cmd.linear.x > 0:
            cmd.linear.x = 0.0
            self.get_logger().warn('Too close to wall, blocking forward motion',
                                   throttle_duration_sec=2.0)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PidController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
