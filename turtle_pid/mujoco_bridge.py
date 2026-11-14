import math
import os

import mujoco
import mujoco.viewer
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose


class MujocoBridge(Node):
    """Simulates a planar robot in MuJoCo, using turtlesim's topic names."""

    def __init__(self):
        super().__init__('mujoco_bridge')

        default_model = os.path.join(
            get_package_share_directory('turtle_pid'),
            'mujoco_models', 'robot.xml')
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('show_viewer', True)
        self.declare_parameter('kv_linear', 10.0)    # speed controller gains
        self.declare_parameter('kv_angular', 5.0)
        self.declare_parameter('k_lateral', 20.0)    # stops sideways sliding
        self.declare_parameter('cmd_timeout', 0.5)   # seconds
        self.declare_parameter('start_x', 5.54)
        self.declare_parameter('start_y', 5.54)

        self.model = mujoco.MjModel.from_xml_path(
            self.get_parameter('model_path').value)
        self.data = mujoco.MjData(self.model)
        self.data.qpos[0] = self.get_parameter('start_x').value
        self.data.qpos[1] = self.get_parameter('start_y').value
        mujoco.mj_forward(self.model, self.data)

        self.cmd_lin = 0.0
        self.cmd_ang = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(
            Twist, '/turtle1/cmd_vel', self.cmd_callback, 10)
        self.pose_pub = self.create_publisher(Pose, '/turtle1/pose', 10)

        self.viewer = None
        if self.get_parameter('show_viewer').value:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.lookat[:] = [5.54, 5.54, 0.0]   
            self.viewer.cam.distance = 18.0                  
            self.viewer.cam.azimuth = 90.0
            self.viewer.cam.elevation = -90.0               

        # 10 ms timer; each tick runs several physics steps (real time)
        self.period = 0.01
        self.substeps = round(self.period / self.model.opt.timestep)
        self.timer = self.create_timer(self.period, self.step)
        self.get_logger().info('MuJoCo bridge started')

    def cmd_callback(self, msg):
        self.cmd_lin = msg.linear.x
        self.cmd_ang = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def apply_control(self, lin_cmd, ang_cmd):
        """Turn wanted speeds into motor forces (a simple speed controller)."""
        vx, vy, w = self.data.qvel[0], self.data.qvel[1], self.data.qvel[2]
        theta = self.data.qpos[2]
        c, s = math.cos(theta), math.sin(theta)

        v_fwd = vx * c + vy * s          # speed along the nose
        v_lat = -vx * s + vy * c         # sideways speed (should be zero)

        f_fwd = self.get_parameter('kv_linear').value * (lin_cmd - v_fwd)
        f_lat = -self.get_parameter('k_lateral').value * v_lat
        torque = self.get_parameter('kv_angular').value * (ang_cmd - w)

        # forward/sideways forces -> world x/y forces
        self.data.ctrl[0] = f_fwd * c - f_lat * s
        self.data.ctrl[1] = f_fwd * s + f_lat * c
        self.data.ctrl[2] = torque

    def step(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.get_parameter('cmd_timeout').value:
            lin_cmd, ang_cmd = 0.0, 0.0   # no recent command: stop
        else:
            lin_cmd, ang_cmd = self.cmd_lin, self.cmd_ang

        for _ in range(self.substeps):
            self.apply_control(lin_cmd, ang_cmd)
            mujoco.mj_step(self.model, self.data)

        self.publish_pose()
        if self.viewer is not None:
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.get_logger().info('Viewer closed, shutting down')
                rclpy.shutdown()

    def publish_pose(self):
        theta = self.data.qpos[2]
        c, s = math.cos(theta), math.sin(theta)
        msg = Pose()
        msg.x = float(self.data.qpos[0])
        msg.y = float(self.data.qpos[1])
        msg.theta = math.atan2(s, c)     # wrap to [-pi, pi]
        msg.linear_velocity = float(
            self.data.qvel[0] * c + self.data.qvel[1] * s)
        msg.angular_velocity = float(self.data.qvel[2])
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.viewer is not None:
            node.viewer.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()