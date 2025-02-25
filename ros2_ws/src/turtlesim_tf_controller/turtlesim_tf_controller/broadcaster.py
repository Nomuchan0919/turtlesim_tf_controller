import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from turtlesim.msg import Pose
import math
import numpy as np
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String

def quaternion_from_eular(ai, aj, ak): #クォータニオンを計算する
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4, ))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss - sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc - sj * ss
    
    return q #q = [x, y, z, w]

def calculate_yaw(current_x, current_y, goal_x, goal_y):
    # 目標地点と現在の位置からyawを計算
    delta_x = goal_x - current_x
    delta_y = goal_y - current_y
    yaw = math.atan2(delta_y, delta_x)
    return yaw

def calculate_distance(current_x, current_y, goal_x, goal_y):
    # 現在の位置と目標位置との距離を計算
    delta_x = goal_x - current_x
    delta_y = goal_y - current_y
    return math.sqrt(delta_x ** 2 + delta_y ** 2)

class TurtlesimBroadcaster(Node):
    def __init__(self):
        super().__init__('turtle_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(String, '/goal_change', self.goal_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.current_turtle_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.goal_positions = {
            "goal_x": (2.0, 5.5),
            "goal_a": (5.5, 2.0),
            "goal_b": (9.0, 5.5),
            "goal_y": (5.5, 9.0)
        }
        self.current_goal = "goal_x"
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False

    def goal_callback(self, msg):
        new_goal = msg.data
        if new_goal in self.goal_positions:
            self.current_goal = new_goal
            self.get_logger().info(f"Goal is changed: {self.current_goal}")
            self.publish_goal()

    def current_turtle_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_yaw = msg.theta
        self.current_turtle()

    def publish_goal(self):
        x, y= self.goal_positions[self.current_goal]
        yaw = calculate_yaw(self.current_x, self.current_y, x, y)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "target_pose"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        quat = quaternion_from_eular(0, 0, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

    def current_turtle(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "turtle1"
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.translation.z = 0.0
        quat = quaternion_from_eular(0, 0, self.current_yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()