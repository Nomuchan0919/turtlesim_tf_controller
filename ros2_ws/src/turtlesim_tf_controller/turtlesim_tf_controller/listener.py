import rclpy
from rclpy.node import Node
import tf2_ros
import math
from geometry_msgs.msg import Twist

class FrameListener(Node):
    def __init__(self):
        super().__init__("turtle_tf2_frame_listener2")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_vel = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        self.tiemr = self.create_timer(1.0, self.update_pose)

    def update_pose(self):
        try:
            if self.tf_buffer.can_transform('turtle1', 'target_pose', rclpy.time.Time()):
                turtle1_to_target_trans = self.tf_buffer.lookup_transform('turtle1', 'target_pose', rclpy.time.Time())

                #現在の位置から目標地点までの差
                dis_x = turtle1_to_target_trans.transform.translation.x
                dis_y = turtle1_to_target_trans.transform.translation.y
        
                angle_to_goal = math.atan2(dis_y, dis_x)
                distance = math.sqrt(dis_x**2 + dis_y**2)
                print(distance)
                print(dis_x)
                print(dis_y)
                msg = Twist()

                if distance < 0.1:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.get_logger().info("Goal reached!")
                else:
                    if abs(angle_to_goal) > 0.05:
                        msg.angular.z = angle_to_goal
                        msg.linear.x = 0.0
                    else:
                        msg.angular.z = 0.0
                        msg.linear.x = 0.5 * distance

                self.cmd_vel.publish(msg)
                self.get_logger().info(f"Publishing cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")
            else:
                self.get_logger().warn("Target pose not yet available in TF.")

        except tf2_ros.LookupException:
            self.get_logger().warn("LookupException: Transform not found. Make sure the TF broadcaster is running.")

        except tf2_ros.ConnectivityException:
            self.get_logger().warn("ConnectivityException: Failed to communicate with TF tree. Check network and TF publisher.")

        except tf2_ros.ExtrapolationException:
            self.get_logger().warn("ExtrapolationException: Transformation data is not available for the requested time.")

def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()