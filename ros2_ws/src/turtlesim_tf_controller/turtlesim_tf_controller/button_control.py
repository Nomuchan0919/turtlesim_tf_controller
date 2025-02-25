import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class ButtonControl(Node):
    def __init__(self):
        super().__init__("button_control")

        #初期設定
        self.goal = None
        self.publisher_ = self.create_publisher(String, '/goal_change', 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_goal)
        self.timer.cancel()

    def joy_callback(self, joy):
        prev_goal = self.goal

        if joy.buttons[0] == 1:  #Aボタンが押されたとき
            self.goal = "goal_a"
        elif joy.buttons[1] == 1:  #Bボタンが押されたとき
            self.goal = "goal_b"
        elif joy.buttons[2] == 1:  #Xボタンが押されたとき
            self.goal = "goal_x"
        elif joy.buttons[3] == 1:  #Yボタンが押されたとき
            self.goal = "goal_y"
        else:
            return

        if prev_goal is None:
            self.timer.reset()

        self.publish_goal()

    def publish_goal(self):
        if self.goal is None:
            return
        goal_msg = String()
        goal_msg.data = self.goal
        self.publisher_.publish(goal_msg)
        self.get_logger().info(f"Published goal change to: {self.goal}")

def main(args=None):
    rclpy.init(args=args)
    node = ButtonControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()