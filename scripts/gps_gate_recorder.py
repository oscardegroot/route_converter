import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv

class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.pose_count = 0
        self.filename = 'saved_poses.csv'

    def pose_callback(self, msg):
        self.pose_count += 1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.pose_count <= 2:  # Save only the first two poses
            with open(self.filename, 'a') as file:
                writer = csv.writer(file)
                writer.writerow([x, y])
            self.get_logger().info(f'Saved pose {self.pose_count}: x={x}, y={y}')

def main(args=None):
    rclpy.init(args=args)
    pose_saver = PoseSaver()
    rclpy.spin(pose_saver)
    pose_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
