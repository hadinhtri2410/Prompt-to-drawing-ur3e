import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pkg_resources
import numpy as np
import os

class JointPublisherCSV(Node):

    def __init__(self):
        super().__init__('joint_publisher_csv')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 1/500 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        filename = 'ha149.csv' # change this line of code. The provided CSV file (ldihel.csv) is incomplete
        csv_file = pkg_resources.resource_filename('py_joint_pub', f'../resource/{filename}')
        self.get_logger().info(f"Found CSV file {csv_file}")
        self.csv_data = np.genfromtxt(csv_file, delimiter=',', skip_header=1)
        self.data_length = len(self.csv_data)
        self.get_logger().info(f"Read CSV file {csv_file} \
            with {np.size(self.csv_data,0)} rows and {np.size(self.csv_data,1)} columns")

    def timer_callback(self):
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        msg.position = self.csv_data[self.i, 1:7]
        
        msg.velocity = []
        msg.effort = []
        
        self.publisher_.publish(msg)
        
        self.i += 1
        self.i %= self.data_length # loop back to the beginning of the csv file


def main(args=None):
    rclpy.init(args=args)

    node = JointPublisherCSV()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()