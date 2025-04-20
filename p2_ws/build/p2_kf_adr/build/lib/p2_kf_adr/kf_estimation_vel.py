import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
import csv

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2 
from .visualization import Visualizer

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
#from kalman_filter import KalmanFilter_2

class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        # GRAPHS
        self.true_states = []
        self.estimated_states = []
        self.times = []
        self.start_time = self.get_clock().now()

        # TODO: Initialize 6D state and covariance
        initial_state = np.zeros((6,1))
        initial_covariance = np.eye(6) * 0.1

        self.kf = KalmanFilter_2(initial_state, initial_covariance)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_estimate',
            10
        )

    def odom_callback(self, msg):

        # TODO: Extract position, orientation, velocities from msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        theta = euler_from_quaternion(quat)[2]
        v = msg.twist.twist.linear.x
        vx = v*np.cos(theta)
        vy = v*np.sin(theta)
        w = msg.twist.twist.angular.z
        z = np.array([[x],[y],[theta],[vx],[vy],[w]])

        # TODO: Run predict() and update() of KalmanFilter_2
        self.kf.predict()
        mu, sigma = self.kf.update(z)


        # GRAPHS
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        self.times.append(elapsed)

        # Guarda estado real
        self.true_states.append([x, y, theta])

        # Guarda estado estimado
        self.estimated_states.append([float(mu[0]), float(mu[1]), float(mu[2])])


        # TODO: Publish estimated full state
        estimate_msg = PoseWithCovarianceStamped()
        estimate_msg.pose.pose.position.x = float(mu[0])
        estimate_msg.pose.pose.position.y = float(mu[1])
        x,y,z,w = quaternion_from_euler(0, 0, float(mu[2]), 'ryxz')
        estimate_msg.pose.pose.orientation.x = x
        estimate_msg.pose.pose.orientation.y = y
        estimate_msg.pose.pose.orientation.z = z
        estimate_msg.pose.pose.orientation.w = w
        estimate_msg.pose.covariance = sigma.flatten().tolist()
        self.publisher.publish(estimate_msg)

    def save_to_csv(self, filename, times, true_states, estimated_states):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'x_true', 'y_true', 'theta_true', 'x_est', 'y_est', 'theta_est'])
            for t, t_state, e_state in zip(times, true_states, estimated_states):
                writer.writerow([t] + t_state + e_state)

        print("DATA SAVED")


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterPureNode()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    finally:
        node.save_to_csv('kalman_output.csv', node.times, node.true_states, node.estimated_states)

