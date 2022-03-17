import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from math import *


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/iiwa_arm_controller/joint_trajectory', 1)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.fac = 1

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6','joint_a7']
        point = JointTrajectoryPoint()
        p = self.fac*0.2
        v = 1.0
        a = 0.2
        point.positions = [p, 0.0, 0.0, radians(50.0), 0.0, 0.0, 0.0]        
        #point.velocities = [v,v,v,v,v,v,v]
        #point.accelerations = [a,a,a,a,a,a,a]

        point.time_from_start = Duration(sec=1)
        msg.points = [point]
        self.fac = -self.fac

        
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

