import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

from math import *
import numpy as np
import operator


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/iiwa_arm_controller/joint_trajectory', 1)
        self.publisherDebug_ = self.create_publisher(JointState, '/debug/jointGoal', 1)
        self.subscriber = self.create_subscription(JointState,'/joint_states',self.listener_callback,1)
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(10*self.timer_period, self.timer_callback)
        self.fac = 1.0
        self.joint_position = np.array([0.0, radians(-45.0), 0.0, radians(80.0), 0.0, radians(35.0), 0.0] )
        self.joint_goal = np.array([0.0, radians(-45.0), 0.0, radians(80.0), 0.0, radians(35.0), 0.0] )
        self.dq = np.array([0.0, 0.0, 0.0, radians(0.5), 0.0, 0.0, 0.0] )

    def listener_callback(self, msg):

        enumerate_object = enumerate(msg.name)
        sorted_pairs = sorted(enumerate_object, key=operator.itemgetter(1))
        sorted_indices = [index for index, element in sorted_pairs]

        self.joint_position = [msg.position[i] for i in sorted_indices]

    def timer_callback(self):
        if self.joint_position[3] > radians(90.0):
            self.fac = -1.0
        elif self.joint_position[3] < -radians(90.0):
            self.fac = 1.0
        
        self.joint_position = self.joint_position + self.fac*self.dq
        v = (self.fac/self.timer_period)*self.dq[3]

        msg = JointTrajectory()
        msg.joint_names = ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6','joint_a7']

        point1 = JointTrajectoryPoint()
        point1.positions = self.joint_position.tolist()
        point1.velocities = [0.0,0.0,0.0,v,0.0,0.0,0.0]
        point1.time_from_start = Duration(nanosec=int(2*self.timer_period*1e9))

        msg.points = [point1]

        # for i in range(1,2):
        #     point2 = JointTrajectoryPoint()
        #     point2.positions = (self.joint_position + i*self.fac*self.dq).tolist()
        #     # point2.velocities = [0.0,0.0,0.0,v,0.0,0.0,0.0]
        #     point2.time_from_start = Duration(nanosec=int((i+1)*self.timer_period*1e9))
        #     msg.points.append(point2)

        msgdebug = JointState()
        msgdebug.header.stamp = self.get_clock().now().to_msg()
        msgdebug.name = ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6','joint_a7']
        msgdebug.position = self.joint_position.tolist()
        # msgdebug.velocity = [0.0,0.0,0.0,v,0.0,0.0,0.0]
        self.publisher_.publish(msg)
        self.publisherDebug_.publish(msgdebug)


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

