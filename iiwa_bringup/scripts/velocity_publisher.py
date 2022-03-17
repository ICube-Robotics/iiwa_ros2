import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

from math import *
import numpy as np
import operator


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 1)
        self.subscriber = self.create_subscription(JointState,'/joint_states',self.listener_callback,1)
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(10*self.timer_period, self.timer_callback)
        self.fac = 1.0
        self.joint_position = np.array([0.0, radians(-45.0), 0.0, radians(80.0), 0.0, radians(35.0), 0.0] )
        self.dq = np.array([0.0, 0.0, 0.0, radians(2), 0.0, 0.0, 0.0] )

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

        msg = Float64MultiArray()
        msg.data = [0.0,0.0,0.0,v,0.0,0.0,0.0]
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

