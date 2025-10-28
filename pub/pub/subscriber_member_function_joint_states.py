# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import serial
import json
import numpy
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.positions = {"A1":0,"A2":6075,"A3":4723,"A4":0,"A5":0,"A6":0,"On":1}
        self.last_sent_time=time.time()
        self.send_rate_hz=1.0
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=10)
        self.get_logger().info('Robot arm connected on /dev/ttyACM0')

    def listener_callback(self, msg):
        now = time.time()

        if now - self.last_sent_time < (1.0/self.send_rate_hz):
            return

        self.last_sent_time = now

        joint_names = msg.name
        joint_positions = msg.position
        
        data =dict(zip(joint_names, joint_positions))

        self.positions['A1']=data['rob_joint_1']*2228.23
        self.positions['A2']=6073# data['rob_joint_2']/numpy.pi*12150 (5092)
        self.positions['A3']=4723#data['rob_joint_3']/numpy.pi*9446
        self.positions['A4']= 0 #data['rob_joint_4']/numpy.pi*180
        self.positions['A5']= 0 #data['rob_joint_5']/numpy.pi*180
        self.positions['A6']= 0 #data['rob_joint_6']/numpy.pi*180

        joints = json.dumps(self.positions, separators=(',', ':'))
        try:
            self.ser.write((joints + "\n").encode())
            self.get_logger().info('Joint states: "%s"' % data)
            self.get_logger().info('JSON: "%s"' % joints)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
    
    def destroy_node(self):
        # if self.ser and self.ser.is_open:
        #     self.ser.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()


    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        minimal_subscriber.get_logger().info("Interrupted by user. Shutting down.")
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finally:
        minimal_subscriber.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
