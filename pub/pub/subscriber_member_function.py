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
import serial

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=10)
        self.get_logger().info('Robot arm connected on /dev/ttyACM0')

    def listener_callback(self, msg):
        
        message = msg.data
        self.ser.write((message + "\n").encode())
        self.get_logger().info('Forwarding message: "%s"' % message)
    
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
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
