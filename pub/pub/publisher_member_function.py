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
import threading
import json
import time



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.positions = {"A1":0,"A2":6075,"A3":4723,"A4":0,"A5":0,"A6":0,"On":1}
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.run = True
        input_thread = threading.Thread(target=self.input_loop, daemon=True)
        input_thread.start()
        self.get_logger().info("Publisher started! Control the robot arm with this command: ")

    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(self.positions, separators=(',', ':'))
        self.publisher_.publish(msg)
        self.i += 1

    

    def input_loop(self):
        while self.run:
            try:
                cmd = input("Enter joint A1-A6 values or q to quit \n").strip()
                if cmd.lower() == 'q':
                    self.get_logger().info("Exit button pressed.")
                    break

                joint, val = cmd.split()
                val=float(val)

                if joint in self.positions:
                    self.positions[joint]=int(val)
                    self.get_logger().info(f"Updated {joint} to {val}")

                else:
                    self.get_logger().warn(f"Unknown joint name: {joint}")
            
            except ValueError:
                self.get_logger().warn("Usage example: A1 1000 / On 1")
            except EOFError:
                break
    def destroy_node(self):
        self.run= False
        return super().destroy_node()
    

def main(args=None):
    rclpy.init(args=args)
    

    minimal_publisher = MinimalPublisher()

    try:
        
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finally:
        minimal_publisher.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
