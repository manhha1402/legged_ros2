#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time

def main():
    rclpy.init()
    node = rclpy.create_node('cmd_vel_continuous_publisher')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # Usage: python publish_cmd_vel_continuous.py linear_x linear_y angular_z
    # Example: python publish_cmd_vel_continuous.py 0.5 0.0 0.2 1.0
    if len(sys.argv) < 5:
        print("Usage: python publish_cmd_vel_continuous.py linear_x linear_y angular_z distance")
        print("Example: python publish_cmd_vel_continuous.py 0.5 0.0 0.2 1.0")
        sys.exit(1)
    linear_x = float(sys.argv[1])
    linear_y = float(sys.argv[2])
    angular_z = float(sys.argv[3])
    move_time = float(sys.argv[4])

    msg = Twist()
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = angular_z

    # # Calculate movement duration
    # linear_vel = (linear_x ** 2 + linear_y ** 2) ** 0.5
    # if linear_vel > 0:
    #     move_time = abs(distance / linear_vel)
    # elif abs(angular_z) > 0:
    #     # For pure rotation, use angular_z and treat distance as angle (radians)
    #     move_time = abs(distance / angular_z)
    # else:
    #     move_time = 0.0


    period = 0.1  # 10 Hz
    print("Publishing to /cmd_vel at 10 Hz. Press Ctrl+C to stop.")
    try:
        start_time = time.time()
        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed >= move_time:
                break
            pub.publish(msg)
            node.get_logger().info(f'Sent: linear.x={msg.linear.x}, linear.y={msg.linear.y}, angular.z={msg.angular.z}')
            time.sleep(period)
        # After move_time, send stop command once
        stop_msg = Twist()  # all fields zero
        pub.publish(stop_msg)
        node.get_logger().info(f'Sent stop command (all zeros) after {move_time:.2f} seconds')
    except KeyboardInterrupt:
        print("\nStopped publishing.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
