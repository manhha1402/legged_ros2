#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController
 
class SimJoyTeleop(Node):
    def __init__(self):
        super().__init__('sim_joy_teleop')
 
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
 
        # Scale config
        self.scale_lin_x = 0.5
        self.scale_lin_y = 0.5
        self.scale_ang_z = 0.3
        self.deadzone = 0.05
 
        self.lb_button_idx = 4
        self.last_buttons = [0] * 15
 
        self.current_twist = Twist()
        self.timer = self.create_timer(0.02, self.timer_callback)
 
        self.get_logger().info("Sim Joystick Teleop initialized. Constant publishing at 50Hz enabled.")
 
    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value
 
    def joy_callback(self, msg):
        self.current_twist.linear.x = self.apply_deadzone(msg.axes[1]) * self.scale_lin_x
        self.current_twist.linear.y = self.apply_deadzone(msg.axes[0]) * self.scale_lin_y
        self.current_twist.angular.z = self.apply_deadzone(msg.axes[3]) * self.scale_ang_z
 
        if msg.buttons[self.lb_button_idx] == 1:
            if msg.buttons[0] == 1 and self.last_buttons[0] == 0: # LB + A
                self.switch_controller(['rl_controller_whole'], ['static_controller', 'rl_controller_legs'])
            elif msg.buttons[2] == 1 and self.last_buttons[2] == 0: # LB + X
                self.switch_controller(['rl_controller_legs'], ['static_controller', 'rl_controller_whole'])
            elif msg.buttons[1] == 1 and self.last_buttons[1] == 0: # LB + B
                self.switch_controller(['static_controller'], ['rl_controller_whole', 'rl_controller_legs'])
            elif msg.buttons[3] == 1 and self.last_buttons[3] == 0: # LB + Y
                self.switch_controller([], ['static_controller', 'rl_controller_whole', 'rl_controller_legs'])
 
        self.last_buttons = msg.buttons
 
    def timer_callback(self):
        self.cmd_vel_pub.publish(self.current_twist)
 
    def switch_controller(self, activate, deactivate):
        if not self.switch_client.service_is_ready():
            self.get_logger().warn("Switch controller service not ready")
            return
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = 2  # BEST_EFFORT
        req.activate_asap = True
        self.get_logger().info(f"Switching: Activate {activate}, Deactivate {deactivate}")
        self.switch_client.call_async(req)
 
def main():
    rclpy.init()
    node = SimJoyTeleop()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()