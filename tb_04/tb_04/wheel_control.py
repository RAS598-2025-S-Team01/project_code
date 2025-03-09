import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, tty, termios
import time

class TurtleBot4Controller(Node):
    def __init__(self):
        super().__init__('turtlebot4_wheel_control')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/rpi_04/cmd_vel', 10)
        self.wheel_control_publisher = self.create_publisher(Twist, '/rpi_04/wheel_control', 10)
        self.timer = self.create_timer(0.1, self.send_velocity_command)
        self.velocity = Twist()
        self.get_logger().info("TurtleBot4 wheel control node started.")

        # Default speeds
        self.linear_speed = 0.2
        self.angular_speed = 0.5

    def send_velocity_command(self):
        self.cmd_vel_publisher.publish(self.velocity)
        self.wheel_control_publisher.publish(self.velocity)

    def move_forward(self):
        self.velocity.linear.x = self.linear_speed
        self.velocity.angular.z = 0.0

    def move_backward(self):
        self.velocity.linear.x = -self.linear_speed
        self.velocity.angular.z = 0.0

    def turn_left(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = self.angular_speed

    def turn_right(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = -self.angular_speed

    def stop(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0

def getKey(timeout=0.1):
    """
    Non-blocking key reader using termios and select.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBot4Controller()

    print("Control TurtleBot4 with keys:")
    print("  w: move forward")
    print("  s: move backward")
    print("  a: turn left")
    print("  d: turn right")
    print("  x: stop")
    print("  i: increase linear speed")
    print("  k: decrease linear speed")
    print("  j: increase angular speed")
    print("  l: decrease angular speed")
    print("  q: quit")

    try:
        while rclpy.ok():
            key = getKey()
            if key == 'w':
                controller.move_forward()
            elif key == 's':
                controller.move_backward()
            elif key == 'a':
                controller.turn_left()
            elif key == 'd':
                controller.turn_right()
            elif key == 'x':
                controller.stop()
            elif key == 'i':  # Increase linear speed
                controller.linear_speed += 0.1
                print("Increased linear speed:", controller.linear_speed)
            elif key == 'k':  # Decrease linear speed (minimum 0)
                controller.linear_speed = max(0, controller.linear_speed - 0.1)
                print("Decreased linear speed:", controller.linear_speed)
            elif key == 'j':  # Increase angular speed
                controller.angular_speed += 0.1
                print("Increased angular speed:", controller.angular_speed)
            elif key == 'l':  # Decrease angular speed (minimum 0)
                controller.angular_speed = max(0, controller.angular_speed - 0.1)
                print("Decreased angular speed:", controller.angular_speed)
            elif key == 'q':
                break
            else:
                controller.stop()

            rclpy.spin_once(controller, timeout_sec=0.1)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

