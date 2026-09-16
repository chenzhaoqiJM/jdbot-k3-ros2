# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of the Willow Garage nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Keyboard teleoperation with configurable x, y, and yaw velocity biases."""

import sys
import threading

import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


HELP = """
This node takes keypresses from the keyboard and publishes them as Twist or
TwistStamped messages. When any of x, y, or yaw is non-zero, x_bias, y_bias,
and yaw_bias are added to the three output components. When all three are
zero, their outputs remain zero.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

MOVE_BINDINGS = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def get_key(settings):
    """Read one key and restore the terminal state."""
    if sys.platform == 'win32':
        return msvcrt.getwch()

    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_terminal_settings():
    """Save terminal settings on platforms that use termios."""
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(settings):
    """Restore previously saved terminal settings."""
    if sys.platform != 'win32':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def velocity_status(speed, turn, x_bias, y_bias, yaw_bias):
    """Return a compact description of the current velocity settings."""
    return (
        f'currently:\tspeed {speed:.2f}\tturn {turn:.2f}\t'
        f'bias(x={x_bias:.2f}, y={y_bias:.2f}, yaw={yaw_bias:.2f})'
    )


def apply_bias(x, y, yaw, x_bias, y_bias, yaw_bias):
    """Add all three biases unless all three commanded velocities are zero."""
    if x == 0.0 and y == 0.0 and yaw == 0.0:
        return 0.0, 0.0, 0.0
    return x + x_bias, y + y_bias, yaw + yaw_bias


def main(args=None):
    """Run the keyboard teleoperation node."""
    settings = save_terminal_settings()
    rclpy.init(args=args)
    node = rclpy.create_node('teleop_twist_keyboard_bias')

    read_only = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    stamped = node.declare_parameter('stamped', False, read_only).value
    frame_id = node.declare_parameter('frame_id', '', read_only).value
    speed = node.declare_parameter('speed', 0.5, read_only).value
    turn = node.declare_parameter('turn', 1.0, read_only).value
    x_bias = node.declare_parameter('x_bias', 0.0, read_only).value
    y_bias = node.declare_parameter('y_bias', 0.0, read_only).value
    yaw_bias = node.declare_parameter('yaw_bias', 0.0, read_only).value

    if not stamped and frame_id:
        raise ValueError("'frame_id' can only be set when 'stamped' is true")

    twist_type = (
        geometry_msgs.msg.TwistStamped if stamped else geometry_msgs.msg.Twist
    )
    publisher = node.create_publisher(twist_type, 'cmd_vel', 10)
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    x = y = z = yaw = 0.0
    status = 0
    message = twist_type()
    twist = message.twist if stamped else message

    if stamped:
        message.header.frame_id = frame_id

    try:
        print(HELP)
        print(velocity_status(speed, turn, x_bias, y_bias, yaw_bias))
        while True:
            key = get_key(settings)
            if key in MOVE_BINDINGS:
                x, y, z, yaw = MOVE_BINDINGS[key]
            elif key in SPEED_BINDINGS:
                speed *= SPEED_BINDINGS[key][0]
                turn *= SPEED_BINDINGS[key][1]
                print(velocity_status(speed, turn, x_bias, y_bias, yaw_bias))
                if status == 14:
                    print(HELP)
                status = (status + 1) % 15
            else:
                x = y = z = yaw = 0.0
                if key == '\x03':
                    break

            output_x, output_y, output_yaw = apply_bias(
                x * speed,
                y * speed,
                yaw * turn,
                x_bias,
                y_bias,
                yaw_bias,
            )
            if stamped:
                message.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = output_x
            twist.linear.y = output_y
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = output_yaw
            publisher.publish(message)
    except Exception as error:  # Preserve the behavior of the upstream node.
        print(error)
    finally:
        if stamped:
            message.header.stamp = node.get_clock().now().to_msg()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        publisher.publish(message)
        rclpy.shutdown()
        spinner.join()
        restore_terminal_settings(settings)


if __name__ == '__main__':
    main()
