#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from rc_car.msg import MotorPercent
import math

class JoystickConverter():
    def __init__(self):
        rospy.init_node("joystick_to_motors")
        rospy.Subscriber("joy", Joy, self.joy_callback)
        self.motor_pub = rospy.Publisher("motor_percent", MotorPercent, queue_size=10)
        self.joy_configuration = "X" # can be "X" or "D", changes button layout
        self.L_left = 0.0; # Joystick left/right value (left is positive)
        self.L_up = 0.0; # Joystick up/down value (up is positive)
        rospy.spin()

    # Converts the joystick input to the differential drive motor percents
    # Based on a joystick grid value where X is left and right where right is
    # the positive direction and Y is up and down where up is the positive
    # direction
    def joystickToDiff(self, x, y, min_joystick=-1, max_joystick=1, min_speed=-1, max_speed=1):
        # If both are 0, no need to perform the calculbation
        if (x == 0 and y == 0):
            return (0., 0.)

        # Compute hypotenuse
        z = math.sqrt(x*x + y*y)

        # Get angle in radians
        rad = math.acos(math.fabs(x)/z)

        # Convert to degrees
        angle = (rad * 180.)/math.pi

        # The angle indicates the measure of turn
        # If the angle remains constant while the magnitude changes, the turn
        # coefficient remains constant
        # This coefficient applies between angles of 0 -> 90
        # 0: -1
        # 45: 0
        # 90: 1
        t_coeff = -1 + (angle/90.)*2.

        turn = t_coeff*math.fabs(math.fabs(y) - math.fabs(x))
        # Round to integer precentage
        turn = round(turn*100, 0)/100

        # Max of Y or X is the movement
        move = max(math.fabs(y), math.fabs(x))

        # First and third quadrant
        if (x >= 0 and y >=0) or (x < 0 and y < 0):
            raw_left = move
            raw_right = turn
        else:
            raw_right = move
            raw_left = turn

        # Reverse polarity
        if (y < 0):
            raw_left = 0 - raw_left
            raw_right = 0 - raw_right

        # Map the values to the defined joystick and speed ranges
        left_out = self.map(raw_left, min_joystick, max_joystick, min_speed, max_speed)
        right_out = self.map(raw_right, min_joystick, max_joystick, min_speed, max_speed)

        return (left_out, right_out)

    def map(self, x_in, in_min, in_max, out_min, out_max):
        # Check that the value is at least in_min
        if (x_in < in_min):
            x_in = in_min
        # Check that the value is at most in_max
        if (x_in > in_max):
            x_in = in_max

        # Map output
        x_out = (x_in - in_min)*(out_max - out_min)/(in_max - in_min) + out_min
        return x_out

    def joy_callback(self, msg):
        # Determine joytick configuration (X or D, based on number of buttons)
        if (len(msg.axes) == 8):
            self.joy_configuration = "X"
        else:
            self.joy_configuration = "D"

        # TODO: Need to allow this code to handle both controller configurations

        # Print out joystick parameters up/down, left/right
        self.L_left = msg.axes[0]
        self.L_up = msg.axes[1]
        motor_percent = MotorPercent()
        self.left_motor, self.right_motor = self.joystickToDiff(-self.L_left, self.L_up)
        motor_percent.data[0] = self.left_motor
        motor_percent.data[1] = self.right_motor
        self.motor_pub.publish(motor_percent)

        # FIXME: need to figure out a solution that actually works!
        # Sometimes the arduino misses a 0 message and doesn't stop
        # To fix this, publish a few more times when last value was 0
        if (motor_percent.data[0] == 0 and motor_percent.data[1] == 0):
            for i in range(10):
                self.motor_pub.publish(motor_percent)

if __name__ == "__main__":
    try:
        joystick = JoystickConverter()
    except rospy.ROSInterruptException:
        pass
