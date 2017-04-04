#!/usr/bin/env python
'''
joystick_controller:
    ros node for controlling the boat's motors using the joystick
'''

from motor_comm import *
import rospy
from sensor_msgs.msg import Joy
from motors.srv import MotorPower2


# define constants for steam controller
spin_axis = 3 # axis that spin of boat will be mapped to
              # corresponds to x-axis on right touchpad
for_back_axis = 2 # axis that forward and backward will be mapped to
                  # corresponds to y-axis on joystick
safety = 5 # corresponds to button/trigger for the safety
safety_threshold = -1 # value safety must be at to trigger actual movement

class JS_obj:

  def __init__(self):
    rospy.init_node('joystick_controller')
    try:
      self.motor_service = rospy.ServiceProxy('motor_power2', MotorPower2)
    except rospy.ServiceException, e:
      print "MotorPower2 service call failed: %s" % e

    self.pow1 = 0;
    self.pow2 = 0;
    self.send_power()

    rospy.Subscriber('joy', Joy, self.joy_callback)

  def clamp(self, max_val, min_val, value):
    return (min(max_val, max(min_val, value)))

  def send_power(self):
    resp = self.motor_service(self.pow1, self.pow2, 2)
    print resp

  def joy_callback(self, data):
    for_back = data.axes[for_back_axis]
    spin = data.axes[spin_axis]

    if data.axes[safety] == safety_threshold:
      self.pow1 = 0.5 * self.clamp(1, -1, for_back + spin)
      self.pow2 = 0.5 * self.clamp(1, -1, for_back - spin)
    else:
      self.pow1 = 0
      self.pow2 = 0

    print self.pow1, self.pow2
    self.send_power()
 
# end def JS_obj



def main():

  js_obj = JS_obj()

  rate = rospy.Rate(20)

  while not rospy.is_shutdown():
    rate.sleep()

# end def main()

if __name__ == "__main__":
  main()
# eof
