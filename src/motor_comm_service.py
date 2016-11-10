#!/usr/bin/env python
'''
'''
from motor_comm import *
import rospy
from motors.srv import MotorPower2

class MotorHandler:

  def __init__(self, motors, powerL=0, powerR=0):
    self.motors = motors
    self.powerL = powerL
    self.powerR = powerR
    self.duration = 0

  def motorHandlerCallback(self, req):
    self.powerL = req.powerL
    self.powerR = req.powerR
    self.duration = req.duration

    self.motors.set_thrust(self.powerL, self.powerR)
    if self.motors.send_motors_power_level():
        return 1
    else: return 0

  def stop(self):
     self.motors.set_thrust(0, 0)
     self.motors.send_motors_power_level()

# end class 

def main():
  motors = motor_comm()
  motor_handler = MotorHandler(motors)

  rospy.init_node('motor_comm_service')
  s = rospy.Service('motor_power2', MotorPower2, 
                    motor_handler.motorHandlerCallback)
  rate = rospy.Rate(20)

  while not rospy.is_shutdown():

    old_time = rospy.get_rostime().to_sec()
    new_time = 0

    while motor_handler.duration > 0:

      new_time = rospy.get_rostime().to_sec()
      motor_handler.duration -= (new_time - old_time)

      if motors.send_motors_power_level():
        pass

      old_time = new_time
      rate.sleep()

    # end while time_left > 0

    motor_handler.stop()
    rate.sleep()

  # end while not rospy.is_shutdown()

# end def main()

if __name__ == "__main__":
  main()
# eof
