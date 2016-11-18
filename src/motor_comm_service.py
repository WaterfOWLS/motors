#!/usr/bin/env python
'''
'''
from motor_comm import *
import rospy
from motors.srv import MotorPower2

'''
MotorHandler - holds stateful information about motors, including power of
               each motor and the remaining duration to spin at that power.
               Also defines callback to set motor power when service request
               is made.
'''
class MotorHandler:

  # default powers to 0 to keep motors stationary
  def __init__(self, motors, powerL=0, powerR=0):
    self.motors = motors
    self.powerL = powerL
    self.powerR = powerR
    self.duration = 0

  # callback to set motor power when ROS service request is made
  def motorHandlerCallback(self, req):
    self.powerL = req.powerL
    self.powerR = req.powerR
    self.duration = req.duration
    # set the thrust to the new values and send it to the motors once
    self.motors.set_thrust(self.powerL, self.powerR)
    if self.motors.send_motors_power_level():
        return 1 # response from motors parsed correctly (currently it is not)
    else: return 0 # response from motors parsed incorrectly  ^ so it will always return 0

  # simple function to stop the motors
  def stop(self):
     self.motors.set_thrust(0, 0)
     self.motors.send_motors_power_level()

# end class 

def main():
  motors = motor_comm() # create a motor communication object to send binary
                        # packets to motor controllers
  motor_handler = MotorHandler(motors) # object to handle motor state and callbacks
   
  rospy.init_node('motor_comm_service') # create a new ros node
  s = rospy.Service('/motor_power2', MotorPower2, 
                    motor_handler.motorHandlerCallback) # start a new service server
  rate = rospy.Rate(20) # use ros to get timing for 20Hz

  while not rospy.is_shutdown(): # infinite loop until shutdown

    # used in inner while loop to log time since previous iteraiton
    old_time = rospy.get_rostime().to_sec()
    new_time = 0

    while motor_handler.duration > 0:

      new_time = rospy.get_rostime().to_sec() # get current time
      # decrement duration by difference between now and time before last sleep
      motor_handler.duration -= (new_time - old_time)

      # send_motors_power level currently doesn't return anything meaningfull
      if motors.send_motors_power_level():
        pass

      # get the time at the bottom of the loop before sleeping
      old_time = new_time
      rate.sleep() # pause to get a 20Hz loop

    # end while time_left > 0

    # after duration of request has elapsed, tell the motors to stop
    motor_handler.stop()
    # pause a bit before going back to top of loop
    rate.sleep()

  # end while not rospy.is_shutdown()

# end def main()

if __name__ == "__main__":
  main()
# eof
