#!/usr/bin/env python
from motor_comm import *
import rospy
from motors.srv import MotorPower2
from math import exp

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
    self.prevPowerL = 0.0
    self.prevPowerR = 0.0
    self.duration = 0
    self.reqTime = 0

  # callback to set motor power when ROS service request is made
  def motorHandlerCallback(self, req):
    self.prevPowerL = self.powerL
    self.prevPowerR = self.powerR
    self.powerL = req.powerL
    self.powerR = req.powerR
    self.duration = req.duration
    self.reqTime = rospy.get_rostime().to_sec()

    return 1

    '''
    # set the thrust to the new values and send it to the motors once
    self.motors.set_thrust(self.powerL, self.powerR)
    if self.motors.send_motors_power_level():
        return 1 # response from motors parsed correctly (currently it is not)
    else: return 0 # response from motors parsed incorrectly  ^ so it will always return 0
    '''

  # simple function to stop the motors
  def stop(self):
    self.prevPowerL = self.powerL
    self.prevPowerR = self.powerR
    self.powerL = 0
    self.powerR = 0
    self.duration = 1
    self.reqTime = rospy.get_rostime().to_sec()

# end class 

def main():

  tau = 3.5 # define time constant 
  motors = motor_comm() # create a motor communication object to send binary
                        # packets to motor controllers
  motor_handler = MotorHandler(motors) # object to handle motor state and callbacks
   
  rospy.init_node('motor_comm_service') # create a new ros node
  s = rospy.Service('/motor_power2', MotorPower2, 
                    motor_handler.motorHandlerCallback) # start a new service server
  rate = rospy.Rate(20) # use ros to get timing for 20Hz

  while not rospy.is_shutdown(): # infinite loop until shutdown

    # used in inner while loop to log time since previous iteraiton
    time_left = 100

    while time_left > 0:

      t = rospy.get_rostime().to_sec() # get current time
      t_diff = t - motor_handler.reqTime
      # decrement duration by difference between now and time before last sleep
      time_left = motor_handler.duration - t_diff

      # get the difference between the new and old powers
      powerLDiff = motor_handler.powerL - motor_handler.prevPowerL
      powerRDiff = motor_handler.powerR - motor_handler.prevPowerR

      newPowerL = motor_handler.powerL - powerLDiff * exp(-t_diff / tau)
      newPowerR = motor_handler.powerR - powerRDiff * exp(-t_diff / tau)
      threshold = 0.02
      if abs(newPowerL) < threshold:
        newPowerL = 0.0
      if abs(newPowerR) < threshold:
        newPowerR = 0.0

      motors.set_thrust(newPowerL, newPowerR)

      # send_motors_power level currently doesn't return anything meaningfull
      if motors.send_motors_power_level():
        pass

     
      # get the time at the bottom of the loop before sleeping
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
