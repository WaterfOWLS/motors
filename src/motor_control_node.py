#!/usr/bin/env python
'''
  ROS node to control the motors
  The motors can only be sent a power percent. This node allows for control
   of RPM as well as looking for fault conditions such as stall and over heat.
'''
import rospy
from pid import PID
from beagleboneblack.msg import MotorPower
from beagleboneblack.msg import MotorResponse
from beagleboneblack.msg import MotorRPM

class motor_control():
  def __init__(self): 
    #pid loop for each motor
    self.pid0=PID(P=0.0001, I=0.00001)
    self.pid1=PID(P=0.0001, I=0.00001)
    
    self.high_power_limit=0.70
    self.low_power_limit=0.15
    self.low_rpm_limit=250
    self.deadband=0.1
      
    self.pub = rospy.Publisher('motor_power', MotorPower, queue_size=10)
    
    self.motor_power=MotorPower()
    self.motor_power.power1=0
    self.motor_power.power2=0
    
  def set_motor_power(self):
    self.motor_power.power1=self.pid0.PID+self.motor_power.power1
    self.motor_power.power2=self.pid1.PID+self.motor_power.power2
    
    if self.pid0.getPoint() > 0 and self.pid0.getPoint() < self.low_rpm_limit:
      self.motor_power.power1=self.low_power_limit
    elif self.pid0.getPoint() < 0 and self.pid0.getPoint() > -self.low_rpm_limit:
      self.motor_power.power1=-self.low_power_limit
    elif self.pid0.getPoint() == 0:
      self.motor_power.power1=0
    elif self.motor_power.power1 > self.high_power_limit:
      self.motor_power.power1=self.high_power_limit
    elif self.motor_power.power1 < self.low_power_limit:
     # if self.motor_power.power1 > -self.low_power_limit+self.deadband and self.motor_power.power1 < self.low_power_limit-self.deadband: 
     #   self.motor_power.power1=0
      if self.motor_power.power1 < 0 and self.motor_power.power1 > -self.low_power_limit:
        self.motor_power.power1 = -self.low_power_limit
      elif self.motor_power.power1 < -self.high_power_limit:
        self.motor_power.power1= -self.high_power_limit
      else:
        self.motor_power.power1=self.low_power_limit

    if self.pid1.getPoint() > 0 and self.pid1.getPoint() < self.low_rpm_limit:
      self.motor_power.power2=self.low_power_limit
    elif self.pid1.getPoint() < 0 and self.pid1.getPoint() > -self.low_rpm_limit:
      self.motor_power.power2=-self.low_power_limit
    elif self.pid1.getPoint() == 0:
      self.motor_power.power2=0
    elif self.motor_power.power2 > self.high_power_limit:
      self.motor_power.power2=self.high_power_limit
    elif self.motor_power.power2 < self.low_power_limit:
      if self.motor_power.power2 < 0 and self.motor_power.power2 > -self.low_power_limit:
        self.motor_power.power2 = -self.low_power_limit
      elif self.motor_power.power2 < -self.high_power_limit:
        self.motor_power.power2= -self.high_power_limit
      else:
        self.motor_power.power2=self.low_power_limit
    
def set_rpm(data,control):  
  control.pid0.setPoint(data.rpm0)
  control.pid1.setPoint(data.rpm1)

def get_data(data,control):
  '''
  Put motor response data into control object
  '''
  if data.motor_id == 0:
    control.data0=data
  elif data.motor_id==1:
    control.data1=data
  else:
    control.error=data
  
def motor_control_node():
  '''
  Top level function to handle control of motors with ROS
  Takes in MotorRPM
  Outputs MotorPower
  '''
  control=motor_control()
  rospy.init_node('motor_control')
  rate = rospy.Rate(2)
  
  while not rospy.is_shutdown(): 
    #incase the motors haven't been turned on yet
    try:
      control.pid0.update(control.data0.rpm)
      control.pid1.update(control.data1.rpm)
    except AttributeError:
      control.pid0.update(0)
      control.pid1.update(0)
    control.set_motor_power()      
    control.pub.publish(control.motor_power)
    
    rospy.Subscriber("motor_rpm", MotorRPM,set_rpm,control)
    rospy.Subscriber("motor_data", MotorResponse,get_data,control)
    rate.sleep()  

if __name__ == '__main__':
  try: 
    motor_control_node()
  except rospy.ROSInterruptException:
    pass
