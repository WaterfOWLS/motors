#!/usr/bin/env python
'''
Node that uses manual input to control the RPM of the motors
'''
import rospy
from beagleboneblack.msg import MotorRPM
from geometry_msgs.msg import Twist

class motor_control ():
  def __init__(self):
    self.motor_rpm=MotorRPM()
    self.motor_rpm.rpm0=0.0
    self.motor_rpm.rpm1=0.0
    self.linear_scalar=300
    self.angular_scalar=50
    self.pub = rospy.Publisher('motor_rpm', MotorRPM, queue_size=10)

def set_rpm (data, motors):
    '''
    Function to send rpm levels to motor 
    Angles are increase CCW
    Motor0 is on left. Motor1 is on right
    Based off the idea that velocity is proportional to RPM
    '''
    linear_x = data.linear.x * motors.linear_scalar 
    angular_z = data.angular.z * motors.angular_scalar
    
    if (angular_z == 0 and linear_x == 0):
        motors.rpm_scalar1 = 0
        motors.rpm_scalar0 = 0
    #turn right like tanks
    elif (angular_z < 0 and linear_x == 0):
        motors.rpm_scalar1 = -190
        motors.rpm_scalar0 = 190
        print "angular_z<0 =", angular_z
        print "linear_x==0 =", linear_x
    #go backwards
    elif (angular_z == 0 and linear_x < 0 or angular_z < 0 and linear_x < 0):
        motors.rpm_scalar1 = -190
        motors.rpm_scalar0 = -190
        print "angular_z==0 =", angular_z
        print "linear_x<0 =", linear_x
    #turn right back not like tank
    elif (angular_z > 0 and linear_x < 0):
        motors.rpm_scalar1 = -190
        motors.rpm_scalar0 = -190
        print "angular_z>0 =", angular_z
        print "linear_x<0 =", linear_x
    #turn left like tank
    elif (angular_z > 0 and linear_x == 0):
        motors.rpm_scalar1 = 190
        motors.rpm_scalar0 = -190
        print "angular_z>0 =", angular_z
        print "linear_x==0 =", linear_x
    else:
        motors.rpm_scalar1 = 190
        motors.rpm_scalar0 = 190

    motors.motor_rpm.rpm0 = linear_x - angular_z + motors.rpm_scalar0
    motors.motor_rpm.rpm1 = linear_x + angular_z + motors.rpm_scalar1

def manual_node():
  '''
  Top level function to handle connection of motors with ROS
  '''
  # Create instance of motor_control class
  # Class contains: ROS RPM Message contents to be published derived from input R and Theta
  motors=motor_control()
  # Initialize ROS Node
  rospy.init_node('manual_node')
  rate = rospy.Rate(10)
 
  # spins at rate and puts the motors response on ROS
  while not rospy.is_shutdown():
    motors.pub.publish(motors.motor_rpm)
    # ROS subscriber handlers - Callback functions: set_rpm, check_stop
    rospy.Subscriber("cmd_vel_mux/input/teleop", Twist, set_rpm, motors)
    rate.sleep()

if __name__ == '__main__':
  try: 
    manual_node()
  except rospy.ROSInterruptException:
    pass
