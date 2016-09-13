from motor_comm import *
motors = motor_comm()
#motors.set_motor_response_node(1)
motors.send_motors_power_level()
print motors.response
