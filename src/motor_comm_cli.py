#!/usr/bin/python

'''file: motor_comm_cli.py

 This file is a script meant to be run as a simple command line
 interface to the motor_comm class in motor_comm.py. It currently
 accepts 5 commands, described below:

 thrust - This command takes two floating point numbers between 1 and
          0 separated by spaces as arguments, and sets the thrust of
          each motor to this value

 left   - set the thrust of the right
          motor to .9 and the left to .1 to turn the boat left

 right  - Same as left, but well, the opposite

 stop   - sets the thrust of both motors to 0

 exit   - exits the prompt, closing the script

 After each command is entered, a status message is printed, as well
 as the status code returned by the motor controllers.

'''

# system modules
# python 3 print for easy fancy in-place output
from __future__ import print_function 
from threading import Thread
from time import sleep

# local, user defined modules
from motor_comm import motor_comm # class for motor communication

# some global variables
thrust = [0.3, 0.3] # thrust value for motors
motors = None # motor_comm instance

# method to be set up as a thread
# implicitly takes thrust and motors as arguments
def run_motors():
    global thrust
    global motors

    while True:
        motors.set_thrust(thrust[0], thrust[1])
        motors.send_motors_power_level() # send the motor thrust over Uart
        sleep(.1)
        
# end def set_thrust()
    
# define the main function
def main():

    global thrust
    global motors
    # create a motor controller object to communicate with the drivers over
    # uart
    motors = motor_comm()

    # string to hold the last status message
    last_msg = ''

    # create and start the thread for sending commands to the motor
    ping_motors = Thread(target=run_motors)
    ping_motors.daemon = True
    ping_motors.start()
    
    # until the user chooses to exit
    while(True):

        # read a line of input from the user
        input = raw_input("Enter command > ")
        toks = input.split(" ") # split the input by spaces

        # If else ladder on the first (and possibly only) word of the command
        if(toks[0].lower() == "thrust"):

            if(len(toks) > 2): # if two thrust values are given
                try:
                    thrust_1 = float(toks[1])
                    thrust_2 = float(toks[1])
                    thrust = [thrust_1, thrust_2]
                    last_msg = "Thrust set to %f %f" % (thrust_1, thrust_2)

                # ran into an error parsing left and right thrust value
                except ValueError: 
                    last_msg = "Error parsing Command. Last command: " + last_msg

            # end if len(toks)

        elif(toks[0].lower() == "left"): # command is to turn left
            thrust = [.9, 0]
            last_msg = "Left > Thrust set to %f %f" % (.9, 0)
    
        elif(toks[0].lower() == "right"): # command is to turn right
            thrust = [0, .9]
            last_msg = "Right > Thrust set to %f %f" % (0, .9)

        elif(toks[0].lower() == "stop"): # command is to turn stop
            thrust = [0, 0]
            last_msg = "Right > Thrust set to %f %f" % (0, 0)
               
        elif(toks[0].lower() == "exit"): # command is to exit
            break # exit the infinite loop

        else: # display the last command entered
            last_msg = "Unknown Command. Last command: " + last_msg
        # end if-else ladder

        print(last_msg) # print the last input message

    # end while true

# end def main()
    
if __name__ == "__main__":
    main()

# eof
