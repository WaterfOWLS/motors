# system modules
from __future__ import print_function # python 3 print for easy fancy in-place output

# local, user defined modules
import motor_comm # class for motor communication

def main():
    motors = motor_comm()

    #
    last_msg = ''
    while(True):

        # read a line of input from the user
        input = raw_input("Enter command > ")
        toks = input.split(" ") # split the input by spaces

        if(toks[0] == "thrust"):
            if(len(toks) > 2):
                try:
                    thrust_1 = float(toks[1])
                    thrust_2 = float(toks[1])
                    motors.set_thrust(thrust_1, thrust_2)
                    last_msg = "Thrust set to %f %f" % (thrust_1, thrust_2)
                except ValueError:
                    last_msg = "Error parsing Command. Last command: " + last_msg
            # end if len(toks)
        elif(toks[0] == "left"):
            motors.set_thrust(.9, 0)
            last_msg = "Left > Thrust set to %f %f" % (.9, 0)
    
        elif(toks[0] == "right"):
            motors.set_thrust(0, .9)
            last_msg = "Right > Thrust set to %f %f" % (0, .9)
               
        elif(toks[0] == "exit"):
            break
        else:
            last_msg = "Unknown Command. Last command: " + last_msg
        # end if-else ladder

        motors.send_motors_power_level() 
        print(last_msg)
    # end while
# end def main()
    
if __name__ == "__main__":
    main()

# eof
