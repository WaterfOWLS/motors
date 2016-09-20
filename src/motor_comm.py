import binascii
import serial
import struct
import sys


class motor_comm:
    def __init__(self):

        # configuration constants
        self.portname = "/dev/ttyAMA0" # serial port name
        self.baudrate = 115200 # bitrate for comm channel
        self.port_timeout = 1 # timeout time in seconds for serial port

        # VRCSR protocol defines  
        self.SYNC_REQUEST  =  0x5FF5
        self.SYNC_RESPONSE =  0x0FF0
        self.PROTOCOL_VRCSR_HEADER_SIZE = 6
        self.PROTOCOL_VRCSR_XSUM_SIZE   = 4
        
        # CSR Address for sending an application specific custom command
        self.ADDR_CUSTOM_COMMAND  = 0xF0

        # The command to send.
        # The Propulsion command has a payload format of:
        # 0xAA R_ID THRUST_0 THRUST_1 THRUST_2 ... THRUST_N 
        # Where:
        #   0xAA is the command byte
        #   R_ID is the NODE ID of the thruster to respond with data
        #   THRUST_X is the thruster power value (-1 to 1) for the thruster with motor id X
        self.PROPULSION_COMMAND   = 0xAA

        #flag for the standard thruster response which contains 
        self.RESPONSE_THRUSTER_STANDARD = 0x2
        #standard response is the device type followed by 4 32-bit floats and 1 byte
        self.RESPONSE_THRUSTER_STANDARD_LENGTH = 1 + 4 * 4 + 1 

        #The propulsion command packets are typically sent as a multicast to a group ID defined for thrusters
        self.THRUSTER_GROUP_ID    = 0x81
        
        #default to 0 thrust for motor 
        self.thrust = [0,0]
        
        #default to 0 motor node for responses
        self.motor_response_node = 0
       
        #deault for send_motor_command
        self.send_motor_command = False
  
        #open the serial port
        try:        
            self.port = serial.Serial(port = self.portname,
                                      baudrate = self.baudrate)
            self.port.timeout = self.port_timeout
        except IOError:
            print ("Error:  Could not open serial port: " + self.portname)     
            sys.exit()
        
    # Function to set the thrust of the motors. If nothing is sent, the current
    # thrust values are used.
    def set_thrust(self,thrust_1='a',thrust_2='a'):

        #if a thrust number isn't sent then set thrust to what it was
        if (thrust_1 == 'a'):
            thrust_1=self.thrust[0]

        if (thrust_2 == 'a'):
            thrust_2=self.thrust[1]

        # TODO: Examine and fix this? seems arbitrary
        # motors take thrust levels between 1 and 0
        # scale inputs to be less than 1
        while (thrust_1 > 1):
            thrust_1=thrust_1/10
        while (thrust_2 > 1):
            thrust_2=thrust_2/10
            
        self.thrust[0]=thrust_1
        self.thrust[1]=thrust_2
        
    # Set motor response node
    # TODO: more descriptive . . . description
    def set_motor_response_node(self,motor_node):
        self.motor_response_node = motor_node
    
    
    def send_motors_power_level(self):
        '''
        Sends communication to motors
        Returns response list from the motor set in self.motor_node
        Contents of response list
            [0] = sync (hex)
            [1]= response node id (int)
            [2] = flag (hex)
            [3] = CSR address (hex)
            [4] = length (hex)
            [5] = header checksum (hex)
            [6] = device type (hex)
            [7] = rpm (float)
            [8] = bus voltage (float)
            [9] = bus current (float)
            [10] = temperature (float C)
            [11] = fault (hex)
            [12] = payload checksum (hex)
        '''
        #Check to see if method is in use by another program/thread
        if self.send_motor_command:
            return False
        else:
            self.send_motor_command = True

            #Create the custom command packet for setting the power level to a group of thrusters
            #generate the header
        flag = self.RESPONSE_THRUSTER_STANDARD
        CSR_address = self.ADDR_CUSTOM_COMMAND
        length = 2 + len(self.thrust) * 4
        command_node=self.THRUSTER_GROUP_ID
        header = bytearray(struct.pack('HBBBB',self.SYNC_REQUEST,int(command_node),flag,CSR_address,length))
        header_checksum = bytearray(struct.pack('i', binascii.crc32(header))) 

        #generate the payload, limiting the thrust to reasonable values
        payload = bytearray(struct.pack('BB', self.PROPULSION_COMMAND, int(self.motor_response_node)))
        for t in self.thrust:
            t = max(t,-1)
            t = min(t, 1)
            payload += bytearray(struct.pack('f',t))

        payload_checksum = bytearray(struct.pack('i', binascii.crc32(payload)))    

        #send the packet and wait for a response
        packet = header + header_checksum + payload + payload_checksum 

        #put the packet on the wire
        self.port.write(bytes(packet))

        #get the response
        expected_response_length = self.PROTOCOL_VRCSR_HEADER_SIZE + self.PROTOCOL_VRCSR_XSUM_SIZE +  self.RESPONSE_THRUSTER_STANDARD_LENGTH +  self.PROTOCOL_VRCSR_XSUM_SIZE

        #read in lines from sent message 
        response_buf = self.port.read(len(bytes(packet)))

        #read in received lines sent from motor
        response_buf = self.port.read(expected_response_length)
        print ("Got response. Length: %d" % len(response_buf))

        #parse the response. If no response all zeros in data
        try:
            self.response = struct.unpack('=HBBBB I BffffB I', response_buf)
        except struct.error:
            self.response=[] 
            self.response.append(0)
            self.response.append(self.motor_response_node)
            for x in range(2,13):
                self.response.append(0)

        self.send_motor_command=False
        return True

    def toggle_node_id(self):
        '''
        Toggles node id between 0 and 1
        '''
        if (self.motor_response_node==1):
            node_id = 0
        else:
            node_id = 1
            
        self.set_motor_response_node(node_id)



