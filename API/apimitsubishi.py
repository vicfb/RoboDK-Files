# This is a Python module that allows driving a Mitsubishi robot.
# This Python module can be run directly in console mode to test its functionality.
# This module allows communicating with a robot through the command line.
# The same commands we can input manually are used by RoboDK to drive the robot from the PC.
# RoboDK Drivers are located in /RoboDK/api/Robot/ by default. Drivers can be PY files or EXE files.
#
# Drivers are modular. They are not part of the RoboDK executable but they must be placed in C:/RoboDK/api/robot/, then, linked in the Connection parameters menu:
#   1. right click a robot in RoboDK, then, select "Connect to robot".
#   2. In the "More options" menu it is possible to update the location and name of the driver.
# Driver linking is automatic for currently available drivers.
#
# Alternatively to the standard programming methods (where a program is generated, then, transferred to the robot and executed) it is possible to run a program simulation directly on the robot
# The robot movement in the simulator is then synchronized with the real robot.
# Programs generated from RoboDK can be run on the robot by right clicking the program, then selecting "Run on robot".
#   Example:
#   https://www.youtube.com/watch?v=pCD--kokh4s
#
# Example of an online programming project:
#   https://robodk.com/blog/online-programming/
#
# It is possible to control the movement of a robot from the RoboDK API (for example, from a Python or C# program using the RoboDK API).
# The same code is used to simulate and optionally move the real robot.
#   Example:
#   https://robodk.com/offline-programming
#
#   To establish connection from RoboDK API:
#   https://robodk.com/doc/en/PythonAPI/robolink.html#robolink.Item.ConnectSafe
#
# Example of a quick manual test in console mode:
#  User entry: CONNECT 192.168.123.1
#  Response:   SMS:Response from the robot or failure to connect
#  Response:   SMS:Ready 
#  User entry: MOVJ 10 20 30 40 50 60
#  Response:   SMS:Working...
#  Response:   SMS:Ready
#  User entry: CJNT
#  Response:   SMS:Working...
#  Response:   JNTS: 10 20 30 40 50 60
#
#---------------------------------------------------------------------------------

import sys
import time
import socket
import threading
import queue
import serial

#----------- communication class for the Mitsubishi robot -------------
# This class handles communication between this driver (PC) and the Mitsubishi robot
class RobotCom:
    """Robot class for programming Mitsubishi robots"""
    LAST_MSG = ""       # Keep a copy of the last message received
    CONNECTED = False   # Connection status is known at all times
    
    # This is executed when the object is created
    def __init__(self):
        self.BUFFER_SIZE = 512 # bytes
        self.TIMEOUT = 60 # seconds # No robot movement should take more than 60 seconds
        #self.TIMEOUT = 10 # seconds
        self.sock = None
        self.sockjnts = None       
        
    # Disconnect from robot
    def disconnect(self):
        """self.CONNECTED = False
        try:
            self.sockjnts.close()
        except:
            pass
        try:
            if self.sock is not None:
                self.Run('1;1;CLOSE')
                self.sock.close()
        except:
            pass       
        return True"""

        #ser = serial.Serial('COM4', 19200, timeout=0,stopbits=2, parity=serial.PARITY_EVEN, rtscts=1)  # open serial port
       
    
    # Connect to robot
    def connect(self, ip, port=10001):
        """global ROBOT_MOVING
        self.disconnect()
        print_message('Connecting to robot %s:%i' % (ip, port))
        # Create new socket connection
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(4)
        self.sockjnts = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sockjnts.settimeout(4)        
        UpdateStatus(ROBOTCOM_WORKING)
        self.sock.connect((ip, port))
        self.CONNECTED = True
        ROBOT_MOVING = False
        print_message('Waiting for welcome message...')
        UpdateStatus(ROBOTCOM_WORKING)
        #time.sleep(2)
        
        # receive welcome message and output to the log
        self.Run('1;1;OPEN=ROBODK',False)
        #print(self.recv_str())
        # notify status that the robot is still working
        UpdateStatus(ROBOTCOM_WORKING)
        
        # send activate robot and read confirmation
        self.Run('1;1;RSTALRM',False)
        self.Run('1;1;CNTLON',False)
        self.Run('1;1;SRVON')
        # RoboDK provides xyzwpr data for the TCP with respect to the robot reference frame for linear movements
        #self.Run('SetWRF', [0, 0, 0, 0, 0, 0])
        #self.sock.settimeout(self.TIMEOUT)        
        #self.sockjnts.connect((ip, 10002))
        
        #q = queue.Queue()
        #t = threading.Thread(target=robot_monitor, args=(q, self.sockjnts))
        #t.daemon = True
        #t.start()
        return True"""

        ser = serial.Serial(ip, 19200, timeout=0,stopbits=2, parity=serial.PARITY_EVEN, rtscts=1)  # open serial port

        ser.write(b'1;0;STOP\r')
        ser.write(b'1;1;SRVOFF\r')
 
        ser.close()

    # Send a line to the robot through the communication port (TCP/IP)
    def send_str(self, msg):
        try:
            sent = self.sock.send(bytes(msg+'\0','ascii'))
            if sent == 0:
                return False
            return True
        except ConnectionAbortedError as e:
            self.CONNECTED = False
            print(str(e))
            return False
    
    # Receive a line from the robot through the communication port (TCP/IP)
    def recv_str(self):
        bdata = b''
        try:
            bdata = self.sock.recv(self.BUFFER_SIZE)
        except ConnectionAbortedError as e:
            self.CONNECTED = False
            print(str(e))
            return
            
        if bdata == b'':
            return None
            
        self.LAST_MSG = bdata.decode('ascii')
        return self.LAST_MSG
    
    # Run a specific command and provide required parameters   
    def Run(self, cmd, send_ready=True):
        # Skip the command if the robot is not connected
        if not self.CONNECTED:
            UpdateStatus(ROBOTCOM_NOT_CONNECTED)
            return

        #---------- Send robot command -------
        # notify RoboDK
        print('sending: %s' % str_send)
        UpdateStatus(ROBOTCOM_WORKING)

        # Try to send the command
        if self.send_str(str_send) is False:
            print_message("Robot connection broken")
            UpdateStatus(ROBOTCOM_NOT_CONNECTED)
            return

        # Try to receive a response
        robot_msg = self.recv_str()
        if self.LAST_MSG is None:
            print_message("Robot connection broken")
            UpdateStatus(ROBOTCOM_NOT_CONNECTED)
            return

        problems = False

        if 'Qer' in robot_msg:
            self.send_str("1;1;ERROR")
            errno = self.recv_str()[3:7]
            self.send_str("1;1;ERRORMES" + str(errno))
            print_message(self.recv_str()[3:])
            return

        if "JPOSF" in cmd:
            # robot response after a GetJoints request
            print_joints(robot_msg.split(";")[1:12:2])
            UpdateStatus(ROBOTCOM_READY)
            return
        
        # Any other acknowledge message (assumed to be successful)
        # By default, we will send the command Ready at every instruction (one Run per instruction in general)
        print(robot_msg)
        if send_ready:
            UpdateStatus(ROBOTCOM_READY)
        else:
            # Save the Ready status to send later and notify RoboDK that the instruction was completed
            global STATUS
            STATUS = ROBOTCOM_READY


# Receives a string through TCP/IP. It reads until if finds NULL character
def read_line(socket):
    data = socket.recv(512)
    s = data.decode("ascii")
    return s


def send_line(socket, msg):
    data = msg.encode("ascii")
    socket.sendall(data)
    
# Specific thread to monitor robot communication
# This thread establishes a permanent link between the robot and the PC to retrieve the robot position at all times
# The robot position is displayed only when the robot is executing a motion command
# When the communication link is broken it will notify the user
def robot_monitor(q, socket):

    try:
        while True:

            send_line(socket, "1;1;JPOSF")
            response = read_line(socket)
            robot_msg = response.split(';')
            #bdata = socket.recv(512)
            #if bdata == b'':
            #    print_message("Invalid monitoring response")
            #    return                
            #robot_msg = bdata.decode('ascii')
            print_joints(robot_msg[1:12:2], True)
            #msg_id = int(robot_msg[1:5])
            #if msg_id == 3007:
                # monitoring stream of data: [3007][j1, j2, j3, j4, j5, j6]
                #print_joints(robot_msg[7:-2].replace(',',''), True)
            #elif msg_id == 3010:
                # position data is also part of the stream. Ignore
                # [3010][212.985, -93.965, 34.273, 180.000, -1.967, 63.936]
            #    pass
            #elif msg_id == 3000:
                # Welcome message is:
                # [3000][Connected to Mecademic Meca500 Robot.]
            #    pass
            #else:
            #    print(robot_msg)
            #    print_message("Unknown monitoring response")
            #    return
                
    except Exception as e:
            print(str(e))
            print_message("Robot not connected")
            return
        
        
#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------
# Generic RoboDK driver for a specific Robot class
global ROBOT
global ROBOT_IP
global ROBOT_PORT
global ROBOT_MOVING

ROBOT = RobotCom()
ROBOT_IP = "127.0.0.1"      # IP of the robot
ROBOT_PORT = 10000          # Communication port of the robot
ROBOT_MOVING = False


#------------ robot connection -----------------
# Establish connection with the robot
def RobotConnect():
    global ROBOT
    global ROBOT_IP
    #global ROBOT_PORT
    ROBOT.connect(ROBOT_IP)
    
# Disconnect from the robot
def RobotDisconnect():
    global ROBOT
    ROBOT.disconnect()
    
        
#-----------------------------------------------------------------------------
# Generic RoboDK driver tools

# Note, a simple print() will flush information to the log window of the robot connection in RoboDK
# Sending a print() might not flush the standard output unless the buffer reaches a certain size

def print_message(message):
    """print_message will display a message in the log window (and the connexion status bar)"""
    print("SMS:" + message)
    sys.stdout.flush() # very useful to update RoboDK as fast as possible

def show_message(message):
    """show_message will display a message in the status bar of the main window"""
    print("SMS2:" + message)
    sys.stdout.flush() # very useful to update RoboDK as fast as possible

def print_joints(joints, ismoving = False):
    if ismoving:
        # Display the feedback of the joints when the robot is moving
        if ROBOT_MOVING:
            #print("CJNT_MOVING " + " ".join(format(x, ".5f") for x in joints)) # if joints is a list of float
            print("JNTS_MOVING " + " ".join(joints))
    else:
        #print("CJNT " + " ".join(format(x, ".5f") for x in joints)) # if joints is a list of float
        print("JNTS " + " ".join(joints))
    sys.stdout.flush() # very useful to update RoboDK as fast as possible

# ---------------------------------------------------------------------------------
# Constant values to display status using UpdateStatus()
ROBOTCOM_UNKNOWN                = -1000
ROBOTCOM_CONNECTION_PROBLEMS    = -3
ROBOTCOM_DISCONNECTED           = -2
ROBOTCOM_NOT_CONNECTED          = -1
ROBOTCOM_READY                  =  0
ROBOTCOM_WORKING                =  1
ROBOTCOM_WAITING                =  2

# Last robot status is saved
global STATUS
STATUS = ROBOTCOM_DISCONNECTED

# UpdateStatus will send an appropriate message to RoboDK which will result in a specific coloring
# for example, Ready will be displayed in green, Waiting... will be displayed in Yellow and other messages will be displayed in red
def UpdateStatus(set_status=None):
    global STATUS
    if set_status is not None:
        STATUS = set_status
        
    if STATUS == ROBOTCOM_CONNECTION_PROBLEMS:
        print_message("Connection problems")
    elif STATUS == ROBOTCOM_DISCONNECTED:
        print_message("Disconnected")
    elif STATUS == ROBOTCOM_NOT_CONNECTED:
        print_message("Not connected")
    elif STATUS == ROBOTCOM_READY:
        print_message("Ready")
    elif STATUS == ROBOTCOM_WORKING:
        print_message("Working...")
    elif STATUS == ROBOTCOM_WAITING:
        print_message("Waiting...")
    else:
        print_message("Unknown status");

# Sample set of commands that can be provided by RoboDK of through the command line
def TestDriver():    
    RunCommand("CONNECT 127.0.0.1 10000")
    #RunCommand("SETTOOL -0.025 -41.046 50.920 60.000 -0.000 90.000")
    #RunCommand("MOVJ -5.362010 46.323420 20.746290 74.878840 -50.101680 61.958500")
    #RunCommand("SPEED 250")
    #RunCommand("MOVL 0 0 0 0 0 0 -5.362010 50.323420 20.746290 74.878840 -50.101680 61.958500")
    #RunCommand("PAUSE 2000") # Pause 2 seconds

#-------------------------- Main driver loop -----------------------------
# Read STDIN and process each command (infinite loop)
# IMPORTANT: This must be run from RoboDK so that RoboDK can properly feed commands through STDIN
# This driver can also be run in console mode providing the commands through the console input
def RunDriver():
    for line in sys.stdin:
        RunCommand(line)
        
# Each line provided through command line or STDIN will be processed by RunCommand    
def RunCommand(linecmd):
    global ROBOT_IP
    global ROBOT
    global ROBOT_MOVING
    
    # strip a line of words into a list of numbers
    def line_2_values(words):
        values = []        
        for word in words:
            try:
                number = float(word)
                values.append(number)
            except:
                pass
        return values
    
    linecmd = linecmd
    words = linecmd.split(' ')
    values = line_2_values(words)
    nvalues = len(values)
    nwords = len(words)
    
    if linecmd == "":
        # Skip if no command is provided
        return
    
    elif nwords >= 2 and linecmd.startswith("CONNECT"):
        # Connect to robot provided the IP and the port
        ROBOT_IP = words[1]
        if nwords >= 3 and nvalues >= 1 and values[0] != 10000:
            #ROBOT_PORT = values[0]
            print("Using default port 10000, not %i" % ROBOT_PORT)
        RobotConnect()
    
    elif nvalues >= 6 and linecmd.startswith("MOVJ"):
        # Activate the monitor feedback
        ROBOT_MOVING = True
        
        # Execute a joint move. RoboDK provides j1,j2,...,j6,x,y,z,w,p,r
        ROBOT.Run("1;1;FDELMRL", False)
        ROBOT.Run("1;1;NEW", False)
        ROBOT.Run("1;1;LOAD=MRL", False)
        ROBOT.Run("1;1;EDATA 1 J1=(" + (','.join(format(vi, ".6f") for vi in values[:6])) + ")", False)
        ROBOT.Run('1;1;EDATA 2 MOV J1', False)
        ROBOT.Run('1;1;EDATA 3 END', False)
        ROBOT.Run("1;1;SAVE", False)
        ROBOT.Run("1;1;RSTPRG", False)
        ROBOT.Run("1;1;PRGLOAD=MRL", False)
        ROBOT.Run("1;1;RSTPRG", False)
        ROBOT.Run("1;1;RUNMRL;1")

    elif nvalues >= 12 and linecmd.startswith("MOVL"):
        # Activate the monitor feedback
        ROBOT_MOVING = True
        
        # Execute a linear move. RoboDK provides j1,j2,...,j6,x,y,z,w,p,r
        ROBOT.Run('1;1;EXECMVS ' + '(' + (','.join(format(vi, ".6f") for vi in values[6:])) + ")(7,0)")
        
    elif linecmd.startswith("CJNT"):
        # Retrieve the current position of the robot
        ROBOT.Run('1;1;JPOSF')    

    elif nvalues >= 1 and linecmd.startswith("SPEED"):
        # First value is linear speed in mm/s
        # IMPORTANT! We should only send one "Ready" per instruction
        if values[0] > 0:            
            # make sure we do not exceed maximum speed (robot turns into error mode)
            ROBOT.Run('1;1;EXECSPD (%.3f)' % min(10000.0, values[0]), False)

        # Provokes sending Ready:
        UpdateStatus()
    elif nvalues >= 1 and linecmd.startswith("SETROUNDING"):
        # Set the rounding/smoothing value. Also known as ZoneData in ABB or CNT for Fanuc
        #ROBOT.Run('SetCornering', [1] if values[0] > 0 else [0])
        pass
    
    elif nvalues >= 1 and linecmd.startswith("PAUSE"):
        UpdateStatus(ROBOTCOM_WAITING)
        # Run a pause
        if values[0] > 0:
            import time
            time.sleep(values[0] * 0.001)
        UpdateStatus(ROBOTCOM_READY)
        
    elif nvalues >= 2 and linecmd.startswith("SETDO"):
        UpdateStatus(ROBOTCOM_WORKING)
        dIO_id = values[0]
        dIO_value = values[1]
        print_message("Warning: Setting DO[%i] = %.1f not implemented" % (dIO_id, dIO_value))
        UpdateStatus(ROBOTCOM_READY)
        
    elif nvalues >= 2 and linecmd.startswith("WAITDI"):
        UpdateStatus(ROBOTCOM_WORKING)
        dIO_id = values[0]
        dIO_value = values[1]
        print_message("Warning: Waiting DI[%i] = %.1f not implemented" % (dIO_id, dIO_value))
        UpdateStatus(ROBOTCOM_READY)
        
    elif nvalues >= 6 and linecmd.startswith("SETTOOL"):
        # Set the Tool reference frame provided the 6 XYZWPR values by RoboDK
        ROBOT.Run('1;1;EXECTOOL (' + (','.join(format(vi, ".6f") for vi in values)) + ')')
        
    elif nvalues >= 1 and nwords >= 2 and linecmd.startswith("RUNPROG"):
        UpdateStatus(ROBOTCOM_WORKING)
        prog_id = int(values[0])
        prog_name = "Program %i" % prog_id
        if nwords >= 3:
            prog_name = words[1]
            
        print_message("Warning: Running program %s not implemented" % (prog_name))
        UpdateStatus(ROBOTCOM_READY)
        
    elif nwords >= 2 and linecmd.startswith("POPUP "):
        UpdateStatus(ROBOTCOM_WORKING)
        message = linecmd[6:]            
        print_message("Warning: Display message %s not implemented" % (message))
        UpdateStatus(ROBOTCOM_READY)
        
    elif linecmd.startswith("DISCONNECT"):
        # Disconnect from robot
        ROBOT.disconnect()
        UpdateStatus(ROBOTCOM_DISCONNECTED)
        
    elif linecmd.startswith("TEST"):
        # Call custom procedure for quick testing
        TestDriver()
        
    elif linecmd.startswith("QUIT"):
        # Stop the driver
        ROBOT.disconnect()
        UpdateStatus(ROBOTCOM_DISCONNECTED)
        quit(0) # Stop the driver
    
    else:
        print("Unknown command: " + linecmd)
    
    # Stop monitoring feedback
    ROBOT_MOVING = False

if __name__ == "__main__":
    """Call Main procedure"""
    # Flush Disconnected message
    UpdateStatus()
    
    # Run the driver from STDIN
    RunDriver()
    
    # Test the driver with a sample set of commands
    #TestDriver()

