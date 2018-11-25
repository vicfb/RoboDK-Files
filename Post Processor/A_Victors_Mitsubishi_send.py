# Copyright 2017 - RoboDK Software S.L. - http://www.robodk.com/
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ----------------------------------------------------
# This file is a POST PROCESSOR for Robot Offline Programming to generate programs 
# for a generic robot with RoboDK
#
# To edit/test this POST PROCESSOR script file:
# Select "Program"->"Add/Edit Post Processor", then select your post or create a new one.
# You can edit this file using any text editor or Python editor. Using a Python editor allows to quickly evaluate a sample program at the end of this file.
# Python should be automatically installed with RoboDK
#
# You can also edit the POST PROCESSOR manually:
#    1- Open the *.py file with Python IDLE (right click -> Edit with IDLE)
#    2- Make the necessary changes
#    3- Run the file to open Python Shell: Run -> Run module (F5 by default)
#    4- The "test_post()" function is called automatically
# Alternatively, you can edit this file using a text editor and run it with Python
#
# To use a POST PROCESSOR file you must place the *.py file in "C:/RoboDK/Posts/"
# To select one POST PROCESSOR for your robot in RoboDK you must follow these steps:
#    1- Open the robot panel (double click a robot)
#    2- Select "Parameters"
#    3- Select "Unlock advanced options"
#    4- Select your post as the file name in the "Robot brand" box
#
# To delete an existing POST PROCESSOR script, simply delete this file (.py file)
#
# ----------------------------------------------------
# More information about RoboDK Post Processors and Offline Programming here:
#     http://www.robodk.com/help#PostProcessor
#     http://www.robodk.com/doc/en/PythonAPI/postprocessor.html
# ----------------------------------------------------


# ----------------------------------------------------
# Import RoboDK tools
from robodk import *
import serial


#ser = serial.Serial('COM4', 19200, timeout=0,stopbits=2, parity=serial.PARITY_EVEN, rtscts=1)
printLineNum = 0


def writeFunction(lineNum):
    """This function is responsible for sending the serial command"""
    global printLineNum
    printLineNum = lineNum
    printLineNum += 1
    return ('1;9;EDATA%d' % printLineNum)
    """command_to_byte = str.encode(command) #converts the string built in bytes to be transmitted in serial
    ser.write(command_to_byte)"""

#Function took from the robodk file and modified. The RV-2AJ robot does not have rotation in y, due to the 5 degrees
#of freedom limit
def pose_2_xyzrw(H):
    """Calculates the equivalent position and euler angles ([x,y,z,r,w] array) of the given pose according to the following operation:
    This robot do not have a rotation in y for this calculation. This function is different than the one found on the robodk file
    :param H: pose
    :type H: :class:`.Mat`
    :return: [x,y,z,w,p,r] in mm and deg
    
    Uses the following order: transl(x,y,z)*rotz(w*pi/180)*rotx(r*pi/180)
        
    .. seealso:: :class:`.Mat`, :func:`~robodk.xyzrpw_2_pose`, :func:`~robodk.TxyzRxyz_2_Pose`
    """
    x = H[0,3]
    y = H[1,3]
    z = H[2,3]
    if (H[2,0] > (1.0 - 1e-6)):
        p = -pi/2
        r = 0
        w = math.atan2(-H[1,2],H[1,1])
    elif H[2,0] < -1.0 + 1e-6:
        p = pi/2
        r = 0
        w = math.atan2(H[1,2],H[1,1])
    else:
        p = math.atan2(-H[2,0],sqrt(H[0,0]*H[0,0]+H[1,0]*H[1,0]))
        w = math.atan2(H[1,0],H[0,0])
        r = math.atan2(H[2,1],H[2,2])    
    return [x, y, z, r*180/pi, w*180/pi]
# ----------------------------------------------------
def pose_2_str(pose):
    """Prints a pose target"""
    [x,y,z,r,w] = pose_2_xyzrw(pose)
    return ('(%.3f,%.3f,%.3f,%.3f,%.3f)' % (x,y,z,r,w))
    
def angles_2_str(angles):
    """Prints a joint target"""
    str = ''
    for i in range(len(angles)):
        str = str + ('%.6f,' % (angles[i]))
    str = str[:-1]
    return "(%s)" % str

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""
    PROG_EXT = 'txt'        # set the program extension
    
    PROG_to_byte = b''

    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = []
    
    PROG = ''
    LOG = ''
    FOOTER = ''
    STRUCT_FLAG = '(7,0)'
    lineno = 0
    nAxes = 6
    nTargets = 0
    POSE_LAST = None
    MSG_VAR = None

    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ''
        self.LOG = ''
        self.nAxes = robot_axes
        
    def ProgStart(self, progname):
        #self.addline('PROC %s()' % progname)
        #mbox('hello')
        return 
        
        
    def ProgFinish(self, progname):
        self.addline('%s END' % writeFunction(printLineNum))
        
    def ProgSave(self, folder, progname, ask_user=False, show_result=False ):
        
        progname = progname + '.' + self.PROG_EXT
        if ask_user or not DirExists(folder):
            filesave = getSaveFile(folder, progname, 'Save program as...')
            if filesave is not None:
                filesave = filesave.name
            else:
                return
        else:
            filesave = folder + '/' + progname
        fid = open(filesave, "w")
        fid.write(self.PROG)
        fid.write(self.FOOTER)
        fid.close()
        print('SAVED: %s\n' % filesave)
        self.PROG_FILES = filesave

       
        
        #---------------------- show result
        if show_result:
            if type(show_result) is str:
                # Open file with provided application
                import subprocess
                p = subprocess.Popen([show_result, filesave])   
            elif type(show_result) is list:
                import subprocess
                p = subprocess.Popen(show_result + [filesave])   
            else:
                # open file with default application
                import os
                os.startfile(filesave)  

            if len(self.LOG) > 0:
                mbox('Program generation LOG:\n\n' + self.LOG)
        
    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        
        SerialCommunication(robot_ip, remote_path, ftp_user, self.PROG_to_byte)
        ProgStart(robot_ip, remote_path, ftp_user)  
            
       

    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        self.POSE_LAST = pose
        self.nTargets = self.nTargets + 1
        self.addline('1;9;EDATA J%i = %s' % (self.nTargets, angles_2_str(joints)))
        self.addline('%s MOV J%i' % (writeFunction(printLineNum), self.nTargets))
        

    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        self.POSE_LAST = pose
        self.nTargets = self.nTargets + 1
        self.addline('1;9;EDATA P%i = %s' % (self.nTargets, pose_2_str(pose)))
        self.addline('%s MVS P%i' % (writeFunction(printLineNum), self.nTargets))

        

    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.nTargets = self.nTargets + 1
        pi = self.nTargets
        self.addline('1;9;EDATA P%i = %s' % (self.nTargets, pose_2_str(POSE_LAST)))
        self.nTargets = self.nTargets + 1
        self.addline('1;9;EDATA P%i = %s' % (self.nTargets, pose_2_str(pose1)))
        self.nTargets = self.nTargets + 1
        self.addline('1;9;EDATA P%i = %s' % (self.nTargets, pose_2_str(pose2)))
        self.addline('%s MVR P%i, P%i, P%i' % (writeFunction(printLineNum), pi, pi+1, pi+2))
                

    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        self.addline('Base %s' % pose_2_str(pose))
        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.addline('%s TOOL%s' % (writeFunction(printLineNum), pose_2_str(pose)))
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        time_ms = time_ms*0.001
        self.addline('%s DLY %.1f' % (writeFunction(printLineNum), time_ms))
    
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        # Assume 5000 mm/s as 100%
        speed_percent = min(speed_mms*100/5000, 100)
        self.addline('%s OVRD %.1f' % (writeFunction(printLineNum), speed_percent))
    
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.addlog('setAcceleration not defined')
    
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.addlog('setSpeedJoints not defined')
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.addlog('setAccelerationJoints not defined')
        
    def setZoneData(self, zone_mm):
        """Changes the smoothing radius (aka CNT, APO or zone data) to make the movement smoother"""
        if zone_mm > 0:
            self.addline('Cnt 1, %.1f' % zone_mm)
        else:
            self.addline('Cnt 0')

    def setDO(self, io_var, io_value):
        """Sets a variable (digital output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'OUT(%s)' % str(io_var)
        if type(io_value) != str: # set default variable value if io_value is a number
            if io_value > 0:
                io_value = '1'
            else:
                io_value = '0'

        # at this point, io_var and io_value must be string values
        self.addline('%s=%s' % (io_var, io_value))

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for a variable (digital input) io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'IN(%s)' % str(io_var)
        if type(io_value) != str: # set default variable value if io_value is a number
            if io_value > 0:
                io_value = '1'
            else:
                io_value = '0'

        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            self.addline('Wait %s=%s' % (io_var, io_value))
        else:
            self.addline('Wait %s=%s Timeout=%.1f' % (io_var, io_value, timeout_ms))
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code.replace(' ','_')
            self.addline('CallP "%s"' % code)
        else:
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Display a message in the robot controller screen (teach pendant)"""
               
        if iscomment:
            self.addline('\'' + message)
        else:
            if self.MSG_VAR is None:
                self.MSG_VAR = 'CMESSAGE'
            self.addline('%s$ = "%s"' % (self.MSG_VAR, message))
        
# ------------------ private ----------------------                
    def addline(self, newline):
        """Add a program line"""
        #self.lineno += 1
        self.PROG += '%s' % (newline) + '\r' + '\n'
        self.PROG_to_byte += str.encode(self.PROG)
        
        
    def addlog(self, newline):
        """Add a log message"""
        self.LOG = self.LOG + newline + '\n'

    def addfooter(self, newline):
        """Add a program line"""
        self.FOOTER = self.FOOTER + newline + '\n'
# -------------------------------------------------
# ------------ For testing purposes ---------------   
def Pose(xyzrpw):
    [x,y,z,r,p,w] = xyzrpw
    a = r*math.pi/180
    b = p*math.pi/180
    c = w*math.pi/180
    ca = math.cos(a)
    sa = math.sin(a)
    cb = math.cos(b)
    sb = math.sin(b)
    cc = math.cos(c)
    sc = math.sin(c)
    return Mat([[cb*ca, ca*sc*sb - cc*sa, sc*sa + cc*ca*sb, x],[cb*sa, cc*ca + sc*sb*sa, cc*sb*sa - ca*sc, y],[-sb, cb*sc, cc*cb, z],[0,0,0,1]])

def test_post():
    """Test the post with a basic program"""

    robot = RobotPost()

    robot.ProgStart('BERNARDES')
    """robot.RunMessage("Program generated by RoboDK using a custom post processor", True)
    robot.setFrame(Pose([807.766544, -963.699898, 41.478944, 0, 0, 0]))
    robot.setTool(Pose([62.5, -108.253175, 100, -60, 90, 0]))"""
    robot.MoveJ(Pose([200, 200, 500, 180, 0, 180]), [-46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752] )
    robot.MoveL(Pose([200, 250, 348.734575, 180, 0, -150]), [-41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418] )
    robot.MoveL(Pose([200, 200, 262.132034, 180, 0, -150]), [-43.73892, -3.91728, -35.77935, 58.57566, 54.11615, -253.81122] )
    """robot.RunMessage("Setting air valve 1 on")
    robot.RunCode("TCP_On", True)
    robot.Pause(1000)
    robot.MoveL(Pose([200, 250, 348.734575, 180, 0, -150]), [-41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418] )
    robot.MoveL(Pose([250, 300, 278.023897, 180, 0, -150]), [-37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677] )
    robot.MoveL(Pose([250, 250, 191.421356, 180, 0, -150]), [-39.75778, -1.04537, -40.37883, 52.09118, 54.15317, -246.94403] )
    robot.RunMessage("Setting air valve off")
    robot.RunCode("TCP_Off", True)
    robot.Pause(1000)"""
    robot.MoveL(Pose([250, 300, 278.023897, 180, 0, -150]), [-37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677] )
    robot.MoveL(Pose([250, 200, 278.023897, 180, 0, -150]), [-41.85389, -1.95619, -34.89154, 57.43912, 52.34162, -253.73403] )
    robot.MoveL(Pose([250, 150, 191.421356, 180, 0, -150]), [-43.82111, 3.29703, -40.29493, 56.02402, 56.61169, -249.23532] )
    robot.MoveJ(None, [-46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752] )
    robot.ProgFinish('Program')
    #robot.ProgSave(".",'BERNARDES', True)
    #robot.ProgSendRobot('COM4', 'BERNARDES')
    print(robot.PROG)
    print(robot.FOOTER)
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    test_post()
