import serial

"""ser = serial.Serial('COM4', 19200, timeout=0,stopbits=2, parity=serial.PARITY_EVEN, rtscts=1)  # open serial port
print(ser.name)""" 

#Write the program name here
ProgName = b'TEST'
#Write the program here
def Robot_Program():
    MOV('P1')
    MOV('P2')
    END()
    
#Initializes the line number
printLineNum=0


"""The string command sentence is built in stages, writeFunction sends the serial command and it is the last stage
it creates the common part of the command sentence, Command_Init functions create the command part 
of the sentence and insert the line number, while Command functions insert the last part of the sentence, 
which means on Command Functions only the Command argument is necessary.
The command functions are the one used to create the robot program, they are equivalent to the ones used on 
Cosirop"""

def writeFunction(cmd, lineNum):
    """This function is responsible for sending the serial command"""
    global printLineNum
    printLineNum = lineNum
    printLineNum += 1
    command = ('1;9;EDATA%d %s' % (printLineNum, cmd))
    """command_to_byte = str.encode(command)
    ser.write(command_to_byte)"""
    print (command)
    
        
#Command_Init Functions
def MOV_Init(lineNum, pos):
    MOV_Buffer = 'MOV %s' % (pos)
    writeFunction(MOV_Buffer, lineNum)

def MVS_Int(lineNum, pos):
    MVS_Buffer = 'MVS %s' % (pos)
    writeFunction(MVS_Buffer, lineNum)

def MVR_Int(lineNum, pos1, pos2, pos3):
    MVR_Buffer = 'MVR %s, %s, %s' % (pos1, pos2, pos3)
    writeFunction(MVR_Buffer, lineNum)

def MVR2_Int(lineNum, pos1, pos2, pos3):
    MVR2_Buffer = 'MVR2 %s, %s, %s' % (pos1, pos2, pos3)
    writeFunction(MVR2_Buffer, lineNum)

def MVR3_Int(lineNum, pos1, pos2, pos3):
    MVR3_Buffer = 'MVR3 %s, %s, %s' % (pos1, pos2, pos3)
    writeFunction(MVR3_Buffer, lineNum)

def MVC_Int(lineNum, pos1, pos2, pos3):
    MVC_Buffer = 'MVC %s, %s, %s' % (pos1, pos2, pos3)
    writeFunction(MVC_Buffer, lineNum)

def CNT_Int(lineNum):
    CNT_Buffer = 'CNT 1'
    writeFunction(CNT_Buffer, lineNum)

def CNT1_Int(lineNum, x1, x2):
    CNT1_Buffer = 'CNT 1, %s, %s' % (x1, x2)
    writeFunction(CNT1_Buffer, lineNum)

def CNT0_Int(lineNum):
    CNT0_Buffer = 'CNT 0'
    writeFunction(CNT0_Buffer, lineNum)

def ACCEL_Int(lineNum, accl, deaccl):
    ACCEL_Buffer = 'ACCEL %s, %s' % (accl, deaccl)
    writeFunction(ACCEL_Buffer, lineNum)

def OVRD_Int(lineNum, speed):
    OVRD_Buffer = 'OVRD %s' % (speed)
    writeFunction(OVRD_Buffer, lineNum)

def JOVRD_Int(lineNum, speed):
    JOVRD_Buffer = 'JOVRD %s' % (speed)
    writeFunction(JOVRD_Buffer, lineNum)

def SPD_Int(lineNum, speed):
    SPD_Buffer = 'SPD %s' % (speed)
    writeFunction(SPD_Buffer, lineNum)

def OADL_Int(lineNum, optaccl):
    OLAD_Buffer = 'OADL %s' % (optaccl)
    writeFunction(OLAD_Buffer, lineNum)

def FINE_Int(lineNum, pulse):
    FINE_Buffer = 'FINE %s' % (pulse)
    writeFunction(FINE_Buffer, lineNum)

def PREC_Int(lineNum, precise):
    PREC_Buffer = 'PREC %s' % (precise)
    writeFunction(PREC_Buffer, lineNum)

def H_CTRL_Int(lineNum, hand):
    H_CTRL_Buffer = 'H%s 1' % (hand)
    writeFunction(H_CTRL_Buffer, lineNum)

def DLY_Int(lineNum, delay):
    DLY_Buffer = 'DLY %s' % (delay)
    writeFunction(DLY_Buffer, lineNum)

def END_Int(lineNum):
    ENDBuffer = 'END'
    writeFunction(ENDBuffer, lineNum)

def TOOL_Int(lineNum, coord):
    TOOL_Buffer = 'TOOL(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)' % (coord[0], coord[1], coord[2], coord[3], coord[4], coord[5])
    writeFunction(TOOL_Buffer, lineNum)



#Command Functions

#Commands that control the robot movement

#Joint Interpolation Movement
def MOV(pos):
    """Moves the robot with Joint Interpolation movement"""
    MOV_Init(printLineNum, pos)

#Linear Interpolation Movement
def MVS(pos):
    """Moves the robot with linear interpolation movement"""
    MVS_Int(printLineNum, pos)

#Circular interpolation Movement
def MVR(pos1, pos2, pos3):
    """Designates start point, transit point and final point and moves with circular interpolation movement, passing in 
    all points on that order"""
    MVR_Int(printLineNum, pos1, pos2, pos3)

def MVR2(pos1, pos2, pos3):
    """Designates start point, end point and reference point. Moves from start point to end point with circular interpolation
    movement, without passing through reference point"""
    MVR2_Int(printLineNum, pos1, pos2, pos3)

def MVR3(pos1, pos2, pos3):
    """Designates start point, end point and center point. Moves from start point to end point with circular interpolation movement"""
    MVR3_Int(printLineNum, pos1, pos2, pos3)

def MVC(pos1, pos2, pos3):
    """Designates start point (end point), transit point 1, transit point 2. Moves from start point to end point
    in order of start point - transit point 1 - transit point 2 - end point with circular interpolation movement"""
    MVC_Int(printLineNum, pos1, pos2, pos3)

#Continuous movement
def CNT():
    """Validates continous movement function"""
    CNT_Int(printLineNum)

def CNT1(x1, x2):
    """Sets the start point and end point neighborhood distances"""
    CNT1_Int(printLineNum, x1, x2)

def CNT0():
    """Invalidades continous movement"""
    CNT0_Int(printLineNum)

#Acceleration and Deacceleration control
def ACCEL(accl, deaccl):
    """designates the acceleration and deacceleration in percentage"""
    ACCEL_Int(printLineNum, accl, deaccl)

def OVRD(speed):
    """Designates the movement speed throughout the whole program as a percentage"""
    OVRD_Int(printLineNum, speed)

def JOVRD(speed):
    """Designates the joint interpolation speed as a percentage of the maximum speed"""
    JOVRD_Int(printLineNum, speed)

def SPD(speed):
    """Designates the linear and circular interpolation speed with the hand end speed in mm/s"""
    SPD_Int(printLineNum, speed)

def OADL(optaccl):
    """Enables or disables the optimum acceleration/decceleration: OADL(ON) or OADL(OFF)"""
    OADL_Int(printLineNum, optaccl)

#Fine Positioning
def FINE(pulse):
    """Defines the final positioning with a number of pulses, more accurate positioning"""
    FINE_Int(printLineNum, pulse)

#High accuracy path
def PREC(precise):
    """Enables or disables the high accuracy path function. ON or OFF"""
    PREC_Int(printLineNum, precise)

#Hand Control
def H_CTRL(hand):
    """Controls hand, H+CTRL('OPEN') or H_CTRL('CLOSE')"""
    H_CTRL_Int(printLineNum, hand)

#Delay
def DLY(delay):
    """Adds a delay in seconds on the program"""
    DLY_Int(printLineNum, delay)

#Tool position

def TOOL(coord):
   """Positions the tool on the robot"""
   TOOL_Int(printLineNum, coord)     

#Function to end the program, last command of the program 
def END():
    END_Int(printLineNum)


"""ser.write(b'1;1;CNTLON\r')
ser.write(b'1;1;SAVE\r')
ser.write(b'1;9;LOAD=%s.MB4\r' % ProgName)
#Function where the robot program is written"""
Robot_Program()
"""ser.write(b'1;1;SAVE\r')
ser.write(b'1;1;CNTLOFF\r')"""



