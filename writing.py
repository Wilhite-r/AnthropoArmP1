#from board import SCL, SDA
from datetime import datetime
from time import time, sleep
import math
#import busio

#from adafruit_pca9685 import PCA9685
#from adafruit_motor import servo

#Arm Dimensions
BicepLength = 9
ElbowHeight = 1.25
ElbowLength = 3.3
ForearmLength = 10
ShoulderHeight = 14.25
hoverHeight = 4.0

#calculated dimensions
DrawHeight = ShoulderHeight - ElbowHeight - hoverHeight

#range
YDisMin = ElbowLength+ForearmLength
YDisMax = 16.9
XDisMin = -3.0
XDisMax = 3.0

#Bias to Correct
ElbowHorizontalAngleBias = -12

#Calculate resolution
forearmYLengthRelative = math.sqrt(ForearmLength**2 - XDisMax**2)
XResolution = 2 * math.degrees(math.acos(XDisMax / ForearmLength))
hypoteneuseLength = math.sqrt(DrawHeight**2 + YDisMax**2)
additionalTriangleTheta1 = math.degrees(math.acos(((ElbowLength + forearmYLengthRelative)**2 + hypoteneuseLength**2 - BicepLength**2)/(2 * hypoteneuseLength*(ElbowLength + forearmYLengthRelative))))
additionalTriangleTheta2 = math.degrees(math.acos((hypoteneuseLength**2 + BicepLength**2 - (ElbowLength + forearmYLengthRelative)**2)/(2 * BicepLength * hypoteneuseLength)))
bigTheta1 = math.degrees(math.asin(DrawHeight/hypoteneuseLength))
bigTheta2 = math.degrees(math.asin(YDisMax/hypoteneuseLength))
ShoulderResolutionMin = bigTheta2 - additionalTriangleTheta2
elbowYResolutionMin = (90 - (bigTheta1 - additionalTriangleTheta1)) - ShoulderResolutionMin

XResolution = math.floor(XResolution)
YResolution = math.floor(elbowYResolutionMin if ShoulderResolutionMin < elbowYResolutionMin else ShoulderResolutionMin)

print("X Range")
print("0, " + str(XResolution))
print("Y Range")
print("0, " + str(YResolution))


# calculate and move servos to a particular angle passed in
#xcoor = int
#ycoor = int
#lift = bool
def GoToXY(xcoor, ycoor, lift):
    if lift:
        servoElbowVerticalAngle = 135
        servoElbowVertical.angle = servoElbowVerticalAngle
        sleep(0.2)

    print("Given Coordinates:")
    print("(" + str(xcoor) + ", " + str(ycoor) + ")")

    if (xcoor > XResolution or xcoor < 0 or ycoor > YResolution or ycoor < 0):
        print("ERROR: Coordinate passed to GoToXY out of the resolution: ")
        print("X Range")
        print("0, " + str(XResolution))
        print("Y Range")
        print("0, " + str(YResolution))
        return 0

    xdis = XDisMax - xcoor * ((XDisMax - XDisMin)/XResolution)
    ydis = YDisMin + ycoor * ((YDisMax - YDisMin)/YResolution)

    #Do math for Elbow X angle
    forearmYLengthRelative = math.sqrt(ForearmLength**2 - xdis**2)
    elbowXAngle = math.degrees(math.acos(xdis / ForearmLength))

    #Do math for shoulder and elbow y angles
    hypoteneuseLength = math.sqrt(DrawHeight**2 + ydis**2)
    additionalTriangleTheta1 = math.degrees(math.acos(((ElbowLength + forearmYLengthRelative)**2 + hypoteneuseLength**2 - BicepLength**2)/(2 * hypoteneuseLength*(ElbowLength + forearmYLengthRelative))))
    additionalTriangleTheta2 = math.degrees(math.acos((hypoteneuseLength**2 + BicepLength**2 - (ElbowLength + forearmYLengthRelative)**2)/(2 * BicepLength * hypoteneuseLength)))
    bigTheta1 = math.degrees(math.asin(DrawHeight/hypoteneuseLength))
    bigTheta2 = math.degrees(math.asin(ydis/hypoteneuseLength))
    shoulderAngle = bigTheta2 - additionalTriangleTheta2
    elbowYAngle = (90 - (bigTheta1 - additionalTriangleTheta1)) - shoulderAngle

    #print("Angles Calculated:")
    #print("Shoulder: " + str(shoulderAngle) + ", Elbow Vertical:" + str(elbowYAngle))
    #print("Elbow Horizontal: " + str(elbowXAngle))
    #If lift is passed as TRUE, we must lift the elbow so it doesn't draw
    elbowYAngle = 135 if lift else elbowYAngle
    elbowXAngle = elbowXAngle - ElbowHorizontalAngleBias

    #TODO: Figure out angle difference and how long we need to wait for completion of servo spin

    #move all servos
    #we save the angles in variables as well as assign to the servos
    #this is because we have no way of reading the angle of the servos
    #so saving it internally allows to access the current angle of the servo
    servoShoulderAngle = shoulderAngle
    #servoShoulder.angle = servoShoulderAngle
    servoElbowVerticalAngle = elbowYAngle
    #servoElbowVertical.angle = servoElbowVerticalAngle
    servoElbowHorizontalAngle = elbowXAngle
    #servoElbowHorizontal.angle = servoElbowHorizontalAngle

    sleep(0.1)
    return 1

#Draw line from (x1,y1) to (x2,y2)
def DrawLine(x1, y1, x2, y2):
    stepsPassed = 0
    stepDifferenceX = (x2 - x1)
    stepDifferenceY = (y2 - y1)
    stepsTotal = abs(stepDifferenceX) if abs(stepDifferenceX) > abs(stepDifferenceY) else abs(stepDifferenceY)
    stepSizeX = stepDifferenceX/stepsTotal
    stepSizeY = stepDifferenceY/stepsTotal
    currentX = float(x1)
    currentY = float(y1)

    GoToXY(currentX,currentY,False)

    while (not (stepDifferenceX == 0 and stepDifferenceY == 0) and stepsPassed < stepsTotal):
        stepDifferenceX = (x2 - currentX)
        stepDifferenceY = (y2 - currentY)
        if not stepDifferenceX == 0:
            currentX = currentX + stepSizeX
        if not stepDifferenceY == 0:
            currentY = currentY + stepSizeY

        GoToXY(currentX, currentY, False)
        stepsPassed = stepsPassed + 1

    return 0

#Get Integer input
def AwaitIntInput(rangeMin, rangeMax):
    isAwaitingCorrectFormat = True

    while (isAwaitingCorrectFormat):
        # Get raw input from user
        userSelection = input("Enter selection in range " + str(rangeMin) + "-" + str(rangeMax))
        # Validate user input
        if not userSelection.isdigit() or int(userSelection) < rangeMin or int(userSelection) > rangeMax:
            print('Incorrect input format detected, please input an integer between ' + str(rangeMin) + ' and ' + str(rangeMax))
        else:
            # If input passes validation, end loop and convert to an integer
            isAwaitingCorrectFormat = False
            selection = int(userSelection)
    
    return selection

def DrawSquare():
    GoToXY(0,0,True)
    lesserResolution = XResolution if XResolution < YResolution else YResolution
    #start at origin
    DrawLine(0, 0, 0, lesserResolution)
    DrawLine(0, lesserResolution, lesserResolution, lesserResolution)
    DrawLine(lesserResolution, lesserResolution, lesserResolution, 0)
    DrawLine(lesserResolution, 0, 0, 0)
 
def DrawTriangle():
    GoToXY(0,0,True)
    lesserResolution = XResolution if XResolution < YResolution else YResolution
    #start at origin
    DrawLine(0, 0, lesserResolution/2, lesserResolution)
    DrawLine(lesserResolution/2, lesserResolution, lesserResolution, 0)
    DrawLine(lesserResolution, 0, 0, 0)

def WriteRW():
    GoToXY(0,0,True)
    letterWidth = XResolution/3
    letterHeight = YResolution

    #Draw R
    DrawLine(0,0,0, YResolution)
    DrawLine(0,YResolution,letterWidth * (5/10), YResolution)
    DrawLine(letterWidth * (5/10), YResolution,letterWidth * (6/10), YResolution * (9/10))
    DrawLine(letterWidth * (6/10), YResolution * (9/10), letterWidth * (7/10), YResolution * (7/10))
    DrawLine(letterWidth * (7/10), YResolution * (7/10), letterWidth * (5/10), YResolution * (6/10))
    DrawLine(letterWidth * (5/10), YResolution * (6/10), 0, YResolution * (5/10))
    DrawLine(0, YResolution * (5/10),letterWidth, 0)

    #Draw W
    XBound = letterWidth
    GoToXY(XBound,YResolution,True)
    DrawLine(XBound, YResolution, XBound + (letterWidth * (1/4)), 0)
    DrawLine(XBound + (letterWidth * (1/4)), 0, XBound + (letterWidth * (1/2)), YResolution * (5/10))
    DrawLine(XBound + (letterWidth * (1/2)), YResolution * (5/10), XBound + (letterWidth * (3/4)), 0)
    DrawLine(XBound + (letterWidth * (3/4)), 0, XBound + letterWidth, YResolution)

def WriteJJ():
    GoToXY(0,YResolution,True)
    letterWidth = XResolution/3
    letterHeight = YResolution

    #Draw 1st J
    DrawLine(0,YResolution,letterWidth, YResolution)
    GoToXY(letterWidth * (3/4),YResolution,True)
    DrawLine(letterWidth * (3/4),YResolution,letterWidth * (3/4), YResolution * (3/10))
    DrawLine(letterWidth * (3/4), YResolution * (3/10), letterWidth * (1/2), 0)
    DrawLine(letterWidth * (1/2), 0, letterWidth * (1/4), YResolution * (2/10))

    #Draw 2nd J
    XBound = letterWidth
    GoToXY(XBound,YResolution,True)
    DrawLine(XBound,YResolution,XBound + letterWidth, YResolution)
    GoToXY(XBound + letterWidth * (3/4),YResolution,True)
    DrawLine(XBound + letterWidth * (3/4),YResolution, XBound + letterWidth * (3/4), YResolution * (3/10))
    DrawLine(XBound + letterWidth * (3/4), YResolution * (3/10), XBound + letterWidth * (1/2), 0)
    DrawLine(XBound + letterWidth * (1/2), 0, XBound + letterWidth * (1/4), YResolution * (2/10))

def WriteLetters():
    namePicker = {
        1: WriteRW,
        2: WriteJJ
    }

    print("Select initials:")
    print("1 = RW")
    print("2 = JJ")

    userSelection = AwaitIntInput(1, 2)
    func = namePicker.get(userSelection, lambda: "Invalid Selection")

    response = func()
 
def Wave():
    #rest angle
    servoShoulderAngle = 0
    #servoShoulder.angle = servoShoulderAngle
    servoElbowVerticalAngle = 90
    #servoElbowVertical.angle = servoElbowVerticalAngle
    servoElbowHorizontalAngle = 90 - ElbowHorizontalAngleBias
    #servoElbowHorizontal.angle = servoElbowHorizontalAngle

    sleep(1)

    #lift shoulder slowly to 90 degrees (4.5 seconds)
    for shoulderAngle in range(0, 135):
        servoShoulderAngle = shoulderAngle
        #servoShoulder.angle = servoShoulderAngle
        sleep(0.01)

    sleep(0.5)

    #extend elbow
    for elbowYAngle in range(0, 45):
        servoElbowVerticalAngle = 90 - elbowYAngle
        #servoElbowVertical.angle = servoElbowVerticalAngle
        sleep(0.01)

    sleep(1)

    for elbowXAngle in range(0,45):
        servoElbowHorizontalAngle = (90 - elbowXAngle) - ElbowHorizontalAngleBias
        #servoElbowHorizontal.angle = servoElbowHorizontalAngle
        print(servoElbowHorizontalAngle)
        sleep(0.01)
 
    for elbowXAngle in range(45,135):
        servoElbowHorizontalAngle = elbowXAngle - ElbowHorizontalAngleBias
        #servoElbowHorizontal.angle = servoElbowHorizontalAngle
        print(servoElbowHorizontalAngle)
        sleep(0.01)

    for elbowXAngle in range(0,90):
        servoElbowHorizontalAngle = (135 - elbowXAngle) - ElbowHorizontalAngleBias
        #servoElbowHorizontal.angle = servoElbowHorizontalAngle
        print(servoElbowHorizontalAngle)
        sleep(0.01)

    for elbowXAngle in range(45,135):
        servoElbowHorizontalAngle = elbowXAngle - ElbowHorizontalAngleBias
        #servoElbowHorizontal.angle = servoElbowHorizontalAngle
        print(servoElbowHorizontalAngle)
        sleep(0.01)

    for elbowXAngle in range(0,45):
        servoElbowHorizontalAngle = (135 - elbowXAngle) - ElbowHorizontalAngleBias
        #servoElbowHorizontal.angle = servoElbowHorizontalAngle
        print(servoElbowHorizontalAngle)
        sleep(0.01)

def DebugGoToXY():
    print("X1: ")
    x1 = AwaitIntInput(0, XResolution)
    print("Y1: ")
    y1 = AwaitIntInput(0, YResolution)
    
    return GoToXY(x1, y1, False)


#i2c = busio.I2C(SCL, SDA)

#create pwm object with 50hz frequency
#pca = PCA9685(i2c)
#pca.frequency = 50

# set pwm pulse width range for each channel
#servoShoulder = servo.Servo(pca.channels[0], min_pulse = 500, max_pulse = 2500)
#servoElbowVertical = servo.Servo(pca.channels[1], min_pulse = 500, max_pulse = 2500)
#servoElbowHorizontal = servo.Servo(pca.channels[2], min_pulse = 500, max_pulse = 2500)

#main loop
while True:
    #rest angle
    servoShoulderAngle = 0
    #servoShoulder.angle = servoShoulderAngle
    servoElbowVerticalAngle = 90
    #servoElbowVertical.angle = servoElbowVerticalAngle
    servoElbowHorizontalAngle = 90 - ElbowHorizontalAngleBias
    #servoElbowHorizontal.angle = servoElbowHorizontalAngle

    functionSwitcher = {
        1: DrawSquare,
        2: DrawTriangle,
        3: WriteLetters,
        4: Wave,
        5: DebugGoToXY
    }

    print("Select a functionality:")
    print("1 = Draw Square")
    print("2 = Draw Triangle")
    print("3 = Write Initials")
    print("4 = Wave")
    print("5 = debug GoToXY")
    
    userSelection = AwaitIntInput(1, 5)
    func = functionSwitcher.get(userSelection, lambda: "Invalid Selection")

    response = func()
pca.deinit()
