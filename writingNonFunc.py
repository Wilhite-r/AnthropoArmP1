#from board import SCL, SDA
from datetime import datetime
from time import time, sleep
import math
import busio

#from adafruit_pca9685 import PCA9685
#from adafruit_motor import servo

#Arm Dimensions
BicepLength = 9
ElbowHeight = 1
ElbowLength = 3.3
ForearmLength = 9
ShoulderHeight = 14.5
hoverHeight = 3.0

#calculated dimensions
DrawHeight = ShoulderHeight - ElbowHeight - hoverHeight

#range
YDisMin = ElbowLength+ForearmLength
YDisMax = 16.9
XDisMin = -4.0
XDisMax = 4.0

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
    ydis = YDisMax - ycoor * ((YDisMax - YDisMin)/YResolution)

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
    elbowYAngle = (90 - (bigTheta1 - additionalTriangleTheta1)) - ShoulderResolutionMin

    #If lift is passed as TRUE, we must lift the elbow so it doesn't draw
    servoElbowVertical = 270 if lift else elbowYAngle

    #move all servos
    #servoShoulderAngle = shoulderAngle
    #servoShoulder.angle = shoulderAngle
    #servoElbowVerticalAngle = servoElbowVertical
    #servoElbowVertical.angle = servoElbowVertical
    #servoElbowHorizontalAngle = elbowXAngle
    #servoElbowHorizontal.angle = elbowXAngle
    return 1

#Draw line from (x1,y1) to (x2,y2)
def DrawLine(x1, y1, x2, y2):
    stepsPassed = 0
    stepDifferenceX = abs(x2 - x1)
    stepDifferenceY = abs(y2 - y1)
    stepsTotal = stepDifferenceX if stepDifferenceX > stepDifferenceY else stepDifferenceY
    currentX = x1
    currentY = y1

    GoToXY(currentX,currentY,False)

    while (not (stepDifferenceX == 0 and stepDifferenceY == 0)):
        stepDifferenceX = (x2 - currentX)
        stepDifferenceY = (y2 - currentY)
        if not stepDifferenceX == 0:
            currentX = currentX + (stepDifferenceX/abs(stepDifferenceX))
        if not stepDifferenceY == 0:
            currentY = currentY + (stepDifferenceY/abs(stepDifferenceY))

        GoToXY(currentX, currentY, False)

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
    lesserResolution = XResolution if XResolution < YResolution else YResolution
    #start at origin
    DrawLine(0, 0, 0, lesserResolution)
    DrawLine(0, lesserResolution, lesserResolution, lesserResolution)
    DrawLine(lesserResolution, lesserResolution, lesserResolution, 0)
    DrawLine(lesserResolution, 0, 0, 0)
 
def DrawCircle():
    return "February"
 
def WriteLetters():
    return "March"
 
def Wave():
    return "April"
 
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

#rest angle
#servoShoulderAngle = 0
#servoShoulder.angle = servoShoulderAngle
#servoElbowVerticalAngle = 90
#servoElbowVertical.angle = servoElbowVerticalAngle
#servoElbowHorizontalAngle = 90
#servoElbowHorizontal.angle = servoElbowHorizontalAngle

#main loop
while True:
    functionSwitcher = {
        1: DrawSquare,
        2: DrawCircle,
        3: WriteLetters,
        4: Wave,
        5: DebugGoToXY
    }

    print("Select a functionality:")
    print("1 = Draw Square")
    print("2 = Draw Circle")
    print("3 = Write letters (4 Max)")
    print("4 = Wave")
    print("5 = debug GoToXY")
    
    userSelection = AwaitIntInput(1, 5)
    func = functionSwitcher.get(userSelection, lambda: "Invalid Selection")

    response = func()
pca.deinit()
