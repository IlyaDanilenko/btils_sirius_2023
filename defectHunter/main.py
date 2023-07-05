import math
from rplidar import RPLidar
lidar = RPLidar('/dev/ttyUSB1')

wheelDiameter = 10
cpr = 360
pi = math.pi
goEps = 180
rotateEps = 3
rotation = 0

mRearLeft = brick.motor(M3).setPower
mRearRight = brick.motor(M4).setPower
mFrontLeft = brick.motor(M1).setPower
mFrontRight = brick.motor(M2).setPower

eRearLeft = brick.encoder(E3)

eRearLeft.reset()

nYaw=0
yawOld = 0
gyroDirection = 0 
gyroscopeDataFile = open('param.txt', 'r')
gyroscopeParams = gyroscopeDataFile.read().split(',')
for p in range(len(gyroscopeParams)):
    gyroscopeParams[p] = int(gyroscopeParams[p])
brick.gyroscope().setCalibrationValues(gyroscopeParams)

def getYaw():
    global gyroDirection, yawOld, nYaw
    yaw = brick.gyroscope().read()[6]/1000
    deltaYaw = yaw - yawOld
    yawOld = yaw
    nYaw += -round(deltaYaw/320)
    gyroDirection = yaw + nYaw * 360
    # return gyroDirection

def go(distance, velocity):
    global gyroDirection
    dir = gyroDirection
    en = distance * cpr / (pi * wheelDiameter)
    kp = 6
    eRearLeft.reset()
    while abs(eRearLeft.read()) < en - goEps:
        err = dir - gyroDirection
        u = kp * err
        mRearLeft(velocity + u)
        mRearRight(velocity - u)
        mFrontLeft(velocity + u)
        mFrontRight(velocity - u)
        script.wait(10)

    mRearLeft(0)
    mRearRight(0)
    mFrontLeft(0)
    mFrontRight(0)

def rotate(deg):
    global gyroDirection
    beta = gyroDirection + deg
    kp = 6
    while abs(beta - gyroDirection) > rotateEps:
        err = beta - gyroDirection
        u = kp * err
        mRearLeft(u)
        mRearRight(-u)
        mFrontLeft(u)
        mFrontRight(-u)
        script.wait(10)
  
    mRearLeft(0)
    mRearRight(0)
    mFrontLeft(0)
    mFrontRight(0)


tim = script.timer(100)
tim.timeout.connect(getYaw)
script.wait(200)

def round(num):
    return int(num + (0.5 if num > 0 else -0.5))

points = []
for i, scan in enumerate(lidar.iter_scans(max_buf_meas=5000)):
    for p in scan:
        x = p[2] * math.cos(p[1])
        y = p[2] * math.sin(p[1])
        x = round(x/10)
        y = round(y/10)
        if not {'x': x, 'y': y} in points:
            points.append({'x': round(x/10), 'y': round(y/10)})
    if i > 10:
        break
print(points)

for point in points:
    brick.display().drawPoint(120+point.x, 160+point.y)
    script.wait(10)
brick.display().redraw()

lidar.stop()
lidar.stop_motor()
lidar.disconnect()