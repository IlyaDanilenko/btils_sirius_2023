import math
from rplidar import RPLidar
from copy import deepcopy

# Connect lidar
lidar = None
# RPLidar("/dev/ttyUSB0")

# Set robots's wheel diameter, cpr and eps
wheelDiameter = 10
cpr = 360
pi = math.pi
goEps = 180
rotateEps = 3
markerEps = 4

# Set map matrix and its size
localMap = []
mapSize = 60
for i in range(mapSize):
    localMap.append([])
    for j in range(mapSize):
        p = 0
        if j == 0 or j == mapSize - 1 or i == 0 or i == mapSize - 1:
            p = 1
        localMap[i].append(p)


# generate test map
def createRect(x, y, h, w):
    global localMap
    for hy in range(h):
        for wx in range(w):
            localMap[y + hy][x + wx] = 1


createRect(16, 24, 6, 20)
createRect(36, 10, 20, 6)
createRect(15, 52, 6, 20)

# Define variables for motors
mRearLeft = brick.motor(M3).setPower
mRearRight = brick.motor(M4).setPower
mFrontLeft = brick.motor(M1).setPower
mFrontRight = brick.motor(M2).setPower

# Define variables for encoders
eFrontLeft = brick.encoder(E1)
eFrontRight = brick.encoder(E2)

# Reset encoders
eFrontLeft.reset()
eFrontRight.reset()

# Set variables for saving encoders states
eFrontRightOld = 0
eFrontLeftOld = 0

# Set robot's start coordinates
x = 50
y = 215

# Define sensors
# frontSensor = brick.sensor(D1).read
# frontLeftSensor = brick.sensor(A1).read
# rearLeftSensor = brick.sensor(A2).read

# Set arrays for checking path availability
variantsX = [0, 1, 0, -1]
variantsY = [-1, 0, 1, 0]

# Set robot size (in matrix cells, 1 cell = 5x5cm with matrix size 60)
robotSize = 4

nYaw = 0
yawOld = 0
gyroDirection = 0
gyroDirectionOld = 0

# Get gyroscope calibration parameters
gyroscopeDataFile = open("param.txt", "r")
gyroscopeParams = gyroscopeDataFile.read().split(",")

# Calibrate gyroscope
for p in range(len(gyroscopeParams)):
    gyroscopeParams[p] = int(gyroscopeParams[p])
brick.gyroscope().setCalibrationValues(gyroscopeParams)


# Function to get yaw from gyroscope data and robot coordinates info from encoders data
def getOdometry():
    global gyroDirection, yawOld, nYaw, gyroDirectionOld, x, y, eFrontLeftOld, eFrontRightOld
    yaw = brick.gyroscope().read()[6] / 1000
    deltaYaw = yaw - yawOld
    yawOld = yaw
    nYaw += -round(deltaYaw / 320)
    gyroDirection = yaw + nYaw * 360
    xOld = x
    yOld = y
    x = xOld + (
        (
            (eFrontRight.read() - eFrontRightOld + eFrontLeft.read() - eFrontLeftOld)
            * pi
            * wheelDiameter
        )
        / (2 * cpr)
    ) * math.cos((gyroDirectionOld + (gyroDirection - gyroDirectionOld) / 2) * pi / 180)
    y = yOld + (
        (
            (eFrontRight.read() - eFrontRightOld + eFrontLeft.read() - eFrontLeftOld)
            * pi
            * wheelDiameter
        )
        / (2 * cpr)
    ) * math.sin((gyroDirectionOld + (gyroDirection - gyroDirectionOld) / 2) * pi / 180)
    eFrontRightOld = eFrontRight.read()
    eFrontLeftOld = eFrontLeft.read()
    gyroDirectionOld = gyroDirection


# correction by wall
# def correctW():
#    global gyroDirection
#    fsData = frontLeftSensor()
#    rsData = rearLeftSensor()
#    if fsData < 40 or rsData < 40:
#        gyroDirection = 0
#        err = rsData - fsData
#        kw = 6
#        u = kw * err
#        cEps = 1
#        while abs(frontLeftSensor() - rearLeftSensor()) > cEps:
#            mRearLeft(u)
#            mRearRight(-u)
#            mFrontLeft(u)
#            mFrontRight(-u)
#            script.wait(10)


# Function to move robot forward and backward
def go(distance, velocity=70):
    global gyroDirection
    dir = gyroDirection
    en = distance * cpr / (pi * wheelDiameter)
    kg = 6
    # kw = 8
    eStart = eFrontLeft.read()
    while abs(eStart - eFrontLeft.read()) < en - 100:
        err = dir - gyroDirection
        ug = kg * err
        # uw = 0
        # sData = frontLeftSensor()
        # if sData < 20:
        #    err = 20 - sData
        #    uw = kw * err
        # u = ug + uw
        u = ug
        mRearLeft(velocity + u)
        mRearRight(velocity - u)
        mFrontLeft(velocity + u)
        mFrontRight(velocity - u)
        script.wait(10)

    brick.motor(M3).brake()
    brick.motor(M4).brake()
    brick.motor(M1).brake()
    brick.motor(M2).brake()
    script.wait(600)


# Function to rotate robot
def rotate(deg):
    global gyroDirection
    beta = gyroDirection + deg
    kp = 3
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


# Timer to get odometry every 100ms
timGyro = script.timer(100)
timGyro.timeout.connect(getOdometry)
script.wait(200)


def round(num):
    return int(num + (0.5 if num > 0 else -0.5))


def drift(time, velocity):
    dir = gyroDirection + 90
    kg = 2.5
    while time > 0:
        err = dir - gyroDirection - 90
        ug = kg * err
        mRearLeft(-(velocity - ug))
        mRearRight(velocity - ug)
        mFrontLeft(velocity + ug)
        mFrontRight(-(velocity + ug))
        time -= 10
        script.wait(10)
    mRearLeft(0)
    mRearRight(0)
    mFrontLeft(0)
    mFrontRight(0)


# Function to make map with lidar
def scan():
    global localMap
    for i, scan in enumerate(lidar.iter_scans(max_buf_meas=5000, scan_type="express")):
        for p in scan:
            if p[1] <= 150 or p[1] >= 210:
                xCoord = p[2] * math.cos(math.radians(p[1]))
                yCoord = p[2] * math.sin(math.radians(p[1]))
                xCoord = round(x / 5 + xCoord / 50)
                yCoord = round(y / 5 + yCoord / 50)
                if (
                    xCoord < mapSize
                    and yCoord < mapSize
                    and xCoord >= 0
                    and yCoord >= 0
                ):
                    localMap[yCoord][xCoord] += 1
        if i > 5:
            break

    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print(
        "\n---------------------------------------------------------------------------------------\n"
    )
    for point in localMap:
        print(point)


# Function to find shortest path from point to point (Lee algorithm)
def findPath(x1, y1, x2, y2):
    global localMap
    for y in range(len(localMap)):
        for x in range(len(localMap[y])):
            if localMap[y][x] != 0:
                localMap[y][x] = 9999

    lMap = deepcopy(localMap)
    for y in range(1, len(localMap) - 1):
        for x in range(1, len(localMap[y]) - 1):
            if localMap[y][x] != 0:
                for cy in range(-5, 6):
                    for cx in range(-5, 6):
                        if -1 < y + cy < mapSize - 1 and -1 < x + cx < mapSize - 1:
                            if lMap[y + cy][x + cx] != 9999:
                                lMap[y + cy][x + cx] = 9999

    localMap = deepcopy(lMap)

    for cy in range(int(robotSize)):
        for cx in range(int(robotSize)):
            if localMap[y1 + cy][x1 + cx] != 0 or localMap[y2 + cy][x2 + cx]:
                return None
    step = 0
    oldWave = [[y1, x1]]
    while localMap[y2][x2] == 0:
        step += 1
        wave = []
        for i in oldWave:
            for j in range(4):
                cellX = i[1] + variantsX[j]
                cellY = i[0] + variantsY[j]
                if cellX != x1 or cellY != y1:
                    cellsFree = 0
                    for k in range(int(robotSize)):
                        for f in range(int(robotSize)):
                            if k == 0 and f == 0:
                                if localMap[cellY + k][cellX + f] == 0:
                                    cellsFree += 1
                            else:
                                if localMap[cellY + k][cellX + f] != 9999:
                                    cellsFree += 1
                    if cellsFree == robotSize**2:
                        wave.append([cellY, cellX])
                        localMap[cellY][cellX] = step
        oldWave = wave

    cellX = x2
    cellY = y2
    wave = [[x2, y2]]
    c = localMap[y2][x2]
    while c > 0:
        for j in range(4):
            nx = cellX + variantsX[j]
            ny = cellY + variantsY[j]
            if c > 1:
                if localMap[cellY][cellX] - 1 == localMap[ny][nx]:
                    cellX = nx
                    cellY = ny
                    wave.insert(0, [cellX, cellY])
                    c -= 1
                    break
            else:
                wave.insert(0, [x1, y1])
                c -= 1
                break
    ang = 1
    path = []
    dist = 0
    for p in range(1, len(wave)):
        if (
            wave[p][0] == wave[p - 1][0] + variantsX[ang]
            and wave[p][1] == wave[p - 1][1] + variantsY[ang]
        ):
            dist += 1
        else:
            for v in range(4):
                if (
                    wave[p][0] == wave[p - 1][0] + variantsX[v]
                    and wave[p][1] == wave[p - 1][1] + variantsY[v]
                ):
                    oldAng = ang
                    ang = v
                    print(v)
                    path.append([go, dist * 5])
                    dist = 0
                    deg = 0
                    if oldAng < ang or (oldAng == 3 and ang == 0):
                        deg = 90
                    elif oldAng > ang or (oldAng == 0 and ang == 3):
                        deg = -90
                    path.append([rotate, deg])
    path.append([go, dist * 5])
    return path


def correctByMarker():
    k = 1.8
    while abs(brick.lineSensor("video2").read()[0]) > markerEps:
        err = brick.lineSensor("video2").read()[0]
        u = k * err
        drift(30, u)


# define video sensor
# brick.configure("video2", "lineSensor")
# brick.lineSensor("video2").init(True)

# while not brick.keys().wasPressed(KeysEnum.Down):
#    script.wait(100)

# brick.lineSensor("video2").detect()

# while not brick.keys().wasPressed(KeysEnum.Up):
#    script.wait(100)

path = findPath(8, 43, 53, 15)
for move in path:
    move[0](move[1])
rotate(90)
mailbox.send(2, "get box")
msg1 = mailbox.receive()
if msg1 == "box in":
    go(distance=35, velocity=-70)
    mailbox.send(2, "lift up")
msg2 = mailbox.receive()
if msg2 == "box out":
    go(100)
    mailbox.send(2, "lower down")
msg3 = mailbox.receive()
if msg3 == "ready":
    print("ok")
    rotate(90)
    go(distance=130, velocity=-70)
    # move to start point
    # for step in range(len(path) - 1, -1, -1):
    #    if path[step][0] == rotate:
    #        path[step][0](-path[step][1])
    #    else:
    #        path[step][0](path[step][1])
