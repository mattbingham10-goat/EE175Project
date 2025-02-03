import math
def manualCoordinateMode(xCoordOld, yCoordOld, xCoordNew, yCoordNew):
    xMove = (xCoordNew - xCoordOld)
    yMove = (yCoordNew - yCoordOld)

    distance = math.sqrt((xMove**2 + yMove**2))
    angle = math.atan2(yMove,xMove) * (180/math.pi)

    return angle, distance

# MANUAL IMPLIMENTATION OF COORDS, LIKELEY OLD INITIALIZED TO 0 AND NEW IS GIVEN EACH TIME
xCoordOld = 3
yCoordOld = 1
xCoordNew = 3
yCoordNew = 6

angle, distance = manualCoordinateMode(xCoordOld, yCoordOld, xCoordNew, yCoordNew)
#MUST UPDATE OLD COORDS TO BE NEW COORDS ON PASS THROUGH

print(angle)
print(' ')
print(distance)