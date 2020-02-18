import serial # Library needed to communicate with arduino

import struct # Used for reading floats in 4 bytes
import time

ser = serial.Serial('/dev/ttyACM0', 9600)
print(ser.name)


def anglesToArduino(angles):
    ser.write(b'\x01') # Tell arduino we will order positions
    for angle in angles:
        ser.write((angle).to_bytes(1, "big"))   
    ser.write(b'\x02') # Tell we finished (arduino will check this as end-code)

def reset():
    ser.write(b'\x03') # Tell arduino to reset
    while not (ser.in_waiting>0):
        pass
    response = ser.read()
    print(response)
    if(response == b'\x04'):
        print("Reset successful ")
    else:
        print("Reset failed")

def getJointsStates():
    ser.write(b'\x00') # Ask for joint angles
    response = ser.read_until(b'\xFF') # 0xFF used as end-code
    if(response[-1] == 255):
        return response[:-1]

def getBalanceAndMovement():
    ser.write(b'\x05') # Tell arduino to send balance and movement data
    xAxis = ser.read(4)
    yAxis = ser.read(4)
    
    xMovement = ser.read()
    yMovement = ser.read()
    return xAxis, yAxis, xMovement, yMovement

def testGets():
    jointStates = getJointsStates()
    xAx, yAx, xMov, yMov = getBalanceAndMovement()
    
    xAx = struct.unpack('f', xAx)[0]
    yAx = struct.unpack('f', yAx)[0]
    print(jointStates)

    print("Joint states {}".format(list(jointStates)))

    print("Axis {}, {}".format(xAx, yAx))
    print("Mov {}, {}".format(int.from_bytes(xMov, "big"), int.from_bytes(yMov, "big")))

    testWriteJoint()

def testWriteJoint():
    anglesToWrite = [0,0,0,0,5,5,5,5,180,180,180,180]
    anglesToArduino(anglesToWrite)
    time.sleep(0.2)
    jointStates = getJointsStates()
    print("Joint angles after write {}".format(list(jointStates)))
