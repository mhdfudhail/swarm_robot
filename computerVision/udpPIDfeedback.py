import cv2
import cv2.aruco as aruco
import numpy as np
import time
import udpSocket
from math import sqrt, pow

frameWidth = 1280
frameHeight = 720
PID = [0.5,0,0.5]
PIDL = [0.55]
prError = 0
plError = 0


def sendPacket(message):
    address = ('192.168.100.123', 9696)
    udpSocket.sock.sendto(message.encode(),address)

def findArucoMarker(img, markerSize=6, totalMarker=250, draw=True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParam = aruco.DetectorParameters_create()
    bbx, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)

    if draw:
        aruco.drawDetectedMarkers(img, bbx)

    return [bbx, ids]

def checkArucoPosition(img, marker, arucoId):
    # print(marker[10])
    
    
    tx,ty,bx,by = marker[arucoId][0],marker[arucoId][1],marker[arucoId][4],marker[arucoId][5]
    t1,t2 = marker[arucoId][2],marker[arucoId][3]
    cv2.circle(img, (t1,t2),3, (0,0,255),3)
    cv2.circle(img, (tx,ty),3, (0,0,255),3)
    # print(f"X:{tx}, Y:{ty}")
    # find centroid of aruco marker
    cx = int((tx+bx)//2)
    cy = int((ty+by)//2)
    # print(f"cx:{cx}, cy:{cy}")
    cv2.circle(img, (cx,cy),2, (255,0,0),2)
    cv2.line(img,(cx,cy),(640,360),(255,0,0),5)
    # calculating distance between points
    X_target,Y_target=640,360
    l_distance = sqrt(pow((X_target-tx),2)+pow((Y_target-ty),2))
    c_distance = sqrt(pow((X_target-cx),2)+pow((Y_target-cy),2))
    r_distance = sqrt(pow((X_target-t1),2)+pow((Y_target-t2),2))
    
    if c_distance>50:
        control_movement(r_distance,l_distance,c_distance)
    # print(f"R-dist: {r_distance}, L-dist: {l_distance}, C-Dist: {c_distance}")

    # generate_control(640,360,cx,cy)
def control_movement(d_right, d_left, d_center):
    global prError,plError
    print(f"Distance: {d_center}")
    # complete opposite towards the goal
    if(d_center<d_right and d_center<d_left):
        print("opposit towards the goal")
        if (d_right>d_left):
            # rotate robot left
            sendPacket(message=f"{10},{10}")
            print("rotate left")  
        else:
            print("rotate right")
            sendPacket(message=f"{10},{10}")
    # head towards the goal 
    else:
        if (d_right<d_left):
            rotationError  = d_left-d_right
            linearError = d_center
            linearVel = PIDL[0]*linearError
            rightRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prError)
            print(f"linear vel: {-1*linearVel}, Right vel: {rightRotateVel}")
            sendPacket(message=f"{-1*linearVel},{rightRotateVel}")
            prError = rotationError

        elif(d_right>d_left):
            rotationError = d_right-d_left
            linearError = d_center
            linearVel = PIDL[0]*linearError
            leftRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prError)
            print(f"linear vel: {-1*linearVel}, Left vel: {-1*leftRotateVel}")
            sendPacket(message=f"{-1*linearVel},{-1*leftRotateVel}")
            prError = rotationError

def generate_control(X_target, Y_target, X_current, Y_current):
    X_error = X_target - X_current
    Y_error = Y_target - Y_current
    # calculating distances

    # Adjust these gains to control robot speed and smoothness
    linear_gain = 0.052
    angular_gain = 3.94

    # Calculate desired linear and angular velocities
    linear_vel = linear_gain * np.linalg.norm([X_error, Y_error])
    angular_vel = angular_gain * (Y_error * X_current - X_error * Y_current) / (X_current**2 + Y_current**2)

    print(f"linear: {linear_vel}, angular: {angular_vel}")
    message = f"{round(linear_vel,3)},{round(angular_vel,3)}"
    # sendPacket(message)



def main():
    udpSocket.init()

    cap = cv2.VideoCapture(0)
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)
    markerDict = {}
    while(cap.isOpened()): 
        while True:
            ret, img = cap.read() 
            arucoFound = findArucoMarker(img)
            if len(arucoFound[0])!=0:
                for bbx, ids in zip(arucoFound[0], arucoFound[1]):
                    idCord = [bbx]
                    # print(f"full cord: {idCord}")
                    # print("fist:",bbx[0][0])
                    # print("second:",bbx[0][1])
                    idNum = ids[0]
                    idCord = [int(bbx[0][0][0]),
                                int(bbx[0][0][1]),
                                int(bbx[0][1][0]),
                                int(bbx[0][1][1]),
                                int(bbx[0][2][0]),
                                int(bbx[0][2][1])]
                    # print(idCord)
                    marker={idNum:idCord}
                    markerDict.update(marker)
                    if 70 in markerDict.keys():
                        checkArucoPosition(img, markerDict, 70)
                    if 130 in markerDict.keys():
                        checkArucoPosition(img, markerDict, 130)

            # print(markerDict)
            cv2.imshow('img', img)
             
            if cv2.waitKey(30) & 0xff == ord('q'): 
                break
                
        cap.release() 
        cv2.destroyAllWindows() 
    else: 
        print("Alert ! Camera disconnected") 

if __name__== '__main__':
    main()