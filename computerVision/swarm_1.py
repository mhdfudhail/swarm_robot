# final orientation towards enemy
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import udpSocket
from math import sqrt, pow, cos, sin, radians, atan2


frameWidth = 1280
frameHeight = 720
PID = [0.5,0,0.5]
PIDL = [0.55]
prErrorx = 0
prErrory = 0
prErrorz = 0
plError = 0
arucoIds = [70, 130]

swarmList=[('192.168.100.122', 9696),('192.168.100.123', 9696),('192.168.100.124', 9696)]

def sendPacket(message, swarmId):
    global swarmList
    if(swarmId==50):
        # address = ('192.168.100.124', 9696)
        udpSocket.sock.sendto(message.encode(),swarmList[2])
    elif(swarmId==70):
        # address = ('192.168.100.123', 9696)
        udpSocket.sock.sendto(message.encode(),swarmList[1])
    elif(swarmId==90):
        # address = ('192.168.100.122', 9696)
        udpSocket.sock.sendto(message.encode(),swarmList[0])
    else:
        for i in swarmList:
            udpSocket.sock.sendto(message.encode(),i)


def findArucoMarker(img, markerSize=6, totalMarker=250, draw=True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParam = aruco.DetectorParameters_create()
    bbx, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)

    if draw:
        aruco.drawDetectedMarkers(img, bbx)

    return [bbx, ids]

def generateTriangle(img, marker, arucoId):
    tx,ty,bx,by = marker[arucoId][0],marker[arucoId][1],marker[arucoId][4],marker[arucoId][5]

    # centroid of aruco marker
    cx = int((tx+bx)//2)
    cy = int((ty+by)//2)
    cv2.circle(img, (cx,cy),2, (255,0,0),2)


    # Calculate centroid
    centroid_x = cx
    centroid_y = cy

    
    triangle_side_length = 200  

    # Calculate vertex positions 
    vertex1_x = centroid_x + int(triangle_side_length * cos(radians(270)))
    vertex1_y = centroid_y + int(triangle_side_length * sin(radians(270)))
    vertex2_x = centroid_x + int(triangle_side_length * cos(radians(150)))
    vertex2_y = centroid_y + int(triangle_side_length * sin(radians(150)))
    vertex3_x = centroid_x + int(triangle_side_length * cos(radians(30)))
    vertex3_y = centroid_y + int(triangle_side_length * sin(radians(30)))

    # Draw triangle
    cv2.line(img, (vertex3_x, vertex3_y), (vertex1_x, vertex1_y), (0, 255, 0), 1)
    cv2.line(img, (vertex1_x, vertex1_y), (vertex2_x, vertex2_y), (0, 255, 0), 1)
    cv2.line(img, (vertex2_x, vertex2_y), (vertex3_x, vertex3_y), (0, 255, 0), 1)

    cv2.circle(img, (vertex1_x, vertex1_y),5, (0,0,255),3)
    cv2.circle(img, (vertex2_x, vertex2_y),5, (0,0,255),3)
    cv2.circle(img, (vertex3_x, vertex3_y),5, (0,0,255),3)
    p1 = [vertex1_x, vertex1_y]
    p2 = [vertex2_x, vertex2_y]
    p3 = [vertex3_x, vertex3_y]
    cntr = [cx, cy]

    return p1, p2, p3, cntr

def arucoPositionTarget(img, marker, arucoId, X_target, Y_target, enemyCxy): #enemy centroid
    print(f"vehicle Id: {arucoId}")
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
    cv2.line(img,(cx,cy),(X_target,Y_target),(255,0,0),2)
    # calculating distance between points
    # X_target,Y_target=640,360

    l_distance = sqrt(pow((X_target-tx),2)+pow((Y_target-ty),2))
    c_distance = sqrt(pow((X_target-cx),2)+pow((Y_target-cy),2))
    r_distance = sqrt(pow((X_target-t1),2)+pow((Y_target-t2),2))
    
    if (c_distance>50):
        if(arucoId==50):
            control_movement_x(r_distance,l_distance,c_distance, 0, arucoId)
        elif(arucoId==70):
            control_movement_y(r_distance,l_distance,c_distance, 0, arucoId)
        elif(arucoId==90):
            control_movement_z(r_distance,l_distance,c_distance, 0, arucoId)
        
        
        # deltaX = cx-enemyCxy[0]
        # deltaY = cy-enemyCxy[1]
        # angle = atan2(deltaY, deltaX)
        # print("angle: ", angle)
    else:
        c_d = sqrt(pow((enemyCxy[0]-cx),2)+pow((enemyCxy[1]-cy),2))
        l_d = sqrt(pow((enemyCxy[0]-tx),2)+pow((enemyCxy[1]-ty),2))
            # c_d = sqrt(pow((enemyCxy[0]-cx),2)+pow((enemyCxy[1]-cy),2))
        r_d = sqrt(pow((enemyCxy[0]-t1),2)+pow((enemyCxy[1]-t2),2))
        difference = -1*(l_d-r_d)
        print("Differance: ", difference)
        if(c_distance<50 and difference>7 or difference<-7):
            print("need correction")
            
        #     # linear velocity must be zero, just correcting orientation.
            if(arucoId==50):
                control_movement_x(r_d, l_d, c_d, 0, arucoId)
            elif(arucoId==70):
                control_movement_y(r_d, l_d, c_d, 0, arucoId)
            elif(arucoId==90):
                control_movement_z(r_d, l_d, c_d, 0, arucoId)
            # pass
        else:
            print("------Target Locked-------")
            sendPacket(message=f"{10},{0},{10},{10}", swarmId=arucoId)
    # print(f"R-dist: {r_distance}, L-dist: {l_distance}, C-Dist: {c_distance}")

    # generate_control(640,360,cx,cy)
def control_movement_x(d_right, d_left, d_center,correctionFlag, id):
    global prErrorx,plError
    # print(f"Distance: {d_center}")
    # complete opposite towards the goal
    if(d_center<d_right and d_center<d_left):
        print("opposit towards the goal")
        if (d_right>d_left):
            # rotate robot left
            print("rotate left")  
            sendPacket(message=f"{0},{0},{-1},{correctionFlag}", swarmId=id)
        else:
            print("rotate right")

            sendPacket(message=f"{0},{0},{1},{correctionFlag}", swarmId=id)
    # head towards the goal 
    else:
        if (d_right<d_left):
            rotationError  = d_left-d_right
            linearError = d_center
            linearVel = PIDL[0]*linearError
            rightRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prErrorx)
            print(f"linear vel: {-1*linearVel}, Right vel: {rightRotateVel}")
            sendPacket(message=f"{-1*linearVel},{rightRotateVel},{correctionFlag},,{correctionFlag}",swarmId=id)
            prErrorx = rotationError

        # elif(d_right>d_left):
        else:
            rotationError = d_right-d_left
            linearError = d_center
            linearVel = PIDL[0]*linearError
            leftRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prErrorx)
            print(f"linear vel: {-1*linearVel}, Left vel: {-1*leftRotateVel}")
            sendPacket(message=f"{-1*linearVel},{-1*leftRotateVel},{correctionFlag},{correctionFlag}",swarmId=id)
            prErrorx = rotationError
def control_movement_y(d_right, d_left, d_center,correctionFlag, id):
    global prErrory,plError
    # print(f"Distance: {d_center}")
    # complete opposite towards the goal
    if(d_center<d_right and d_center<d_left):
        print("opposit towards the goal")
        if (d_right>d_left):
            # rotate robot left
            print("rotate left")  
            sendPacket(message=f"{0},{0},{-1},{correctionFlag}", swarmId=id)
        else:
            print("rotate right")

            sendPacket(message=f"{0},{0},{1},{correctionFlag}", swarmId=id)
    # head towards the goal 
    else:
        if (d_right<d_left):
            rotationError  = d_left-d_right
            linearError = d_center
            linearVel = PIDL[0]*linearError
            rightRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prErrory)
            print(f"linear vel: {-1*linearVel}, Right vel: {rightRotateVel}")
            sendPacket(message=f"{-1*linearVel},{rightRotateVel},{correctionFlag},,{correctionFlag}",swarmId=id)
            prErrory = rotationError

        # elif(d_right>d_left):
        else:
            rotationError = d_right-d_left
            linearError = d_center
            linearVel = PIDL[0]*linearError
            leftRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prErrory)
            print(f"linear vel: {-1*linearVel}, Left vel: {-1*leftRotateVel}")
            sendPacket(message=f"{-1*linearVel},{-1*leftRotateVel},{correctionFlag},{correctionFlag}",swarmId=id)
            prErrory = rotationError
def control_movement_z(d_right, d_left, d_center,correctionFlag, id):
    global prErrorz,plError
    # print(f"Distance: {d_center}")
    # complete opposite towards the goal
    if(d_center<d_right and d_center<d_left):
        print("opposit towards the goal")
        if (d_right>d_left):
            # rotate robot left
            print("rotate left")  
            sendPacket(message=f"{0},{0},{-1},{correctionFlag}", swarmId=id)
        else:
            print("rotate right")

            sendPacket(message=f"{0},{0},{1},{correctionFlag}", swarmId=id)
    # head towards the goal 
    else:
        if (d_right<d_left):
            rotationError  = d_left-d_right
            linearError = d_center
            linearVel = PIDL[0]*linearError
            rightRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prErrorz)
            print(f"linear vel: {-1*linearVel}, Right vel: {rightRotateVel}")
            sendPacket(message=f"{-1*linearVel},{rightRotateVel},{correctionFlag},,{correctionFlag}",swarmId=id)
            prErrorz = rotationError

        # elif(d_right>d_left):
        else:
            rotationError = d_right-d_left
            linearError = d_center
            linearVel = PIDL[0]*linearError
            leftRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prErrorz)
            print(f"linear vel: {-1*linearVel}, Left vel: {-1*leftRotateVel}")
            sendPacket(message=f"{-1*linearVel},{-1*leftRotateVel},{correctionFlag},{correctionFlag}",swarmId=id)
            prErrorz = rotationError



def main():
    global arucoIds
    udpSocket.init()

    cap = cv2.VideoCapture(0)
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)
    markerDict = {}
    swarms = [70,90,50]
    
    while(cap.isOpened()): 
        while True:
            ret, img = cap.read() 
            arucoFound = findArucoMarker(img)
            if len(arucoFound[0])!=0:
                # for i in arucoFound[1]:
                #     if 
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


                for key in list(markerDict.keys()):
                    if key not in [id[0] for id in arucoFound[1]]:
                        del markerDict[key]

            #  <<<<<<<<<<<<<<<<<<<<< back shifted
                # detecting enemy-aruco and generating triangle coordinates
                if 130 in markerDict.keys():
                    p1,p2,p3,cntr = generateTriangle(img, markerDict, 130)
                    # triangulating target swarm robot_1
                    if swarms[0] in markerDict.keys():
                        arucoPositionTarget(img, markerDict, swarms[0],p1[0],p1[1],cntr)
                    if swarms[1] in markerDict.keys():
                        arucoPositionTarget(img, markerDict, swarms[1] ,p2[0],p2[1],cntr)
                    if swarms[2] in markerDict.keys():
                        arucoPositionTarget(img, markerDict, swarms[2] ,p3[0],p3[1],cntr)
                else:
                    print("target not found!")
                    sendPacket(message=f"{0},{0},{0},{0}",swarmId=-1)


            # print(markerDict)
            cv2.imshow('img', img)
             
            if cv2.waitKey(30) & 0xff == ord('q'): 
                break
                
        cap.release() 
        cv2.destroyAllWindows() 
    else: 
        print("Camera disconnected") 

if __name__== '__main__':
    main()