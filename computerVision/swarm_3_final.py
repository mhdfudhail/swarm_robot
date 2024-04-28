# final orientation towards enemy
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import udpSocket
from math import sqrt, pow, cos, sin, radians


frameWidth = 1280
frameHeight = 720
PID = [0.5,0,0.5]
PIDL = [0.55]
prError50, prError70, prError90 = 0, 0, 0
plError = 0
arucoIds = [70, 130]

swarmList=[0,0,0]
# swarmList=[('192.168.100.122', 9696),('192.168.100.123', 9696),('192.168.100.124', 9696)]

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
    
    # X_target, Y_target = xyTarget[0],xyTarget[1]
    tx,ty,bx,by = marker[arucoId][0],marker[arucoId][1],marker[arucoId][4],marker[arucoId][5]
    t1,t2 = marker[arucoId][2],marker[arucoId][3]
    cv2.circle(img, (t1,t2),3, (0,0,255),3)
    cv2.circle(img, (tx,ty),3, (0,0,255),3)
    # print(f"X:{tx}, Y:{ty}")
    # find centroid of aruco marker
    cx = int((tx+bx)//2)
    cy = int((ty+by)//2)
    print(f"vehicle Id: {arucoId} -- {cx,cy}-- {X_target, Y_target}")
    # print(f"cx:{cx}, cy:{cy}")
    cv2.circle(img, (cx,cy),2, (255,0,0),2)
    cv2.line(img,(cx,cy),(X_target,Y_target),(255,0,0),2)
    # calculating distance between points
    # X_target,Y_target=640,360

    l_distance = sqrt(pow((X_target-tx),2)+pow((Y_target-ty),2))
    c_distance = sqrt(pow((X_target-cx),2)+pow((Y_target-cy),2))
    r_distance = sqrt(pow((X_target-t1),2)+pow((Y_target-t2),2))
    
    if (c_distance>30):
        control_movement(r_distance,l_distance,c_distance, 0, arucoId)
        
    else:
        c_d = sqrt(pow((enemyCxy[0]-cx),2)+pow((enemyCxy[1]-cy),2))
        l_d = sqrt(pow((enemyCxy[0]-tx),2)+pow((enemyCxy[1]-ty),2))
            # c_d = sqrt(pow((enemyCxy[0]-cx),2)+pow((enemyCxy[1]-cy),2))
        r_d = sqrt(pow((enemyCxy[0]-t1),2)+pow((enemyCxy[1]-t2),2))
        difference = -1*(l_d-r_d)
        # print("Differance: ", difference)
        if(c_distance<60 and difference>7 or difference<-7):
            print("need correction")
            control_movement(r_d, l_d, c_d, 0, arucoId)

        # linear velocity must be zero, just correcting orientation.
            
        else:
            print("------Target Locked-------")
            sendPacket(message=f"{10},{0},{10},{10}", swarmId=arucoId)
    # print(f"R-dist: {r_distance}, L-dist: {l_distance}, C-Dist: {c_distance}")

    # generate_control(640,360,cx,cy)
def control_movement(d_right, d_left, d_center,correctionFlag, id):
    global prError50, prError70, prError90, plError
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
        # id:50
        if (id==50 and d_right<d_left):
            rotationError  = d_left-d_right
            linearError = d_center
            linearVel = PIDL[0]*linearError
            rightRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prError50)
            # print(f"linear vel: {-1*linearVel}, Right vel: {rightRotateVel}")
            sendPacket(message=f"{-1*linearVel},{rightRotateVel},{correctionFlag},,{correctionFlag}",swarmId=id)
            prError50 = rotationError
# else:
        elif(id==50 and d_right>d_left):
            rotationError = d_right-d_left
            linearError = d_center
            linearVel = PIDL[0]*linearError
            leftRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prError50)
            # print(f"linear vel: {-1*linearVel}, Left vel: {-1*leftRotateVel}")
            sendPacket(message=f"{-1*linearVel},{-1*leftRotateVel},{correctionFlag},{correctionFlag}",swarmId=id)
            prError50 = rotationError
        
        # id:70
        if (id==70 and d_right<d_left):
            rotationError  = d_left-d_right
            linearError = d_center
            linearVel = PIDL[0]*linearError
            rightRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prError70)
            # print(f"linear vel: {-1*linearVel}, Right vel: {rightRotateVel}")
            sendPacket(message=f"{-1*linearVel},{rightRotateVel},{correctionFlag},,{correctionFlag}",swarmId=id)
            prError70 = rotationError

        elif(id==70 and d_right>d_left):
            rotationError = d_right-d_left
            linearError = d_center
            linearVel = PIDL[0]*linearError
            leftRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prError70)
            # print(f"linear vel: {-1*linearVel}, Left vel: {-1*leftRotateVel}")
            sendPacket(message=f"{-1*linearVel},{-1*leftRotateVel},{correctionFlag},{correctionFlag}",swarmId=id)
            prError70 = rotationError

        # id:90
        if (id==90 and d_right<d_left):
            rotationError  = d_left-d_right
            linearError = d_center
            linearVel = PIDL[0]*linearError
            rightRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prError90)
            # print(f"linear vel: {-1*linearVel}, Right vel: {rightRotateVel}")
            sendPacket(message=f"{-1*linearVel},{rightRotateVel},{correctionFlag},,{correctionFlag}",swarmId=id)
            prError90 = rotationError
            
        elif(id==90 and d_right>d_left):
            rotationError = d_right-d_left
            linearError = d_center
            linearVel = PIDL[0]*linearError
            leftRotateVel = PID[0]*rotationError+PID[2]*(rotationError-prError90)
            # print(f"linear vel: {-1*linearVel}, Left vel: {-1*leftRotateVel}")
            sendPacket(message=f"{-1*linearVel},{-1*leftRotateVel},{correctionFlag},{correctionFlag}",swarmId=id)
            prError90 = rotationError

def getPosition(img, marker, arucoId): #enemy centroid
    global coFlagx, coFlagy, coFlagz
    # print(f"vehicle Id: {arucoId}")
    tx,ty,bx,by = marker[arucoId][0],marker[arucoId][1],marker[arucoId][4],marker[arucoId][5]
    t1,t2 = marker[arucoId][2],marker[arucoId][3]
    cv2.circle(img, (t1,t2),3, (0,0,255),3)
    cv2.circle(img, (tx,ty),3, (0,0,255),3)

    cx = int((tx+bx)//2)
    cy = int((ty+by)//2)
    # print(f"cx:{cx}, cy:{cy}")
    cv2.circle(img, (cx,cy),2, (255,0,0),2)

    return cx,cy, arucoId

def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# nearest corner algorithm
def find_nearest_corner(corners, robots):
    corner_assigned = {}
    dist_list = []
    robot_list=robots.copy()
    k=0
    for i in corners:
        
        for j in robots:
            dist_list.append(distance(i,j))
        # print("min dist : ",robots[dist_list.index(min(dist_list))])
        
        if len(robots)>=1:
            corner_assigned[k]=robots[dist_list.index(min(dist_list))]
            robots.pop(dist_list.index(min(dist_list)))
            dist_list.clear()
        else:
            corner_assigned[k]=robot_list
        k+=1   
        # if 
            
    
    print(corner_assigned)
    return corner_assigned


def main():
    global arucoIds, swarmList
    udpSocket.init()

    cap = cv2.VideoCapture(0)
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)
    markerDict = {}
    swarms = [70,90,50]
    robot_positions=[]
    robot_positions_ids=[]
    id50,id70,id90 = False,False,False
    # nearest_dict = {}
    
    while(cap.isOpened()): 
        try:
            ip = udpSocket.get_ip()
            print(f"System IP: {ip}.")

            print("System is ready to pair...")
            while True:
                # Fetching Ip from Robots
                data, addr = udpSocket.sock.recvfrom(1024)
                print("received message: {} from {}\n".format(data[1:-1],addr))
                if(int(data)==50):
                    id50=True
                    swarmList[2]=addr
                    print(f"Robot ID: {data}  ip: {addr}")
                elif(int(data)==70):
                    id70=True
                    swarmList[1]=addr
                    print(f"Robot ID: {data}  ip: {addr}")
                elif(int(data)==90):
                    id90=True
                    swarmList[0]=addr
                    print(f"Robot ID: {data}  ip: {addr}")

                if (id50 and id70 and id90):
                    print(f"Swarm Ip list: {swarmList}")
                    break
        except:
            # print(f"---Camera : {cap.isOpened()}---")
            print("---No network Connection!---")

        while True:
            ret, img = cap.read() 
            arucoFound = findArucoMarker(img)
            if len(arucoFound[0])!=0:
                
                for bbx, ids in zip(arucoFound[0], arucoFound[1]):
                    idCord = [bbx]
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

            #
                # detecting enemy-aruco and generating triangle coordinates
                if 130 in markerDict.keys():
                    p1,p2,p3,cntr = generateTriangle(img, markerDict, 130)
                    if len(robot_positions)==0:
                        if (swarms[0] in markerDict.keys() and swarms[1] in markerDict.keys() and swarms[2] in markerDict.keys()):
                            for i in markerDict.keys():
                                if i !=130:
                                    posx,posy, id = getPosition(img,markerDict,i)
                                    robot_positions.append([posx, posy])
                                    robot_positions_ids.append(id)
                    print("Robot position: ",robot_positions)            
                    corners = [p1, p2, p3]
                    print("Corners: ",corners)
                    if(len(corners)==3 and len(robot_positions)==3):
                        print("Generating nearest corner")
                        robot_positions_copy = robot_positions.copy()
                        dict = find_nearest_corner(corners, robot_positions)
                        flag= False
                        for i in dict.keys():
                            cv2.line(img, tuple(corners[i]), tuple(dict[i]), (0, 255, 0), 2)
                            if dict[i] in robot_positions_copy:
                                # print(robot_positions_ids[robot_positions_copy.index(dict[i])], i)
                                x=robot_positions_ids[robot_positions_copy.index(dict[i])]
                                # print(x, dict[i])
                                # nearest_dict[x] = dict[i] 
                                if x in markerDict.keys():
                                    # cv2.line(img,(cntr[0],cntr[1]),(),(255,0,0),2)
                                    arucoPositionTarget(img, markerDict, x ,corners[i][0],corners[i][1],cntr)
                                if x in markerDict.keys():
                                    arucoPositionTarget(img, markerDict, x ,corners[i][0],corners[i][1],cntr)
                                if x in markerDict.keys():
                                    arucoPositionTarget(img, markerDict, x ,corners[i][0],corners[i][1],cntr)
                                
                                # arucoPositionTarget(img, markerDict, x, dict[i],cntr)
                    # triangulating target swarm robot_1
                    # print("-----",dict)
                    # print(swarms[0], p1)
                    # print(swarms[0],nearest_dict[swarms[0]][1])
                    # break
                else:
                    print("target not found!")
                    sendPacket(message=f"{0},{0},{0},{0}",swarmId=-1)

            robot_positions.clear()
            robot_positions_ids.clear()
            # robot_positions_copy.clear()
            # print(markerDict)
            cv2.imshow('img', img)
             
            if cv2.waitKey(30) & 0xff == ord('q'): 
                break
                
        cap.release() 
        cv2.destroyAllWindows() 
    else: 
        print("Camera disconnected!") 

if __name__== '__main__':
    main()