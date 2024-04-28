import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math

frameWidth = 1280
frameHeight = 720

def generateTriangle(img, marker, arucoId):
    tx,ty,bx,by = marker[arucoId][0],marker[arucoId][1],marker[arucoId][4],marker[arucoId][5]
    t1,t2 = marker[arucoId][2],marker[arucoId][3]
    
    # centroid of aruco marker
    cx = int((tx+bx)//2)
    cy = int((ty+by)//2)
    cv2.circle(img, (cx,cy),2, (255,0,0),2)


    # Calculate centroid
    centroid_x = cx
    centroid_y = cy

    # Calculate triangle side length based on marker size (adjust as needed)
    triangle_side_length = 200  # Example: half the marker size

    # Calculate vertex positions (adjust angles for desired orientation)
    vertex1_x = centroid_x + int(triangle_side_length * math.cos(math.radians(270)))
    vertex1_y = centroid_y + int(triangle_side_length * math.sin(math.radians(270)))
    vertex2_x = centroid_x + int(triangle_side_length * math.cos(math.radians(150)))
    vertex2_y = centroid_y + int(triangle_side_length * math.sin(math.radians(150)))
    vertex3_x = centroid_x + int(triangle_side_length * math.cos(math.radians(30)))
    vertex3_y = centroid_y + int(triangle_side_length * math.sin(math.radians(30)))

    # Draw triangle
    # cv2.line(img, (centroid_x, centroid_y), (vertex1_x, vertex1_y), (0, 255, 0), 1)
    # cv2.line(img, (centroid_x, centroid_y), (vertex2_x, vertex2_y), (0, 255, 0), 1)
    # cv2.line(img, (centroid_x, centroid_y), (vertex3_x, vertex3_y), (0, 255, 0), 1)

    cv2.circle(img, (vertex1_x, vertex1_y),5, (0,0,255),3)
    cv2.circle(img, (vertex2_x, vertex2_y),5, (0,0,255),3)
    cv2.circle(img, (vertex3_x, vertex3_y),5, (0,0,255),3)


def findArucoMarker(img, markerSize=6, totalMarker=250, draw=True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParam = aruco.DetectorParameters_create()
    bbx, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)

    if draw:
        aruco.drawDetectedMarkers(img, bbx)

    return [bbx, ids]

def main():
    cap = cv2.VideoCapture(0)
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)
    markerDict = {}
    targets = [(0,0),(0,0),(0,0)]
    while(cap.isOpened()): 
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

                    # detecting target and generating triangle coordinates
                    if 130 in markerDict.keys():
                        generateTriangle(img, markerDict, 130)

            print(markerDict)
            cv2.imshow('img', img)
             
            if cv2.waitKey(30) & 0xff == ord('q'): 
                break
                
        cap.release() 
        cv2.destroyAllWindows() 
    else: 
        print("Camera disconnected") 

if __name__== '__main__':
    main()