import cv2
import numpy as np
import random as rng
import time

from ..interop import DetectedObject, ObjectType

def Initialize():
    cap = cv2.VideoCapture(0)
    # Check if camera opened successfully

    if (cap.isOpened() == False):
        print("Error opening video stream or file")
    return cap


def ObjectDetection(cap):
    ret, frame = cap.read()
    objects = []
    if ret == True:
            newtime = time.time()
            # Display the resulting frame
            # image = cv2.imread("image.png")
            # hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            frame = cv2.flip(frame,0)
            hsv_stream = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            

            ORANGE_MIN = np.array([170,50,50])
            ORANGE_MAX = np.array([179, 255, 255])

            ORANGE_MIN1 = np.array([0,50,50])
            ORANGE_MAX1 = np.array([5, 255, 255])

            BLUE_MIN = np.array([100,30,30])
            BLUE_MAX = np.array([110,255,255])

            GREEN_MIN = np.array([35,30,30])
            GREEN_MAX = np.array([50,255,255])

            YELLOW_MIN = np.array([25,100,100])
            YELLOW_MAX = np.array([30,255,255])

            BLACK_MIN = np.array([0,0,0])
            BLACK_MAX = np.array([0,0,0])

            hsv_thresholded_orange1 = cv2.inRange(hsv_stream, ORANGE_MIN, ORANGE_MAX)
            hsv_thresholded_orange2 = cv2.inRange(hsv_stream, ORANGE_MIN1, ORANGE_MAX1)
            hsv_thresholded_orange = hsv_thresholded_orange1 + hsv_thresholded_orange2
            
            hsv_thresholded_blue = cv2.inRange(hsv_stream, BLUE_MIN, BLUE_MAX)
            
            hsv_thresholded_green = cv2.inRange(hsv_stream, GREEN_MIN, GREEN_MAX)

            hsv_thresholded_yellow = cv2.inRange(hsv_stream, YELLOW_MIN, YELLOW_MAX)
            hsv_thresholded_black = cv2.inRange(hsv_stream, BLACK_MIN, BLACK_MAX)



            # green
            canny_edge_green  = cv2.blur(hsv_thresholded_green, (10,10))
            _,contours, hierarchy = cv2.findContours(canny_edge_green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                rect = cv2.boundingRect(c)
                if rect[2] < 60 or rect[3] < 60: continue
                
                x,y,w,h = rect
                Extent = (cv2.contourArea(c)/(w*h))
                if Extent > 0.7:
                    cv2.rectangle(canny_edge_green,(x,y),(x+w,y+h),(150,255,0),2)
                    if(x>(cap.get(3)/2)):
                         xtext = x
                    else:
                         xtext = x 
                    cv2.putText(canny_edge_green,('Green, Height:'+str(h)),(xtext+w+10,y+h),0,0.3,(150,255,0))
                    D_DetermanentG = 8000
                    Distance = round((D_DetermanentG/h),2)
                    cv2.putText(canny_edge_green,('Distance'+str(Distance)),(xtext+w+10,y-20),0,0.3,(150,255,0))
                    capwidth  = cap.get(3)
                    Bearing = (((x+(w/2))-(capwidth/2))/(7.25))
                    Bearing1 = round(Bearing,2)
                    cv2.putText(canny_edge_green,('Bearing'+str(Bearing1)),(xtext+w+10,y-30),0,0.3,(150,255,0))

                    objects.append(DetectedObject("Green",Bearing1,Distance,0)) #0 placeholder for orientation


            #blue
            canny_edge_blue = cv2.blur(hsv_thresholded_blue, (10,10))
            _, contoursb, hierarchy = cv2.findContours(canny_edge_blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for c in contoursb:
                rect = cv2.boundingRect(c)
                if rect[2] < 30 or rect[3] < 30: continue
               
                x,y,w,h = rect
                Extent = (cv2.contourArea(c)/(w*h))
                if Extent > 0.7:
                    cv2.rectangle(canny_edge_blue,(x,y),(x+w,y+h),(150,255,0),2)
                    if(x>(cap.get(3)/2)):
                         xtext = x
                    else:
                         xtext = x 
                    cv2.putText(canny_edge_blue,('Blue, Height:'+str(h)),(x+w+10,y+h),0,0.3,(150,0,0))
                    Oriantation = round(h/w,2)
                    if Oriantation >1:
                        Oriantation = 1
                    Oriantation = round(90*(float(Oriantation))**3.5,3)
                    if Oriantation > 85:
                        OriantationStr = "Straight"
                    else:
                        OriantationStr = str(Oriantation)
                    cv2.putText(canny_edge_blue,('Oriantation'+OriantationStr),(xtext+w+10,y-10),0,0.3,(150,255,0))
                    D_DetermanentB = 4200
                    Distance = round((D_DetermanentB/h),2)
                    cv2.putText(canny_edge_blue,('Distance'+str(Distance)),(xtext+w+10,y-20),0,0.3,(150,255,0))
                    capwidth  = cap.get(3)
                    Bearing = (((x+(w/2))-(capwidth/2))/(7.25))
                    Bearing1 = round(Bearing,2)
                    cv2.putText(canny_edge_blue,('Bearing'+str(Bearing1)),(xtext+w+10,y-30),0,0.3,(150,255,0))

                    objects.append(DetectedObject("Blue",Bearing1,Distance,0)) #0 placeholder for orientation


            #Orange
            canny_edge_orange = cv2.blur(hsv_thresholded_orange, (10,10))
            _, contourso, hierarchy = cv2.findContours(canny_edge_orange,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for c in contourso:
                 rect = cv2.boundingRect(c)
                 if rect[2] < 30 or rect[3] < 30: continue
                 
                 x,y,w,h = rect
                 Extent = (cv2.contourArea(c)/(w*h))
                 if Extent > 0.7:
                     cv2.rectangle(canny_edge_orange,(x,y),(x+w,y+h),(150,255,0),2)
                     if(x>(cap.get(3)/2)):
                         xtext = x
                     else:
                         xtext = x 
                     
                     cv2.putText(canny_edge_orange,('Orange, Height:'+str(h)),(xtext+w+10,y+h),0,0.3,(150,255,0))
                     D_DetermanentO = 2200
                     Distance = round((D_DetermanentO/h),2)
                     cv2.putText(canny_edge_blue,('Distance'+str(Distance)),(xtext+w+10,y-20),0,0.3,(150,255,0))
                     capwidth  = cap.get(3)
                     Bearing = ((x+(w/2))-(capwidth/2))/(7.25)
                     Bearing1 = round(Bearing,2)
                     cv2.putText(canny_edge_orange,('Bearing'+str(Bearing1)),(xtext+w+10,y-30),0,0.3,(150,255,0))

                     objects.append(DetectedObject("Orange",Bearing1,Distance,0)) #0 placeholder for orientation
            
            #Yellow
            canny_edge_yellow = cv2.blur(hsv_thresholded_yellow, (10,10))
            _, contourso, hierarchy = cv2.findContours(canny_edge_yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for c in contourso:
                 rect = cv2.boundingRect(c)
                 if rect[2] < 30 or rect[3] < 30: continue
                
                 x,y,w,h = rect
                 Extent = (cv2.contourArea(c)/(w*h))
                 if Extent > 0.3 and cv2.contourArea(c) > 10000:
                     cv2.rectangle(canny_edge_yellow,(x,y),(x+w,y+h),(150,255,0),2)
                     if(x>(cap.get(3)/2)):
                         xtext = x
                     else:
                         xtext = x 
                     
                     cv2.putText(canny_edge_yellow,('Yellow, Height:'+str(h)),(xtext+w+10,y+h),0,0.3,(150,255,0))
                     D_DetermanentY = 4000
                     Distance = int(round((D_DetermanentY/h),2))
                     cv2.putText(canny_edge_yellow,('Distance'+str(Distance)),(xtext+w+10,y-20),0,0.3,(150,255,0))
                     capwidth  = cap.get(3)
                     Bearing = ((x+(w/2))-(capwidth/2))/(7.25)
                     Bearing1 = round(Bearing,2)
                     cv2.putText(canny_edge_yellow,('Bearing'+str(Bearing1)),(x+w+10,y-30),0,0.3,(150,255,0))

                     objects.append(DetectedObject("Yellow",Bearing1,Distance,0)) #0 placeholder for orientation


            #Black
            canny_edge_black = cv2.blur(hsv_thresholded_black, (10,10))
            _, contourso, hierarchy = cv2.findContours(canny_edge_black,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for c in contourso:
                 rect = cv2.boundingRect(c)
                 if rect[2] < 80 or rect[3] < 80: continue
                
                 x,y,w,h = rect
                 Extent = (cv2.contourArea(c)/(w*h))
                 if Extent > 0.2 and cv2.contourArea(c) > 10000:
                     cv2.rectangle(canny_edge_black,(x,y),(x+w,y+h),(150,255,0),2)
                     if(x>(cap.get(3)/2)):
                         xtext = x   
                     else:
                         xtext = x   
                     #print(str(cv2.contourArea(c))
                     cv2.putText(canny_edge_black,('Black, Height:'+str(h)),(xtext+w+10,y+h),0,0.3,(150,255,0))
                     D_DetermanentO = 2200
                     Distance = int(round((D_DetermanentO/h),2))
                     cv2.putText(canny_edge_black,('Distance'+str(Distance)),(xtext+w+10,y-20),0,0.3,(150,255,0))
            
                     objects.append(DetectedObject("Wall",Bearing1,Distance,0)) #0 placeholder for orientation


            if ((time.time()-newtime)!=0):
                Freq = round(1/(time.time()-newtime),1)
            else:
                Freq = 0


            img = canny_edge_blue + canny_edge_green + canny_edge_orange + canny_edge_yellow +canny_edge_black
            
            cv2.putText(img,('Frequency: '+str(Freq)+"Hz"),(30,30),0,0.3,(150,255,0))

            cv2.imshow('Frame', hsv_stream)
            #cv2.imshow('Frame_Threshold_Orange', hsv_thresholded_orange)
            #cv2.imshow('Frame_Threshold_Blue', hsv_thresholded_blue)
            #cv2.imshow('Frame_Threshold_Green', hsv_thresholded_green)
            #cv2.imshow('Frame_Threshold_Greenlines', canny_edge_green)
            cv2.imshow('Frame_Detections', img)
            # cv2.imshow('Frame_Threshold_edges_blue',  blue_edges)
            # cv2.imshow('Frame_Threshold_edges_orange',  orange_edges)

    return objects
