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
            scale = 0.5
            frame = cv2.resize(frame, (int(frame.shape[1] * scale), int(frame.shape[0] * scale)))
            frame = cv2.flip(frame,0)
            # frame = cv2.flip(frame,1)



            # BRIGHTNESS ADJUSTING


        
            cols, rows = frame.shape[:-1]
            brightness = np.sum(frame) / (255 * cols * rows)

            minimum_brightness = 1.1
            
            ratio = brightness / minimum_brightness

            if ratio >= 1:
                pass
            elif ratio >= 1.3:
                cv2.convertScaleAbs(frame, alpha = 1 / ratio, beta = 0)
                return []
            else:    
                pass
                # Otherwise, adjust brightness to get the target brightness
                cv2.convertScaleAbs(frame, alpha = 1 / ratio, beta = 0)
                return []

            
            capwidth = frame.shape[1]
            # print(frame.shape)
            hsv_stream = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            

            ORANGE_MIN = np.array([170, 70,80])
            ORANGE_MAX = np.array([179, 255, 255])

            ORANGE_MIN1 = np.array([0, 70, 80])
            ORANGE_MAX1 = np.array([10, 255, 255])

            BLUE_MIN = np.array([100,80,30])
            BLUE_MAX = np.array([110,255,255])

            GREEN_MIN = np.array([35,60,25])
            GREEN_MAX = np.array([55,255,255])

            YELLOW_MIN = np.array([20,50,50])
            YELLOW_MAX = np.array([35,255,255])

            BLACK_MIN = np.array([0,0,0])
            BLACK_MAX = np.array([179,100,30])

            hsv_thresholded_orange1 = cv2.inRange(hsv_stream, ORANGE_MIN, ORANGE_MAX)
            hsv_thresholded_orange2 = cv2.inRange(hsv_stream, ORANGE_MIN1, ORANGE_MAX1)
            hsv_thresholded_orange = hsv_thresholded_orange1 + hsv_thresholded_orange2
            
            hsv_thresholded_blue = cv2.inRange(hsv_stream, BLUE_MIN, BLUE_MAX)
            
            hsv_thresholded_green = cv2.inRange(hsv_stream, GREEN_MIN, GREEN_MAX)

            hsv_thresholded_yellow = cv2.inRange(hsv_stream, YELLOW_MIN, YELLOW_MAX)
            hsv_thresholded_black = cv2.inRange(hsv_stream, BLACK_MIN, BLACK_MAX)

            

            # green
            canny_edge_green  = cv2.blur(hsv_thresholded_green, (5, 5))
            _,contours, hierarchy = cv2.findContours(canny_edge_green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                rect = cv2.boundingRect(c)
                if rect[2] < 30 or rect[3] < 30: continue
                
                x,y,w,h = rect
                Extent = (cv2.contourArea(c)/(w*h))
                if Extent > 0.7 and ((y+h)>(150*scale)):
                    cv2.rectangle(canny_edge_green,(x,y),(x+w,y+h),(150,255,0),2)
                    if(x>(cap.get(3)/2)):
                         xtext = x
                    else:
                         xtext = x 
                    Oriantation = round(h/w,2)
                    if Oriantation >1:
                        Oriantation = 1
                    Oriantation = round(90*(float(Oriantation))**3.5,3)
                    
                    if Oriantation > 85:
                        OriantationStr = "Straight"
                    else:
                        OriantationStr = str(Oriantation)
                    
                    cv2.putText(canny_edge_green,('Oriantation: '+OriantationStr),(xtext+w+10,y-10),0,0.3,(150,255,0))
                    cv2.putText(canny_edge_green,('Green, Height:'+str(h)),(xtext+w+10,y+h),0,0.3,(150,255,0))
                    D_DetermanentG = scale * 8200
                    Distance = round((D_DetermanentG/h),2)
                    cv2.putText(canny_edge_green,('Distance'+str(Distance)),(xtext+w+10,y-20),0,0.3,(150,255,0))
                    #capwidth  = cap.get(3)
                    Bearing = (((x+(w/2))-(capwidth/2))/(7.25*scale))
                    Bearing1 = round(Bearing,2)
                    cv2.putText(canny_edge_green,('Bearing'+str(Bearing1)),(xtext+w+10,y-30),0,0.3,(150,255,0))

                    objects.append(DetectedObject(ObjectType.OBSTACLE,(Bearing1),Distance,Oriantation)) #0 placeholder for orientation


            #blue
            canny_edge_blue = cv2.blur(hsv_thresholded_blue, (5,5))
            _, contoursb, hierarchy = cv2.findContours(canny_edge_blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for c in contoursb:
                rect = cv2.boundingRect(c)
                if rect[2] < 30 or rect[3] < 30: continue
               
                x,y,w,h = rect
                Extent = (cv2.contourArea(c)/(w*h))
                if Extent > 0.6 and ((y+h)>(150*scale)):
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
                    cv2.putText(canny_edge_blue,('Oriantation: '+OriantationStr),(xtext+w+10,y-10),0,0.3,(150,255,0))
                    D_DetermanentB = scale * 4400
                    Distance = round((D_DetermanentB/h),2)
                    cv2.putText(canny_edge_blue,('Distance: '+str(Distance)),(xtext+w+10,y-20),0,0.3,(150,255,0))
                    # capwidth  = cap.get(3)
                    Bearing = (((x+(w/2))-(capwidth/2))/(7.25*scale))
                    Bearing1 = round(Bearing,2)
                    cv2.putText(canny_edge_blue,('Bearing: '+str(Bearing1)),(xtext+w+10,y-30),0,0.3,(150,255,0))

                    objects.append(DetectedObject(ObjectType.ROCK,Bearing1,Distance,Oriantation)) #0 placeholder for orientation


            #Orange
            canny_edge_orange = hsv_thresholded_orange # cv2.blur(hsv_thresholded_orange, (10,10))
            _, contourso, hierarchy = cv2.findContours(canny_edge_orange,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for c in contourso:
                 rect = cv2.boundingRect(c)
                 if rect[2] < 10 or rect[3] < 10: continue
                 
                 x,y,w,h = rect
                 Extent = (cv2.contourArea(c)/(w*h))
                 if Extent > 0.4 and ((y+h)>(150*scale)):
                     cv2.rectangle(canny_edge_orange,(x,y),(x+w,y+h),(150,255,0),2)
                     if(x>(cap.get(3)/2)):
                         xtext = x
                     else:
                         xtext = x 
                     
                     cv2.putText(canny_edge_orange,('Orange, Height:'+str(h)),(xtext+w+10,y+h),0,0.3,(150,255,0))
                     D_DetermanentO = scale * 2600
                     Distance = round((D_DetermanentO/h),2)
                     cv2.putText(canny_edge_blue,('Distance'+str(Distance)),(xtext+w+10,y-20),0,0.3,(150,255,0))
                     # capwidth  = cap.get(3)
                     Bearing = ((x+(w/2))-(capwidth/2))/(7.25*scale)
                     Bearing1 = round(Bearing,2)
                     cv2.putText(canny_edge_orange,('Bearing'+str(Bearing1)),(xtext+w+10,y-30),0,0.3,(150,255,0))

                     objects.append(DetectedObject(ObjectType.SAMPLE,(Bearing1),Distance,0)) #0 placeholder for orientation
            
            #Yellow
            canny_edge_yellow = cv2.blur(hsv_thresholded_yellow, (5,5))
            _, contourso, hierarchy = cv2.findContours(canny_edge_yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for c in contourso:
                 rect = cv2.boundingRect(c)
                 if rect[2] < 30 or rect[3] < 30: continue
                
                 x,y,w,h = rect
                 Extent = (cv2.contourArea(c)/(w*h))
                 if Extent > 0.3 and cv2.contourArea(c) > 1000 and ((y+h)>(150*scale)):
                     cv2.rectangle(canny_edge_yellow,(x,y),(x+w,y+h),(150,255,0),2)
                     if(x>(cap.get(3)/2)):
                         xtext = x
                     else:
                         xtext = x 
                     
                     cv2.putText(canny_edge_yellow,('Yellow, Height:'+str(h)),(xtext+w+10,y+h),0,0.3,(150,255,0))
                     D_DetermanentY = scale * 5000
                     Distance = int(round((D_DetermanentY/h),2))
                     cv2.putText(canny_edge_yellow,('Distance'+str(Distance)),(xtext+w+10,y-20),0,0.3,(150,255,0))
                     #capwidth  = cap.get(3)
                     Bearing = ((x+(w/2))-(capwidth/2))/(7.25*scale)
                     Bearing1 = round(Bearing,2)
                     cv2.putText(canny_edge_yellow,('Bearing'+str(Bearing1)),(x+w+10,y-30),0,0.3,(150,255,0))

                     objects.append(DetectedObject(ObjectType.LANDER,(Bearing1),Distance,0)) #0 placeholder for orientation


            #Black
            canny_edge_black = cv2.blur(hsv_thresholded_black, (5,5))
            _, contourso, hierarchy = cv2.findContours(canny_edge_black,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for c in contourso:
                 rect = cv2.boundingRect(c)
                 if rect[2] < 80 or rect[3] < 80: continue
                
                 x,y,w,h = rect
                 Extent = (cv2.contourArea(c)/(w*h))
                 if Extent > 0.3 and cv2.contourArea(c) > 10000:
                     cv2.rectangle(canny_edge_black,(x,y),(x+w,y+h),(150,255,0),2)
                     if(x>(cap.get(3)/2)):
                         xtext = x   
                     else:
                         xtext = x   
                     #print(str(cv2.contourArea(c))
                     cv2.putText(canny_edge_black,('Black, Height:'+str(h)),(xtext+w+10,y+h),0,0.3,(150,255,0))
                     B_DetermanentO = scale * 1000
                     Distance = int(round((B_DetermanentO/h),2))
                     cv2.putText(canny_edge_black,('Distance'+str(Distance)),(xtext+w+10,y-20),0,0.3,(150,255,0))
            
                     objects.append(DetectedObject(ObjectType.WALL,0,Distance,0)) #0 placeholder for orientation


            if ((time.time()-newtime)!=0):
                Freq = round(1/(time.time()-newtime),1)
            else:
                Freq = 0


            img = canny_edge_blue + canny_edge_green + canny_edge_orange + canny_edge_yellow 
            
            cv2.putText(img,('Frequency: '+str(Freq)+"Hz"),(30,30),0,0.3,(150,255,0))

            #cv2.imshow('Frame', hsv_stream)
            #cv2.imshow('Frame_Threshold_Orange', hsv_thresholded_orange)
            #cv2.imshow('Frame_Threshold_Blue', hsv_thresholded_blue)
            #cv2.imshow('Frame_Threshold_Green', hsv_thresholded_green)
            #cv2.imshow('Frame_Threshold_Greenlines', canny_edge_green)
            cv2.imshow('Frame_Detections', img)
            #cv2.imshow('walls',  canny_edge_black)
            # cv2.imshow('Frame_Threshold_edges_orange',  orange_edges)

    cv2.waitKey(1)
    return objects
            

if __name__ == "__main__":
    cap = Initialize()
    current = True
    while current:
        Objectarray = ObjectDetection(cap)
    
        if (len(Objectarray)!=0):
            print(Objectarray[0].type)
    





        if cv2.waitKey(20) & 0xFF == ord('q'):
            current = False



