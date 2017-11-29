"""
Detects yellow objects by thresholding the Hue values of captured images.
Calculates and displays the distance of detected objects 
assuming they are tennis ball markers.
"""

import cv2
import numpy as np

# intialise kernels and thresholds
kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(25,25))
kernel_close = np.ones((20,20))
area_thresh = 2000.
hue_thresh_low = 30
hue_thresh_high = 50
distance_constant = 85000        #actual width * focal length
marker_colour = [(255,0,0),(0,255,0),(0,0,255)]
should_run = True

# intialise video stream object
cam= cv2.VideoCapture(0)
cam.set(6, 15)
cv2.namedWindow('output', cv2.WINDOW_NORMAL)
#cv2.resizeWindow('output', (800, 800))
frame_width = int(cam.get(3))
frame_height = int(cam.get(4))

out = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 15, (frame_width,frame_height))

def find_distance(rad):
    distance = distance_constant / (2 * rad * 1000)
    return distance

def draw_target(i, num):
    (x, y), radius= cv2.minEnclosingCircle(conts[i])
    target_distance = find_distance(radius)
    centre = (int(x), int(y))
    radius_int = int(radius)
    height = 450 - (num * 30)
    cv2.circle(img, centre, radius_int, marker_colour[num-1], 2)
    cv2.circle(img, centre, 2, (0,0, 255), 5)
    cv2.putText(img, "Distance: %.2f m" % target_distance, (20, height) , 5, 1, marker_colour[num-1], 2, cv2.LINE_AA)

def display_images():
    cv2.imshow("output",img)
    out.write(img)
    #cv2.imshow("hsv", hsv_image)    
    #cv2.imshow("mask", mask_final)
    #cv2.imshow("h", h)

def morphological_ops(img):
    mask_open=cv2.morphologyEx(img,cv2.MORPH_OPEN,kernel_open)
    mask_close=cv2.morphologyEx(mask_open,cv2.MORPH_CLOSE,kernel_close)
    return mask_close

while(should_run == True):
    # capture a frame from the video stream
    ret, img = cam.read()
    
    marker_num = 0
    
    # check that a frame was read
    if(ret == False):
        print "No image taken"
        continue
    else:
        # convert frame to HSV colour space and split h, s and v channels
        hsv_image = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_image)

        # create the Mask
        mask=cv2.inRange(h, hue_thresh_low, hue_thresh_high)
    
        # opening and closing
        mask_final = morphological_ops(mask)
        
        # create array of detected contours
        _,conts,h=cv2.findContours(mask_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        # loop through contours and check size, if above size of 
        # area_thresh, draw in blue onto frame with distance
        # then display any images in display_images
        for i in range(len(conts)):
            contour_area = cv2.contourArea(conts[i])

            if(contour_area > area_thresh):
                cv2.drawContours(img,conts[i],-1,(255,0,0),3)
                marker_num = marker_num + 1
                draw_target(i, marker_num)                
            display_images()
            
            #wait for q to exit and release camera
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("q"):
                cv2.destroyAllWindows()
                cam.release()
                out.release()
                should_run = False
                break