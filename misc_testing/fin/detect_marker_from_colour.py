"""
Detects yellow objects by thresholding the Hue values of captured images.
Calculates and displays the distance of detected objects 
assuming they are tennis ball markers.
"""
import cv2
import numpy as np

# intialise kernels and thresholds
kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
kernel_close = np.ones((30,30))
area_thresh = 3000.
hue_thresh_low = 30
hue_thresh_high = 60
distance_constant = 85000        #actual width * focal length
should_run = True

# intialise video stream object
cam= cv2.VideoCapture(0)

def find_distance(rad):
    distance = distance_constant / (2 * rad)
    return distance

def draw_target(i):
    (x, y), radius= cv2.minEnclosingCircle(conts[i])
    target_distance = find_distance(radius)
    centre = (int(x), int(y))
    radius_int = int(radius)
    cv2.circle(img, centre, radius_int, (0,255,0), 2)
    cv2.circle(img, centre, 2, (0,0, 255), 5)
    cv2.putText(img, "%.2f mm" % target_distance, centre, 5, 2, (0,255,0), 2, cv2.LINE_AA)

def display_images():
    cv2.imshow("hsv", hsv_image)
    cv2.imshow("cam",img)
    cv2.imshow("mask", mask_final)
    #cv2.imshow("h", h)

def morphological_ops(img):
    mask_open=cv2.morphologyEx(img,cv2.MORPH_OPEN,kernel_open)
    mask_close=cv2.morphologyEx(mask_open,cv2.MORPH_CLOSE,kernel_close)
    return mask_close

while(should_run == True):
    # capture a fram from the video stream
    ret, img = cam.read()
    
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
                draw_target(i)                
            display_images()
            
            #wait for q to exit and release camera
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("q"):
                cv2.destroyAllWindows()
                cam.release()
                should_run = False
                break