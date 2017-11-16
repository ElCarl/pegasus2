import cv2
import numpy as np

#intialise kernels and thresholds
kernelOpen = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(25,25))
kernelClose = np.ones((30,30))
area_thresh = 3000
hue_thresh_low = 30
hue_thresh_high = 60

cam= cv2.VideoCapture(1)



while(True):
    ret, img = cam.read()

    #convert BGR to HSV and split the channels
    hsv_image = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_image)
    
    # create the Mask
    mask=cv2.inRange(h, hue_thresh_low, hue_thresh_high)
    
    #morphology
    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
    maskFinal=maskClose
    
    _,conts,h=cv2.findContours(maskFinal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    for i in range(len(conts)):
        contour_area = cv2.contourArea(conts[i])
        print contour_area
        if(contour_area > area_thresh):
            cv2.drawContours(img,conts,-1,(255,0,0),3)
            (x, y), radius= cv2.minEnclosingCircle(conts[i])
            centre = (int(x), int(y))
            radius_int = int(radius)
            cv2.circle(img, centre, radius_int, (0,255,0), 2)
            cv2.putText(img, "%f" % radius, centre, 5, 6, (0,255,255), 2, cv2.LINE_AA)
    #display images
    cv2.imshow("h", h)
    cv2.imshow("cam",img)
    cv2.imshow("mask", mask)
    #wait for q to exit
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        cv2.destroyAllWindows()
        cam.release()
        break