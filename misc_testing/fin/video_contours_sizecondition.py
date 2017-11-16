import cv2
import numpy as np

lowerBound=np.array([33,80,40])
upperBound=np.array([102,255,255])

cam= cv2.VideoCapture(1)

kernelOpen = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
kernelClose = np.ones((20,20))

while(True):
    #read image form video stream
    ret, img = cam.read()

    #convert BGR to HSV
    imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    # create the Mask
    mask=cv2.inRange(imgHSV,lowerBound,upperBound)
    
    #morphology
    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
    maskFinal=maskClose
    
    # find contours of binary image
    _,conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    
    for i in range(len(conts)):
        #check contour is of sufficient size to be tennis ball
        contour_area = cv2.contourArea(conts[i])
        print contour_area
        if(contour_area > 2000):
            cv2.drawContours(img,conts[i],-1,(255,0,0),3)
            (x, y), radius= cv2.minEnclosingCircle(conts[i])
            centre = (int(x), int(y))
            radius_int = int(radius)
            cv2.circle(img, centre, radius_int, (0,255,0), 2)
            cv2.putText(img, "Radius: %.2f" % radius, centre, 5, 2, (0,255,255), 2, cv2.LINE_AA)

    cv2.imshow("maskClose",maskClose)
    cv2.imshow("cam",img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        cv2.destroyAllWindows()
        cam.release()