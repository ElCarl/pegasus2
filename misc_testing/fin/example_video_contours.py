import cv2
import numpy as np

lowerBound=np.array([33,80,40])
upperBound=np.array([102,255,255])

cam= cv2.VideoCapture(0)
print(cam.isOpened())
kernelOpen = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
kernelClose = np.ones((20,20))

while(True):
    ret, img = cam.read()
    #print(ret)
    #img=cv2.resize(img,(640,480))

    #convert BGR to HSV
    imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    # create the Mask
    mask=cv2.inRange(imgHSV,lowerBound,upperBound)
    #morphology
    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

    maskFinal=maskClose
    _,conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    
    cv2.drawContours(img,conts,-1,(255,0,0),3)
    for i in range(len(conts)):
        (x, y), radius= cv2.minEnclosingCircle(conts[i])
        centre = (int(x), int(y))
        radius_int = int(radius)
        cv2.circle(img, centre, radius_int, (0,255,0), 2)
        cv2.putText(img, "%f" % radius, centre, 5, 6, (0,255,255), 2, cv2.LINE_AA)
    cv2.imshow("maskClose",maskClose)
    #cv2.imshow("maskOpen",maskOpen)
    #cv2.imshow("mask",mask)
    cv2.imshow("cam",img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        cv2.destroyAllWindows()
        cam.release()
        break