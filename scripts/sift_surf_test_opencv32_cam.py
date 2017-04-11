import numpy as np
import cv2

print("OpenCV Version: {}".format(cv2.__version__))

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    sift = cv2.xfeatures2d.SIFT_create()
    kp = sift.detect(gray,None)
    
    imgOutSift = frame
    cv2.drawKeypoints(frame,kp, imgOutSift, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Display the resulting frame
    cv2.imshow('sift detection',imgOutSift)

    surf = cv2.xfeatures2d.SURF_create()
    kp = surf.detect(gray,None)
    
    imgOutSurf = frame
    cv2.drawKeypoints(frame,kp, imgOutSurf, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Display the resulting frame
    cv2.imshow('surf detection',imgOutSurf)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
