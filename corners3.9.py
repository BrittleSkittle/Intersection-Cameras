#might need to run as sudo
import cv2 as cv
import pickle
import numpy as np
#Try different /video numbers in /dev folder
input_video = cv.VideoCapture('/dev/video2')
input_video.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
input_video.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
parameters = cv.aruco.DetectorParameters()
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
detector = cv.aruco.ArucoDetector(dictionary,parameters)

print('Type the number of the node you are currently on.\n')
Node = input("")
print('Type the number of markers you want to capture.\n')
markerMin = int(input(""))
prevIds = None
prevCorners = None
while input_video.grab():
    ret, image = input_video.retrieve()
    image_copy = image.copy()
    corners, ids, rejected = detector.detectMarkers(image)
    print (corners)
    print(ids)

    
    n = 0
    if ids is not None and len(ids) > 0:
        print(len(ids))
        if prevIds is not None:
        #if False:
            idlen = len(ids)
            newids = None
            newcorners = None

            for i in range(len(prevIds)):
                match = False
                for j in range(len(ids)):
                    if prevIds[i] == ids[j]:
                        match = True
                if not match:
                    if newids is None:
                        newids = prevIds[i]
                    else:
                        newids = np.row_stack((newids,prevIds[i]))
                    ids = np.row_stack((ids,prevIds[i]))
                    corners = list(corners)
                    corners.append(prevCorners[i])

            print("new len: "+str(len(ids)))
            print("new corners len: "+str(len(corners)))
            #print(ids)

        prevIds = ids
        prevCorners = corners
        cv.aruco.drawDetectedMarkers(image_copy, corners)
        n = len(ids)
        
    cv.imshow("out", image_copy)
    key = cv.waitKey(20) & 0xFF
    if key == 27:
        break
    if n>=markerMin:
        break

pickle.dump(ids,open('ids'+Node+'.pkl', 'wb'))
pickle.dump(corners, open('corners'+Node+'.pkl','wb'))
input_video.release()
cv.destroyAllWindows()
print('\n\n'+str(len(ids))+' markers detected, ids downloaded to ids'+Node+'.pkl, corners downloaded to corners'+Node+'.pkl')
if(Node!=1):
    print('Now run server.py on Node 1, and then client.py on this Node and input ids'+Node+'.pkl to transfer to Node 1. Then run 2Dto3D.py.')
