#might need to run as sudo
import cv2 as cv
import pickle
#Try different /video numbers in /dev folder
input_video = cv.VideoCapture('/dev/video2')
input_video.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
input_video.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
#detector_params = cv.aruco.DetectorParameters()
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
#detector = cv.aruco.ArucoDetector(dictionary)

print('Type the number of the node you are currently on.\n')
Node = input("")
print('Type the number of markers you want to capture.\n')
markerMin = int(input(""))

while input_video.grab():
    ret, image = input_video.retrieve()
    image_copy = image.copy()
    corners, ids, rejected = cv.aruco.detectMarkers(image, dictionary)
    print(ids)
    print (corners)
    # si au moins un marqueur est détecté
    n = 0
    if ids is not None and len(ids) > 0:
        #liste_ids = sorted(ids.flatten())
        #print(ids)
        #print (corners)
        n = len(ids)
        print (n)
        cv.aruco.drawDetectedMarkers(image_copy, corners)
        for i in range (n):
            center = tuple(corners[i][0][0])
            cv.circle(image_copy, center, 5, (0,0,255), -1)
    cv.imshow("out", image_copy)
    key = cv.waitKey(20) & 0xFF
    if key == 27:
        break
    if n>markerMin:
        break

pickle.dump(ids,open('ids'+Node+'.pkl', 'wb'))
pickle.dump(corners, open('corners'+Node+'.pkl','wb'))
input_video.release()
cv.destroyAllWindows()
print('\n\n'+str(len(ids))+' markers detected, ids downloaded to ids'+Node+'.pkl, corners downloaded to corners'+Node+'.pkl')
if(Node!=1):
    print('Now run server.py on Node 1, and then client.py on this Node and input ids'+Node+'.pkl to transfer to Node 1. Then run 2Dto3D.py.')
