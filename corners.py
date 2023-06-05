import cv2 as cv
input_video = cv.VideoCapture('/dev/video4')
input_video.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
input_video.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
#detector_params = cv.aruco.DetectorParameters()
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
#detector = cv.aruco.ArucoDetector(dictionary)
while input_video.grab():
    ret, image = input_video.retrieve()
    image_copy = image.copy()
    corners, ids, rejected = cv.aruco.detectMarkers(image, dictionary)
    print(ids)
    print (corners)
    # si au moins un marqueur est détecté
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
input_video.release()
cv.destroyAllWindows()
