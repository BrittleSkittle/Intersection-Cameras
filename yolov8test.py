import cv2 as cv
from ultralytics import YOLO

model = YOLO('yolov8m.pt')
input_video = cv.VideoCapture('/dev/video3')
input_video.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
input_video.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
while input_video.grab():
    ret, frame = input_video.read()
    detect = model.predict(frame)
    detect = detect[0]
    detectplot = detect.plot()
    cv.imshow("output", detectplot)
    key = cv.waitKey(20) & 0xFF
    if key == 27:
        break

input_video.release()
cv.destroyAllWindows()
