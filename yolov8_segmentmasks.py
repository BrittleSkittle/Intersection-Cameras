import cv2 as cv
from ultralytics import YOLO
from PIL import Image

model = YOLO('yolov8m-seg.pt')
input_video = cv.VideoCapture('/dev/video2')
input_video.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
input_video.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
while input_video.grab():
    ret, frame = input_video.read()
    detect = model.predict(frame)
    detect = detect[0]
    detectplot = detect.plot()
    cv.imshow("output",detectplot)
    key = cv.waitKey(1)
    if key == ord("s"):
        break
    
masks = detect.masks
mask1 = 0 #masks[0]
mask = 0 #mask1.data[0].numpy()
detectplot = detect.plot()
box = detect.boxes[0]

for box in detect.boxes:
    class_id = detect.names[box.cls[0].item()]
    print("output", class_id)

while True:
    for x in range(len(masks)):
        mask1 = masks[x]
        mask = mask1.data[0].numpy()
        cv.imshow("output"+str(x), mask)
    y = cv.waitKey(1)
    if y == ord("q"):
        break
    

input_video.release()
cv.destroyAllWindows()
