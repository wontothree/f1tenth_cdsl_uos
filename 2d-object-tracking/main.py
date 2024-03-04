from ultralytics import YOLO
import cv2

# load yolov8 model
model = YOLO('yolov8n.pt')

# use default camera (camera index 0)
cap = cv2.VideoCapture(0)

ret = True
# read frames
while ret:
    ret, frame = cap.read()

    if ret:

        # detect objects
        # track objects
        results = model.track(frame, persist=True)

        # plot results
        frame_ = results[0].plot()

        # visualize
        cv2.imshow('frame', frame_)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

# release the camera when the loop is done
cap.release()
cv2.destroyAllWindows()
