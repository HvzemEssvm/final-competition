from ultralytics import YOLO
import cv2 as cv

model = YOLO("best.pt")
cap = cv.VideoCapture(0)

while True:

    cam_on, frame = cap.read()
    if not cam_on:
        break

    flipped_frame = cv.flip(frame, 1)

    results = model.track(source=flipped_frame, conf=0.8, iou=0.5, show=False)

    annotated_frame = results[0].plot()

    cv.imshow("Flipped View", annotated_frame )

    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Release the capture and close windows
cap.release()
cv.destroyAllWindows()
