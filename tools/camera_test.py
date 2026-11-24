import cv2

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)   # 0 = laptop webcam, 1 = usually the USB camera

#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
    ok, frame = cap.read()
    if not ok:
        print("No image from camera")
        break
    cv2.imshow("camera", frame)
    if cv2.waitKey(1) & 0xFF == ord("c"):   # press c to capture image
        cv2.imwrite("data/captured_image3.png", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):   # press q to quit
        break

print("Frame shape:", frame.shape)
cap.release()
cv2.destroyAllWindows()