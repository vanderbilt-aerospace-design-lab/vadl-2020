import cv2
''' Simply records a video'''

VIDEO_FILE = "./videos/flight_0.mp4"

# Set up OpenCV video capture
cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(VIDEO_FILE, fourcc, 15.0, (640, 480))
print("OpenCV video established")
print("Frame rate: {}".format(cap.get(5)))

while True:
    ret, img = cap.read()
    if ret:
        out.write(img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()