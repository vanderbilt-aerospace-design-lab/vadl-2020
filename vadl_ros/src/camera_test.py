import cv2
import time


cap = cv2.VideoCapture(0)
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter("flight.avi", fourcc, 30.0, (640, 480))
print("Frame rate: {}".format(cap.get(5)))
print("Pixels: {}, {}".format(cap.get(3),cap.get(4)))
print("Codec: {}".format(cap.get(6)))
# cap.set(3, 1280)
# cap.set(4, 720)
# cap.set(6, cv2.VideoWriter_fourcc('M','J','P','G'))
# print("Codec: {}".format(cap.get(6)))
# print("Frame rate: {}".format(cap.get(5)))
# print("Pixels: {}, {}".format(cap.get(3),cap.get(4)))

ct = 0
ret, img = cap.read()
start_time = time.time()
while time.time() - start_time <= 1:
    ret, img = cap.read()
    ct += 1

print("FPS: {}".format(ct))
