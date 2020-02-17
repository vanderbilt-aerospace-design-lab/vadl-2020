import cv2
from marker_detection.camera import VideoStreamer

def main():
    vs = VideoStreamer(use_pi=1)
    while True:
        frame = vs.read()

        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()