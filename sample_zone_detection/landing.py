import cv2
import numpy as np

IMAGE_FILE = 'test_image.jpg'
CAMERA_VIDEO_FILE = 'videos/depth_estimation_1.mp4'
BOX_LENGTH = 22 # inches
FOCAL_LENGTH = 362  # This is just approximate for now

def find_target():
    cap = cv2.VideoCapture(1)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(CAMERA_VIDEO_FILE, fourcc, 15.0, (640, 480))

    while True:
        ret, img = cap.read()

        # TODO: Choose between LAB and HSV color space
        lab_space_img = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)

        '''Display converted color space'''
        # cv2.imshow('lab_image', lab_space_img)
        # k = cv2.waitKey(0)
        # if k == 27:  # wait for ESC key to exit
        #     cv2.destroyAllWindows()

        # Threshold image based on yellow; otsu thresholding
        retval1, thresh1 = cv2.threshold(lab_space_img[:, :, 2], 0, 255, cv2.THRESH_OTSU)

        # TODO: Is this necessary?
        # Binary OR Otsu thresholding
        retval2, thresh2 = cv2.threshold(lab_space_img[:, :, 0], 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        sum_thresh = cv2.bitwise_and(thresh1, thresh2)


        # TODO: Is this necessary?
        # TODO: Try adding a Guassian blur
        # Dilate and erode the image
        # This reduces the noise residual in the threshold
        se1 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(20,20))
        se2 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(7, 7))
        sum_thresh = cv2.dilate(sum_thresh, se2)
        sum_thresh = cv2.erode(sum_thresh, se1)

        # Find contours
        # TODO: The return values here are dependent on Opencv 2,3,or 4, so either be sure of which version
        #  we are using or add some code to check the version
        _, contours, hierarchy = cv2.findContours(sum_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # Find corners of the contour
        if len(contours) >= 1:
            final_contour = None
            # Take contour w/ max area
            c = max(contours, key=cv2.contourArea)

            # Approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)

            if len(approx) == 4:
                c = approx
                peri = cv2.arcLength(c, True)

            # Find extreme points
            extLeft = tuple(c[c[:, :, 0].argmin()][0])
            extRight = tuple(c[c[:, :, 0].argmax()][0])
            extTop = tuple(c[c[:, :, 1].argmin()][0])
            extBot = tuple(c[c[:, :, 1].argmax()][0])

            # Find the contour center
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # Find the distance to the platform
            pixel_length = peri / 4
            depth = (FOCAL_LENGTH / pixel_length) * BOX_LENGTH
            scale_factor = BOX_LENGTH / pixel_length

            # Find positional error from center of platform
            # Pixels
            err_x = cX - (640/2)
            err_y = (480/2) - cY

            # Convert to inches
            err_x *= scale_factor
            err_y *= scale_factor

            '''Visualize'''
            # Draw extreme points on image
            cv2.circle(img, extLeft, 8, (255, 0, 0), -1)
            cv2.circle(img, extRight, 8, (255, 0, 0), -1)
            cv2.circle(img, extTop, 8, (255, 0, 255), -1)
            cv2.circle(img, extBot, 8, (0, 0, 255), -1)

            # Draw center
            cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
            cv2.putText(img, "center", (cX + 20, cY + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw depth
            cv2.putText(img, "Distance: {}".format(np.around(depth, 1)), (cX - 40, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw XY error
            cv2.putText(img, "XY Error: {}, {}".format(np.around(err_x, 1), np.around(err_y, 1)), (cX - 40, cY - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw contours on image
            cv2.drawContours(img, [c], 0, (0, 255, 0), 3)
            '''        '''

            # Write to video
            out.write(img)

        '''Display thresholded image'''
        cv2.imshow('img', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    out.release()


def main():

    find_target()

if __name__ == "__main__":
    main()
