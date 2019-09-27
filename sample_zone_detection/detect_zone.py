import cv2

IMAGE_FILE = 'test_image.jpg'


def find_target(img):

    # TODO: Choose between LAB and HSV color space
    lab_space_img = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)

    # cv2.imshow('lab_image', lab_space_img)
    # k = cv2.waitKey(0)
    # if k == 27:  # wait for ESC key to exit
    #     cv2.destroyAllWindows()

    # Threshold yellow values
    retval1, thresh1 = cv2.threshold(lab_space_img[:, :, 2], 0, 255, cv2.THRESH_OTSU)

    # TODO: Is this necessary?
    retval2, thresh2 = cv2.threshold(lab_space_img[:, :, 0], 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    sum_thresh = cv2.bitwise_and(thresh1, thresh2)

    # cv2.imshow('img', thresh1)
    # k = cv2.waitKey(0)
    # if k == 27:  # wait for ESC key to exit
    #     cv2.destroyAllWindows()
    # elif k == ord('s'):  # wait for 's' key to save and exit
    #     cv2.imwrite('sum.png', sum_thresh)
    #     cv2.destroyAllWindows()

    # TODO: Is this necessary?
    # se1 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(20,20))
    # se2 = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(7, 7))
    # sum_thresh = cv2.dilate(sum_thresh, se2)
    # sum_thresh = cv2.erode(sum_thresh, se1)

    # Find contours
    contours, hierarchy = cv2.findContours(sum_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # Draw contours
    sum_thresh = cv2.drawContours(sum_thresh, contours, -1, (0, 0, 255), 3)

    cv2.imshow('img', sum_thresh)
    k = cv2.waitKey(0)
    if k == 27:  # wait for ESC key to exit
        cv2.destroyAllWindows()
    elif k == ord('s'):  # wait for 's' key to save and exit
        cv2.imwrite('sum.png', sum_thresh)
        cv2.destroyAllWindows()


    print("x")


def main():

    # Load color image
    img = cv2.imread(IMAGE_FILE, 1)

    find_target(img)


if __name__ == "__main__":
    main()