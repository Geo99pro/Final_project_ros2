import cv2
import imutils

def get_hsv_value_based_on_click(image_path):
    image = cv2.imread(image_path)
    global image_hsv
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    cv2.imshow("Original Image", image)
    cv2.setMouseCallback("Original Image", pick_color)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def pick_color(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_value = image_hsv[y, x]
        print(f"HSV Value at ({x}, {y}): {hsv_value}")

def main():
    input_image_path = 'D:/ros_2_docker/final_project_ros2_2024/Final_project_ros2/square.png'

    get_hsv_value_based_on_click(image_path=input_image_path)
# if __name__ == '__main__':
#    main()


def get_shape(img, contours, eps):
    """
    This function is used to approximate the contour of an object in an image and determine the shape of the object
    Helpfull for the link: #https://pyimagesearch.com/2021/10/06/opencv-contour-approximation/
    
    Args:

        img: The image where the object is located, should be in matrix form. If the image is in a file, it should be read using cv2.imread() to get the matrix form
        contours: The contours of the object in the image
        eps: The epsilon value to approximate the contour of the object

    Returns:

        object_form: The shape of the object in the image
    """
    object_form = None
    img_copy = img.copy()
    #for eps in np.linspace(0.01, 0.05, 10):
    cnts = imutils.grab_contours(contours)
    c = max(cnts, key=cv2.contourArea)
    (x, y, w, h) = cv2.boundingRect(c)
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, eps*peri, True)
    text = "eps={:.2f}, num_points = {}".format(eps, len(approx))
    cv2.putText(img_copy, text, (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    print(f"By approximating the contour, the value is: {approx}")
    
    if len(approx) == 3:
        object_form = "Triangle"
        print(f"The object countour after approximation is a Triangle either {object_form} points.")
    elif len(approx) == 4:
        object_form = "Square"
        print(f"The object countour after approximation is a Square either {object_form} points.")
    else:
        object_form = "Circle"
        print(f"The object countour after approximation is a Circle either {object_form} points.")
    
    return object_form, c, text, x, y