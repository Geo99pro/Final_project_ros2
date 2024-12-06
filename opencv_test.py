import cv2
import imutils
import numpy as np

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





"""input_image_path = 'D:/ros_2_docker/final_project_ros2_2024/Final_project_ros2/square.png'

def find_contours(image_path: str, hsv_lower_values: list, hsv_upper_values: np.array):
    image = cv2.imread(image_path)
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #purple hsv
    lower_hsv = np.array(hsv_lower_values)
    upper_hsv = np.array(hsv_upper_values)
    
    mask = cv2.inRange(image_hsv, lower_hsv, upper_hsv)
    result = cv2.bitwise_and(image, image, mask=mask)

    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(contours)
    c = max(cnts, key=cv2.contourArea)
    #print(c)
    output = image.copy()
    cv2.drawContours(output, [c], -1, (0, 255, 0), 3)
    (x, y, w, h) = cv2.boundingRect(c)
    #print(x, y, w, h)

    text = "num_points = {}".format(len(c))
    cv2.putText(output, text, (x+5, y +50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    print("[INFO] {}".format(text))
    #cv2.imshow("Image", output)
    #cv2.imshow("Purple Mask", mask)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return c, output, x, y, mask


for eps in np.linspace(0.01, 0.05, 10):
 
    The purpose of this script is to find the contours of a given image and determine the shape of the object in the image

    print("Starting training --->")
    print(f"eps: {eps}")
    #lower_hsv = [0, 255, 255] # RED TRIANGLE
    #upper_hsv = [0, 255, 255] # RED TRIANGLE
    lower_hsv = [178, 255, 255] # RED SQUARE
    upper_hsv = [178, 255, 255] # RED SQUARE
    c, output, x, y, mask = find_contours(input_image_path, lower_hsv, upper_hsv)
    print(output)
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, eps*peri, True)
    text = "eps={:.2f}, num_points = {}".format(eps, len(approx))
    image_hsv = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(image_hsv, np.array(lower_hsv), np.array(upper_hsv))
    cv2.putText(output, text, (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    print(f"approx: {approx}")
    if len(approx) == 3:
        print("Triangle")
    elif len(approx) == 4:
        print("square")
    else:
        print("circle")
        
    #cv2.imshow("Image", output)
    #cv2.imshow("Mask", mask)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    break





















import cv2
import numpy as np


input_image_path = '/home/lci/Downloads/ros2_final_prject_github/Final_project_ros2/src/gpg_fproject/rectangle.jpg'

image = cv2.imread(input_image_path)
image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#purple hsv
lower_hsv = np.array([125, 50, 50])
upper_hsv = np.array([115, 255, 255])
mask = cv2.inRange(image_hsv, lower_hsv, upper_hsv)
result = cv2.bitwise_and(image, image, mask=mask)
cv2.imshow("Original Image", image)
cv2.imshow("HSV Image", image_hsv)
cv2.imshow("Purple Mask", mask)
cv2.imshow("Result", result)
#edge = cv2.Canny(image_hsv, 30, 200)

#contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
cv2.waitKey(0)
cv2.destroyAllWindows()"""