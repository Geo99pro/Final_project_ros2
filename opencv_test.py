import cv2
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
    input_image_path = '/home/lci/Downloads/ros2_final_prject_github/Final_project_ros2/src/gpg_fproject/purple_rectangle.jpg'
    get_hsv_value_based_on_click(image_path=input_image_path)


input_image_path = '/home/lci/Downloads/ros2_final_prject_github/Final_project_ros2/src/gpg_fproject/purple_rectangle.jpg'
image = cv2.imread(input_image_path)
image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#purple hsv
lower_hsv = np.array([152, 156, 157])
upper_hsv = np.array([152, 156, 157])
mask = cv2.inRange(image_hsv, lower_hsv, upper_hsv)
result = cv2.bitwise_and(image, image, mask=mask)

contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
cv2.imshow("Original Image", image)
cv2.imshow("HSV Image", image_hsv)
cv2.imshow("Purple Mask", mask)
cv2.imshow("Result", result)
cv2.waitKey(0)
cv2.destroyAllWindows()









"""

"""





























"""import cv2
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