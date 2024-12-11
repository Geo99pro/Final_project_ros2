import cv2
#import imutils

def get_hsv_value_based_on_click(image_path):
    """
    This function is used to get the HSV value of a pixel in an image by clicking on the pixel"""
    image = cv2.imread(image_path)
    global image_hsv
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    cv2.imshow("Original Image", image)
    cv2.setMouseCallback("Original Image", pick_color)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def pick_color(event, x, y):
    """
    This function is used to get the HSV value of a pixel in an image by clicking on the pixel"""
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_value = image_hsv[y, x]
        print(f"HSV Value at ({x}, {y}): {hsv_value}")

def get_shape(img, mask, eps):
    """
    This function is used to approximate the contour of an object in an image and determine the shape of the object
    Helpful the link: #https://pyimagesearch.com/2021/10/06/opencv-contour-approximation/
    
    Args:

        img: The image containing the object. This can be a path to the image or the image itself as a NumPy array
        contours: The contours of the object in the image
        eps: The approximation accuracy parameter

    Returns:

        object_form: The shape of the object in the image
        c : The contour of the object in the image
        text: The text to be displayed on the image
        x: The x-coordinate of the object in the image
        y: The y-coordinate of the object in the image
    """
    if isinstance(img, str):
        img = cv2.imread(img)
    
    object_form = None
    img_copy = img.copy()
    #for eps in np.linspace(0.01, 0.05, 10):
    cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#imutils.grab_contours(contours)
    c = max(cnts, key=cv2.contourArea)
    coords = cv2.boundingRect(c)
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, eps*peri, True)
    text = "eps={:.2f}, num_points = {}".format(eps, len(approx))
    cv2.putText(img_copy, text, (coords[0], coords[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
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

if __name__ == "__main__":
    #get_hsv_value_based_on_click("test.jpg")
    get_shape("test.jpg", 0.04)