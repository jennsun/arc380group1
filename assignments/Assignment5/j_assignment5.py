
# from ARC380_Assignment5_helper import capture_img
import numpy as np
import cv2
from matplotlib import pyplot as plt
from cv2 import aruco
 
"""
 
P2: Extract features (shape and color) and store each object in a class like and return an annotated image
Input: image
Output: annotated image
and return an annotatted image with 
P3: 
t features shape () and color and store them each object in a class where each object is a dictionary
object = {
    color : "COLOR",
    shape : "SHAPE",
    size : DIMENSION,
    position: {x: "X", y: "Y", z: "Z"},
    orientation: DEGREES
}
 
"""

def extract_features(img_path: str):
    dark_green = np.array([0, 100, 0])
    red = np.array([100, 0, 0])
    blue = np.array([0, 0, 100])
    colors = [dark_green, red, blue]
    # There are two shapes: circle and square. We need to know which type of shape we are looking for and we find that via expected area
    # So check every possible color/shape combination
    features = []
    for color in colors:
        features.append(extract_shape_with_color(img_path, "circle", color))
        features.append(extract_shape_with_color(img_path, "square", color))
    return features
 
def extract_shape_with_color(img_path: str, shape: str, color: np.array) -> dict:
    """
    sample image path: "sample-image.png"
    from an image, extracts color, shape, size, position, and orientation of objects
    returns: dict of the objects in the image
    """
    # Given an image, separate objects and then loop through all objects to assign traits on each one
    img = cv2.imread(img_path)
    # Convert the image from BGR to RGB and display using matplotlib
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Define the dimensions of the output image
    width = 10      # inches
    height = 7.5    # inches
    ppi = 96        # pixels per inch (standard resolution for most screens - can be any arbitrary value that still preserves information)
 
    # Reshape our image data to a flattened list of RGB values
    img_data = img_rgb.reshape((-1, 3))
    img_data = np.float32(img_data)
 
    # Define the number of clusters
    k = 6 # 3 or 4 + white background + black aruco markers
 
    # Define the criteria for the k-means algorithm
    # This is a tuple with three elements: (type of termination criteria, maximum number of iterations, epsilon/required accuracy)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
 
    # Run the k-means algorithm
    # Parameters: data, number of clusters, best labels, criteria, number of attempts, initial centers
    _, labels, centers = cv2.kmeans(img_data, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
    # centers = np.uint8(centers)
    print(f'Labels shape: {labels.shape}')
    print(f'Centers shape: {centers.shape}')
    print(f'Centers: \n{centers}')

    centers = np.uint8(centers)

    kmeans_data = centers[labels.flatten()]
    kmeans_img = kmeans_data.reshape(img.shape)
    labels = labels.reshape(img.shape[:2])

    # Display the k-means image
    plt.imshow(kmeans_img)
    plt.title(f'Image classification using k-means clustering (k = {k})')
    plt.gca().invert_yaxis()
    plt.show()
 
    # identify the cluster closest to each color
    # for color in colors: , do for green first
    distances = np.linalg.norm(centers - color, axis=1)
    color_label = np.argmin(distances)
 
    # All pixels that belong to this cluster will be white, and all others will be black
    mask_img = np.zeros(kmeans_img.shape[:2], dtype='uint8')
    mask_img[labels == color_label] = 255

    # Segment cluster into separate regions
    # Segment continuous regions
    # Parameters: input image, contour retrieval mode, contour approximation method
    contours, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    print("number of contours: ", len(contours))

    # remove the contours that are OpenCV Aruco markers as detected using the cv::aruco::ArucoDetector::detectMarkers() function
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, markerIds, rejectedCandidates = detector.detectMarkers(mask_img)
    print("corners: ", corners)

    area_of_marker = 2*2 * ppi**2

    
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        # If the center's area contains one of the corners of the aruco markers, remove it
        for corner in corners:
            if x < corner[0][0][0] < x+w and y < corner[0][0][1] < y+h:
                contours.remove(contour)
                break
        

    # Visualize the contours
    # Parameters for drawContours: input image, contours, contour index (-1 means all contours), color, thickness
    contour_img = img.copy()
    cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 3)
    plt.title(f'Contour image for cluster {color_label}')
    plt.imshow(cv2.cvtColor(contour_img, cv2.COLOR_BGR2RGB))
    plt.show()

    # Get size of object (area) -- actual disk has 1" radius
    # Get area of each region
    areas = [cv2.contourArea(contour) for contour in contours]
    print(f'Area of each region: {areas}')

    # Calculate the expected area of object given shape
    if shape == "circle":
        radius = 1
        expected_area = radius**2 * np.pi * ppi**2
    else:
        expected_area = 2*2 * ppi**2
    print(f'Expected area for 1 in radius circle: {expected_area}')
    
    perimeters = [cv2.arcLength(contour, closed=True) for contour in contours]

    # # Calculate the roundness of each contour
    roundness = []
    for i, contour in enumerate(contours):
        roundness.append((4 * np.pi * areas[i]) / perimeters[i]**2)

    print(f'Roundness of each region: {roundness}')
    
    # Find the contour with the closest area to the expected area
    closest_area_idx = np.argmin(np.abs(np.array(areas) - expected_area))

    # Alternatively, find the roundest contour
    # roundest_idx = np.argmax(roundness)

    # These should theoretically be the same for this example
    print(f'Closest area index: {closest_area_idx}')
    # print(f'Roundest index: {roundest_idx}')

    # Visualize the selected contour
    selected_contour_img = img.copy()
    cv2.drawContours(selected_contour_img, contours, closest_area_idx, (0, 255, 0), 3)

    plt.imshow(cv2.cvtColor(selected_contour_img, cv2.COLOR_BGR2RGB))
    plt.title(f'Selected contour image for label {color_label}')
    plt.gca().invert_yaxis()
    plt.show()
    
    # Compute the centroid of the region:
    selected_contour = contours[closest_area_idx]

    # Get the center using a bounding box
    x, y, w, h = cv2.boundingRect(selected_contour)
    u_c = x + w//2
    v_c = y + h//2

    # # Alternate method:
    # # Get the center of the selected contour using the moments
    # moments = cv2.moments(selected_contour)
    # u_c = int(moments['m10']/moments['m00'])
    # v_c = int(moments['m01']/moments['m00'])

    # Draw the center of the selected contour
    center_img = img.copy()
    cv2.circle(center_img, (u_c, v_c), 5, (255, 255, 0), -1)

    plt.imshow(cv2.cvtColor(center_img, cv2.COLOR_BGR2RGB))
    plt.title(f'Center of the selected contour for label {color_label}')
    plt.gca().invert_yaxis()
    plt.show()

    # Convert pixels to inches
    x_in = u_c / ppi
    y_in = v_c / ppi

    # Inches to mm
    x_mm = x_in * 25.4
    y_mm = y_in * 25.4

    # 2D position of the dark green disk relative to the task frame
    p_task = (x_mm, y_mm)

    print(p_task)

    # IF SQUARE, find orientation
    # Use minAreaRect to find the orientation of the blue square
    if shape == "square":
        rect = cv2.minAreaRect(selected_contour) # minAreaRect returns a Box2D structure. A Box2D structure is a tuple of ((x, y), (w, h), angle).
        print(rect)
        angle = rect[2]
    else:
        angle = 0

    # create a dictionary for the object
    object = {
        "color": color,
        "shape": shape,
        "size": (w, h),
        "position": {"x": x_mm, "y": y_mm},
        "orientation": angle
    }
    print(object)
    # append the object to a list of objects

# def annotate_features(img: np.nda, objects: dict) -> np.ndarray:
#     # Convert the image from BGR to RGB and display using matplotlib
#     # img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
 
#     # Run k-means clustering on the image
 
#     # Reshape our image data to a flattened list of RGB values
#     img_data = img.reshape((-1, 3))
#     img_data = np.float32(img_data)

#     # run pixel classification using k-means to classify by color
#     k = 6 # change based on number of colors
#     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
#     _, labels, centers = cv2.kmeans(img_data, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
#     centers = np.uint8(centers)
#     kmeans_data = centers[labels.flatten()]
#     kmeans_img = kmeans_data.reshape(img.shape)
#     labels = labels.reshape(img.shape[:2])
 
#     # Identify the different clusters via color
#     # example of initializing colors
#     dark_green = np.array([0, 100, 0])
#     red = np.arr([100, 0, 0])
 
#     # identify the cluster closest to each color
#     # for color in colors: , do for green first
#     distances = np.linalg.norm(centers - dark_green, axis=1)
#     green_cluster_label = np.argmin(distances)
 
#     # All pixels that belong to this cluster will be white, and all others will be black
#     mask_img = np.zeros(kmeans_img.shape[:2], dtype='uint8')
#     mask_img[labels == green_cluster_label] = 255
 
#     plt.imshow(mask_img, cmap='gray')
#     plt.title(f'Mask image for cluster {green_cluster_label} corresponding to dark green')
#     plt.gca().invert_yaxis()
#     plt.show()
 
 
 
 
 
 
 


# Given an input image, use OpenCV to extract the different pieces in the image and extract the color and shape of it
def process_image(img_path: str):
    return 

extract_features("test-image.png")
