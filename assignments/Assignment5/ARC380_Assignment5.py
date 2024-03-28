from ARC380_Assignment5_helper import capture_img
import numpy as np
import cv2
from matplotlib import pyplot as plt
from cv2 import aruco
 
 
"""
 
P2: Extract features (shape and color) and store each object in a class like and return an annotated image
Input: image
Output: annotated image
and return an annotated image with 
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

# 2b
def get_img(save_path):
    """
    captures and saves an image, and returns the BGR version of the image.
    """
    img = capture_img(visualize=False, save=True, path=save_path)
    # convert image from RGB (from realsense) to BGR (opencv)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    # return img


def transform_img(path):
    """
    transforms a given image by un-warping it and cropping it to the aruco markers
    """
    img = cv2.imread(path)
    # aruco imports
    dictionary = aruco.getPrePredefinedDictionary(aruco.DICT_6X6_250)
    detector_params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, detector_params)

    # detect aruco markers
    corners, ids, rejected = detector.detectMarkers(img)

    # parameters of final image
    width = 10
    height = 7.5
    ppi = 96

    # sort markers and define source and destination points
    ids = ids.flatten()
    corners = np.array([corners[i] for i in np.argsort(ids)])
    corners = np.squeeze(corners)
    ids = np.sort(ids)
    src_pts = np.array([corners[0][0], corners[1][1], corners[2][2], corners[3][3]], dtype='float32')
    dst_pts = np.array([[0, 0], [0, height*ppi], [width*ppi, height*ppi], [width*ppi, 0]], dtype='float32')

    # do the transform and return the corrected image
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    corrected_img = cv2.warpPerspective(img, M, (img_rgb.shape[1], img_rgb.shape[0]))
    corrected_img = corrected_img[:int(height*ppi), :int(width*ppi)]
    cv2.imwrite(path, corrected_img)
    return corrected_img

 
# 2c
def extract_features(img_path: str) -> dict:
    """
    sample image path: "sample-image.png"
    from an image, extracts color, shape, size, position, and orientation of objects
    returns: dict of the objects in the image
    """
    # Given an image, separate objects and then loop through all objects to assign traits on each one
    img = cv2.imread(img_path)
    # Convert the image from BGR to RGB and display using matplotlib
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    plt.imshow(img_rgb)
    plt.show()
 
    # Run k-means clustering on the image
    # Reshape our image data to a flattened list of RGB values
    img_data = img_rgb.reshape((-1, 3))
    img_data = np.float32(img_data)
 
    # Define the number of clusters
    k = 7 # 3 or 4 + white background + black aruco markers
 
    # Define the criteria for the k-means algorithm
    # This is a tuple with three elements: (type of termination criteria, maximum number of iterations, epsilon/required accuracy)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.5)
 
    # Run the k-means algorithm
    # Parameters: data, number of clusters, best labels, criteria, number of attempts, initial centers
    _, labels, centers = cv2.kmeans(img_data, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
    centers = np.uint8(centers)
    # print(f'Labels shape: {labels.shape}')
    # print(f'Centers shape: {centers.shape}')
    # print(f'Centers: \n{centers}')

    kmeans_data = centers[labels.flatten()]
    kmeans_img = kmeans_data.reshape(img.shape)
    labels = labels.reshape(img.shape[:2])

    # define colors in image
    colors = [
            #   ("green", np.array([0, 101, 55])), 
            #   ("red", np.array([96, 9, 5])),
            #   ("yellow", np.array([166, 132, 0])),
              ("orange", np.array([149, 37, 0])),
            #   ("pink", np.array([241, 101, 122])),
              ("turquoise", np.array([17, 129, 123])),
            #   ("blue", np.array([0, 105, 237])),
            #   ("dark blue", np.array([11, 13, 200])),
              ("purple", np.array([14, 11, 16]))
            ]
    
    # identify the cluster closest to each color
    objects = []

    for color_name, color_value in colors:
        distances = np.linalg.norm(centers - color_value, axis=1)
        cluster_label = np.argmin(distances)
    
        # All pixels that belong to this cluster will be white, and all others will be black
        mask_img = np.zeros(kmeans_img.shape[:2], dtype='uint8')
        mask_img[labels == cluster_label] = 255

        contours, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Visualize the contours
        # Parameters for drawContours: input image, contours, contour index (-1 means all contours), color, thickness
        contour_img = img.copy()
        cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 3)
    
        # plt.imshow(cv2.cvtColor(contour_img, cv2.COLOR_BGR2RGB))
        # plt.title(f'Contour image for cluster {cluster_label} corresponding to {color_name}')
        # plt.gca().invert_yaxis()
        # plt.show()

        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, closed=True)
            roundness = 4 * np.pi * area / perimeter**2
            is_square = roundness < 0.785

            # calculate position
            x, y, w, h = cv2.boundingRect(contour)
            u = x + w/2
            v = y + h/2

            # if aruco marker, skip this object
            # aruco markers are at the corners of the image
            if (u < 100 or u > 850) and (v < 100 or v > 600):
                continue
            
            # calculate orientation of square
            rect = cv2.minAreaRect(contour)
            orientation = rect[2]

            # add to list of objects
            shape = "square" if is_square else "circle"
            objects.append(
                {
                    "color" : color_name,
                    "shape" : shape,
                    "size" : area,
                    "position": {"x": u, "y": v},
                    "orientation": orientation
                }
            )
    print("number of objects found:", len(objects))
    for o in objects:
        print(o)
    return objects          
 
# 2d
def annotate_features(path, objects):
    colors = {
        "orange": (0, 37, 149),
        "turquoise": (123, 129, 17),
        "purple": (16, 11, 14)
    }
    font = cv2.FONT_HERSHEY_SIMPLEX 
    img = cv2.imread(path)

    for object in objects:
        color = object["color"]
        shape = object["shape"]
        area = object["size"]
        x, y = (int(object["position"]["x"]), int(object["position"]["y"]))
        orientation = object["orientation"]

        if shape == "circle":
            radius = int((area / np.pi)**0.5)
            img = cv2.circle(img, center=(x, y), radius=radius, color=colors[color], thickness=5)
            img = cv2.putText(img, text=color + " " + shape, org=(x, y), fontScale=1, fontFace=font, color=colors[color], thickness=1)
        else:
            pass
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.title("Annotated Image")
    # plt.gca().invert_yaxis()
    plt.show()


if __name__ == "__main__":
    path = "test-image.png"
    # img = get_img(path)
    # img = transform_img(img)
    objects = extract_features(path)
    annotate_features(path, objects)