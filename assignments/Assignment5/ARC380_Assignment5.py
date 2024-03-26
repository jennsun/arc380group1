from ARC380_Assignment5_helper import capture_img
import numpy as np
import cv2
from matplotlib import pyplot as plt
 
 
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
    # convert image from RGB (from realsense) to BGR (opencv) (do we need to do this?)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    # return img
 
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
    print(f'Labels shape: {labels.shape}')
    print(f'Centers shape: {centers.shape}')
    print(f'Centers: \n{centers}')

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
    for color_name, color_value in colors:
        distances = np.linalg.norm(centers - color_value, axis=1)
        print(distances)
        cluster_label = np.argmin(distances)
        print(cluster_label)
    
        # All pixels that belong to this cluster will be white, and all others will be black
        mask_img = np.zeros(kmeans_img.shape[:2], dtype='uint8')
        mask_img[labels == cluster_label] = 255

        contours, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Visualize the contours
        # Parameters for drawContours: input image, contours, contour index (-1 means all contours), color, thickness
        contour_img = img.copy()
        cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 3)
    
        plt.imshow(cv2.cvtColor(contour_img, cv2.COLOR_BGR2RGB))
        plt.title(f'Contour image for cluster {cluster_label} corresponding to {color_name}')
        plt.gca().invert_yaxis()
        plt.show()
        # plt.imshow(mask_img, cmap='gray')
        # plt.title(f'Mask image for cluster {cluster_label} corresponding to {color_name}')
        # plt.gca().invert_yaxis()
        # plt.show()
 
 
# 2d
def annotate_features(img: np.ndarray, objects: dict):
    pass


if __name__ == "__main__":
    path = "test-image.png"
    # get_img(path)
    extract_features(path)