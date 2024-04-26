"""
Final Project Part 2
Develop the 2D and 3D perception required to precisely determine the pose of each object to pick and
place them. You will need to utilize the ArUco markers to calibrate the frame of the camera, then process
the RGB image and point cloud data to extract the relevant features. Feel free to explore other code
libraries and techniques for perception
"""

import cv2
import time
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
from cv2 import aruco

def pts_dist(pt1, pt2):
    return ((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)**0.5

# capture color and depth data from realsense camera
def capture(date:str):
    # start and configure camera
    pipe = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    profile = pipe.start(config)

    profile = pipe.get_active_profile()
    device = profile.get_device()
    color_sensor = device.first_color_sensor()

    color_sensor.set_option(rs.option.enable_auto_exposure, 1)
    color_sensor.set_option(rs.option.enable_auto_white_balance, 1)

    # wait for the auto exposure and white balance to stabilize
    time.sleep(2)
    
    print("capturing data...")

    try:
        frames = pipe.wait_for_frames()

        # get color frame
        cf = frames.get_color_frame()
        if not cf:
            print('no color frame captured')
        cf.keep()
        color_frame = cf
    finally:
        print("finished capturing frame")
        pipe.stop()
    
    # save color data
    print("saving data...")
    color_img = np.asanyarray(color_frame.get_data())
    # color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
    cv2.imwrite('color-img-' + date + '.png', color_img)
    print('saved color image')

    return color_img

# un-warp image and crop to aruco marker
def transform_img(color_path, show=False):
    img = cv2.imread(color_path)

    if show:
        plt.figure(figsize=(16,9))
        plt.imshow(img)
        plt.title('Original Color Image')
        plt.show()

    # aruco imports
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    detector_params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, detector_params)

    # detect aruco markers
    corners, ids, rejected = detector.detectMarkers(img)
    markers_img = img.copy()
    aruco.drawDetectedMarkers(markers_img, corners, ids)
    
    if show:
        plt.figure(figsize=(16,9))
        plt.imshow(markers_img)
        plt.title('Detected ArUco markers')
        plt.show()
    
    # parameters of final image
    # TODO: change these?
    width = 19
    height = 12
    ppi = 96

    # parameters of initial images
    color_width = 1920
    color_height = 1080

    # sort markers and define source and destination points
    ids = ids.flatten()
    corners = np.array([corners[i] for i in np.argsort(ids)])
    corners = np.squeeze(corners)
    ids = np.sort(ids)
    # print("corners", corners)
    src_pts = np.array([corners[0][1], corners[1][2], corners[2][3], corners[3][0]], dtype='float32')
    # print(src_pts)
    dst_pts = np.array([[0, 0], [0, height*ppi], [width*ppi, height*ppi], [width*ppi, 0]], dtype='float32')

    if show:
        markers_img = img.copy()
        # aruco.drawDetectedMarkers(markers_img, corners, ids)
        plt.figure(figsize=(16,9))
        plt.imshow(markers_img)
        plt.plot(*zip(*src_pts), marker='o', color='r', ls='')
        plt.title('Detected ArUco markers')
        plt.show()

    # do the transform and return the corrected image
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    corrected_color_img = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    corrected_color_img = corrected_color_img[:int(height*ppi), :int(width*ppi)]
    corrected_color_img = cv2.cvtColor(corrected_color_img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(color_path[:-4] + '-corrected-2.png', corrected_color_img)

    if show:
        plt.imshow(corrected_color_img)
        plt.title('Corrected IMG')
        plt.show()

    return corrected_color_img

# extract features from color image
def extract_2d_features(color_path, show=False):
    img = cv2.imread(color_path)

    # flatten data
    img_data = img.reshape((-1, 3))
    img_data = np.float32(img_data)

    # run k-means clustering
    k = 8 # define number of clusters TODO
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.5)
    _, labels, centers = cv2.kmeans(img_data, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
    centers = np.uint8(centers)
    kmeans_data = centers[labels.flatten()]
    kmeans_img = kmeans_data.reshape(img.shape)
    labels = labels.reshape(img.shape[:2])

    # find objects
    colors = [
            #   ("green", np.array([0, 101, 55])), 
            #   ("red", np.array([96, 9, 5])),
              ("yellow", np.array([0, 132, 166])),
              ("orange", np.array([0, 52, 172])),
            #   ("pink", np.array([241, 101, 122])),
              ("turquoise", np.array([116, 127, 23])),
            #   ("blue", np.array([0, 105, 237])),
            #   ("dark blue", np.array([11, 13, 200])),
            #   ("purple", np.array([13, 12, 14]))
                ('block', np.array([108, 141, 155]))
            ]
    
    objects = []
    for color_name, color_value in colors:
        distances = np.linalg.norm(centers - color_value, axis=1)
        cluster_label = np.argmin(distances)
    
        # all pixels that belong to this cluster will be white, and all others will be black
        mask_img = np.zeros(kmeans_img.shape[:2], dtype='uint8')
        mask_img[labels == cluster_label] = 255
        contours, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # visualize the contours
        contour_img = img.copy()
        # cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 3)

        for c in contours:
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            cv2.drawContours(contour_img, [box], 0, (0, 255, 0), 2)

        if show:
            plt.imshow(cv2.cvtColor(contour_img, cv2.COLOR_BGR2RGB))
            plt.title(f'Contour image for cluster {cluster_label} corresponding to {color_name}')
            plt.gca().invert_yaxis()
            plt.show()

        # define features
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, closed=True)
            if area < 5000 or area > 100000:
                continue
            if perimeter < 100:
                continue

            roundness = 4 * np.pi * area / perimeter**2
            # print('roundness:', roundness)
            is_square = roundness < 0.7
            is_block = area < 30000
            if is_block:
                print(area, perimeter)

            # calculate position
            x, y, w, h = cv2.boundingRect(contour)
            u = x + w/2
            v = y + h/2

            orientation = None
            box = None
            # calculate orientation of square
            if is_square:
                rect = cv2.minAreaRect(contour)
                orientation = rect[2]
            if is_block:
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                length1 = pts_dist(box[0], box[1])
                length2 = pts_dist(box[1], box[2])
                if length1 < length2:
                    orientation += 90 # ORIENTATION GOING CW FROM VERTICAL


            # add to list of objects
            shape = "square" if is_square else "circle"
            shape = 'block' if is_block else shape
            print(shape, color_name)

            if color_name == 'block':
                if shape == 'block':
                    print("found a block")
                    objects.append(
                        {
                            "color" : color_name,
                            "shape" : shape,
                            "size" : area,
                            "position": {"x": u, "y": v},
                            "orientation": orientation,
                            "box": box
                        }
                    )
                else:
                    print("color is block but shape is not block")
            else:
                if shape != 'block':
                    objects.append(
                        {
                            "color" : color_name,
                            "shape" : shape,
                            "size" : area,
                            "position": {"x": u, "y": v},
                            "orientation": orientation,
                            "box": box
                        }
                    )
    print("number of objects found:", len(objects))
    for o in objects:
        print(o)
    return objects

def annotate_features(img_path, annotated_path, objects):
    # define colors
    colors = {
        "orange": (0, 37, 149),
        "turquoise": (123, 129, 17),
        "purple": (16, 11, 14),
        "green": (55, 101, 0),
        "yellow": (0, 132, 166),
        "block": (0, 0, 0)
    }
    font = cv2.FONT_HERSHEY_SIMPLEX # for text
    img = cv2.imread(img_path) # read in image

    # annotate each object
    for object in objects:
        # extract features
        color = object["color"]
        shape = object["shape"]
        area = object["size"]
        x, y = (int(object["position"]["x"]), int(object["position"]["y"]))
        orientation = object["orientation"]
        box = object["box"]

        if shape == "circle":
            side_length = np.sqrt(area)
            # Ask candace about side_length
            radius = int((area / np.pi)**0.5)
            img = cv2.circle(img, center=(x, y), radius=radius, color=(255, 0, 0), thickness=5)
            img = cv2.putText(img, text=f"{color} {shape}, ({x}, {y})", 
                              org=(x-25, y + int(side_length / 2) + 25), 
                              fontScale=1, fontFace=font, color=colors[color], thickness=1)
        elif shape == "square":
            side_length = int(area**0.5)
            square = ((x, y), (side_length, side_length), orientation)
            square = cv2.boxPoints(square)
            square = np.intp(square)
            cv2.drawContours(img, [square], 0, (255, 0, 0), 5)
            img = cv2.putText(img, text=f"{color} {shape}, ({x}, {y}), {orientation}", 
                              org=(x-25, y + int(side_length / 2) + 25), 
                              fontScale=1, fontFace=font, color=colors[color], thickness=1)
        else: 
            cv2.drawContours(img, [box], 0, (255, 0, 0), 5)
            img = cv2.putText(img, text=f"{color} {shape}, ({x}, {y}), {orientation}", 
                              org=(x-50, y + 35), 
                              fontScale=1, fontFace=font, color=(255, 255, 255), thickness=1)
    cv2.imwrite(annotated_path, img)
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.title("Annotated Image")
    # plt.gca().invert_yaxis()
    plt.show()

if __name__ == '__main__':
    # color_img = capture('4-21-1')
    transform_img('color-img-4-21-2.png', show=True)

    objects = extract_2d_features('color-img-4-21-2-corrected.png', show=True)
    annotate_features('color-img-4-21-2-corrected.png', 'color-img-4-21-2-corrected-annotated.png', objects)
