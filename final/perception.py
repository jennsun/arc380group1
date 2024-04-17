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
import open3d as o3d

# capture color and depth data from realsense camera
def capture(date:str):
    # start and configure camera
    pipe = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
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
        
        # get depth frame
        df = frames.get_depth_frame()
        if not df:
            print('no depth frame captured')
        df.keep()
        depth_frame = df
    finally:
        print("finished capturing frame")
        pipe.stop()
    
    # save color data
    print("saving data...")
    color_img = np.asanyarray(color_frame.get_data())
    color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
    cv2.imwrite('color-img-' + date + '.png', color_img)

    # save depth data
    depth_data = np.asanyarray(depth_frame.get_data())
    cv2.imwrite('depth-img-' + date + '.png', depth_data)
    depth_file = 'depth-data-' + date +'.npy'
    with open(depth_file, 'wb') as f:
        np.save(f, depth_data)
    print('saved color image and depth data')

    return color_img, depth_data

# un-warp image and crop to aruco marker
def transform_img(color_path, depth_path, show=False):
    img = cv2.imread(color_path)
    depth_img = cv2.imread(depth_path)
    depth_data = None

    with open('depth-data-4-15.npy', 'rb') as f:
        depth_data = np.load(f)
        plt.imshow(depth_data)
        plt.show()
    print(depth_data.shape)

    if show:
        plt.figure(figsize=(16,9))
        plt.imshow(img)
        plt.title('Original Color Image')
        plt.show()

        plt.figure(figsize=(16,9))
        plt.imshow(depth_img)
        plt.title('Original Depth Image')
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
    # TODO: change these
    width = 19
    height = 12
    ppi = 96

    # parameters of initial images
    color_width = 1920
    color_height = 1080
    depth_width = 640
    depth_height = 480

    # sort markers and define source and destination points
    ids = ids.flatten()
    corners = np.array([corners[i] for i in np.argsort(ids)])
    corners = np.squeeze(corners)
    ids = np.sort(ids)
    print("corners", corners)
    src_pts = np.array([corners[0][1], corners[1][2], corners[2][3], corners[3][0]], dtype='float32')

    print(src_pts)
    dst_pts = np.array([[0, 0], [0, height*ppi], [width*ppi, height*ppi], [width*ppi, 0]], dtype='float32')

    # do the transform and return the corrected image
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    corrected_color_img = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    # corrected_img = corrected_img[:int(height*ppi), :int(width*ppi)]
    cv2.imwrite(color_path[:-4] + '-corrected.png', corrected_color_img)

    # corrected_depth_img = cv2.warpPerspective(depth_img, M, (depth_img.shape[1], depth_img.shape[0]))
    # cv2.imwrite(depth_path[:-4] + '-corrected.png', corrected_depth_img)
    if show:
        plt.imshow(corrected_color_img)
        plt.title('Corrected IMG')
        plt.show()

        # plt.imshow(corrected_depth_img)
        # plt.title('Corrected Depth IMG')
        # plt.show()

    return corrected_color_img# , corrected_depth_img

# create point cloud from depth data
def get_point_cloud(depth_path, pcd_path):
    depth_img = cv2.imread(depth_path)

    # define points
    width, height = depth_img.shape[1], depth_img.shape[0]
    x = np.linspace(-0.5, 0.5, width)
    y = np.linspace(-0.5, 0.5, height)
    xx, yy = np.meshgrid(x, y)
    points = np.vstack(xx.flatten(), yy.flatten(), depth_img.flatten())

    # create point cloud
    pcd = o3d.geoemtry.PointCloud()
    pcd.points = o3d.utility.Vector3DVector(points)
    pcd.paint_uniform_coor([0, 0, 1]) # paint all points blue

    # visualize and save point cloud
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([pcd, coordinate_frame])
    o3d.io.write_point_cloud(pcd_path, pcd)

    return pcd

# remove background and outliers from point cloud
def filter_pcd(pcd, filtered_pcd_path):
    max_distance = 0.5 # max distance in meters in image
    np_pcd = np.asarray(pcd.points)

    # remove background
    within_dist_idx = np.where(np.abs(np_pcd[:,2]) < max_distance)[0]
    filtered_pcd = pcd.select_by_index(within_dist_idx)

    # remove outliers
    nb_neighbors = 20
    std_ratio = 2
    filtered_pcd, idx = filtered_pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    
    # visualize
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([filtered_pcd, coordinate_frame])
    o3d.io.write_point_cloud(filtered_pcd_path, filtered_pcd)

    return filtered_pcd

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
              ("yellow", np.array([166, 132, 0])),
              ("orange", np.array([172, 52, 0])),
            #   ("pink", np.array([241, 101, 122])),
              ("turquoise", np.array([23, 127, 116])),
            #   ("blue", np.array([0, 105, 237])),
            #   ("dark blue", np.array([11, 13, 200])),
            #   ("purple", np.array([13, 12, 14]))
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
        cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 3)

        if show:
            plt.imshow(cv2.cvtColor(contour_img, cv2.COLOR_BGR2RGB))
            plt.title(f'Contour image for cluster {cluster_label} corresponding to {color_name}')
            plt.gca().invert_yaxis()
            plt.show()

        # define features
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, closed=True)
            if area < 10000:
                continue
            if perimeter < 100:
                continue

            roundness = 4 * np.pi * area / perimeter**2
            is_square = roundness < 0.78
            # TODO: find roundness of bricks (probably less than square?)

            # calculate position
            x, y, w, h = cv2.boundingRect(contour)
            u = x + w/2
            v = y + h/2

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

# extract block features from pointcloud
def extract_3d_features(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)

    # use RANSAC to fit a plane and locate the table surface
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f'Plane model: {a}x + {b}y + {c}z + {d} = 0')

    # visualize the inliers and outliers of the plane
    inlier_pcd = pcd.select_by_index(inliers)
    inlier_pcd.paint_uniform_color([1, 0, 0])
    print(f'Plane inliers point cloud has {len(inlier_pcd.points)} points.')

    outlier_pcd = pcd.select_by_index(inliers, invert=True)
    print(f'Plane outliers point cloud has {len(outlier_pcd.points)} points.')

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([inlier_pcd, outlier_pcd, coordinate_frame])

    # basic segmentation of the outlier point cloud using DBSCAN clustering
    labels = outlier_pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True)
    labels = np.asarray(labels)
    print(f'Found {len(np.unique(labels))} clusters.')

    # visualize the clusters in different colors
    clusters = []
    centroids = []
    for i in range(len(labels)):
        cluster = outlier_pcd.select_by_index(np.where(labels == i)[0])
        random_color = list(np.random.choice(range(256), size=3))
        cluster.paint_uniform_color(random_color)

        points = cluster.points
        centroid = np.mean(points, axis=1) # TODO: check axis
        centroids.append(centroid)

    o3d.visualization.draw_geometries(clusters.append(coordinate_frame))
    # TODO: get orientation of clusters? (or get this from 2D image?)

    return clusters, centroids


if __name__ == '__main__':
    color_img, depth_data = capture('4-17')
    transform_img('color-img-4-17.png', 'depth-img-4-17.png', show=False)

    # extract_2d_features('color-img-4-15.png', show=True)
