import numpy as np
import compas_rrc as rrc
import compas.geometry as cg
import json
import math
import cv2
import time
# import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
from cv2 import aruco


from perception import *
# from tower1movement import *
from towermovement import * 

task_frame = cg.Frame.from_points([248.49, 192.44, 26.81], [-229.4, 192.41, 24], [247.98, 494.92, 26.35])
speed = 900

if __name__ == '__main__':

    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb_rrc = rrc.AbbClient(ros, '/rob1-rw6')
    print('Connected to ROS.')

    # ================================== YOUR CODE HERE ==================================

    # Set tools
    abb_rrc.send(rrc.SetTool('vac_gripper'))
    print('set tool to vac_gripper.')


    # Set speed [mm/s]
    speed = 900 # 
    print("speed set to", speed)

    home = rrc.RobotJoints([0, 0, 0, 0, 90, 0])
    print("about to send, home is", home)
    done = abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))
    print("moved to home position.")

    # ====================================================================================

    # Define the task space points: top left, bottom left, top right
    task_frame = cg.Frame.from_points([248.49, 192.44, 25], [248.49, 494.92, 25], [-229.4, 192.44, 25])

    # add perception stuff
    date = '4-29-2'
    img = capture_mac(date)
    transform_img('color-img-' + date + '.png', show=False)
    ground_objects = extract_2d_features('color-img-' + date + '-corrected.png', show=False)
    annotate_features('color-img-' + date + '-corrected.png', 'color-img-' + date + '-corrected-annotated.png', ground_objects)

    # read in simpletower.json which is a list of dicts
    with open("json/tower1.json") as json_file:
        tower_objects = json.load(json_file)

    for object in tower_objects:
        pick_and_place(abb_rrc, object, ground_objects)

    # ====================================================================================

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()