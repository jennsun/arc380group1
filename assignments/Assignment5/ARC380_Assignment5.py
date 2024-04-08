import compas_rrc as rrc
import compas.geometry as cg
import math
from Assignment5P2 import get_img, extract_features, get_features

# Assuming there is a robot object already initialized

y_offset = -13

def transform_task_to_world_frame(ee_frame_t: cg.Frame, task_frame: cg.Frame) -> cg.Frame:
    """Transform a task frame to the world frame.

    Args:
        ee_frame_t (cg.Frame): The end-effector frame defined in task space.
        task_frame (cg.Frame): The task frame.

    Returns:
        cg.Frame: The task frame in the world frame.
        FIX: Returning The END EFFECTOR frame in the world frame
    """
    ee_frame_w = None
    # ================================== YOUR CODE HERE ==================================

    # Part 1.d.
    # transform a target end effector frame from task space to world frame
    T = cg.Transformation.from_frame(task_frame)
    print("T is", T)
    ee_frame_t.transform(T)
    ee_frame_w = ee_frame_t
    print("ee_frame_w is", ee_frame_w)
    
    # ====================================================================================
    return ee_frame_w

def move_to_t_point(abb_rrc, x: float, y: float, z: float):
    """
    Move end effector to a point in the task frame. 
    """
    ppi = 96
    inch_to_mm = 25.4
    
    x = x / ppi * inch_to_mm
    y = y / ppi * inch_to_mm - y_offset
    # z = z / ppi * inch_to_mm
    print("x", x, "y", y, "z", z)
    # Convert task frame position to world frame position
    ee_frame_w = abb_rrc.send_and_wait(rrc.GetFrame())
    ee_frame_t = cg.Frame([x, y, z], [1, 0, 0], [0, 1, 0]) # where we want to move the EE to
    ee_frame_w = transform_task_to_world_frame(ee_frame_t, task_frame) 

    # Move the robot to the new position
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(ee_frame_w, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))
    print("moved to new position: ", x, y, z)

def pick_object(abb_rrc, object):
    # move to object, then down on object
    # one block is about 3.125 mm
    print("pick object", object)
    move_to_t_point(abb_rrc, object["position"]["x"], object["position"]["y"], -20)
    move_to_t_point(abb_rrc, object["position"]["x"], object["position"]["y"], 14.0)
    # turn gripper on
    abb_rrc.send_and_wait(rrc.SetDigital('DO00', 1))
    abb_rrc.send_and_wait(rrc.WaitTime(1.0))
    # move object upwards
    move_to_t_point(abb_rrc, object["position"]["x"], object["position"]["y"], -20)


def place_object(abb_rrc, object, largest_object_position, angle):
    print("place object", object)
    # object should ideally be on robot's grip at this point
    x = largest_object_position[object["color"]][0]
    y = largest_object_position[object["color"]][1]
    z = largest_object_position[object["color"]][2]
    print("pile position is", x, y, z)
    # move item on top of largest object (base of pile)'s position
    move_to_t_point(abb_rrc, x, y, z - 12)
    
    # rotate object by angle
    # TODO: CHECK ROTATION IMPLEMENTATION
    if object["shape"] == "square":
        current_frame = abb_rrc.send_and_wait(rrc.GetFrame())
        # create rotation transformation around Z axis at the current location
        angle = math.radians(angle)
        rotation = cg.Rotation.from_axis_and_angle([0, 0, 1], angle, point=current_frame.point)
        # Apply the rotation to the current end effector frame
        rotated_frame = current_frame.transformed(rotation)
        # Move the robot to the rotated frame
        abb_rrc.send_and_wait(rrc.MoveToFrame(rotated_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))
    else:
        # move gripper down
        move_to_t_point(abb_rrc, x, y, z + 2.5)
        
    # # turn gripper off
    abb_rrc.send_and_wait(rrc.SetDigital('DO00', 0))
    abb_rrc.send_and_wait(rrc.WaitTime(0.5))

    # move gripper up
    move_to_t_point(abb_rrc, x, y, z - 20)

    # update values since pile height has increased. 1/8 inch is 3.175 mm
    largest_object_position[object["color"]] = (x, y, z - 2)


def sort_objects_into_piles(abb_rrc, objects):
    # object = {
    #     "color": color,
    #     "shape": shape,
    #     "size": (w, h),
    #     "position": {"x": x_mm, "y": y_mm},
    #     "orientation": angle
    # }
    # get the mapping of the color to the position coordinates of the LARGEST object of each color

    objects = sorted(objects, key=lambda x:x['size'], reverse=True)
    largest_object_position = {}
    largest_object_size = {}
    for obj in objects:
        color = obj["color"]
        size = obj["size"]
        shape = obj["shape"]
        x = obj["position"]["x"]
        y = obj["position"]["y"]

        # bases should be circles
        if color not in largest_object_position:
            # tuple consists of x coordinate, y coordinate, and height to place next block
            largest_object_position[color] = (x, y, 3.125)
            largest_object_size[color] = size
        else:
            if size > largest_object_size[color] and shape == "circle":
                largest_object_position[color] = (x, y, 3.125)

    # sort all the objects by size in decreasing order (larger to small)
    objects = sorted(objects, key=lambda x: x["size"], reverse=True)

    for obj in objects:
        color = obj["color"]
        size = obj["size"]
        x = obj["position"]["x"]
        y = obj["position"]["y"]
        angle = obj["orientation"]

        # if object is biggest object of that color, skip
        if x == largest_object_position[color][0] and y == largest_object_position[color][1]:
            continue

        pick_object(abb_rrc, obj)
        place_object(abb_rrc, obj, largest_object_position, angle)


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
    speed = 150 # 
    print("speed set to", speed)

    home = rrc.RobotJoints([0, 0, 0, 0, 90, 0])
    print("about to send, home is", home)
    done = abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))
    print("moved to home position.")

    # Define the task space points: top left, top right, bottom left
    # task_frame = cg.Frame.from_points([254.71, 192.44, 17.51], [-229.4, 192.41, 15.51], [253.71, 491.79, 19.13])
    task_frame = cg.Frame.from_points([248.49, 192.44, 26.81], [-229.4, 192.41, 24], [247.98, 494.92, 26.35])
    # move_to_t_point(abb_rrc, x=971, y=626, z=30)
    # top left: [254.71, 192.44, 17.51]
    # top right: [-229.4, 192.41, 15.51]
    # bottom left: [253.71, 491.79, 19.13] 
    # height: 30cm is 11.81 inches (12)
    # width: 48cm is 18.89 inches (19)

    # Define the base pose of the first pile
    # path = "test-image-4-7.png"
    # img = get_img(path)
    objects = get_features() # CANDACE'S CODE
    print("objects is", objects)
    sort_objects_into_piles(abb_rrc, objects)

    # ====================================================================================

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
