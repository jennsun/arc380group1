import compas_rrc as rrc
import compas.geometry as cg

# Assuming there is a robot object already initialized

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
    # Convert task frame position to world frame position
    ee_frame_w = abb_rrc.send_and_wait(rrc.GetFrame())
    ee_frame_t = cg.Frame([x, y, z], [1, 0, 0], [0, -1, 0]) # where we want to move the EE to
    ee_frame_w = transform_task_to_world_frame(ee_frame_t, task_frame) 

    # Move the robot to the new position
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(ee_frame_w, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))
    print("moved to new position: ", x, y, z)

def pick_object(abb_rrc, object):
    # move to object, then down on object
    move_to_t_point(abb_rrc, object["position"]["x"], object["position"]["y"], 0.3)
    move_to_t_point(abb_rrc, object["position"]["x"], object["position"]["y"], 0.1)
    # turn gripper on
    abb_rrc.send_and_wait(rrc.SetDigital('DO00', True))
    abb_rrc.send_and_wait(rrc.WaitTime(0.5))
    # move object upwards
    move_to_t_point(abb_rrc, object["position"]["x"], object["position"]["y"], 0.3)


def place_object(abb_rrc, object, largest_object_position, angle):
    # object should ideally be on robot's grip at this point
    x = largest_object_position[object["color"]][0]
    y = largest_object_position[object["color"]][1]
    z = largest_object_position[object["color"]][2]

    # move item on top of largest object (base of pile)'s position
    move_to_t_point(abb_rrc, x, y, z + 0.3)
    # move gripper down
    move_to_t_point(abb_rrc, x, y, z + 0.1)
    # rotate object by angle
    # TODO: IMPLEMENT THIS
    
    # turn gripper off
    abb_rrc.send_and_wait(rrc.SetDigital('DO00', False))
    abb_rrc.send_and_wait(rrc.WaitTime(0.5))

    # move gripper up
    move_to_t_point(abb_rrc, x, y, z + 0.3)

    # update values since pile height has increased. 1/8 inch is 3.175 mm
    largest_object_position[object["color"]] = (x, y, z + 3.175)


def sort_objects_into_piles(abb_rrc, objects):
    # object = {
    #     "color": color,
    #     "shape": shape,
    #     "size": (w, h),
    #     "position": {"x": x_mm, "y": y_mm},
    #     "orientation": angle
    # }
    # get the mapping of the color to the position coordinates of the LARGEST object of each color
    largest_object_position = {}
    for obj in objects:
        color = obj["color"]
        size = obj["size"]
        x = obj["position"]["x"]
        y = obj["position"]["y"]

        if color not in largest_object_position:
            # tuple consists of x coordinate, y coordinate, and height to place next block
            largest_object_position[color] = (x, y, 0)
        else:
            if size > largest_object_position[color]:
                largest_object_position[color] = (x, y, 0)

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
    speed = 80 # 
    print("speed set to", speed)

    home = rrc.RobotJoints([0, 0, 0, 0, 90, 0])
    print("about to send, home is", home)
    done = abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))
    print("moved to home position.")

    # Define the base pose of the first pile
    objects = get_objects() # CANDACE'S CODE
    sort_objects_into_piles(abb_rrc, objects)

    # ====================================================================================

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
