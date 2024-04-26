import compas_rrc as rrc
import compas.geometry as cg
import json
import math

# task_frame = cg.Frame.from_points([248.49, 192.44, 26.81], [-229.4, 192.41, 24], [247.98, 494.92, 26.35])
task_frame = cg.Frame.from_points([244.76, 189.79, 25], [-228.99, 203.04, 25], [249.35, 489.06, 25])
speed = 150

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
    # print("T is", T)
    ee_frame_t.transform(T)
    ee_frame_w = ee_frame_t
    # print("ee_frame_w is", ee_frame_w)
    
    # ====================================================================================
    return ee_frame_w

def move_to_t_point(abb_rrc, x: float, y: float, z: float):
    """
    Move end effector to a point in the task frame. 
    """
    ppi = 96
    inch_to_mm = 25.4
    
    x = x / ppi * inch_to_mm
    y = y / ppi * inch_to_mm
    x = x + 5
    y = y + 7
    # z = z / ppi * inch_to_mm
    print("moving to point: x", x, "y", y, "z", z)
    # Convert task frame position to world frame position
    ee_frame_w = abb_rrc.send_and_wait(rrc.GetFrame())
    ee_frame_t = cg.Frame([x, y, z], [1, 0, 0], [0, 1, 0]) # where we want to move the EE to
    ee_frame_w = transform_task_to_world_frame(ee_frame_t, task_frame) 
    print("after transformation, ee_frame_w is ", ee_frame_w)
    # Move the robot to the new position
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(ee_frame_w, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))

def move_down_keep_rotation(abb_rrc, dz, orientation):
    print("Rotation in degrees", orientation)
    angle = math.radians(orientation)
    print("Rotation in radians", angle)
    current_frame = abb_rrc.send_and_wait(rrc.GetFrame())
    rotation = cg.Rotation.from_axis_and_angle([0, 0, 1], angle, point=current_frame.point)
    # Apply the rotation to the current end effector frame
    rotated_frame = current_frame.transformed(rotation)

    point = rotated_frame.point
    point[2] += dz
    rotated_frame.point = point
    # Move the robot to the rotated frame
    print("move keep rotation, moving to frame", rotated_frame)
    abb_rrc.send_and_wait(rrc.MoveToFrame(rotated_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))

def move_to_t_point_rhino(abb_rrc, x: float, y: float, z: float):
    """
    Move end effector to a point in the task frame. 
    """
    print("moving to point rhino: x", x, "y", y, "z", z)
    # Convert task frame position to world frame position
    ee_frame_w = abb_rrc.send_and_wait(rrc.GetFrame())
    ee_frame_t = cg.Frame([x, y, z], [1, 0, 0], [0, 1, 0]) # where we want to move the EE to
    ee_frame_w = transform_task_to_world_frame(ee_frame_t, task_frame) 
    print("after transformation, ee_frame_w is ", ee_frame_w)

    # Move the robot to the new position
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(ee_frame_w, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))
    

def pick_and_place(abb_rcc, object, ground_objects):
    # extract current object's features
    shape = object["shape"]
    color = object["color"]
    rotation = object["rotation"]
    x = object["position"][0]
    y = object["position"][1]
    z = object["position"][2]
    place_position = (x, y, z)
    pick_location = [0, 0, 0]
    pick_angle = 0

    # get an appropriate shape from the perception (list of dicts)
    # ground_objects = []
    # get object with same shape and color as current object from ground_objects
    for obj in ground_objects:
        if obj["shape"] == shape and obj["color"] == color:
            pick_location = [obj["position"]["x"], obj["position"]["y"], 0]
            pick_angle = obj["orientation"]
            # blocks are 13 mm tall, others are 3 mm tall (rounded up)
            if shape == "block":
                pick_location[2] = -13
            else:
                pick_location[2] = -1
            ground_objects.remove(obj)
            break

    if pick_angle is None:
        pick_angle = 0

    print("PICK ANGLE IS")
    print(pick_angle)

    # find where the object is and pick it up
    print("picking", color, shape)
    pick_object(abb_rcc, pick_location, shape, pick_angle)

    # place the object according to position specified in object
    print("placing", color, shape)
    place_object(abb_rcc, place_position, shape, rotation)

    print("-----------------------------------------------------")


def pick_object(abb_rrc, pick_location, shape, angle):
    # move to object, then down on object
    x = pick_location[0]
    y = pick_location[1]
    z = pick_location[2]
    # get orientation
    # one block is about 3.125 mm
    move_to_t_point(abb_rrc, x, y, z - 20)
    angle = 90 - angle

    # if shape == "block":
    #     # rotate gripper to match block
    #     current_frame = abb_rrc.send_and_wait(rrc.GetFrame())
    #     # move_keep_rotation(abb_rrc, x, y, z, orientation)
    #     # # create rotation transformation around Z axis at the current location
    #     # # rotation is already in radians
    #     print("Degrees is:", angle)
    #     orientation = math.radians(angle)
    #     print("Rotating it by radians:", orientation)
    #     rotation = cg.Rotation.from_axis_and_angle([0, 0, 1], orientation, point=current_frame.point)
    #     # # Apply the rotation to the current end effector frame
    #     rotated_frame = current_frame.transformed(rotation)
    #     # # Move the robot to the rotated frame
    #     print("rotating to frame", rotated_frame)
    #     abb_rrc.send_and_wait(rrc.MoveToFrame(rotated_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # Move the robot downwards in the z-axis
    # move_keep_rotation(abb_rrc, x, y, z, angle)
    print("moving down")
    if shape == "circle":
        move_to_t_point(abb_rrc, x, y, z)
    else:
        # square and block
        move_down_keep_rotation(abb_rrc, -20, angle - 20)

    # turn gripper on
    print("turning vacuum on")
    abb_rrc.send_and_wait(rrc.SetDigital('DO00', 1))
    abb_rrc.send_and_wait(rrc.WaitTime(1.0))
    # move object upwards
    move_to_t_point(abb_rrc, x, y, z - 20)


    # rotate it back to 0 degrees/original orientation
    # current_frame = abb_rrc.send_and_wait(rrc.GetFrame())
    # # # rotation is already in radians
    # orientation = -1 * math.radians(angle)
    # rotation = cg.Rotation.from_axis_and_angle([0, 0, 1], orientation, point=current_frame.point)
    # # # Apply the rotation to the current end effector frame
    # rotated_frame = current_frame.transformed(rotation)
    # # # Move the robot to the rotated frame
    # print("rotating to frame", rotated_frame)
    # abb_rrc.send_and_wait(rrc.MoveToFrame(rotated_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # Move the robot 50 mm upwards in the z-axis
    # move_keep_rotation(abb_rrc, x, y, z - 20, orientation)


def place_object(abb_rrc, place_position, shape, angle):
    # object should ideally be on robot's grip at this point
    x = place_position[0] 
    y = place_position[1] 
    z = - (place_position[2])

    # move item on top of largest object (base of pile)'s position
    move_to_t_point_rhino(abb_rrc, x, y, z - 20)

     # move gripper down
    move_to_t_point_rhino(abb_rrc, x, y, z - 1)
    
    # rotate object by angle
    if shape == "block" or shape == "square":
        current_frame = abb_rrc.send_and_wait(rrc.GetFrame())
        # create rotation transformation around Z axis at the current location
        # rotation is already in radians
        # angle = math.radians(angle)
        print("Rotating it by radians:", angle)
        rotation = cg.Rotation.from_axis_and_angle([0, 0, 1], angle, point=current_frame.point)
        # Apply the rotation to the current end effector frame
        rotated_frame = current_frame.transformed(rotation)
        # Move the robot to the rotated frame
        abb_rrc.send_and_wait(rrc.MoveToFrame(rotated_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))
        
    # # turn gripper off
    abb_rrc.send_and_wait(rrc.SetDigital('DO00', 0))
    abb_rrc.send_and_wait(rrc.WaitTime(0.5))

    # move gripper up
    move_to_t_point_rhino(abb_rrc, x, y, z - 20)

    # Reset gripper to 0 rotation

def convert_clockwise_to_counterclockwise(angle):
    """
    Converts an angle from clockwise to counterclockwise notation.

    Parameters:
    angle (float): An angle in degrees, measured clockwise from the positive x-axis.

    Returns:
    float: The equivalent angle in degrees, measured counterclockwise from the positive x-axis.
    """
    # Normalize the input angle to be within the range [0, 360)
    angle = angle % 360
    
    # Convert to counterclockwise
    counterclockwise_angle = (360 - angle) % 360

    return counterclockwise_angle


# if __name__ == '__main__':

#     # Create Ros Client
#     ros = rrc.RosClient()
#     ros.run()

#     # Create ABB Client
#     abb_rrc = rrc.AbbClient(ros, '/rob1-rw6')
#     print('Connected to ROS.')

#     # ================================== YOUR CODE HERE ==================================

#     # Set tools
#     abb_rrc.send(rrc.SetTool('vac_gripper'))
#     print('set tool to vac_gripper.')


#     # Set speed [mm/s]
#     speed = 150 # 
#     print("speed set to", speed)

#     home = rrc.RobotJoints([0, 0, 0, 0, 90, 0])
#     print("about to send, home is", home)
#     done = abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))
#     print("moved to home position.")

#     # ====================================================================================

#     # Define the task space points: top left, top right, bottom left

#     # read in simpletower.json which is a list of dicts
#     with open("simpletower_sorted.json") as json_file:
#         tower_json = json.load(json_file)


#     # ====================================================================================

#     # End of Code
#     print('Finished')

#     # Close client
#     ros.close()
#     ros.terminate()