import compas_rrc as rrc
import compas.geometry as cg
import json
import math

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
    y = y / ppi * inch_to_mm
    # z = z / ppi * inch_to_mm
    print("x", x, "y", y, "z", z)
    # Convert task frame position to world frame position
    ee_frame_w = abb_rrc.send_and_wait(rrc.GetFrame())
    ee_frame_t = cg.Frame([x, y, z], [1, 0, 0], [0, 1, 0]) # where we want to move the EE to
    ee_frame_w = transform_task_to_world_frame(ee_frame_t, task_frame) 

    # Move the robot to the new position
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(ee_frame_w, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))
    print("moved to new position: ", x, y, z)

def pick_and_place(abb_rcc, object):
    # extract current object's features
    shape = object["shape"]
    color = object["color"]
    rotation = object["rotation"]
    x = object["position"][0]
    y = object["position"][1]
    z = object["position"][2]
    place_position = (x, y, z)

    # get an appropriate shape from the perception (list of dicts)
    ground_objects = []
    # get object with same shape and color as current object from ground_objects
    for obj in ground_objects:
        if obj["shape"] == shape and obj["color"] == color:
            pick_location = obj["position"]
            # blocks are 13 mm tall, others are 3 mm tall (rounded up)
            if shape == "block":
                pick_location[2] = pick_location[2] - 15
            else:
                pick_location[2] = pick_location[2] - 5
            ground_objects.remove(obj)
            break

    # find where the object is and pick it up
    pick_object(abb_rcc, pick_location)

    # place the object according to position specified in object
    place_object(abb_rcc, place_position, rotation)


def pick_object(abb_rrc, pick_location):
    # move to object, then down on object
    x = pick_location[0]
    y = pick_location[1]
    z = pick_location[2]
    # one block is about 3.125 mm
    print("pick object", object)
    # move_to_t_point(abb_rrc, x, y, z -20)
    move_to_t_point(abb_rrc, x, y, z)
    # turn gripper on
    abb_rrc.send_and_wait(rrc.SetDigital('DO00', 1))
    abb_rrc.send_and_wait(rrc.WaitTime(1.0))
    # move object upwards
    move_to_t_point(abb_rrc, x, y, z -20)


def place_object(abb_rrc, place_position, shape, angle):
    print("place object", object)
    # object should ideally be on robot's grip at this point
    x = place_position[0]
    y = place_position[1]
    z = place_position[2]

    # move item on top of largest object (base of pile)'s position
    move_to_t_point(abb_rrc, x, y, z - 3)
    
    # rotate object by angle
    # TODO: CHECK ROTATION IMPLEMENTATION
    if shape == "block" or shape == "square":
        current_frame = abb_rrc.send_and_wait(rrc.GetFrame())
        # create rotation transformation around Z axis at the current location
        # rotation is already in radians
        # angle = math.radians(angle)
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

    # ====================================================================================

    # Define the task space points: top left, top right, bottom left
    task_frame = cg.Frame.from_points([248.49, 192.44, 26.81], [-229.4, 192.41, 24], [247.98, 494.92, 26.35])

    # read in simpletower.json which is a list of dicts
    with open("simpletower_sorted.json") as json_file:
        tower_json = json.load(json_file)


    # ====================================================================================

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()