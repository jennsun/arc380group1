import numpy as np
import compas.geometry as cg
import compas_rrc as rrc

# Define any additional imports here if needed
def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to a 3x3 rotation matrix
    :param quaternion: Quaternion as a NumPy array, in the format (x, y, z, w)
    :return: 3x3 rotation matrix as a NumPy array
    """
    matrix = None

    # ================================== YOUR CODE HERE ==================================

    # Using formula from https://www.songho.ca/opengl/gl_quaternion.html
    # Also on https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    w, x, y, z = quaternion

    matrix = np.array([[1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                          [2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
                            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]])
    
    # ====================================================================================

    return matrix



def create_frame_from_points(point1: cg.Point, point2: cg.Point, point3: cg.Point) -> cg.Frame:
    """Create a frame from three points.

    Args:
        point1 (cg.Point): The first point (origin).
        point2 (cg.Point): The second point (along x axis).
        point3 (cg.Point): The third point (within xy-plane).

    Returns:
        cg.Frame: The frame that fits the given 3 points.
    """
    frame = None
    # ================================== YOUR CODE HERE ==================================

    frame = cg.Frame(point1, point2, point3)

    # ====================================================================================
    return frame


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


# ====================== Drawing effects and helper functions ============================

# Part 2

# ========================================================================================


if __name__ == '__main__':

    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb_rrc = rrc.AbbClient(ros, '/rob1-rw6')
    print('Connected.')

    # ================================== YOUR CODE HERE ==================================

    # Set tools
    abb_rrc.send(rrc.SetTool('pen_group1'))
    print('set tool to pen_group1.')


    # Set speed [mm/s]
    print("setting speed")
    speed = 30 # start with slower speed at first
    print("speed set to", speed)

    # Go to home position (linear joint move)
    # robot_joints, external_axes = abb_rrc.send_and_wait(rrc.GetJoints())
    # print("robot_joints is", robot_joints)
    # print("external_axes is", external_axes)
    home = rrc.RobotJoints([0, 0, 0, 0, 90, 0])
    print("about to send, home is", home)
    done = abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))
    print("moved to home position.")

    # Parts 1.e. and 2
    # move the robot to (0, 0, 0) in the task space
    # Define the task frame
    task_frame = cg.Frame([424.52, 194.86, 29.45], [207.21, 190.83, 28.80], [204.03, 473.65, 31.88])
    print("task_frame is", task_frame)

    # Read current frame positions
    ee_frame_w = abb_rrc.send_and_wait(rrc.GetFrame())
    print(f'Frame = {ee_frame_w}')
    
    # Create a new frame at position (0, 0, 0)
    ee_frame_t = cg.Frame([0.0, 0.0, 0.0], ee_frame_w.xaxis, ee_frame_w.yaxis) # where we want to move the EE to
    print("ee_frame_t is", ee_frame_t)
    ee_frame_w = transform_task_to_world_frame(ee_frame_t, task_frame) 
    print("ee_frame_w is", ee_frame_w)

    # Move the robot to the new position
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(ee_frame_w, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))
    print("moved to new position.")

    
    """
     # Go to home position (linear joint move)
    home = rrc.RobotJoints([0, 0, 0, 0, 90, 0])
    done = abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))

    # Read current frame positions
    frame = abb_rrc.send_and_wait(rrc.GetFrame())
    print(f'Frame = {frame}')

    # Create a new frame with the same orientation but a different position
    new_frame = cg.Frame(cg.Point(340, 315, 275), frame.xaxis, frame.yaxis)

    # Move robot the new pos
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(new_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))

    """

    # ====================================================================================

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
