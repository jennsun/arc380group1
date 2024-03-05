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
def move_to_t_point (x: float, y: float, z: float):
    """
    Move end effector to a point in the task frame. 
    """
    # Convert task frame position to world frame position
    ee_frame_w = abb_rrc.send_and_wait(rrc.GetFrame())
    ee_frame_t = cg.Frame([x, y, z], ee_frame_w.xaxis, ee_frame_w.yaxis) # where we want to move the EE to
    ee_frame_w = transform_task_to_world_frame(ee_frame_t, task_frame) 

    # Move the robot to the new position
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(ee_frame_w, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))
    print("moved to new position: ", x, y, z)

    """
    2a. Draw a rectangle given its dimensions, position, and rotation.
    """
def draw_rectangle (width, height, position, rotation):
    # Create a frame for the rectangle
    frame = Frame(position, (1, 0, 0), (0, 1, 0))

    # Rotate the frame around its z-axis
    frame.rotate(compas.geometry.Vector(0, 0, 1), rotation)

    # Create a box representing the rectangle
    rectangle = Box(frame, width, height, 0.1)

    # Get the vertices of the rectangle
    vertices = rectangle.vertices

    # Print the vertices coordinates
    for vertex in vertices:
        print(vertex)

    """
    2b. Draw any regular polygon given a number of sides and position.
    """
def draw_regular_polygon (num_sides, position):
    center = np.array(position) #convert the position to a numpy array
    radius = 1.0 #set the radius of the polygon 
    
    #calculate the vertices of the polygon
    vertices = []
    for i in range(num_sides):
        angle = i * (2 * np.pi / num_sides)
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        vertices.append([x, y])

    polygon = cg.Polygon(vertices)

    # Print the vertices coordinates
    for vertex in vertices:
        print(vertex)
        
def draw_dashed_line (start: cg.Point, end: cg.Point, dash_gap_ratio: float):
    """
    2g. Draw a dashed line given a start point, end point, and dash-gap ratio.
    """
    z = start.z
    points = []

    # define x and y coords of dashes
    x_vals = np.linspace(start.x, end.x, num = np.abs(start.x - end.x) // 5)
    y_vals = np.linspace(start.y, end.y, num=len(x_vals))

    # add dash and gap points to points array
    points.append([x_vals[0], y_vals[0]])
    for i in range(1, len(x_vals)):
        intermmediate_point = [(x_vals[i] - x_vals[i-1]) * dash_gap_ratio + x_vals[i-1], 
                               (y_vals[i] - y_vals[i-1]) * dash_gap_ratio + y_vals[i-1]]
        points.append(intermmediate_point)
        points.append([x_vals[i], y_vals[i]])

    # draw dashed line
    for i in range(len(points) - 1):
        if i % 2 == 0: # draw a dash between this point and next point
            move_to_t_point(points[i][0], points[i][1], z)
            move_to_t_point(points[i+1][0], points[i+1][1], z)
        else: # draw a gap between this point and next point
            move_to_t_point(points[i][0], points[i][1], z - 3) # TODO: check direction of positive z-axis
            move_to_t_point(points[i+1][0], points[i+1][1], z - 3)
    
def draw_hatch_pattern(corner1: cg.Point, corner2: cg.Point):
    """
    2j. Draw a hatch pattern given a rectangular boundary. 

    corner1: top left corner of rectangle. 
    corner2: bottom right corner of rectangle.
    """
    # define coordinates of hatch lines
    z = corner1.z
    x_vals = np.linspace(corner1.x, corner2.x, 10)
    y_vals = np.linspace(corner1.y, corner2.y, 10)

    lines = []
    for i in range(len(x_vals)):
        lines.append([[x_vals[i], y_vals[0]], [x_vals[i], y_vals[-1]]])
    for i in range(len(y_vals)):
        lines.append([[x_vals[0], y_vals[i]], [x_vals[-1], y_vals[i]]])
    
    # draw lines
    for line in lines:
        point1, point2 = line
        move_to_t_point(point1[0], point1[1], z - 3)
        move_to_t_point(point1[0], point1[1], z)
        move_to_t_point(point2[0], point2[1], z)
        move_to_t_point(point2[0], point2[1], z - 3)


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
