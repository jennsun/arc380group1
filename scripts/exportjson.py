import json

import Rhino.Geometry as rg
import rhinoscriptsyntax as rs

def get_objects():
    objects = []
    
    # Prompt the user to select Brep objects in Rhino
    brep_ids = rs.GetObjects("Select Breps", rs.filter.polysurface)
    
    if not brep_ids:
        print("No Breps selected.")
        return
    
    # Extract geometry and layer information for each object
    for id in brep_ids:
        brep = rs.coercebrep(id)
        layer_name = rs.ObjectLayer(id)
        
        # Get object shape, size, and color from the layer name
        shape, size, color = parse_layer_name(layer_name)
        
        # Get object pose (located at center of top face, for vacuum)
        pos, rot = get_pose(brep, shape)
        
        objects.append((pos, rot, shape, size, color))
    
    return objects

def parse_layer_name(layer_name):
    shape, size, color = None, None, None
    
    if 'block' in layer_name.lower():
        shape = "block"
    
    else:   # (the object is a disk or square)
        sublayer = layer_name.split('::')[-1]
        tag, color = sublayer.split('_')
        color = color.lower()
        
        if tag[0].lower() == 'd':
            shape = "disk"
            size = float(tag[1:])
        elif tag[0].lower() == 's':
            shape = "square"
            size = 2
    
    return shape, size, color

def get_pose(brep, shape):
    # Compute position of the brep by projecting the centroid along the Z axis
    # Get brep centroid
    vmp = rg.VolumeMassProperties.Compute(brep,
                                          volume=False,
                                          firstMoments=True,
                                          secondMoments=False,
                                          productMoments=False)
    centroid = vmp.Centroid
    
    # Note: Intersection will get both bottom and top points, sorted by Z.
    # We want the top (second) point, hence the [1].
    pos = rg.Intersect.Intersection.ProjectPointsToBreps([brep],
                                                         [centroid],
                                                         rg.Vector3d.ZAxis,
                                                         1e-6)[1]
    
    # Compute rotation using the longest axis
    rot = 0.0 # In radians
    if shape != "disk":
        # Get longest edge in brep
        sorted_edges = sorted(brep.Edges, key=lambda edge : edge.GetLength())
        longest_edge = sorted_edges[-1]
        # Construct vector from longest_edge vertices and compute angle
        edge_vec = rg.Vector3d(longest_edge.PointAtEnd - longest_edge.PointAtStart)
        rot = rg.Vector3d.VectorAngle(rg.Vector3d.XAxis, edge_vec, rg.Vector3d.ZAxis)
    
    # Convert position Rhino.Point3d to tuple
    pos_tuple = (pos.X, pos.Y, pos.Z)
    
    return pos_tuple, rot

def export_JSON(obj_tuples, filename):
    # Convert the tuple list into a dictionary of dictionaries
    obj_dict = {}
    keys = ["position", "rotation", "shape", "size", "color"]
    for i in range(len(obj_tuples)):
        obj_key = "object" + str(i)
        obj_dict[obj_key] = dict(zip(keys, obj_tuples[i]))
    
    # Save dict as JSON file
    with open(filename, 'w') as f:
        json.dump(obj_dict, f, indent=4)
    
    print("Objects have been written to " + filename)


# Get objects and sort them into assembly order (i.e., by height / z coord)
objects = get_objects() # Returns a list of tuples with object information
sorted_objects = sorted(objects, key=lambda obj : obj[0][2])

# Export the object information as a JSON file
export_JSON(sorted_objects, '/Users/jenny/Documents/2024Spring/ARC380/arc380group1/scripts/tower4-27-2.json')
