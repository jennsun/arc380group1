"""
Final Project Part 4
Match perceived objects to their locations in the tower.
"""

import json

def get_tower_objects(json_path):
    tower_objects = []

    with open(json_path) as json_file:
        tower_json = json.load(json_file)

        for key in tower_json.keys():
            info = tower_json[key]
            tower_objects.append({
                                "color": info["color"],
                                "shape": info["shape"],
                                "position": info["position"],
                                "orientation": info["rotation"],
                                "size": info["size"]
                                })
    tower_objects.sort(key=lambda x:x["position"][2])
    return tower_objects

if __name__ == "__main__":
    tower_objects = get_tower_objects('tower.json')
    print(tower_objects)


        

