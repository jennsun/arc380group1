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
            pass

        

