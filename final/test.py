import json

# read in simpletower.json and sort as json
with open("json/tower-x-pos-y-neg.json") as json_file:
    tower_json = json.load(json_file)

# convert tower_json to list of objects
tower_objects = []
for key in tower_json.keys():
    # include the key in the object as well
    tower_json[key]["name"] = key
    tower_objects.append(tower_json[key])
    # print(tower_objects)

# sort list of objects by their z value
tower_objects.sort(key=lambda x: x["position"][0])

# write sorted list of objects to simpletower_sorted.json
with open("json/tower.json", "w") as json_file:
    json.dump(tower_objects, json_file, indent=4)

# read in simpletower.json which is a list of dicts
with open("json/tower.json") as json_file:
    tower_json = json.load(json_file)

print("tower_json is", tower_json)
