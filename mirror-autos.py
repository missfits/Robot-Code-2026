#!/usr/bin/env python3
import json
import os

# CORRECT field width from 2026-official.json welded field
FIELD_WIDTH = 8.069  # meters

def mirror_y_coordinate(y):
    """Mirror a Y coordinate across the horizontal centerline"""
    return FIELD_WIDTH - y

def mirror_rotation(rotation):
    """Mirror a rotation angle across the horizontal centerline"""
    # For mirroring across horizontal line: new_angle = -old_angle
    return -rotation

def mirror_path(input_file, output_file):
    """Mirror a PathPlanner path file across the horizontal centerline"""
    with open(input_file, 'r') as f:
        data = json.load(f)
    
    # Mirror waypoints
    for waypoint in data['waypoints']:
        # Mirror anchor Y coordinate
        waypoint['anchor']['y'] = mirror_y_coordinate(waypoint['anchor']['y'])
        
        # Mirror prevControl Y coordinate if it exists
        if waypoint['prevControl'] is not None:
            waypoint['prevControl']['y'] = mirror_y_coordinate(waypoint['prevControl']['y'])
        
        # Mirror nextControl Y coordinate if it exists
        if waypoint['nextControl'] is not None:
            waypoint['nextControl']['y'] = mirror_y_coordinate(waypoint['nextControl']['y'])
        
        # Update linked names if they contain "outpost"
        if waypoint.get('linkedName'):
            waypoint['linkedName'] = waypoint['linkedName'].replace('outpost', 'depot')
    
    # Mirror rotation targets
    if 'rotationTargets' in data:
        for target in data['rotationTargets']:
            target['rotationDegrees'] = mirror_rotation(target['rotationDegrees'])
    
    # Mirror goal end state rotation
    if 'goalEndState' in data and 'rotation' in data['goalEndState']:
        data['goalEndState']['rotation'] = mirror_rotation(data['goalEndState']['rotation'])
    
    # Mirror ideal starting state rotation
    if 'idealStartingState' in data and 'rotation' in data['idealStartingState']:
        data['idealStartingState']['rotation'] = mirror_rotation(data['idealStartingState']['rotation'])
    
    # Write output file
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"Created mirrored path: {output_file}")

def mirror_auto(input_file, output_file):
    """Mirror a PathPlanner auto file by replacing outpost path names with depot equivalents"""
    with open(input_file, 'r') as f:
        data = json.load(f)
    
    # Recursively replace path names in the command structure
    def replace_outpost_with_depot(obj):
        if isinstance(obj, dict):
            for key, value in obj.items():
                if key == 'pathName' and isinstance(value, str):
                    obj[key] = value.replace('outpost', 'depot')
                else:
                    replace_outpost_with_depot(value)
        elif isinstance(obj, list):
            for item in obj:
                replace_outpost_with_depot(item)
    
    replace_outpost_with_depot(data)
    
    # Write output file
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"Created mirrored auto: {output_file}")

def main():
    base_path = "src/main/deploy/pathplanner"

    # Automatically find all files with "outpost" in the name
    paths_dir = os.path.join(base_path, "paths")
    autos_dir = os.path.join(base_path, "autos")

    outpost_paths = []
    outpost_autos = []

    # Scan for all .path files containing "outpost"
    if os.path.exists(paths_dir):
        for filename in os.listdir(paths_dir):
            if filename.endswith(".path") and "outpost" in filename:
                # Remove the .path extension to get the path name
                path_name = filename[:-5]
                outpost_paths.append(path_name)

    # Scan for all .auto files containing "outpost"
    if os.path.exists(autos_dir):
        for filename in os.listdir(autos_dir):
            if filename.endswith(".auto") and "outpost" in filename:
                # Remove the .auto extension to get the auto name
                auto_name = filename[:-5]
                outpost_autos.append(auto_name)

    # Sort for consistent output
    outpost_paths.sort()
    outpost_autos.sort()

    # Mirror paths
    print(f"Mirroring outpost paths to depot paths (field width: {FIELD_WIDTH} m)...")
    print(f"Found {len(outpost_paths)} outpost path(s) to mirror")
    for path_name in outpost_paths:
        input_file = os.path.join(base_path, "paths", f"{path_name}.path")
        output_name = path_name.replace("outpost", "depot")
        output_file = os.path.join(base_path, "paths", f"{output_name}.path")

        mirror_path(input_file, output_file)

    # Mirror autos
    print(f"\nMirroring outpost autos to depot autos...")
    print(f"Found {len(outpost_autos)} outpost auto(s) to mirror")
    for auto_name in outpost_autos:
        input_file = os.path.join(base_path, "autos", f"{auto_name}.auto")
        output_name = auto_name.replace("outpost", "depot")
        output_file = os.path.join(base_path, "autos", f"{output_name}.auto")

        mirror_auto(input_file, output_file)

    print("\nMirroring complete!")
    print(f"Total: {len(outpost_paths)} path(s) and {len(outpost_autos)} auto(s) mirrored")

if __name__ == "__main__":
    main()
