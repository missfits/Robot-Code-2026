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
    
    # Define outpost paths to mirror (based on the original outpost files)
    outpost_paths = [
        "1st outpost shoot",
        "fast outpost shoot",
        "outpost flex pickup",
        "outpost flex bump",
        "outpost flex center",
        "2nd outpost cleanup",
        "2nd outpost shoot",
        "outpost center half neutral zone"
    ]
    
    # Define outpost autos to mirror
    outpost_autos = [
        "outpost cleanup",
        "outpost fast",
        "outpost shoot",
        "outpost flex",
        "outpost center flex"
    ]
    
    # Mirror paths
    print(f"Mirroring outpost paths to depot paths (field width: {FIELD_WIDTH} m)...")
    for path_name in outpost_paths:
        input_file = os.path.join(base_path, "paths", f"{path_name}.path")
        output_name = path_name.replace("outpost", "depot")
        output_file = os.path.join(base_path, "paths", f"{output_name}.path")
        
        if os.path.exists(input_file):
            mirror_path(input_file, output_file)
        else:
            print(f"Warning: Input file not found: {input_file}")
    
    # Mirror autos
    print("\nMirroring outpost autos to depot autos...")
    for auto_name in outpost_autos:
        input_file = os.path.join(base_path, "autos", f"{auto_name}.auto")
        output_name = auto_name.replace("outpost", "depot")
        output_file = os.path.join(base_path, "autos", f"{output_name}.auto")
        
        if os.path.exists(input_file):
            mirror_auto(input_file, output_file)
        else:
            print(f"Warning: Input file not found: {input_file}")
    
    print("\nMirroring complete!")

if __name__ == "__main__":
    main()
