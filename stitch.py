# this file is for placing 3D objects (meshes) side-by-side
import open3d as o3d
import numpy as np
import copy

def place_objects_side_by_side(source_path, target_path, output_path, gap=1.0):
    """
    Combines two .ply 3D objects into one file by placing them next to each other.
    
    :param source_path: Path to the first .ply file (the one that will be moved).
    :param target_path: Path to the second .ply file (the stationary one).
    :param output_path: Path to save the combined .ply file.
    :param gap: The distance to leave between the two objects so they don't touch. 
                Adjust this based on the scale of your 3D data.
    """
    #print("1. Loading 3D objects...")
    # Changed to read_triangle_mesh to load them as solid objects rather than just points
    source_mesh = o3d.io.read_triangle_mesh(source_path)
    target_mesh = o3d.io.read_triangle_mesh(target_path)
    
    # Optional check: Let the user know if the .ply file lacks actual surface geometry
    #if not source_mesh.has_triangles() or not target_mesh.has_triangles():
     #   print("Warning: One or both files might not contain mesh faces (triangles) and may still act like point clouds.")

    #print("2. Calculating bounding boxes...")
    # Get the bounding boxes to find the outermost edges of the objects
    target_max_bound = target_mesh.get_max_bound()
    source_min_bound = source_mesh.get_min_bound()

   # print("3. Moving the source object...")
    # Calculate how far to shift the source object along the X-axis.
    # Shift = (Target's rightmost edge) - (Source's leftmost edge) + (desired gap)
    shift_x = target_max_bound[0] - source_min_bound[0] + gap
    
    # Create a translation vector (moving it purely along the X-axis)
    translation_vector = np.array([shift_x, 0.0, 0.0])

    # Create a copy so we don't alter the original loaded data
    source_moved = copy.deepcopy(source_mesh)
    source_moved.translate(translation_vector)

    #print("4. Combining the objects...")
    # Open3D allows you to easily add meshes together into the same coordinate space
    combined_mesh = source_moved + target_mesh

   # print(f"5. Saving side-by-side model to {output_path}...")
    # Changed to write_triangle_mesh to save the combined object
    o3d.io.write_triangle_mesh(output_path, combined_mesh)
    print("Placement complete!")

# --- Execute the code ---
if __name__ == "__main__":
    # Replace these with your actual file names
    file1 = "testbedroom (1).ply"
    file2 = "testloft.ply"
    output = "side_by_side_rooms.ply"
    
    # You may need to adjust the 'gap' parameter depending on whether your 
    # coordinates are in meters, millimeters, inches, etc.
    place_objects_side_by_side(file1, file2, output, gap=2)
