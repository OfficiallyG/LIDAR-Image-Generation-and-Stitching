# this file is for placing point clouds side-by-side
import open3d as o3d
import numpy as np
import copy

def place_ply_side_by_side(source_path, target_path, output_path, gap=1.0):
    """
    Combines two .ply point clouds into one file by placing them next to each other.
    
    :param source_path: Path to the first .ply file (the one that will be moved).
    :param target_path: Path to the second .ply file (the stationary one).
    :param output_path: Path to save the combined .ply file.
    :param gap: The distance to leave between the two point clouds so they don't touch. 
                Adjust this based on the scale of your LiDAR data.
    """
    print("1. Loading point clouds...")
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)

    print("2. Calculating bounding boxes...")
    # Get the bounding boxes to find the outermost edges of the point clouds
    target_max_bound = target.get_max_bound()
    source_min_bound = source.get_min_bound()

    print("3. Moving the source point cloud...")
    # Calculate how far to shift the source cloud along the X-axis.
    # Shift = (Target's rightmost edge) - (Source's leftmost edge) + (desired gap)
    shift_x = target_max_bound[0] - source_min_bound[0] + gap
    
    # Create a translation vector (moving it purely along the X-axis)
    translation_vector = np.array([shift_x, 0.0, 0.0])

    # Create a copy so we don't alter the original loaded data
    source_moved = copy.deepcopy(source)
    source_moved.translate(translation_vector)

    print("4. Combining the point clouds...")
    # Open3D allows you to easily add point clouds together into the same coordinate space
    combined_cloud = source_moved + target

    # Note: We removed the voxel downsampling here because the clouds no longer 
    # overlap, so there are no duplicate points at a "seam" to worry about.

    print(f"5. Saving side-by-side model to {output_path}...")
    o3d.io.write_point_cloud(output_path, combined_cloud)
    print("Placement complete!")

# --- Execute the code ---
if __name__ == "__main__":
    # Replace these with your actual file names
    file1 = "testbedroom (1).ply"
    file2 = "testloft.ply"
    output = "side_by_side_rooms.ply"
    
    # You may need to adjust the 'gap' parameter depending on whether your 
    # coordinates are in meters, millimeters, inches, etc.
    place_ply_side_by_side(file1, file2, output, gap=0.5)
