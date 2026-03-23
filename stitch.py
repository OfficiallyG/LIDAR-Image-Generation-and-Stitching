# this file is for the stitching algorithm.
import open3d as o3d
import numpy as np
import copy

def stitch_ply_files(source_path, target_path, output_path, distance_threshold=0.2):
    """
    Combines two .ply point clouds using ICP registration.
    
    :param source_path: Path to the first .ply file (the one that will be moved).
    :param target_path: Path to the second .ply file (the stationary one).
    :param output_path: Path to save the stitched .ply file.
    :param distance_threshold: Max distance for points to be considered matching. 
                               Adjust based on the scale of your LiDAR data.
    """
    print("1. Loading point clouds...")
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)

    print("2. Running ICP registration to align the scans...")
    # This initial transformation assumes the scans are already roughly 
    # pointing in the same direction. 
    initial_guess = np.identity(4)

    # Perform point-to-point ICP
    icp_result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, initial_guess,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    print("3. Applying transformation to the source cloud...")
    # Create a copy so we don't alter the original loaded data
    source_aligned = copy.deepcopy(source)
    source_aligned.transform(icp_result.transformation)

    print("4. Merging the point clouds...")
    # Open3D allows you to easily add point clouds together
    merged_cloud = source_aligned + target

    # Optional but recommended: Downsample the combined cloud to remove 
    # duplicate/overlapping points right at the seam where the walls meet.
    print("5. Filtering overlapping points...")
    voxel_size = distance_threshold / 5.0 
    merged_cloud = merged_cloud.voxel_down_sample(voxel_size=voxel_size)

    print(f"6. Saving stitched model to {output_path}...")
    o3d.io.write_point_cloud(output_path, merged_cloud)
    print("Stitching complete!")

# --- Execute the code ---
if __name__ == "__main__":
    # Replace these with your actual file names
    file1 = "testbedroom (1).ply"
    file2 = "testloft.ply"
    output = "stitched_rooms.ply"
    
    stitch_ply_files(file1, file2, output)
