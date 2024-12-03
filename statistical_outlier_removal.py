import open3d as o3d

def apply_statistical_outlier_removal(input_file, output_file, nb_neighbors=20, std_ratio=2.0):
    """
    Apply Statistical Outlier Removal to a mesh or point cloud.

    Parameters:
    - input_file: Path to the input mesh or point cloud file.
    - output_file: Path to save the cleaned mesh or point cloud.
    - nb_neighbors: Number of neighbors to analyze for each point.
    - std_ratio: Standard deviation multiplier for the threshold.

    Returns:
    - None (saves the cleaned result to `output_file`).
    """
    # Load the input file
    print(f"Loading file: {input_file}")
    try:
        pcd = o3d.io.read_point_cloud(input_file)
    except RuntimeError:
        mesh = o3d.io.read_triangle_mesh(input_file)
        pcd = mesh.sample_points_uniformly(number_of_points=50000)

    # Apply statistical outlier removal
    print("Applying Statistical Outlier Removal...")
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    # Keep the inlier points
    clean_pcd = pcd.select_by_index(ind)

    # Save the cleaned point cloud
    if isinstance(pcd, o3d.geometry.PointCloud):
        o3d.io.write_point_cloud(output_file, clean_pcd)
    else:
        clean_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(clean_pcd, alpha=0.1)
        o3d.io.write_triangle_mesh(output_file, clean_mesh)

    print(f"Cleaned file saved as: {output_file}")
