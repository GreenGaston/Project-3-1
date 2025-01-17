import open3d as o3d
import numpy as np

def apply_statistical_outlier_removal(pcd, nb_neighbors=20, std_ratio=2.0):
    """
    Apply Statistical Outlier Removal to a point cloud.

    Parameters:
    - pcd: The input point cloud.
    - nb_neighbors: Number of neighbors to analyze for each point.
    - std_ratio: Standard deviation multiplier for the threshold.

    Returns:
    - clean_pcd: The cleaned point cloud.
    """
    if not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError("Input must be a PointCloud")

    # Apply statistical outlier removal
    print("Applying Statistical Outlier Removal...")
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    # Keep the inlier points
    clean_pcd = pcd.select_by_index(ind)

    return clean_pcd

def apply_normal_deviation_removal(pcd, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30), threshold=0.1):
    """
    Remove points where the normal vector is too different from its neighbors.

    Parameters:
    - pcd: The input point cloud.
    - search_param: Search parameter for normal estimation.
    - threshold: Threshold for normal deviation.

    Returns:
    - clean_pcd: The cleaned point cloud.
    """
    if not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError("Input must be a PointCloud")

    # Estimate normals
    print("Estimating normals...")
    pcd.estimate_normals(search_param=search_param)

    # Compute the deviation of normals
    print("Computing normal deviations...")
    normals = np.asarray(pcd.normals)
    kd_tree = o3d.geometry.KDTreeFlann(pcd)
    indices_to_remove = []

    for i in range(len(normals)):
        [_, idx, _] = kd_tree.search_knn_vector_3d(pcd.points[i], search_param.max_nn)
        neighbor_normals = normals[idx[1:]]  # Exclude the point itself
        mean_normal = np.mean(neighbor_normals, axis=0)
        deviation = np.linalg.norm(normals[i] - mean_normal)
        if deviation > threshold:
            indices_to_remove.append(i)

    # Remove points with high normal deviation
    clean_pcd = pcd.select_by_index(indices_to_remove, invert=True)

    return clean_pcd