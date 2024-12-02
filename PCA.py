import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA


def pca_denoising(point_cloud, k=10, threshold=0.01):
    """
    Apply PCA denoising to a 3D point cloud.
    
    Parameters:
    - point_cloud: open3d.geometry.PointCloud
    - k: Number of nearest neighbors to consider for PCA
    - threshold: Eigenvalue threshold to determine noise points

    Returns:
    - denoised_point_cloud: open3d.geometry.PointCloud
    """
    # Get point cloud as numpy array
    points = np.asarray(point_cloud.points)

    # Nearest neighbor search
    pcd_tree = o3d.geometry.KDTreeFlann(point_cloud)

    # Lists to store denoised points
    denoised_points = []

    for i, point in enumerate(points):
        # Find k nearest neighbors
        [_, idx, _] = pcd_tree.search_knn_vector_3d(point, k)
        neighbors = points[idx]

        # Perform PCA
        pca = PCA(n_components=3)
        pca.fit(neighbors)
        eigenvalues = pca.explained_variance_

        # Check if the smallest eigenvalue is below the threshold
        if eigenvalues[-1] > threshold:
            denoised_points.append(point)

    # Create a new point cloud with denoised points
    denoised_point_cloud = o3d.geometry.PointCloud()
    denoised_point_cloud.points = o3d.utility.Vector3dVector(np.array(denoised_points))

    return denoised_point_cloud


if __name__ == "__main__":
    # Load the point cloud
    pcd = o3d.io.read_point_cloud("Armadillo.ply")

    # Apply PCA denoising (adjust paramaters if needed)
    k_neighbors = 20
    noise_threshold = 0.001
    denoised_pcd = pca_denoising(pcd, k=k_neighbors, threshold=noise_threshold)

    # Save and visualize the result
    o3d.io.write_point_cloud("denoised_dragon.ply", denoised_pcd)
    print(f"Denoised point cloud saved as 'denoised_dragon.ply'")

    o3d.visualization.draw_geometries([denoised_pcd])
