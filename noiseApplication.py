# file for applying noise to a mesh, denoising it, and comparing the results

import open3d as o3d
import numpy as np
import copy

from PCA import pca_denoising
from statistical_outlier_removal import apply_statistical_outlier_removal
from statistical_outlier_removal import apply_normal_deviation_removal

def simulate_scanning_error(pcd, distance_threshold=0.01, noise_level=0.02):

    if not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError("Input must be a PointCloud")

    # Create a copy of the point cloud to avoid modifying the original
    noisy_pcd = copy.deepcopy(pcd)

    # Generate a random plane
    plane_normal = np.random.rand(3) - 0.5
    plane_normal /= np.linalg.norm(plane_normal)
    plane_point = np.random.rand(3) - 0.5

    # Calculate distances of points to the plane
    points = np.asarray(noisy_pcd.points)
    distances = np.dot(points - plane_point, plane_normal)

    # Select points close to the plane
    mask = np.abs(distances) < distance_threshold
    noisy_points = points[mask]



    # Apply noise to the selected points
    noise = np.random.normal(scale=distance_threshold, size=noisy_points.shape)
    points[mask] += noise

    # Update the points in the copied point cloud
    noisy_pcd.points = o3d.utility.Vector3dVector(points)

    return noisy_pcd

def apply_random_noise(pcd, noise_level=0.02):
    """
    Apply random noise to the points of a point cloud.

    Parameters:
    - pcd: The input point cloud.
    - noise_level: The maximum magnitude of the noise to be added.

    Returns:
    - noisy_pcd: The point cloud with random noise added to its points.
    """
    if not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError("Input must be a PointCloud")

    noisy_pcd = copy.deepcopy(pcd)
    points = np.asarray(noisy_pcd.points)
    noise = 2 * noise_level * np.random.rand(*points.shape) - noise_level
    noisy_pcd.points = o3d.utility.Vector3dVector(points + noise)
    return noisy_pcd

def salt_and_pepper_noise(pcd, noise_level=0.02):
    """
    Apply salt-and-pepper noise to the points of a point cloud.

    Parameters:
    - pcd: The input point cloud.
    - noise_level: The probability of adding noise to each point.

    Returns:
    - noisy_pcd: The point cloud with salt-and-pepper noise added to its points.
    """
    if not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError("Input must be a PointCloud")

    noisy_pcd = copy.deepcopy(pcd)
    points = np.asarray(noisy_pcd.points)
    noise_mask = np.random.rand(*points.shape) < noise_level
    noise = 2 * noise_level * np.random.rand(*points.shape) - noise_level
    noisy_pcd.points = o3d.utility.Vector3dVector(points + noise * noise_mask)
    return noisy_pcd