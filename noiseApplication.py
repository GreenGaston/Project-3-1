

# file for applying noise to a mesh, denoising it, and comparing the results

import open3d as o3d
import numpy as np

from PCA import pca_denoising
from statistical_outlier_removal import apply_statistical_outlier_removal
from statistical_outlier_removal import apply_normal_deviation_removal

def simulate_scanning_error(mesh_file, distance_threshold=0.01):
    """
    Simulate a scanning error by generating a random plane that intersects with the mesh
    and selecting all points close to that plane in the same random direction.

    Parameters:
    - mesh_file: Path to the input mesh file.
    - distance_threshold: Distance threshold to select points close to the plane.

    Returns:
    - modified_pcd: The modified point cloud with simulated scanning error.


    not completely applicable as scans dont cause both sides of the mesh to be affected
    but it will have to do for now
    """
    # Load the mesh
    print(f"Loading mesh: {mesh_file}")
    mesh = o3d.io.read_triangle_mesh(mesh_file)
    pcd = mesh.sample_points_uniformly(number_of_points=50000)

    # Generate a random plane
    plane_normal = np.random.rand(3)
    plane_normal /= np.linalg.norm(plane_normal)
    plane_point = np.mean(np.asarray(pcd.points), axis=0)  # Use the centroid of the point cloud as a point on the plane

    # Select points close to the plane
    print("Selecting points close to the plane...")
    points = np.asarray(pcd.points)
    distances = np.dot(points - plane_point, plane_normal)
    indices_to_select = np.where(np.abs(distances) < distance_threshold)[0]

    # Simulate scanning error by perturbing the selected points
    error_direction = plane_normal * distance_threshold
    points[indices_to_select] += error_direction

    # Create a new point cloud with the modified points
    modified_pcd = o3d.geometry.PointCloud()
    modified_pcd.points = o3d.utility.Vector3dVector(points)
    modified_pcd.normals = pcd.normals  # Keep the original normals

    print("Scanning error simulation complete.")
    return modified_pcd

def apply_random_noise(mesh, noise_level=0.02):
    """
    Apply random noise to the vertices of a mesh.

    Parameters:
    - mesh: The input mesh.
    - noise_level: The maximum magnitude of the noise to be added.

    Returns:
    - noisy_mesh: The mesh with random noise added to its vertices.
    """
    noisy_mesh = mesh.copy()
    vertices = np.asarray(noisy_mesh.vertices)
    noise = 2 * noise_level * np.random.rand(*vertices.shape) - noise_level
    noisy_mesh.vertices = o3d.utility.Vector3dVector(vertices + noise)
    return noisy_mesh

def salt_and_pepper_noise(mesh, noise_level=0.02):
    """
    Apply salt-and-pepper noise to the vertices of a mesh.

    Parameters:
    - mesh: The input mesh.
    - noise_level: The probability of adding noise to each vertex.

    Returns:
    - noisy_mesh: The mesh with salt-and-pepper noise added to its vertices.
    """
    noisy_mesh = mesh.copy()
    vertices = np.asarray(noisy_mesh.vertices)
    noise_mask = np.random.rand(*vertices.shape) < noise_level
    noise = 2 * noise_level * np.random.rand(*vertices.shape) - noise_level
    noisy_mesh.vertices = o3d.utility.Vector3dVector(vertices + noise * noise_mask)
    return noisy_mesh




