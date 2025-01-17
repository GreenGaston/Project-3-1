import open3d as o3d
import numpy as np
from scipy.spatial import distance

# Load the original PLY model
def load_model(file_path):
    mesh = o3d.io.read_triangle_mesh(file_path)
    mesh.compute_vertex_normals()
    return mesh

# Generate a point cloud from the mesh
def sample_point_cloud(mesh, num_points=10000):
    point_cloud = mesh.sample_points_uniformly(number_of_points=num_points)
    return point_cloud

# Perform surface reconstruction
def poisson_reconstruction(point_cloud):
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud, depth=9)
    return mesh

# Evaluate reconstruction
def evaluate_reconstruction(original_mesh, reconstructed_mesh):
    # Convert to point clouds for comparison
    original_pcd = sample_point_cloud(original_mesh)
    reconstructed_pcd = sample_point_cloud(reconstructed_mesh)

    original_points = np.asarray(original_pcd.points)
    reconstructed_points = np.asarray(reconstructed_pcd.points)

    # Chamfer Distance
    chamfer_1 = np.mean([np.min(distance.cdist([p], reconstructed_points)) for p in original_points])
    chamfer_2 = np.mean([np.min(distance.cdist([p], original_points)) for p in reconstructed_points])
    chamfer_distance = chamfer_1 + chamfer_2

    # Hausdorff Distance
    d1 = np.max([np.min(distance.cdist([p], reconstructed_points)) for p in original_points])
    d2 = np.max([np.min(distance.cdist([p], original_points)) for p in reconstructed_points])
    hausdorff_distance = max(d1, d2)

    # Volume Overlap
    # original_volume = "does not exist"
    # reconstructed_volume = reconstructed_mesh.get_volume()
    # volume_overlap = min(original_volume, reconstructed_volume) / max(original_volume, reconstructed_volume)

    return {
        'chamfer_distance': chamfer_distance,
        'hausdorff_distance': hausdorff_distance,
        'volume_overlap': volume_overlap
    }
def mesh_to_mesh_chamfer_distance(original_mesh, reconstructed_mesh):
    # Sample points from both meshes
    original_pcd = sample_point_cloud(original_mesh)
    reconstructed_pcd = sample_point_cloud(reconstructed_mesh)

    original_points = np.asarray(original_pcd.points)
    reconstructed_points = np.asarray(reconstructed_pcd.points)

    chamfer_1 = np.mean([np.min(distance.cdist([p], reconstructed_points)) for p in original_points])
    chamfer_2 = np.mean([np.min(distance.cdist([p], original_points)) for p in reconstructed_points])
    return chamfer_1 + chamfer_2

def mesh_to_mesh_hausdorff_distance(original_mesh, reconstructed_mesh):
    # Sample points from both meshes
    original_pcd = sample_point_cloud(original_mesh)
    reconstructed_pcd = sample_point_cloud(reconstructed_mesh)

    original_points = np.asarray(original_pcd.points)
    reconstructed_points = np.asarray(reconstructed_pcd.points)

    d1 = np.max([np.min(distance.cdist([p], reconstructed_points)) for p in original_points])
    d2 = np.max([np.min(distance.cdist([p], original_points)) for p in reconstructed_points])
    return max(d1, d2)

def volume_overlap(original_mesh, reconstructed_mesh):
    # Compute mesh volumes
    original_volume = original_mesh.get_volume()
    reconstructed_volume = reconstructed_mesh.get_volume()

    # Approximation: Use the smaller volume divided by the larger
    return min(original_volume, reconstructed_volume) / max(original_volume, reconstructed_volume)


def poisson_reconstruction(input_file, depth):
    pcd = o3d.io.read_point_cloud(input_file)

    pcd.estimate_normals()
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    mesh.compute_vertex_normals()


    # vertices_to_remove = densities < np.quantile(densities, 0.01)
    # mesh.remove_vertices_by_mask(vertices_to_remove)

    return mesh

def alpha_shape_reconstruction(input_file,  alpha):
    pcd = o3d.io.read_point_cloud(input_file)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

    return mesh
def ball_pivoting_reconstruction(input_file, radii):

    print("radii= ", radii)
    pcd = o3d.io.read_point_cloud(input_file)
    pcd.estimate_normals()
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
    return mesh




# Visualization
def visualize(original_mesh, reconstructed_mesh):
    original_mesh.paint_uniform_color([1, 0, 0])  # Red
    reconstructed_mesh.paint_uniform_color([0, 1, 0])  # Green
    o3d.visualization.draw_geometries([original_mesh, reconstructed_mesh])






def TestAllMethods(filename):
    # Load and process
    original_mesh = load_model(filename)
    ##point_cloud = sample_point_cloud(original_mesh,)

    poisson_re=poisson_reconstruction(filename, 13)
    alpha_shape_re=alpha_shape_reconstruction(filename, 0.05)
    ball_pivoting_re=ball_pivoting_reconstruction(filename, [0.01, 0.05, 0.1])

    # Evaluation
    score_poisson = evaluate_reconstruction(original_mesh, poisson_re)
    score_alpha_shape = evaluate_reconstruction(original_mesh, alpha_shape_re)
    score_ball_pivoting = evaluate_reconstruction(original_mesh, ball_pivoting_re)


    print(f"Poisson Reconstruction: {score_poisson}")
    print(f"Alpha Shape Reconstruction: {score_alpha_shape}")
    print(f"Ball Pivoting Reconstruction: {score_ball_pivoting}")

    # Visualization
    visualize(original_mesh, poisson_re)
    visualize(original_mesh, alpha_shape_re)
    visualize(original_mesh, ball_pivoting_re)

    #visualize the meshes sepperately
    o3d.visualization.draw_geometries([poisson_re])
    o3d.visualization.draw_geometries([alpha_shape_re])
    o3d.visualization.draw_geometries([ball_pivoting_re])


import numpy as np

def convert_point_to_global_space(point, point_of_measurement, angle_x, angle_y, angle_z):


    

    angle_x_rad = np.radians(angle_x)
    angle_y_rad = np.radians(angle_y)
    angle_z_rad = np.radians(angle_z)
    
    # Define the rotation matrix for rotation around the X-axis
    rotation_matrix_x = np.array([
        [1, 0, 0],
        [0, np.cos(angle_x_rad), -np.sin(angle_x_rad)],
        [0, np.sin(angle_x_rad), np.cos(angle_x_rad)]
    ])
    
    # Define the rotation matrix for rotation around the Y-axis
    rotation_matrix_y = np.array([
        [np.cos(angle_y_rad), 0, np.sin(angle_y_rad)],
        [0, 1, 0],
        [-np.sin(angle_y_rad), 0, np.cos(angle_y_rad)]
    ])
    
    # Define the rotation matrix for rotation around the Z-axis
    rotation_matrix_z = np.array([
        [np.cos(angle_z_rad), -np.sin(angle_z_rad), 0],
        [np.sin(angle_z_rad), np.cos(angle_z_rad), 0],
        [0, 0, 1]
    ])
    # Convert point and point_of_measurement to numpy 
    
    rotation_matrix = np.dot(rotation_matrix_z, np.dot(rotation_matrix_y, rotation_matrix_x))

    point = np.array(point)
    point_of_measurement = np.array(point_of_measurement)
    
    # Apply the rotation
    rotated_point = np.dot(rotation_matrix, point)
    
    # Apply the translation
    global_point = rotated_point + point_of_measurement
    
    return global_point



# Main Pipeline
if __name__ == "__main__":
    original_file = "xyzrgb_dragon.ply"

    TestAllMethods(original_file)
