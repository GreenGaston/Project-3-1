import open3d as o3d
import numpy as np
from scipy.spatial import distance


def alpha_shape_reconstruction(input_file,  alpha):
    pcd = o3d.io.read_point_cloud(input_file)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    return mesh

def mesh_to_mesh_chamfer_distance(original_pcd, reconstructed_pcd):
    original_points = np.asarray(original_pcd.points)
    reconstructed_points = np.asarray(reconstructed_pcd.points)

    chamfer_1 = np.mean([np.min(distance.cdist([p], reconstructed_points)) for p in original_points])
    chamfer_2 = np.mean([np.min(distance.cdist([p], original_points)) for p in reconstructed_points])
    return chamfer_1 + chamfer_2

def sample_point_cloud(mesh, num_points=10000):
    point_cloud = mesh.sample_points_uniformly(number_of_points=num_points)
    return point_cloud

def load_model(file_path):
    mesh = o3d.io.read_triangle_mesh(file_path)
    mesh.compute_vertex_normals()
    return mesh

def grid_search(alpha_values, file):
        """
        Perform grid search to find the best depth value based on Chamfer distance.
        """
        original_mesh = load_model(file)
        best_alpha = None
        best_chamfer_distance = float('inf')

        original_pcd = sample_point_cloud(original_mesh)
        for alpha in alpha_values:
            reconstructed_mesh = alpha_shape_reconstruction(file, alpha)
            reconstructed_pcd = sample_point_cloud(reconstructed_mesh)
            chamfer_distance = mesh_to_mesh_chamfer_distance(original_pcd, reconstructed_pcd)
            
            print(f"Alpha={alpha}, Chamfer Distance={chamfer_distance}")
            
            # Update best depth
            if chamfer_distance < best_chamfer_distance:
                best_chamfer_distance = chamfer_distance
                best_alpha = alpha
        
        print(f"Best alpha: {best_alpha} with Chamfer Distance: {best_chamfer_distance}")
        return best_alpha

if __name__ == '__main__':
    file = "xyzrgb_dragon.ply"
    alpha_values = [0.1, 0.5, 0.75, 1.0, 2.0, 5.0, 10.0]
    grid_search(alpha_values, file)