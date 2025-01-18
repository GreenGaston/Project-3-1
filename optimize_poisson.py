import open3d as o3d
import numpy as np
from scipy.spatial import distance


def poisson_reconstruction(input_file, depth):
    pcd = o3d.io.read_point_cloud(input_file)

    pcd.estimate_normals()
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    mesh.compute_vertex_normals()
    vertices_to_remove = densities < np.quantile(densities, 0.01)
    mesh.remove_vertices_by_mask(vertices_to_remove)
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

def grid_search(depth_values, file):
        """
        Perform grid search to find the best depth value based on Chamfer distance.
        """
        original_mesh = load_model(file)
        best_depth = None
        best_chamfer_distance = float('inf')

        original_pcd = sample_point_cloud(original_mesh)
        
        for depth in depth_values:
            reconstructed_mesh = poisson_reconstruction(file, depth)
            reconstructed_pcd = sample_point_cloud(reconstructed_mesh)
            chamfer_distance = mesh_to_mesh_chamfer_distance(original_pcd, reconstructed_pcd)
            
            print(f"Depth={depth}, Chamfer Distance={chamfer_distance}")
            
            # Update best depth
            if chamfer_distance < best_chamfer_distance:
                best_chamfer_distance = chamfer_distance
                best_depth = depth
        
        print(f"Best Depth: {best_depth} with Chamfer Distance: {best_chamfer_distance}")
        return best_depth

if __name__ == '__main__':
    file = "xyzrgb_dragon.ply"
    depth_values = [7, 8, 9, 10, 11, 12, 13]
    grid_search(depth_values, file)
    