import open3d as o3d	
import numpy as np


pcd = o3d.io.read_point_cloud("dragon.ply")
pcd.estimate_normals()


print('Performing Poisson surface reconstruction...')
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=12)
mesh.compute_vertex_normals()

yellow_color = np.array([[237/255, 202/255, 29/255] for _ in range(len(mesh.vertices))])
mesh.vertex_colors = o3d.utility.Vector3dVector(yellow_color)

# Calculate number of verticess
num_vertices = len(mesh.vertices)
print(f'Number of vertices in the mesh: {num_vertices}')


vertices_to_remove = densities < np.quantile(densities, 0.01)
mesh.remove_vertices_by_mask(vertices_to_remove)

# Visualize the resulting mesh
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)