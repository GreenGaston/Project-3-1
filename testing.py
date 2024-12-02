import open3d as o3d
import numpy as np



# pcd = o3d.io.read_point_cloud("dragon.ply")
# pcd.paint_uniform_color([237/255, 202/255, 29/255])
# pcd.estimate_normals()
# o3d.visualization.draw_geometries([pcd])





# reading the mesh from poisson file
# mesh = o3d.io.read_triangle_mesh("mesh_poisson.ply")
# mesh.compute_vertex_normals() 
# mesh.paint_uniform_color([237/255, 202/255, 29/255])
# o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)





# poisson training 
# pcd = o3d.io.read_point_cloud("dragon.ply")
# pcd.estimate_normals()

# # Perform Poisson surface reconstruction
# # The depth parameter controls the resolution of the output mesh
# # A depth of around 12 to 13 typically yields millions of vertices
# print('Performing Poisson surface reconstruction...')
# mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=12)

# # Calculate number of vertices
# num_vertices = len(mesh.vertices)
# print(f'Number of vertices in the mesh: {num_vertices}')

# # Optionally, you can clean the mesh by removing low-density vertices if needed
# # This will remove some of the less reliable parts of the mesh
# vertices_to_remove = densities < np.quantile(densities, 0.01)
# mesh.remove_vertices_by_mask(vertices_to_remove)

# # Visualize the resulting mesh
# o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

# # write the mesh to a file
# o3d.io.write_triangle_mesh("mesh_poisson.ply", mesh)







# display the whole mesh
mesh = o3d.io.read_triangle_mesh("output.ply")
mesh.compute_vertex_normals()
# make the mesh gray

mesh.paint_uniform_color([237/255, 202/255, 29/255])
o3d.visualization.draw_geometries([mesh])





# alpha 

pcd = o3d.io.read_point_cloud("output.ply") # for all points

# pcd = mesh.sample_points_poisson_disk(number_of_points=15000)
# o3d.visualization.draw_geometries([pcd])
alpha = 1
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()
mesh.paint_uniform_color([237/255, 202/255, 29/255])
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

##save the mash as ply file
o3d.io.write_triangle_mesh("dragonoutput.ply", mesh)

# pcd2= mesh.sample_points_poisson_disk(number_of_points=15000)
# # compute the mean distance between the sampled points of the 2 meshes
# distances = pcd.compute_point_cloud_distance(pcd2)
# mean_distance = np.mean(distances)
# print(f"mean distance: {mean_distance:.3f}")