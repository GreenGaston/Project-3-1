import open3d as o3d
import numpy as np
from statistical_outlier_removal import apply_statistical_outlier_removal


# pcd = o3d.io.read_point_cloud("dragon.ply")
# pcd.paint_uniform_color([237/255, 202/255, 29/255])
# pcd.estimate_normals()
# o3d.visualization.draw_geometries([pcd])





# reading the mesh from poisson file
# mesh = o3d.io.read_triangle_mesh("mesh_poisson.ply")
# mesh.compute_vertex_normals() 
# yellow_color = np.array([[1.0, 1.0, 0.0] for _ in range(len(mesh.vertices))])
# mesh.vertex_colors = o3d.utility.Vector3dVector(yellow_color)
# # mesh.paint_uniform_color([1, 1, 0])
# o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)





# poisson training 
# pcd = o3d.io.read_point_cloud("dragon.ply")
# pcd.estimate_normals()

# # Perform Poisson surface reconstruction
# # The depth parameter controls the resolution of the output mesh
# # A depth of around 12 to 13 typically yields millions of vertices
# print('Performing Poisson surface reconstruction...')
# mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=12)
# mesh.compute_vertex_normals()

# yellow_color = np.array([[237/255, 202/255, 29/255] for _ in range(len(mesh.vertices))])
# mesh.vertex_colors = o3d.utility.Vector3dVector(yellow_color)

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
# o3d.io.write_triangle_mesh("mesh_poisson_chair.ply", mesh)







# display the whole mesh
# mesh = o3d.io.read_triangle_mesh("dragon.ply")
# mesh.compute_vertex_normals()
# # make the mesh gray

# mesh.paint_uniform_color([237/255, 202/255, 29/255])
# o3d.visualization.draw_geometries([mesh])





#alpha 

# pcd = o3d.io.read_point_cloud("dragon.ply") # for all points

# pcd = mesh.sample_points_poisson_disk(number_of_points=15000)
# o3d.visualization.draw_geometries([pcd])
# alpha = 3.5
# print(f"alpha={alpha:.3f}")
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
# mesh.compute_vertex_normals()
# mesh.paint_uniform_color([237/255, 202/255, 29/255])
# o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

##save the mash as ply file
# o3d.io.write_triangle_mesh("dragonoutput.ply", mesh)

# pcd2= mesh.sample_points_poisson_disk(number_of_points=15000)
# # compute the mean distance between the sampled points of the 2 meshes
# distances = pcd.compute_point_cloud_distance(pcd2)
# mean_distance = np.mean(distances)
# print(f"mean distance: {mean_distance:.3f}")


#ball pivoting

#pcd = o3d.io.read_point_cloud("dragon.ply")

#Techniques to reduce nr of points used.
# downsample_factor = 0.5  # Keep 10% of the original points (adjust as needed)
# pcd = pcd.random_down_sample(downsample_factor)

# #voxel_size = 0.01  # Larger voxel size for faster processing (adjust as needed)
# #pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

# pcd.estimate_normals()
# radii = [0.075, 0.125, 0.225]
# print("radii: ", radii)
# mesh_bpa = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
# mesh_bpa.compute_vertex_normals()
# mesh_bpa.paint_uniform_color([237 / 255, 202 / 255, 29 / 255])
# o3d.visualization.draw_geometries([mesh_bpa], mesh_show_back_face=True)

def poisson_reconstruction(input_file, output_file, depth):
    pcd = o3d.io.read_point_cloud(input_file)

    pcd.estimate_normals()


    print('Performing Poisson surface reconstruction...')
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
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


def alpha_shape_reconstruction(input_file, output_file, alpha):
    pcd = o3d.io.read_point_cloud(input_file)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    yellow_color = np.array([[237/255, 202/255, 29/255] for _ in range(len(mesh.vertices))])

    mesh.vertex_colors = o3d.utility.Vector3dVector(yellow_color)

    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

def ball_pivoting_reconstruction(input_file, output_file, radii):
    #Techniques to reduce nr of points used.
    # downsample_factor = 0.5  # Keep 10% of the original points (adjust as needed)
    # pcd = pcd.random_down_sample(downsample_factor)

    # #voxel_size = 0.01  # Larger voxel size for faster processing (adjust as needed)
    # #pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    print("radii= ", radii)
    pcd = o3d.io.read_point_cloud(input_file)
    pcd.estimate_normals()
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(output_file, mesh)
    mesh.paint_uniform_color([237 / 255, 202 / 255, 29 / 255])
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

def main():
    input_file = "dragon.ply"
    depth = 12 #optimal for poisson for dragon
    alpha = 3.5 #optimal for alpha for dragon
    radii = [0.075, 0.125, 0.225] #good setting for ball pivoting for dragon
    poisson_output = "mesh_poisson.ply"
    alpha_output = "mesh_poisson.ply"
    ballpiv_output = "mesh_poisson.ply"
    apply_statistical_outlier_removal(input_file, poisson_output, nb_neighbors=30, std_ratio=1.5)

    poisson_reconstruction(input_file, poisson_output, depth)
    alpha_shape_reconstruction(input_file, alpha_output, alpha)
    ball_pivoting_reconstruction(input_file, ballpiv_output, radii)
    #can also add output files to other methods if necessary.

if __name__ == "__main__":
    main()


#ball pivoting

#pcd = o3d.io.read_point_cloud("dragon.ply")

#Techniques to reduce nr of points used.
# downsample_factor = 0.5  # Keep 10% of the original points (adjust as needed)
# pcd = pcd.random_down_sample(downsample_factor)

# #voxel_size = 0.01  # Larger voxel size for faster processing (adjust as needed)
# #pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

# pcd.estimate_normals()
# radii = [0.075, 0.125, 0.225]
# print("radii: ", radii)
# mesh_bpa = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
# mesh_bpa.compute_vertex_normals()
# mesh_bpa.paint_uniform_color([237 / 255, 202 / 255, 29 / 255])
# o3d.visualization.draw_geometries([mesh_bpa], mesh_show_back_face=True)

def poisson_reconstruction(input_file, output_file, depth):
    pcd = o3d.io.read_point_cloud(input_file)
    pcd.estimate_normals()
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)

    # Optionally clean the mesh by removing low-density vertices
    vertices_to_remove = densities < np.quantile(densities, 0.01)
    mesh.remove_vertices_by_mask(vertices_to_remove)

    num_vertices = len(mesh.vertices)
    print(f"Number of vertices in the mesh: {num_vertices}")
    o3d.io.write_triangle_mesh(output_file, mesh)
    mesh.paint_uniform_color([237 / 255, 202 / 255, 29 / 255])
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

def alpha_shape_reconstruction(input_file, output_file, alpha):

    print("alpha= ", alpha)
    pcd = o3d.io.read_point_cloud(input_file)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(output_file, mesh)
    mesh.paint_uniform_color([237 / 255, 202 / 255, 29 / 255])
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)


def ball_pivoting_reconstruction(input_file, output_file, radii):
    #Techniques to reduce nr of points used.
    # downsample_factor = 0.5  # Keep 10% of the original points (adjust as needed)
    # pcd = pcd.random_down_sample(downsample_factor)

    # #voxel_size = 0.01  # Larger voxel size for faster processing (adjust as needed)
    # #pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    print("radii= ", radii)
    pcd = o3d.io.read_point_cloud(input_file)
    pcd.estimate_normals()
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(output_file, mesh)
    mesh.paint_uniform_color([237 / 255, 202 / 255, 29 / 255])
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

def main():
    input_file = "dragon.ply"
    depth = 12 #optimal for poisson for dragon
    alpha = 3.5 #optimal for alpha for dragon
    radii = [0.075, 0.125, 0.225] #good setting for ball pivoting for dragon
    poisson_output = "mesh_poisson.ply"
    alpha_output = "mesh_poisson.ply"
    ballpiv_output = "mesh_poisson.ply"
    
    poisson_reconstruction(input_file, poisson_output, depth)
    alpha_shape_reconstruction(input_file, alpha_output, alpha)
    ball_pivoting_reconstruction(input_file, ballpiv_output, radii)
    #can also add output files to other methods if necessary.

if __name__ == "__main__":
    main()