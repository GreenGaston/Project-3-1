import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("dragon.ply") 
alpha = 3.5
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()
yellow_color = np.array([[237/255, 202/255, 29/255] for _ in range(len(mesh.vertices))])

mesh.vertex_colors = o3d.utility.Vector3dVector(yellow_color)

o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
