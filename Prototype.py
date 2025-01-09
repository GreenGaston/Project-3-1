import open3d as o3d
import numpy as np
import math

def visualize_points_with_poisson(points, depth=8):
    """
    Visualizes the given points using Poisson surface reconstruction.

    :param points: A list of points to visualize.
    :param depth: The depth parameter for Poisson reconstruction.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.concatenate(points))
    pcd.estimate_normals()

    pcd.normals= o3d.utility.Vector3dVector(-np.asarray(pcd.normals))

    print('Performing Poisson surface reconstruction...')
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    mesh.compute_vertex_normals()

    yellow_color = np.array([[237/255, 202/255, 29/255] for _ in range(len(mesh.vertices))])
    mesh.vertex_colors = o3d.utility.Vector3dVector(yellow_color)

    o3d.visualization.draw_geometries([mesh])

def visualize_points_with_ball_pivoting(points, radii):
    """
    Visualizes the given points using Ball Pivoting surface reconstruction.

    :param points: A list of points to visualize.
    :param radii: A list of radii to use for Ball Pivoting.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.concatenate(points))
    pcd.estimate_normals()

    print('Performing Ball Pivoting surface reconstruction...')
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))

    yellow_color = np.array([[237/255, 202/255, 29/255] for _ in range(len(mesh.vertices))])
    mesh.vertex_colors = o3d.utility.Vector3dVector(yellow_color)

    o3d.visualization.draw_geometries([mesh])

def convert_point_to_global_space(point, point_of_measurement):
    """
    Converts a point from relative space to global space without rotation.

    :param point: The point in relative space (x, y, z).
    :param point_of_measurement: The point of measurement in global space (x, y, z).
    :return: The point in global space (x, y, z).
    """
    point = np.array(point)
    #point_of_measurement = np.array(point_of_measurement)
    global_point = point + point_of_measurement
    return global_point

# Define the rotation matrix for 60 degrees around the y-axis
angle_rad = np.radians(300)
rotation_matrix = np.array([
    [np.cos(angle_rad), np.sin(angle_rad),0],
    
    [-np.sin(angle_rad),np.cos(angle_rad),0],
    [0, 0, 1]
])

# Define the movement vector in the unrotated plane
movement_vector = np.array([0.1, 0, 0])

# Apply the rotation to the movement vector
rotated_movement = np.dot(rotation_matrix, movement_vector)

print(f"Rotated movement vector: {rotated_movement}")

# File one has coordinates 0,0,0
# Each next file has an x+0.10
# Convert these points to global space and visualize them
pointsxy=[]
pointsxz=[]
pointsyz=[]
movemenx=0
movementy=0
movementz=0




for i in range(1, 2):
    filename = f"output{i}.csv"
    print(f"Reading file {filename}")
    data = np.loadtxt(filename, delimiter=',', skiprows=1, usecols=(1, 2, 3))

    #rotate all points by n degreese around the z axis
    for i in range(len(data)):

        data[i]=np.dot(rotation_matrix, data[i])
        #scale all points down to 1/1000
        #data[i]=data[i]/1000
        

    # Convert the points
    points_of_measurement_xy = [0+(i-1)*0.000001,0,0]


    converted_points = convert_point_to_global_space(data, points_of_measurement_xy)
    # Include points_of_measurement_xy in the points array
    #also add the origin point
    points_with_measurement = np.vstack([converted_points, points_of_measurement_xy])
    pointsxy.append(points_with_measurement)
  

# for i in range(len(pointsxy)):
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(pointsxy[i])
#     o3d.visualization.draw_geometries([pcd])



# # Visualize the points with adjusted point size
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.concatenate(pointsxy))

# Create a visualization window and set the point size
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)
opt = vis.get_render_option()
opt.point_size = 0.01  # Default point size
vis.run()
vis.destroy_window()
visualize_points_with_poisson(pointsxy, 6)
# visualize_points_with_ball_pivoting(pointsxy, [0.075, 0.125, 0.225])

