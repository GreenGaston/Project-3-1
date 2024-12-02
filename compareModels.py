import trimesh
import numpy as np
from scipy.spatial import cKDTree

def load_model(filename):
    return trimesh.load(filename)

def hausdorff_distance(mesh1, mesh2):
    points1 = mesh1.vertices
    points2 = mesh2.vertices

    tree1 = cKDTree(points1)
    tree2 = cKDTree(points2)

    distances1, _ = tree1.query(points2, k=1)
    distances2, _ = tree2.query(points1, k=1)

    hausdorff_dist = max(np.max(distances1), np.max(distances2))
    return hausdorff_dist

def chamfer_distance(mesh1, mesh2):
    points1 = mesh1.vertices
    points2 = mesh2.vertices

    tree1 = cKDTree(points1)
    tree2 = cKDTree(points2)

    distances1, _ = tree1.query(points2, k=1)
    distances2, _ = tree2.query(points1, k=1)

    chamfer_dist = np.mean(distances1) + np.mean(distances2)
    return chamfer_dist

def surface_area_difference(mesh1, mesh2):
    area1 = mesh1.area
    area2 = mesh2.area

    return abs(area1 - area2)

def compare_models(file1, file2):
    mesh1 = load_model(file1)
    mesh2 = add_noise(load_model(file2), 0.0006)

    comparison_metrics = {
        'hausdorff_distance': hausdorff_distance(mesh1, mesh2),
        'chamfer_distance': chamfer_distance(mesh1, mesh2),
        'surface_area_difference': surface_area_difference(mesh1, mesh2),
        'volume_overlap': volume_overlap(mesh1, mesh2)
    }

    return comparison_metrics

def volume_overlap(mesh1, mesh2):
    return True
    volume1 = mesh1.volume
    volume2 = mesh2.volume

    intersection = mesh1.intersection(mesh2)
    if intersection.is_empty:
        intersection_volume = 0
    else:
        intersection_volume = intersection.volume

    union_volume = volume1 + volume2 - intersection_volume
    overlap_ratio = intersection_volume / union_volume

    return overlap_ratio

def add_noise(mesh, noise_level):
    #noise level is an integer describing the range from 0 to noise_level
    #to be added uniformly to each vertex
    for i in range(len(mesh.vertices)):
        noise = 2 * noise_level * np.random.rand(3) - noise_level


        mesh.vertices[i] += noise

    return mesh
if __name__ == '__main__':
    file1 = 'xyzrgb_dragon.ply'
    #make a noisy copy of the original mesh
    file2 = 'xyzrgb_dragon.ply'
    
    metrics = compare_models(file1, file2)
    print(f"Comparison metrics: {metrics}")