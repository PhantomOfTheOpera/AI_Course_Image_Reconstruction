import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy


from merger import PCDMerging


def calculate_transformation(source_path, target_path):

    result = o3d.io.read_point_cloud(source_path, format='pcd')
    original = o3d.io.read_point_cloud(target_path, format='ply')

    down_ = original.voxel_down_sample(voxel_size=1)
    a = PCDMerging([result, original], 0.03)

    trans_init, _ = a.pairwise_registration(result, down_)

    return trans_init, result, down_


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def calculate_distance(source, target):
    distances = source.compute_point_cloud_distance(target)
    mean_error = np.mean(distances)
    diameter = np.linalg.norm(target.get_max_bound() - target.get_min_bound())

    return distances, mean_error / diameter


def plot_histogram(vector):

    plt.hist(vector, bins=100, range=(0, 10))
    plt.show()


if __name__ == "__main__":

    trans_matrix, source, target = calculate_transformation("data/output_data/garuda.pcd",
                                                            "data/Original/Garuda and Vishnu.ply")
    print(trans_matrix)

    # apply calculated transformations to align the source and target
    source.transform(trans_matrix)
    dist_vector, error = calculate_distance(source, target)

    # Vizualize original and transformed object
    draw_registration_result(source, target, np.identity(4))

    # print the metrics and plot the distance histogram
    print(error)
    print(max(dist_vector))
    plot_histogram(dist_vector)