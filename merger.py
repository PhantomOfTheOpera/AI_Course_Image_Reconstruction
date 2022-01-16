import open3d as o3d
import numpy as np


class PCDMerging:

    voxel_size = 1.0
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5

    def __init__(self, pcds, pcds_reduced_voxel_size):
        self.pcds = pcds
        self.pcds_reduced_voxel_size = pcds_reduced_voxel_size

    @staticmethod
    def set_voxel_size(new_value):
        PCDMerging.voxel_size = new_value

    @staticmethod
    def visualize(pcds_down):
        o3d.visualization.draw_geometries(pcds_down)

    @staticmethod
    def pairwise_registration(source, target, matrix=np.identity(4)):

        icp_coarse = o3d.pipelines.registration.registration_icp(
            source, target, PCDMerging.max_correspondence_distance_coarse, matrix,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))

        icp_fine = o3d.pipelines.registration.registration_icp(
            source, target, PCDMerging.max_correspondence_distance_fine,
            icp_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))

        transformation_icp = icp_fine.transformation

        information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            source, target, PCDMerging.max_correspondence_distance_fine,
            icp_fine.transformation)

        return transformation_icp, information_icp

    @staticmethod
    def full_registration(pcds_down):

        pose_graph = o3d.pipelines.registration.PoseGraph()
        odometry = np.identity(4)
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))

        for source_id in range(len(pcds_down)):

            for target_id in range(source_id + 1, len(pcds_down)):
                transformation_icp, information_icp = PCDMerging.pairwise_registration(
                    pcds_down[source_id], pcds_down[target_id])

                if target_id == source_id + 1:  # odometry case
                    odometry = np.dot(transformation_icp, odometry)
                    pose_graph.nodes.append(
                        o3d.pipelines.registration.PoseGraphNode(
                            np.linalg.inv(odometry)))
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=False))
                else:  # loop closure case
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=True))
        return pose_graph

    @staticmethod
    def optimize_pose_graph(pcds_down):
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            pose_graph = PCDMerging.full_registration(pcds_down)
        return pose_graph

    @staticmethod
    def global_optimization(pose_graph):
        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=PCDMerging.max_correspondence_distance_fine,
            edge_prune_threshold=0.1,
            reference_node=0)

        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug):
            o3d.pipelines.registration.global_optimization(
                pose_graph,
                o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
                o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
                option)

    def join_pcd(self):
        pcd_down = self.pcds

        # pose_graph optimization
        pose_graph = PCDMerging.optimize_pose_graph(pcd_down)
        PCDMerging.set_voxel_size(0.03)

        # global optimization
        PCDMerging.global_optimization(pose_graph)

        pcds = self.pcds_reduced_voxel_size
        pcd_combined = o3d.geometry.PointCloud()

        for point_id in range(len(pcds)):
            pcds[point_id].transform(pose_graph.nodes[point_id].pose)
            pcd_combined += pcds[point_id]
        pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=PCDMerging.voxel_size)

        return pcd_combined_down


class Visualize:
    @staticmethod
    def visualize(pcds_down):
        o3d.visualization.draw_geometries(pcds_down)
