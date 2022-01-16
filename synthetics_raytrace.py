from __future__ import division
import trimesh
import numpy as np
import open3d as o3d


class RayTrace:

    def __init__(self, filepath, number_of_rotations: int = 16,
                 camera_resolution: tuple = (640, 480),
                 color: tuple = (.5, .5, .5)
                 ):
        self.filename = filepath.split('/')[1]
        self.short_name = self.filename.split()[0]
        self.mesh = trimesh.load(filepath, force='mesh')
        self.number_of_rotations = number_of_rotations
        self.camera_resolution = camera_resolution
        self.paint_color = color
        self.scene = None

    @staticmethod
    def calculate_rotation_matrix(number_of_rotations):
        angle = np.deg2rad(360 / number_of_rotations)

        rot_matrix = np.array([[np.cos(angle), 0, np.sin(angle), 0],
                               [0, 1, 0, 0],
                               [-np.sin(angle), 0, np.cos(angle), 0],
                               [0, 0, 0, 1]])
        return rot_matrix

    def apply_transform(self, rot_matrix):
        self.mesh.apply_transform(rot_matrix)

    def create_scene(self):
        self.scene = self.mesh.scene()

    def set_camera_resolution_and_fov(self):
        self.scene.camera.resolution = self.camera_resolution

        self.scene.camera.fov = 60 * (self.scene.camera.resolution /
                                      self.scene.camera.resolution.max())

    def calculate_rays_intersects_location(self):
        origins, vectors, pixels = self.scene.camera_rays()

        points, index_ray, index_tri = self.mesh.ray.intersects_location(origins, vectors, multiple_hits=False)
        return points

    @staticmethod
    def create_and_write_pcd_file(index, points, filename):

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        pcd.paint_uniform_color([.5, .5, .5])
        pcd.estimate_normals()
        o3d.io.write_point_cloud(f"input_data/{filename}_part_{index}.pcd", pcd)

    def slice_image(self):
        matrix = RayTrace.calculate_rotation_matrix(self.number_of_rotations)
        print(matrix)
        for rotation in range(self.number_of_rotations):
            self.apply_transform(matrix)
            print('transform applied')
            self.create_scene()
            print('scene created')
            self.set_camera_resolution_and_fov()
            print('parameters set')

            points = self.calculate_rays_intersects_location()
            RayTrace.create_and_write_pcd_file(rotation, points, self.short_name)


if __name__ == '__main__':

    a = RayTrace(filepath='data/Original/Garuda and Vishnu.ply', number_of_rotations=16, camera_resolution=(100, 100))
    a.slice_image()

