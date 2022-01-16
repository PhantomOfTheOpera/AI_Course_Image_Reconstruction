import open3d as o3d


class LoadPCD:

    voxel_size = 1.0
    reduced_voxel_size = 0.03

    def __init__(self, filepath, number_of_slices=16):
        self.filepath = filepath
        self.filename = filepath.split('/')[-1]
        self.number_of_slices = number_of_slices

    @staticmethod
    def set_voxel_size(new_value):
        LoadPCD.voxel_size = new_value

    def load_pcl(self):
        pcds = list()

        for i in range( self.number_of_slices):
            pcd = o3d.io.read_point_cloud(self.filepath % i, format='pcd')
            print(self.filepath % i)
            pcd_down = pcd.voxel_down_sample(voxel_size=LoadPCD.voxel_size)
            pcds.append(pcd_down)

        return pcds
