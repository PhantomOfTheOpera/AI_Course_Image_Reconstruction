import open3d as o3d


class WritePCD:

    def __init__(self,  pcd_combined_down, path):

        self.path = path
        self.pcd_combined_down = pcd_combined_down

    def write_pcd(self):

        o3d.io.write_point_cloud(self.path, self.pcd_combined_down)