from merger import PCDMerging, Visualize
from writer import WritePCD
from dataloader import LoadPCD

if __name__ == '__main__':
    reader = LoadPCD("data/input_data/Garuda/Garuda_part_%d.pcd")

    pcds = reader.load_pcl()

    # LoadPCD.set_voxel_size(1)
    pcds_reduced_voxel_size = reader.load_pcl()

    # before optimization
    Visualize.visualize(pcds)

    merger = PCDMerging(pcds, pcds_reduced_voxel_size)
    combined_pcd = merger.join_pcd()

    # after optimization
    Visualize.visualize([combined_pcd])

    writer = WritePCD(combined_pcd, "data/output_data/garuda.pcd")
    writer.write_pcd()
