import sys
import open3d as o3d

''' Down sample point cloud
    Usage: python down_sample_point_cloud.py <pcd_or_ply_file> <voxel_size> <outfile_type>
'''


def main():
    if len(sys.argv) != 3 and len(sys.argv) != 4:
        print("Usage: python down_sample_point_cloud.py <pcd_or_ply_file> <voxel_size> <outfile_type>")
        exit(1)
    pcd_file = sys.argv[1]
    voxel_size = sys.argv[2]
    outfile_type = pcd_file[-3:]
    if (len(sys.argv) == 5 and (sys.argv[4] == 'ply' or sys.argv[4] == 'pcd')):
        outfile_type = sys.argv[4]

    print("Reading pcd file...")
    pcd = o3d.io.read_point_cloud(pcd_file)
    # 下采样
    print("Down sampling point cloud...")
    pcd = pcd.voxel_down_sample(voxel_size=float(voxel_size))
    print("Writing bin file...")
    new_pcd_file = pcd_file[:-4] + '_down_sample' + '.' + outfile_type
    o3d.io.write_point_cloud(new_pcd_file, pcd)
    print("Done.")
    

if __name__ == "__main__":
    main()

