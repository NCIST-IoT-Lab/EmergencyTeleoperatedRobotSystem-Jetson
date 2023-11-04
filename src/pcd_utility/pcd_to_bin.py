import sys
import numpy as np
import open3d as o3d

''' Convert pcd to bin
    Usage: python pcd_to_bin.py <pcd_file> <bin_file>
'''


def main():
    if len(sys.argv) != 3:
        print("Usage: python pcd_to_bin.py <pcd_file> <bin_file>")
        exit(1)
    pcd_file = sys.argv[1]
    bin_file = sys.argv[2]
    print("Reading pcd file...")
    pcd = o3d.io.read_point_cloud(pcd_file)
    # 下采样
    # pcd = pcd.voxel_down_sample(voxel_size=0.04)
    print("Writing bin file...")
    points = np.asarray(pcd.points)
    points = points.astype(np.float32)
    points.tofile(bin_file)
    print("Done.")
    

if __name__ == "__main__":
    main()

