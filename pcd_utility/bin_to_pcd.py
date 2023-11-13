import sys
import numpy as np
import open3d as o3d
import struct

''' Convert bin to pcd
    Usage: python bin_to_pcd.py <bin_file> <pcd_file>
'''


def main():
    if len(sys.argv) != 3:
        print("Usage: python bin_to_pcd.py <bin_file> <pcd_file>")
        exit(1)
    bin_file = sys.argv[1]
    pcd_file = sys.argv[2]
    print("Reading bin file...")
    size_float = 3
    list_pcd = []
    with open(bin_file, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z = struct.unpack("fff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pcd)
    print("Writing pcd file...")
    o3d.io.write_point_cloud(pcd_file, pcd)
    print("Done.")
    

if __name__ == "__main__":
    main()

