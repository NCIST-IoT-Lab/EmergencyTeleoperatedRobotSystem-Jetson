import sys
import open3d as o3d

''' Convert pcd to ply
    Usage: python pcd_to_ply.py <pcd_file> <ply_file>
'''

def main():
    if len(sys.argv) != 3:
        print("Usage: python pcd_to_ply.py <pcd_file> <ply_file>")
        exit(1)
    pcd_file = sys.argv[1]
    ply_file = sys.argv[2]
    print("Reading ply file...")
    pcd = o3d.io.read_point_cloud(pcd_file)
    print("Writing pcd file...")
    o3d.io.write_point_cloud(ply_file, pcd)
    print("Done.")

if __name__ == "__main__":
    main()
    
