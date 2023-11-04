import sys
import open3d as o3d

''' Convert ply to pcd 
    Usage: python ply_to_pcd.py <ply_file> <pcd_file>
'''

def main():
    if len(sys.argv) != 3:
        print("Usage: python ply_to_pcd.py <ply_file> <pcd_file>")
        exit(1)
    ply_file = sys.argv[1]
    pcd_file = sys.argv[2]
    print("Reading ply file...")
    pcd = o3d.io.read_point_cloud(ply_file)
    print("Writing pcd file...")
    o3d.io.write_point_cloud(pcd_file, pcd)
    print("Done.")

if __name__ == "__main__":
    main()
    
