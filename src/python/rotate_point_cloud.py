import sys
import open3d as o3d
import numpy as np

''' Rotate point cloud 
    Usage: python rotate_point_cloud.py <ply_or_pcd_file> <axis> <angle> <outfile_type>
'''

def get_rotation_matrix(axis, angle):
    """Get the rotation matrix for an axis and angle"""
    if axis == 'x':
        return np.array([[1, 0, 0],
                         [0, np.cos(angle), -np.sin(angle)],
                         [0, np.sin(angle), np.cos(angle)]])
    elif axis == 'y':
        return np.array([[np.cos(angle), 0, np.sin(angle)],
                         [0, 1, 0],
                         [-np.sin(angle), 0, np.cos(angle)]])
    elif axis == 'z':
        return np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0],
                         [0, 0, 1]])
    else:
        raise Exception("Invalid axis")
    

def main():
    print(len(sys.argv))
    if len(sys.argv) != 4 and len(sys.argv) != 5:
        print("Usage: python rotate_point_cloud.py <ply_or_pcd_file> <axis> <angle> <outfile_type>")
        exit(1)
    pcd_file = sys.argv[1]
    axis = sys.argv[2]
    angle = float(sys.argv[3])
    outfile_type = pcd_file[-3:]
    if (len(sys.argv) == 5 and (sys.argv[4] == 'ply' or sys.argv[4] == 'pcd')):
        outfile_type = sys.argv[4]


    print("Reading point cloud...")
    pcd = o3d.io.read_point_cloud(pcd_file)
    rm = get_rotation_matrix(axis, angle)
    center = np.array([0, 0, 0])
    print("Rotating point cloud...")
    pcd = pcd.rotate(rm, center)
    new_pcd_file = pcd_file[:-4] + '_r_' + axis + '_' + str(angle) + '.' + outfile_type
    print("Saving point cloud to " + new_pcd_file)
    o3d.io.write_point_cloud(new_pcd_file, pcd)
    print("Done!")

if __name__ == "__main__":
    main()
    
