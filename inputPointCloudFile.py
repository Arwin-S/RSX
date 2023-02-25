import numpy as np
import open3d as o3d
import time

if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("CSite3_orig-utm.pcd", format='pcd')
    print (pcd)
    o3d.visualization.draw_geometries([pcd])

    write = np.asarray(pcd.points)
    np.savetxt('pointcloud.csv', write, delimiter=',')

