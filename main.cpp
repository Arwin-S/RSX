#include "custom_ransac.h"
#include "pcl_traversibility_score.h"

using namespace std;
using Eigen::MatrixXd;

int main(int, char **)
{
    // Use Point Cloud File

    int error;
    MatrixXd A;

    // tic();
    error = csvRead(A, "../pointcloud.csv", 25);
    // toc();

    if (error == 0)
    {
        cout << "Matrix (" << A.rows() << "x" << A.cols() << "):" << endl;
        //   cout << A << endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(A.rows());
    // loop over each element in the matrix and add it to the point cloud
    for (int i = 0; i < A.rows(); ++i) {

            // create a new point with the matrix element as its x coordinate
            pcl::PointXYZ point;
            point.x = A(i,0);
            point.y = A(i,1);
            point.z = A(i,2);

            // add the point to the point cloud
           (*cloud)[i] = point;
    }
    // print cloud
    // std::cout << "PointCloud:" << std::endl;
    // for (const auto& point : *cloud) {
    //     std::cout << "  (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    // }

    // calculate the traversability score
    double score = calculateTraversabilityScore(cloud);
    std::cout << "traversability score: " << score << std::endl;

    // // Use Randomly Generated Points to test and visualize RANSAC

    // int numPoints = 10;
    // MatrixXd pc = (MatrixXd::Random(numPoints, 3) + MatrixXd::Constant(numPoints, 3, 1)) * 50; // random numbers 0 - 100
    // pc.col(2) = pc.col(2).array() * 0.1 + 50;

    // // 1 point at 0 and 1 point at 100 forces matplotlib plot z axis in range 0 - 100
    // pc(0, 2) = 0;
    // pc(numPoints - 1, 2) = 100;

    // Experimenting with Segmentation into X-by-X grid


    // Eigen::Vector4d plane = ransac(A);
    // cout << "Coefficents of Plane (Ax + By +Cz + D):" << endl
    //      << plane << endl
    //      << endl;
}
