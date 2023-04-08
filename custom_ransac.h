#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <ctime>
#include <stack>
#include <cstdlib>
#include "csv_io.h"

using namespace std;
using Eigen::MatrixXd;

// ransac helper functions
Eigen::Vector4d pointsToPlane(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3);
int distPlanePoint(const Eigen::Vector4d &plane, const MatrixXd &in, double dist);

Eigen::Vector4d ransac(MatrixXd &in);


//Below is sample main function which uses custom ransac function

/*
int main(int, char **)
{
    // // Use Point Cloud File

    // int error;
    // MatrixXd A;

    // // tic();
    // error = csvRead(A, "pointcloud.csv", 25);
    // // toc();

    // if (error == 0)
    // {
    //     cout << "Matrix (" << A.rows() << "x" << A.cols() << "):" << endl;
    //     //   cout << A << endl;
    // }


    // Use Randomly Generated Points to test and visualize RANSAC

    int numPoints = 10;
    MatrixXd pc = (MatrixXd::Random(numPoints, 3) + MatrixXd::Constant(numPoints, 3, 1)) * 50; // random numbers 0 - 100
    pc.col(2) = pc.col(2).array() * 0.1 + 50;

    // 1 point at 0 and 1 point at 100 forces matplotlib plot z axis in range 0 - 100
    pc(0, 2) = 0;
    pc(numPoints - 1, 2) = 100;

    // Experimenting with Segmentation into X-by-X grid


    Eigen::Vector4d plane = ransac(pc);
    cout << "Coefficents of Plane (Ax + By +Cz + D):" << endl
         << plane << endl
         << endl;
}
*/