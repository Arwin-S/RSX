#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <ctime>
#include <stack>
#include <cstdlib>

using namespace std;
using Eigen::MatrixXd;

//time measurement
std::stack<clock_t> tictoc_stack;
void tic();
void toc();

int csvRead(MatrixXd &outputMatrix, const string &fileName, const streamsize dPrec);
int csvWrite(const MatrixXd &inputMatrix, const string &fileName, const streamsize dPrec);
Eigen::Vector4d ransac(MatrixXd &in);

// ransac helper functions
Eigen::Vector4d pointsToPlane(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3);
int distPlanePoint(const Eigen::Vector4d &plane, const MatrixXd &in, double dist);

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
   // cout << pc << endl<< endl<< endl;
}

Eigen::Vector4d ransac(MatrixXd &in)
{
    // Use current time as seed for random generator
    srand(time(0));

    // Parameters for RANSAC
    int nIterations = 10;
    double distThreshold = 0.01;
    double inlierThreshold = 0.8; // If this percentage of all points lie on plane, stop testing new iterations

    int numPoints = in.rows(); // conversion from long int to int should be fine as point cloud shouldnt have more than 2 billion points
    double numInliers = 0;
    double inlierPercent = 0;
    double bestInlierPercent = -1;

    Eigen::Vector4d bestPlane;
    int it = 0;

    tic();
    while (it < nIterations)
    {
        int p1 = (rand() % in.rows());
        int p2 = (rand() % in.rows());
        int p3 = (rand() % in.rows());
        Eigen::Vector4d ranPlane = pointsToPlane(in.row(p1), in.row(p2), in.row(p3));

        numInliers = distPlanePoint(ranPlane, in, distThreshold);

        inlierPercent = numInliers / numPoints;
        if (inlierPercent >= inlierThreshold)
        {
            bestPlane = ranPlane;
            it = nIterations;
        }
        else
        {
            if (inlierPercent > bestInlierPercent)
            {
                bestInlierPercent = inlierPercent;
                bestPlane = ranPlane;
            }
            inlierPercent = 0;
            numInliers = 0;
        }

        it++;
    }
    toc();

    // write to CSV to plot in matPlotLib
    csvWrite(in, "plotData.csv", 10);
    return bestPlane;
}

// Vectorized, should run much faster
// Returns # of points of input matrix that lie dist or closer to plane
int distPlanePoint(const Eigen::Vector4d &plane, const MatrixXd &in, double dist)
{
    Eigen::VectorXd values = (in.col(0).array() * plane(0) + in.col(1).array() * plane(1) + in.col(2).array() * plane(2) + plane(3)).abs();
    values.array() /= (sqrt(pow(plane(0), 2) + pow(plane(1), 2) + pow(plane(2), 2)));
    return (values.array() <= dist).count();
}

// Takes 3 xyz points and returns plane equation coefficients a,b,c,d as a vector
Eigen::Vector4d pointsToPlane(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3)
{
    Eigen::Vector3d AB = p2 - p1;
    Eigen::Vector3d AC = p3 - p1;

    Eigen::Vector3d cr = AB.cross(AC);

    double a = cr(0);
    double b = cr(1);
    double c = cr(2);

    double d = (-a * p1(0) - b * p1(1) - c * p1(2));
    Eigen::Vector4d coeff(a, b, c, d);
    return coeff;
}

// reads point cloud csv file and populates matrix
int csvRead(MatrixXd &outputMatrix, const string &fileName, const streamsize dPrec)
{
    ifstream inputData;
    inputData.open(fileName);
    cout.precision(dPrec);
    if (!inputData)
    {
        return -1;
    }
    string fileline, filecell;
    unsigned int prevNoOfCols = 0, noOfRows = 0, noOfCols = 0;
    while (getline(inputData, fileline))
    {
        noOfCols = 0;
        stringstream linestream(fileline);
        while (getline(linestream, filecell, ','))
        {
            try
            {
                stod(filecell);
            }
            catch (...)
            {
                return -1;
            }
            noOfCols++;
        }
        if (noOfRows++ == 0)
            prevNoOfCols = noOfCols;
        if (prevNoOfCols != noOfCols)
            return -1;
    }
    inputData.close();
    outputMatrix.resize(noOfRows, noOfCols);
    inputData.open(fileName);
    noOfRows = 0;

    // no populate matrix
    while (getline(inputData, fileline))
    {
        noOfCols = 0;
        stringstream linestream(fileline);
        while (getline(linestream, filecell, ','))
        {
            outputMatrix(noOfRows, noOfCols++) = stod(filecell);
        }
        noOfRows++;
    }
    return 0;
}

// writes to CSV for python plot.py to visualize
int csvWrite(const MatrixXd &inputMatrix, const string &fileName, const streamsize dPrec)
{
    int i, j;
    ofstream outputData;
    outputData.open(fileName);
    if (!outputData)
        return -1;
    outputData.precision(dPrec);
    for (i = 0; i < inputMatrix.rows(); i++)
    {
        for (j = 0; j < inputMatrix.cols(); j++)
        {
            outputData << inputMatrix(i, j);
            if (j < (inputMatrix.cols() - 1))
                outputData << ",";
        }
        if (i < (inputMatrix.rows() - 1))
            outputData << endl;
    }
    outputData.close();
    if (!outputData)
        return -1;
    return 0;
}

// time measurement functions
void tic()
{
    tictoc_stack.push(clock());
}

void toc()
{
    cout.precision(10);
    std::cout << "Time elapsed: "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack.pop();
}