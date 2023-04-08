#include "custom_ransac.h"


//time measurement
std::stack<clock_t> tictoc_stack;
void tic();
void toc();


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

