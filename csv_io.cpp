#include "csv_io.h"
using Eigen::MatrixXd;
using namespace std;

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
