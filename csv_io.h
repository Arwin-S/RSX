#pragma once

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>

int csvRead(Eigen::MatrixXd &outputMatrix, const std::string &fileName, const std::streamsize dPrec);
int csvWrite(const Eigen::MatrixXd &inputMatrix, const std::string &fileName, const std::streamsize dPrec);