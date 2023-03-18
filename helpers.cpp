#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <stdlib.h>
#include <cmath>

// Calculate slope from a plane
double calculateSlopeFromPlane(double a, double b, double c, double d) {
    double magnitudeOfNormal = std::sqrt(a*a + b*b + c*c);

    // Normalize coefficients
    a = a / magnitudeOfNormal;
    b = b / magnitudeOfNormal;
    c = c / magnitudeOfNormal;
    d = d / magnitudeOfNormal;

    double slopeAngle = std::acos(c / magnitudeOfNormal);

    return slopeAngle;
}

// Calculate the roughness of the plane by measuring the standard deviation of each point from the plane
double calculateRoughnessFromPlane(double a, double b, double c, double d, double threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    double roughness = 0.0;
    int numPoints = 0;
    double sumDistanceSquared = 0.0;
    double sumDistance = 0.0;

    int totalPoints = cloud->points.size();
    auto points = cloud->points;

    for (int idx=0; idx<totalPoints; idx++) {
        auto point = points[idx];
        double distance = std::abs(a * point.x + b * point.y + c * point.z + d) / std::sqrt(a*a + b*b + c*c);
        if (distance < threshold) {
            numPoints++;
            sumDistanceSquared += distance*distance;
            sumDistance += distance;
        }
    }

    if (numPoints >= 2) {
        double meanDistance = sumDistance / numPoints;
        double varianceDistance = (sumDistanceSquared / numPoints) - (meanDistance * meanDistance);
        roughness = std::sqrt(varianceDistance);
    }

    return roughness;
}

// Ransac function on a pcl PointCloud
std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> ransac_on_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) 
{

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  //Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);

  // Computes the model
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    return {coefficients, inliers};
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (const auto& idx: inliers->indices)
    std::cerr << idx << "    " << cloud->points[idx].x << " "
                               << cloud->points[idx].y << " "
                               << cloud->points[idx].z << std::endl;

  return {coefficients, inliers};
}

// Calculates the traversabiliy score from a point cloud
double calculateTraversabilityScore(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // call ransac on the generated point cloud
    int numPoints = cloud->points.size();
    std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> result = ransac_on_cloud(cloud);
    // need a condition to check if the ransac was successful
    // Get coefficients of the plane
    double a = result.first->values[0];
    double b = result.first->values[1];
    double c = result.first->values[2];
    double d = result.first->values[3];

    // Calculate parameters from the ransac plane
    int inlierRatio = result.second->indices.size() / numPoints;
    double slope = calculateSlopeFromPlane(a, b, c, d);
    double roughnessThreshold = 2.0;
    double roughness = calculateRoughnessFromPlane(a, b, c, d, roughnessThreshold, cloud);

    // Hyperparameters 
    double roughnessParameter = 1.0;
    double slopeParameter = 1.0;
    double inlierRatioParameter = 1.0;

    double traversabilityScore = inlierRatio * inlierRatioParameter + slopeParameter * slope + roughness * roughnessParameter;
    return traversabilityScore;

}