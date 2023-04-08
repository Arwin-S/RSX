#include "segment_pcl.h"
#include "pcl_traversibility_score.h"

void segment_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr rover_cloud, int local_map_dim)
{
    using namespace pcl;


    //get min_max x and y of input cloud to segment
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D (*rover_cloud, min, max);
    double x_sz = max.x - min.x;
    double y_sz = max.y - min.y;
    double x_grid_sz = x_sz / local_map_dim;
    double y_grid_sz = y_sz / local_map_dim;

    std::cout << min.x << " " << min.y << "\n";
    std::cout << max.x << " " << max.y << "\n\n";

    // length of local map sides (is a square)
    double local_map_len = local_map_tr.x - local_map_tl.x;
    double cell_len = local_map_len / local_map_dim;
    //std::cout << cell_len << std::endl;

    // // Translate crop box up by 1
    // cropBoxFilter.setTranslation(Eigen::Vector3f(0, 1, 0));

    for (int i = 0; i < local_map_dim; i++)
    {
        for (int j = 0; j < local_map_dim; j++)
        {
            // std::cout << "Min point: X : " << local_map_tl.x + (cell_len * j) << " Y : " << local_map_bl.y + (cell_len * i);
            // std::cout << "      Max point: X : " << local_map_tl.x + (cell_len * (j + 1)) << " Y : " << local_map_bl.y + (cell_len * (i + 1));
            // std::cout << std::endl;

            CropBox<PointXYZ> cropBoxFilter (true);
            cropBoxFilter.setInputCloud (rover_cloud);
            //order is bottom left --> top right
            // Eigen::Vector4f min_pt (local_map_tl.x + (cell_len * j), local_map_bl.y + (cell_len * i), -1000.0f, -1000.0f);
            // Eigen::Vector4f max_pt (local_map_tl.x + (cell_len * (j + 1)), local_map_bl.y + (cell_len * (i + 1)), 1000.0f, 1000.0f);

            Eigen::Vector4f min_pt (min.x + (x_grid_sz * j), min.y + (y_grid_sz * i), -1000.0f, -1000.0f);
            Eigen::Vector4f max_pt (min.x + (x_grid_sz * (j + 1)), min.y + (y_grid_sz * (i + 1)), 1000.0f, 1000.0f);

            cropBoxFilter.setMin (min_pt);
            cropBoxFilter.setMax (max_pt);

            // Cloud
            PointCloud<PointXYZ> cloud_out;
            cropBoxFilter.filter (cloud_out);
            std::cout << cloud_out.width << std::endl;

            // for (const auto& point: cloud_out)
            // {
            //     std::cout << i << "th Row, " << j << "th Col: " << point.x << " "
            //               << point.y << " "
            //               << std::endl;
            // }
            // do some traversibility stuff here...
            PointCloud<PointXYZ>::Ptr cloud_out_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            *cloud_out_ptr = cloud_out;
            double score = calculateTraversabilityScore(cloud_out_ptr);
        }
    }
}
//translates all points to local map frame
void translate_rover_to_local(pcl::PointCloud<pcl::PointXYZ> &rover_cloud, pcl::PointXYZ local_origin)
{
    for (auto &point : rover_cloud)
    {
        point.x = point.x - local_origin.x;
        point.y = point.y - local_origin.y;
    }
}