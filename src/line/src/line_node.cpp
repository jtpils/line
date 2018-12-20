#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/pca.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile("line.pcd", *laser_ptr);

    pcl::PointCloud<pcl::PointXYZ> line1;

    while(1)
    {
        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(laser_ptr));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);
        ransac.setDistanceThreshold(0.03);
        ransac.computeModel();
        pcl::PointCloud<pcl::PointXYZ> line;
        std::vector<int> indices;
        ransac.getInliers(indices);
        
        if(indices.size() < 4)
        {
            break;
        }
        pcl::copyPointCloud(*laser_ptr, indices, line);

        pcl::PointCloud<pcl::PointXYZ>::Ptr line_ptr(new pcl::PointCloud<pcl::PointXYZ>(line));
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(line_ptr);
        std::cout << "..........." << std::endl;
        Eigen::Vector3f yang = pca.getEigenVectors().col(0);
        Eigen::Vector3f jia(0.027775198, -0.085673995, 0.50992984);
        float angle = acos(yang.dot(jia) / (yang.norm() * jia.norm())) * 180.0 / M_PI;
        std::cout << angle << std::endl;

        if(angle < 10 || angle > 170)
        {
            line1 += line;
        }
        pcl::PointCloud<pcl::PointXYZ> tmp;
        for(int j = 0; j < laser_ptr->points.size(); j ++)
        {
            std::vector<int>::iterator iter = find(indices.begin(), indices.end(), j);
            if(iter == indices.end())
            {
                tmp.push_back(laser_ptr->points[j]);
            }
        }

        *laser_ptr = tmp;

    }

    pcl::io::savePCDFile("2.pcd", line1);
    return 0;
}
