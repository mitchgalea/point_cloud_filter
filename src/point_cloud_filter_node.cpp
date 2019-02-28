
#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <random>
#include <math.h>

class PointCloudFilterNode{

    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;
    ros::Publisher pc_pub_;
    
    double lower_limit_;
    double upper_limit_;
    double bracket_;
    double z_cutoff_;
    double voxel_;

    bool noise_;

public:
    PointCloudFilterNode(ros::NodeHandle nh)
    : nh_(nh)
    {
        pc_sub_= nh_.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, &PointCloudFilterNode::pcCallback, this);
        pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth/points/filtered", 100);
        
        ros::NodeHandle pn("~");
        pn.param<double>("bracket", bracket_, 0.01);
        pn.param<double>("z_cutoff", z_cutoff_, 4.00);
        pn.param<double>("voxel", voxel_, 0.01);
        pn.param<bool>("noise", noise_, false);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        bool listened = false;

        while(!listened)
        {
            listened = true;
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = tfBuffer.lookupTransform("camera_rgb_optical_frame", "base_scan",
                                                            ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ros::Duration(1.0).sleep();
                listened = false;
            }
            if(listened)
            {
                lower_limit_ = transformStamped.transform.translation.y - 0.001;
                upper_limit_ = transformStamped.transform.translation.y + 0.001;
            }
        }
    }

    double noise(double z)
    {
        double std_dev = 0.004 * pow(z, 1.5);
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);
        std::normal_distribution<double> normal_distribution(0.0, std_dev);

        double n = normal_distribution(generator);

    }

    void pcCallback(const sensor_msgs::PointCloud2ConstPtr& pCloud)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_y_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_z_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg (*pCloud, *cloud);

        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize (voxel_, voxel_, voxel_);
        voxel_grid.filter (*cloud_voxel_filtered);

        pcl::PassThrough<pcl::PointXYZRGB> y_pass;
        y_pass.setInputCloud(cloud_voxel_filtered);
        y_pass.setFilterFieldName ("y");
        y_pass.setFilterLimits (lower_limit_, upper_limit_);
        y_pass.filter (*cloud_y_filtered);

        for(size_t i = 0; i < cloud_y_filtered->points.size(); i++)
        {
            cloud_y_filtered->points[i].z += noise(cloud_y_filtered->points[i].z);
        }
        
        pcl::PassThrough<pcl::PointXYZRGB> z_pass;
        z_pass.setInputCloud (cloud_y_filtered);
        z_pass.setFilterFieldName ("z");
        z_pass.setFilterLimits (0.1, z_cutoff_);
        z_pass.filter (*cloud_z_filtered);
        pc_pub_.publish(*cloud_z_filtered);

    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_filter_node");

  ros::NodeHandle nh;

  PointCloudFilterNode a(nh);

  ros::spin();

  ros::shutdown();

  return 0;
}

