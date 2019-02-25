
#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include <pcl_ros/point_cloud.h>

#include <iostream>
#include <string>
#include <vector>

class PointCloudFilterNode{

    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;
    ros::Publisher pc_pub_;
    
    double lower_limit_;
    double upper_limit_;

public:
    PointCloudFilterNode(ros::NodeHandle nh)
    : nh_(nh)
    {
        pc_sub_= nh_.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, &PointCloudFilterNode::pcCallback, this);
        pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth/points/filtered", 100);
        
        ros::NodeHandle pn("~");
        pn.param<double>("lower", lower_limit_, 0.0);
        pn.param<double>("upper", upper_limit_, 2.0);
    }


    void pcCallback(const sensor_msgs::PointCloud2ConstPtr& pCloud)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg (*pCloud, *cloud);

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (lower_limit_, upper_limit_);

        pass.filter (*cloud_filtered);
        
		for (size_t l = 0; l < cloud_filtered->points.size(); l++)
        {
			auto b_hold = cloud_filtered->points[l].b;
			cloud_filtered->points[l].b = cloud_filtered->points[l].r;
			cloud_filtered->points[l].r = b_hold;
        }
        pc_pub_.publish(*cloud_filtered);

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

