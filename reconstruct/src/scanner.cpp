/// \file
/// \brief 
///
/// PARAMETERS:
///     
/// PUBLISHES:
///     
/// SUBSCRIBES:
///     /camera/depth/color/points (sensor_msgs/PointCloud2): camera pointcloud data
/// BROADCASTS:
///     
/// SERVICES:
///  

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

//global variables
static ros::Subscriber pc_sub;
static ros::Publisher pc_pub;

static sensor_msgs::PointCloud2Ptr PCfused;

static pcl::PCLPointCloud2 cloud_in_pcl;
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

static const int frequency = 2;
static bool first = true;

/*
Perform ICP to find transformation
transform PointCloud
Concatenate PointCloud
publish concatenated pointcloud
*/
void pcCallback(const sensor_msgs::PointCloud2Ptr &PCmsg)
{
    if(first)
    {
        //store pointcloud
        PCfused = PCmsg;

        //publish pointcloud
        pc_pub.publish(PCfused);

        first = false;
    }
    else
    {
        pcl_conversions::toPCL(*PCmsg,cloud_in_pcl);
        pcl::fromPCLPointCloud2(cloud_in_pcl,*cloud_in);
    }
}

int main(int argc, char** argv)
{
    //initialize node
    ros::init(argc, argv, "scanner");
    ros::NodeHandle nh;

    //initialize subscriber
    pc_sub = nh.subscribe("/camera/depth/color/points", 10, pcCallback);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/fused", 10, true);

    //looping frequency
    ros::Rate r(frequency);

    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}