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
#include <pcl_ros/transforms.h>
#include <iostream>

//global variables
static ros::Subscriber pc_sub;
static ros::Publisher pc_pub;

static sensor_msgs::PointCloud2Ptr PCold;
static sensor_msgs::PointCloud2Ptr PCtrans;
static sensor_msgs::PointCloud2Ptr PCfused;

static pcl::PCLPointCloud2 cloud_in_pcl;    //source
static pcl::PCLPointCloud2 cloud_out_pcl;   //target
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);       //source
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);      //target
static pcl::PointCloud<pcl::PointXYZ> cloud_fused;                                              //fused
static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
static Eigen::Matrix4f transformation;

static const int frequency = 5;
static bool first = true;

/*
Perform ICP to find transformation
transform PointCloud
Concatenate PointCloud
publish concatenated pointcloud
*/
void pcCallback(const sensor_msgs::PointCloud2Ptr &PCnew)
{
    if(first)
    {
        //store pointcloud
        PCold = PCnew;
        PCfused = PCnew;
        PCtrans = PCnew;

        //publish pointcloud
        pc_pub.publish(PCfused);

        first = false;
    }
    else
    {
        //convert to PCLPointCloud2 then to PointCloud<pcl::PointXYZ>
        pcl_conversions::toPCL(*PCnew,cloud_in_pcl);
        pcl_conversions::toPCL(*PCold,cloud_out_pcl);
        pcl::fromPCLPointCloud2(cloud_in_pcl,*cloud_in);
        pcl::fromPCLPointCloud2(cloud_out_pcl,*cloud_out);

        //perform ICP
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_out);
        icp.align(cloud_fused);
        transformation = icp.getFinalTransformation();
        // std::cout << transformation << std::endl;

        //Transform Point Cloud
        pcl_ros::transformPointCloud(transformation, *PCnew, *PCtrans);

        // ROS_ERROR_STREAM(PCtrans->header);
        // ROS_ERROR_STREAM(PCfused->header);

        //Fuse point clouds
        pcl::concatenatePointCloud(*PCfused,*PCtrans,*PCfused);


        //publish pointcloud
        ROS_WARN_STREAM("Publishing");
        pc_pub.publish(PCfused);

        PCold = PCnew;
    }
}

int main(int argc, char** argv)
{
    //initialize node
    ros::init(argc, argv, "scanner");
    ros::NodeHandle nh;

    //initialize subscriber
    pc_sub = nh.subscribe("/voxel_grid/output", 10, pcCallback);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/fused", 10, true);

    //looping frequency
    ros::Rate r(frequency);

    //icp settings
    icp.setMaximumIterations(50);

    transformation = Eigen::Matrix4f::Identity();

    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}