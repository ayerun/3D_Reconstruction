/// \file
/// \brief Creates point cloud map using ICP.
///
/// PARAMETERS:
///     
/// PUBLISHES:
///     /fused (sensor_msgs/PointCloud2): concatenated point cloud
/// SUBSCRIBES:
///     /camera/depth/color/points (sensor_msgs/PointCloud2): camera point cloud data
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

static sensor_msgs::PointCloud2 PCold;
static sensor_msgs::PointCloud2 PCtrans;
static sensor_msgs::PointCloud2 PCfused;

static pcl::PCLPointCloud2 cloud_in_pcl;    //source
static pcl::PCLPointCloud2 cloud_out_pcl;   //target
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);       //source
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);      //target
static pcl::PointCloud<pcl::PointXYZ> cloud_fused;                                              //fused
static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
static Eigen::Matrix4f Tba;
static Eigen::Matrix4f Tcb;
static Eigen::Matrix4f transform;
static Eigen::Matrix4f Tca;

static const int frequency = 1;
static bool first = true;


/// \brief subscriber callback that performs ICP, transforms point cloud, fuses point clouds, publishes fused point cloud
/// \param PCnew - incoming point cloud
void pcCallback(const sensor_msgs::PointCloud2 PCnew)
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
        pcl_conversions::toPCL(PCnew,cloud_in_pcl);
        pcl_conversions::toPCL(PCold,cloud_out_pcl);
        pcl::fromPCLPointCloud2(cloud_in_pcl,*cloud_in);
        pcl::fromPCLPointCloud2(cloud_out_pcl,*cloud_out);

        //perform ICP
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_out);
        icp.align(cloud_fused);
        Tcb = icp.getFinalTransformation();

        //calculate transform
        Tca = Tcb*Tba;

        //Transform Point Cloud
        pcl_ros::transformPointCloud(Tca, PCnew, PCtrans);
        ROS_WARN_STREAM(icp.getFitnessScore());

        //Fuse point clouds
        pcl::concatenatePointCloud(PCfused,PCtrans,PCfused);


        //publish pointcloud
        pc_pub.publish(PCfused);

        //iterate
        PCold = PCnew;
        Tba = Tca;
    }
}

/// \brief initializes node, subscriber, publisher, parameters, and objects
/// \param argc - initialization arguement
/// \param argv - initialization arguement
/// \return 0 at end of function
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
    icp.setMaxCorrespondenceDistance(0.05);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);

    //intialize matricies
    Tba = Eigen::Matrix4f::Identity();
    Tcb = Eigen::Matrix4f::Identity();
    transform = Eigen::Matrix4f::Identity();
    Tca = Eigen::Matrix4f::Identity();

    //spin
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}