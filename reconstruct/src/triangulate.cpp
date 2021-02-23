#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/processing.h>
#include <reconstruct/create_mesh.h>

static sensor_msgs::PointCloud2Ptr scan;
static const int frequency = 200;

void pcCallback(const sensor_msgs::PointCloud2Ptr &PCmsg)
{
    scan = PCmsg;
}

bool meshCallback(reconstruct::create_mesh::Request &req, reconstruct::create_mesh::Response &res)
{
    //convert to PCLPointCloud2 then to PointCloud<pcl::PointXYZ>
    pcl::PCLPointCloud2 cloud_transition;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*scan, cloud_transition);
    pcl::fromPCLPointCloud2(cloud_transition, *cloud);

    //Processing
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proc (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::CloudSurfaceProcessing<pcl::PointXYZ,pcl::PointXYZ>::Ptr proc;
    // proc->setInputCloud(cloud);
    // proc->process(*cloud_proc);
    // cloud = cloud_proc; //used processed cloud from here

    //Normal Estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    //Concatenate XYZ with normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    
    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    //Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    //Set triangulation parameters
    gp3.setSearchRadius(0.05); //maximum distance between connected points (max edge length)
    gp3.setMu(2);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI/4);
    gp3.setMaximumAngle(2*M_PI/3);
    gp3.setMinimumAngle(M_PI/18);
    gp3.setNormalConsistency(false);

    //Create mesh
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // export as stl
    pcl::io::savePolygonFileSTL(req.filename,triangles,true);

    return true;
}

bool plyCallback(reconstruct::create_mesh::Request &req, reconstruct::create_mesh::Response &res)
{
    //convert to PCLPointCloud2 then to PointCloud<pcl::PointXYZ>
    pcl::PCLPointCloud2 cloud_transition;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*scan, cloud_transition);
    pcl::fromPCLPointCloud2(cloud_transition, *cloud);

    //Normal Estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    //Concatenate XYZ with normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    //save normal cloud
    pcl::PLYWriter writer;
    writer.write(req.filename,*cloud_with_normals);

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"triangulate");
    ros::NodeHandle nh;

    const ros::Subscriber pc_sub = nh.subscribe("/rtabmap/cloud_map", 10, pcCallback);
    const ros::ServiceServer meshservice = nh.advertiseService("create_mesh",meshCallback);
    const ros::ServiceServer plyservice = nh.advertiseService("create_ply",plyCallback);

    ros::Rate r(frequency);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 1;
}
