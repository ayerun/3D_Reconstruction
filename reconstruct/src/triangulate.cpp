/// \file
/// \brief Provides services to create .stl meshes or a .ply point cloud
///
/// PARAMETERS:
///     
/// PUBLISHES:
///     
/// SUBSCRIBES:
///     /rtabmap/cloud_map (sensor_msgs/PointCloud2): camera point cloud data
/// BROADCASTS:
///     
/// SERVICES:
///     create_gp_mesh: uses greedy projection triangulation to create .stl
///     create_mc_mesh: uses marching cubes to create .stl
///     create_ply: saves point cloud with normals as .ply

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>
#include <reconstruct/create_mesh.h>

//global variables
static sensor_msgs::PointCloud2Ptr scan;
static const int frequency = 200;

/// \brief subscriber callback stores address of latest cloud map
/// \param PCmsg - incoming cloud map
void pcCallback(const sensor_msgs::PointCloud2Ptr &PCmsg)
{
    scan = PCmsg;
}

/// \brief create .stl using greedy projection triangulation
/// \param req - service request
///              filename: location to save .stl
/// \param resp - service response (empty)
/// \return true when callback complete
bool gp_meshCallback(reconstruct::create_mesh::Request &req, reconstruct::create_mesh::Response &res)
{
    //convert to PCLPointCloud2 then to PointCloud<pcl::PointXYZ>
    pcl::PCLPointCloud2 cloud_transition;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*scan, cloud_transition);
    pcl::fromPCLPointCloud2(cloud_transition, *cloud);

    //Create search tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    //Moving Least Squares Normal Estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);
    mls.process(*cloud_with_normals);
    
    //Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    //Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    //Set triangulation parameters
    gp3.setSearchRadius(0.1); //maximum distance between connected points (max edge length)
    gp3.setMu(2.5);
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

/// \brief create .stl using marching cubes
/// \param req - service request
///              filename: location to save .stl
/// \param resp - service response (empty)
/// \return true when callback complete
bool mc_meshCallback(reconstruct::create_mesh::Request &req, reconstruct::create_mesh::Response &res)
{
    //convert to PCLPointCloud2 then to PointCloud<pcl::PointXYZ>
    pcl::PCLPointCloud2 cloud_transition;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*scan, cloud_transition);
    pcl::fromPCLPointCloud2(cloud_transition, *cloud);

    //Create search tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    //Moving Least Squares Normal Estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);
    mls.process(*cloud_with_normals);
    
    //Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    //Initialize objects
    pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
    pcl::PolygonMesh triangles;

    //Create mesh
    mc.setInputCloud(cloud_with_normals);
    mc.setSearchMethod(tree2);
    mc.setIsoLevel(0.5);
    mc.setGridResolution(32,32,32);
    mc.reconstruct(triangles);

    // export as stl
    pcl::io::savePolygonFileSTL(req.filename,triangles,true);

    return true;
}

/// \brief calculate normals and create .ply
/// \param req - service request
///              filename: location to save .ply
/// \param resp - service response (empty)
/// \return true when callback complete
bool plyCallback(reconstruct::create_mesh::Request &req, reconstruct::create_mesh::Response &res)
{
    //convert to PCLPointCloud2 then to PointCloud<pcl::PointXYZ>
    pcl::PCLPointCloud2 cloud_transition;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*scan, cloud_transition);
    pcl::fromPCLPointCloud2(cloud_transition, *cloud);

    //Create search tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    //Moving Least Squares Normal Estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);
    mls.process(*cloud_with_normals);

    //save normal cloud
    pcl::PLYWriter writer;
    writer.write(req.filename,*cloud_with_normals);

    return true;
}

/// \brief initializes node, subscriber, publisher, parameters, and objects
/// \param argc - initialization arguement
/// \param argv - initialization arguement
/// \return 0 at end of function
int main(int argc, char** argv)
{
    ros::init(argc,argv,"triangulate");
    ros::NodeHandle nh;

    //initialize subscriber and services
    const ros::Subscriber pc_sub = nh.subscribe("/rtabmap/cloud_map", 10, pcCallback);
    const ros::ServiceServer gp_mesh_service = nh.advertiseService("create_gp_mesh",gp_meshCallback);
    const ros::ServiceServer mc_mesh_service = nh.advertiseService("create_mc_mesh",mc_meshCallback);
    const ros::ServiceServer ply_service = nh.advertiseService("create_ply",plyCallback);

    //spin
    ros::Rate r(frequency);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 1;
}
