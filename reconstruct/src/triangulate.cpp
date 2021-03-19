/// \file
/// \brief Provides services to create .stl meshes or a .ply point cloud
///
/// PARAMETERS:
///     SearchRadius: Greedy Projection Triangulation search radius
///     Mu: Greedy Projection Triangulation Mu
///     MaximumNearestNeighbors: Greedy Projection Triangulation Maximum Nearest Neighbors
///     MaximumSurfaceAngle: Greedy Projection Triangulation Maximum Surface Angle
///     MaximumAngle: Greedy Projection Triangulation Maximum Angle
///     MinimumAngle: Greedy Projection Triangulation
///     Iso_Level: Marching Cubes iso level
///     GR_x: Marching Cubes grid resolution x
///     GR_y: Marching Cubes grid resolution y
///     GR_z: Marching Cubes grid resolution z
///     PolynomialOrder: Moving Least Squares order of polynomial
///     mls_SearchRadius: Moving Least Squares search radius
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

//Greedy Project Triangulation parameters
static double SearchRadius;
static double Mu;
static int MaximumNearestNeighbors;
static double MaximumSurfaceAngle;
static double MaximumAngle;
static double MinimumAngle;

//Marching Cubes parameters
static double Iso_Level;
static double GR_x;
static double GR_y;
static double GR_z;

//Moving Least Squares parameters
static int PolynomialOrder;
static double mls_SearchRadius;

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
    mls.setPolynomialOrder(PolynomialOrder);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(mls_SearchRadius);
    mls.process(*cloud_with_normals);
    
    //Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    //Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    //Set triangulation parameters
    gp3.setSearchRadius(SearchRadius); //maximum distance between connected points (max edge length)
    gp3.setMu(Mu);
    gp3.setMaximumNearestNeighbors(MaximumNearestNeighbors);
    gp3.setMaximumSurfaceAngle(MaximumSurfaceAngle);
    gp3.setMaximumAngle(MaximumAngle);
    gp3.setMinimumAngle(MinimumAngle);
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
    mls.setPolynomialOrder(PolynomialOrder);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(mls_SearchRadius);
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
    mc.setIsoLevel(Iso_Level);
    mc.setGridResolution(GR_x,GR_y,GR_z);
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
    mls.setPolynomialOrder(PolynomialOrder);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(mls_SearchRadius);
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

    //get parameters
    //greedy projection triangulation
    ros::param::get("/SearchRadius", SearchRadius);
    ros::param::get("/Mu", Mu);
    ros::param::get("/MaximumNearestNeighbors", MaximumNearestNeighbors);
    ros::param::get("/MaximumSurfaceAngle", MaximumSurfaceAngle);
    ros::param::get("/MaximumAngle", MaximumAngle);
    ros::param::get("/MinimumAngle", MinimumAngle);
    //marching cubes
    ros::param::get("/Iso_Level", Iso_Level);
    ros::param::get("/GR_x", GR_x);
    ros::param::get("/GR_y", GR_y);
    ros::param::get("/GR_z", GR_z);
    //moving least squares
    ros::param::get("/PolynomialOrder", PolynomialOrder);
    ros::param::get("/mls_SearchRadius", mls_SearchRadius);


    //spin
    ros::Rate r(frequency);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 1;
}
