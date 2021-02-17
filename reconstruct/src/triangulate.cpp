#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

static ros::Subscriber pc_sub;
static sensor_msgs::PointCloud2Ptr scan;

//create meshing service
//use GreedyProjectionTriangulation to generate PolygonMesh
//savePolygonFileSTL()

void pcCallback(const sensor_msgs::PointCloud2Ptr &PCmsg)
{
    scan = PCmsg;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"triangulate");
    ros::NodeHandle nh;

    pc_sub = nh.subscribe("/rtabmap/cloud_map", 10, pcCallback);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 1;
}

//bill strong works with 3D scanning