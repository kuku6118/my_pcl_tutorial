#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_nl.h>

#include <sensor_msgs/PointCloud2.h>
using std::cout;
using std::endl;
using std::stringstream;
using std::string;

using namespace pcl;

unsigned int filesNum = 0;
bool saveCloud(false);
bool combineCloud(false);


boost::shared_ptr<visualization::CloudViewer> viewer;
pcl::PointCloud<pcl::PointXYZRGBA> g_cloud;

void combineCloudto(pcl::PointCloud<pcl::PointXYZRGBA> &cloud, float *Rot, float *Tran)
{
    pcl::PointCloud<pcl::PointXYZRGBA> t_cloud;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    transform(1, 3) = 0.1;

    pcl::transformPointCloud(cloud, t_cloud, transform);

    


    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZRGBA> Final;
    icp.align(Final);

    //pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icpnl;
/*
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icpnl;

    icpnl.setInputCloud(cloud);
    icpnl.setInputTarget(g_cloud);

    icp.align(g_cloud);

    std::cout << "has converged:" << icpnl.hasConverged() << " score: " <<
    icpnl.getFitnessScore() << std::endl;
    std::cout << icpnl.getFinalTransformation() << std::endl;
    //g_cloud = g_cloud + t_cloud;
*/
}

void cloudCB(const sensor_msgs::PointCloud2& input)
{
    pcl::PointCloud<pcl::PointXYZRGBA> cloud; // With color

    pcl::fromROSMsg(input, cloud); // sensor_msgs::PointCloud2 ----> pcl::PointCloud<T>

    if(! viewer->wasStopped()) viewer->showCloud(g_cloud.makeShared());

    if(saveCloud)
    {
        stringstream stream;
        stream << "inputCloud"<< filesNum<< ".pcd";
        string filename = stream.str();

        if(io::savePCDFile(filename, cloud, true) == 0)
        {
            filesNum++;
            cout << filename<<" Saved."<<endl;
        }
        else PCL_ERROR("Problem saving %s.\n", filename.c_str());

        saveCloud = false;

    }

    if(combineCloud)
    {
        float R[9], t[3];
        combineCloudto(cloud, &R[0], &t[0]);
        combineCloud = false;
    }

}


void
keyboardEventOccured(const visualization::KeyboardEvent& event, void* nothing)
{
    if(event.getKeySym() == "space"&& event.keyDown())
        saveCloud = true;
    if((event.getKeySym() == "s" || event.getKeySym() == "S") && event.keyDown())
        combineCloud = true;
}

// Creates, initializes and returns a new viewer.
boost::shared_ptr<visualization::CloudViewer> createViewer()
{
    boost::shared_ptr<visualization::CloudViewer> v(new visualization::CloudViewer("OpenNI viewer"));
    v->registerKeyboardCallback(keyboardEventOccured);

    return(v);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "pcl_write");
    ros::NodeHandle nh;
    cout<< "Press space to record point cloud to a file."<<endl;

    viewer = createViewer();

    ros::Subscriber pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, cloudCB);

    ros::Rate rate(30.0);

    while (ros::ok() && ! viewer->wasStopped())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

