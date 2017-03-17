// Implementation of the processor library.
// See processor.h for documentation.

// See processor_node.cpp for explanations of what each header provides.
#include "my_pcl_tutorial/processor.h"

#include <ros/package.h>
#include "pcl/point_types.h"                 // pcl::PointXYZRGB
#include "pcl_conversions/pcl_conversions.h" // pcl::fromROSMsg
#include "pcl_ros/point_cloud.h"             // pcl::PointCloud
#include "ros/console.h"                     // ROS_INFO
#include "sensor_msgs/PointCloud2.h"         // sensor_msgs::PointCloud2

#include <math.h> // is_nan

namespace pcl_sample {

//pcl::visualization::CloudViewer viewer("Cloud Viewer");

int user_data = 0; 
bool saveCloud(false);

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    // set background to black (R = 0, G = 0, B = 0)
#if 0
    viewer.setBackgroundColor (0.1, 0.1, 0.1);  
    pcl::PointXYZ o;  
    o.x = 1.0;  
    o.y = 0;  
    o.z = 0;  
    viewer.addSphere (o, 0.25, "sphere", 0);  
    std::cout << "i only run once" << std::endl;
#else
    viewer.setBackgroundColor (0.1, 0.1, 0.1);
#endif
}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    // you can add something here, ex:  add text in viewer
#if 0
    static unsigned count = 0;  
    std::stringstream ss;  
    ss << "Once per viewer loop: " << count++;  
    viewer.removeShape ("text", 0);  
    viewer.addText (ss.str(), 200, 300, "text", 0);  
      
    //FIXME: possible race condition here:  
    user_data++;
#endif
}

void
keyboardEventOccured(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
  if(event.getKeySym() == "space"&& event.keyDown())
    saveCloud = true;
}

Processor::Processor(const ros::Time& prev_time)
  : prev_time_(prev_time),
    time_taken_(ros::Duration(0)),
    num_calls_(0) {

  boost::shared_ptr<visualization::CloudViewer> v(new visualization::CloudViewer("OpenNI viewer"));
  v->registerKeyboardCallback(keyboardEventOccured);
  m_viewer = v;

  m_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  //viewer = std::shared_ptr<pcl::visualization::CloudViewer>(new pcl::visualization::CloudViewer);
  // blocks until the cloud is actually rendered
  m_viewer->showCloud(m_cloud);

  // use the following functions to get access to the underlying more advanced/powerful
  // PCLVisualizer

  // This will only get called once
  m_viewer->runOnVisualizationThreadOnce (viewerOneOff);

  // This will get called once per visualization iteration
  m_viewer->runOnVisualizationThread (viewerPsycho);
}

Processor::~Processor()
{
}

void Processor::Average(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    pcl::PointXYZRGB* average) { 
  double average_x = 0;
  double average_y = 0;
  double average_z = 0;
  for (const pcl::PointXYZRGB& point : cloud.points) {
    // Some points may have NaN values for position.
    if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
      continue;
    }
    average_x += point.x;
    average_y += point.y;
    average_z += point.z;
  }
  average->x = average_x / cloud.points.size();
  average->y = average_y / cloud.points.size();
  average->z = average_z / cloud.points.size();
  
  ros::Time current_time = ros::Time::now();
  time_taken_ += current_time - prev_time_;
  prev_time_ = current_time;
  num_calls_ += 1;
}

void Processor::Callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  ROS_INFO("Processor::Callback start.\n");
  // Use PCL's conversion.


  try
  {
    pcl::fromROSMsg(*msg, *m_cloud);
    if (!m_viewer->wasStopped())
      m_viewer->showCloud(m_cloud->makeShared());
  }
  catch (std::runtime_error e)
  {
    ROS_ERROR_STREAM("Error in converting cloud to image message: "
                      << e.what());
  }
/*
  pcl::PointXYZRGB average;

  Average(*m_cloud, &average);

  // Log the average point at the INFO level. You can specify other logging
  // levels using ROS_DEBUG, ROS_WARN, etc. Search for "ros logging" online.
  ROS_INFO("x: %f y: %f z: %f, time/point: %fs",
    average.x, average.y, average.z, time_taken_.toSec() / num_calls_);*/
  ROS_INFO("Processor::Callback end.\n");
}

bool Processor::wasStopped()
{
  return m_viewer->wasStopped();
}

}
