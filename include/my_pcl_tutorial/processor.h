#ifndef PCL_SAMPLE_PROCESSOR_H
#define PCL_SAMPLE_PROCESSOR_H

// See processor_node.cpp for explanations of what each header provides.
#include "pcl/point_types.h"         // pcl::PointXYZRGB
#include "pcl_ros/point_cloud.h"     // pcl::PointCloud
#include "sensor_msgs/PointCloud2.h" // sensor_msgs::PointCloud2
#include "ros/time.h"                // ros::Duration, ros::Time
#include <pcl/visualization/cloud_viewer.h>

namespace pcl_sample {
// Contains methods for processing a point cloud.
//
// Sample usage:
// Processor processor(ros::Time::now());
class Processor {
 public:
  Processor(const ros::Time& prev_time);
  ~Processor();

  // Computes the average point in the given point cloud, and stores the result
  // in the given PointXYZRGB.
  void Average(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    pcl::PointXYZRGB* average);

  // Subscriber callback to receive point cloud messages.
  void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);

  bool wasStopped();

 private:
  // Keep track of the average time between calls of the callback.
  ros::Time prev_time_;      // The time of the last callback.
  ros::Duration time_taken_; // The time between the last two callbacks.
  int num_calls_;            // The number of times the callback has been called.

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
  //std::shared_ptr<pcl::visualization::CloudViewer> viewer;
  //pcl::visualization::CloudViewer viewer;
};
};

// The type signature for Average is an example of a particular C++ convention
// used by some. Input parameters are first, followed by outputs. Inputs are
// generally const references or const pointers, indicating that they cannot be
// changed. Small parameters like ints are passed in by value as usual. Outputs
// are usually pointers, indicating that they may change. A function that has
// just one small output can return it as usual.

#endif
