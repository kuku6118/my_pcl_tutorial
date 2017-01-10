#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//  BOOST_FOREACH (const pcl::PointXYZRGB& pt, msg->points)
//    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  ros::NodeHandle nh;
  std::string topic = nh.resolveName("point_cloud");
  uint32_t queue_size = 1;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (topic, queue_size, callback);

  ros::spin ();

  return 0;
}
