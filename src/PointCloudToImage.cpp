// ROS core
#include <ros/ros.h>
//Image message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
//stl stuff
#include <string>

union DataType
{
  float f;
  int d;
  uint8_t p[4];
};

class PointCloudToImage
{
public:
  void
  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    if ((cloud->width * cloud->height) == 0)
      return; //return if the cloud is not dense!

    ROS_INFO_STREAM("Point Cloud width " << cloud->width);
    ROS_INFO_STREAM("Point Cloud height " << cloud->height);
    ROS_INFO_STREAM("Point Cloud data size " << cloud->data.size());
    DataType x, y, z, color;
    
    x.p[0] = cloud->data[1228800];
    x.p[1] = cloud->data[1228801];
    x.p[2] = cloud->data[1228802];
    x.p[3] = cloud->data[1228803];
    
    y.p[0] = cloud->data[1228804];
    y.p[1] = cloud->data[1228805];
    y.p[2] = cloud->data[1228806];
    y.p[3] = cloud->data[1228807];
    
    z.p[0] = cloud->data[1228808];
    z.p[1] = cloud->data[1228809];
    z.p[2] = cloud->data[1228810];
    z.p[3] = cloud->data[1228811];
    
    color.p[0] = cloud->data[1228812];
    color.p[1] = cloud->data[1228813];
    color.p[2] = cloud->data[1228814];
    color.p[3] = cloud->data[1228815];
    ROS_INFO_STREAM("Point Cloud data " << x.f << " " << y.f << " " << z.f << " " << color.d);

    try
    {
      pcl::toROSMsg (*cloud, image_); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }
    image_pub_.publish (image_); //publish our cloud image
  }
  PointCloudToImage () : cloud_topic_("input"),image_topic_("output")
  {
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2> (cloud_topic_, 30,
                          &PointCloudToImage::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
  }
private:
  ros::NodeHandle nh_;
  sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "convert_pointcloud_to_image");
  PointCloudToImage pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}
