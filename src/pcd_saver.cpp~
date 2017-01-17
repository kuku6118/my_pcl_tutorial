#include <iostream>
#include <cstdlib>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_nl.h>

using std::cout;
using std::endl;
using std::stringstream;
using std::string;

using namespace pcl;

unsigned int filesNum = 0;
bool saveCloud(false);
bool combineCloud(false);
bool loadCloud(false);


boost::shared_ptr<visualization::CloudViewer> viewer;
pcl::PointCloud<pcl::PointXYZRGBA> g_cloud;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudw(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB> cloudw;

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

#define _LOAD_POSE_
int LoadPoseArray(const char *filename, double** (&Rot), double** (&Tran))
{
	int num = 0;
	//FILE *file = fopen("data\\pose_data.txt", "r");
	FILE *file = fopen(filename, "r");
	fscanf(file, "%d\n", &num);
	if (num > 0)
	{
		Rot = new double*[num];
		Tran = new double*[num];
		for (int i = 0; i < num; ++i)
		{
			Rot[i] = new double[9];
			Tran[i] = new double[3];
#ifdef _LOAD_POSE_
			fscanf(file, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n",
				&Rot[i][0], &Rot[i][1], &Rot[i][2],
				&Rot[i][3], &Rot[i][4], &Rot[i][5],
				&Rot[i][6], &Rot[i][7], &Rot[i][8],
				&Tran[i][0], &Tran[i][1], &Tran[i][2]);
			Tran[i][0] /= 1000.0;
			Tran[i][1] /= 1000.0;
			Tran[i][2] /= 1000.0;
#else
			double Rx = 0, Ry = 10 * (i + 1), Rz = 0;
			Rx = Rx * M_PI / 180.0;
			Ry = Ry * M_PI / 180.0;
			Rz = Rz * M_PI / 180.0;
			double cx = cos(Rx), sx = sin(Rx),
				cy = cos(Ry), sy = sin(Ry),
				cz = cos(Rz), sz = sin(Rz);
			Rot[i][0] = cx * cy;
			Rot[i][1] = cx * sy * sz - sx * cz;
			Rot[i][2] = sx * sz + cx * sy * cz;
			Rot[i][3] = sx * cy;
			Rot[i][4] = sx * sy * sz + cx * cz;
			Rot[i][5] = sx * sy * cz - cx * sz;
			Rot[i][6] = -sy;
			Rot[i][7] = cy * sz;
			Rot[i][8] = cy * cz;
#endif
		}
		fclose(file);
		return num;
	}
	fclose(file);
	return 0;
}

int data_num = 0;
int data_size = 0;
double **Rot, **Tran;
int LoadPCDandCombineCloud()
{
	std::string path = ros::package::getPath("my_pcl_tutorial");
	std::string data_path = path + "/data/";
	//std::string planeinfo_path = data_path + "cloud";
	//std::string camera_mat_path = data_path + "pose_0000000";
	char cloud_data[256];
	char pose_data[256];

	sprintf(&cloud_data[0], "%scloud%d.pcd", data_path.c_str(), data_num);
	//sprintf(&pose_data[0], "%spose_%08d.txt", data_path.c_str(), data_num);
	sprintf(&pose_data[0], "%spose_data.txt", data_path.c_str());
	
	if (data_num == 0)
		data_size = LoadPoseArray(pose_data, Rot, Tran);

	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudl (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudt (new pcl::PointCloud<pcl::PointXYZRGB>);

	if (data_size > 0)
	{
		if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (cloud_data, *cloudl) == -1)
		{
			PCL_ERROR("Couldn't read file %s", cloud_data);
			return -1;
		}

		Eigen::Matrix4f transform_1;// = Eigen::Matrix4f::Identity();

		transform_1 << Rot[data_num][0], Rot[data_num][1], Rot[data_num][2], Tran[data_num][0],
		Rot[data_num][3], Rot[data_num][4], Rot[data_num][5], Tran[data_num][1],
		Rot[data_num][6], Rot[data_num][7], Rot[data_num][8], Tran[data_num][2],
		0, 0, 0, 1;

		std::cout << transform_1 << std::endl;

		pcl::transformPointCloud(*cloudl, *cloudt, transform_1);

		//*cloudw += *cloudt;
		cloudw += *cloudt;

		data_num++;
	}
	else
	{
		data_num = -1;
	}
	return data_num;
}

void cloudCB(const sensor_msgs::PointCloud2& input)
{
    pcl::PointCloud<pcl::PointXYZRGBA> cloud; // With color

    pcl::fromROSMsg(input, cloud); // sensor_msgs::PointCloud2 ----> pcl::PointCloud<T>

    if(! viewer->wasStopped()) viewer->showCloud(g_cloud.makeShared());
    //if(! viewer->wasStopped()) viewer->showCloud(cloudw.makeShared());

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
/*
    if(loadCloud)
    {
		LoadPCDandCombineCloud();
		loadCloud = false;
    }*/
}


void
keyboardEventOccured(const visualization::KeyboardEvent& event, void* nothing)
{
    if(event.getKeySym() == "space"&& event.keyDown())
        saveCloud = true;
    if((event.getKeySym() == "s" || event.getKeySym() == "S") && event.keyDown())
        combineCloud = true;
    if((event.getKeySym() == "l" || event.getKeySym() == "L") && event.keyDown())
        loadCloud = true;
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

