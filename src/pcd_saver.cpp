#include <iostream>
#include <cstdlib>
#include <chrono>

#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transforms.h>
#include <pcl/point_representation.h>

#include <pcl/registration/lum.h>

#include <opencv2/opencv.hpp>
#include "4pcs.h"
#include "io/io.h"

using std::cout;
using std::endl;
using std::stringstream;
using std::string;

using namespace pcl;
using namespace match_4pcs;

unsigned int filesNum = 0;
bool saveCloud(false);
bool combineCloud(false);
bool loadCloud(false);
bool saveAll(false);


boost::shared_ptr<visualization::CloudViewer> viewer;
pcl::PointCloud<pcl::PointXYZRGBA> g_cloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudw(new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB> cloudw;
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudwn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

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

void CleanInvalidNormals( vector<Point3D> &v, 
                          vector<cv::Point3f> &normals){
  if (v.size() == normals.size()){
    vector<Point3D>::iterator itV = v.begin();
    vector<cv::Point3f>::iterator itN = normals.begin();
  
    float norm;
    unsigned int nb = 0;
    for( ; itV != v.end(); ){
      norm = cv::norm((*itV).normal());
      if (norm < 0.1){
        itN = normals.erase(itN);
        itV = v.erase(itV);
        nb++;
      }else{
        if (norm != 1.){
          (*itN).x /= norm;
          (*itN).y /= norm;
          (*itN).z /= norm;
        }
        itV++;
        itN++;
      }
    }
    
    if (nb != 0){
      cout << "Removed " << nb << " invalid points/normals" << endl; 
    }
  }
}

bool is_nan(double dVal)
{
	return !(dVal == dVal);
}

#if 0
int isNumber(double d)
{
	return (d==d);
}
#else
int isNumber(double d)
{
	return (d==d)&&(d<=DBL_MAX&&d>=-DBL_MAX);
}
#endif

int isFiniteNumber(double d)
{
	return (d<=DBL_MAX&&d>=-DBL_MAX);
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
	char obj_data[256];
	char pose_data[256];

	sprintf(&cloud_data[0], "%scloud%d.pcd", data_path.c_str(), data_num);
	sprintf(&obj_data[0], "%scloud%d.obj", data_path.c_str(), data_num);
	//sprintf(&pose_data[0], "%spose_%08d.txt", data_path.c_str(), data_num);
	sprintf(&pose_data[0], "%spose_data.txt", data_path.c_str());
	
	if (data_num == 0)
		data_size = LoadPoseArray(pose_data, Rot, Tran);

	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudl (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudt (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::registration::LUM<pcl::PointXYZRGB> lum;

	if (data_size > 0)
	{
		if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (cloud_data, *cloudl) == -1)
		{
			PCL_ERROR("Couldn't read file %s", cloud_data);
			return -1;
		}

		for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloudl->end(); it != cloudl->begin(); --it)
		{
#if 1 //new nan and inf number check
			if (!(isNumber(it->x)) || !(isNumber(it->y)) || !(isNumber(it->z)))
				cloudl->erase(it);
#else
			if (is_nan(it->x) || is_nan(it->y) || is_nan(it->z))
				cloudl->erase(it);
#endif //new nan and inf number check
		}
		cloudl->erase(cloudl->begin());

		Eigen::Matrix4f transform_1;// = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f transform_tmp = Eigen::Matrix4f::Identity();;

#if 1 // global registration


#if 1 // camera extrinsic parameters to transform

		//pcl::copyPointCloud(*cloudl, *cloudt); //4pcs

		std::chrono::_V2::steady_clock::time_point t3 = std::chrono::_V2::steady_clock::now();
		transform_1 << Rot[data_num][0], Rot[data_num][1], Rot[data_num][2], Tran[data_num][0],
		Rot[data_num][3], Rot[data_num][4], Rot[data_num][5], Tran[data_num][1],
		Rot[data_num][6], Rot[data_num][7], Rot[data_num][8], Tran[data_num][2],
		0, 0, 0, 1;

		std::cout << transform_1 << std::endl;
#if 1
		pcl::transformPointCloud(*cloudl, *cloudt, transform_1);
#else
		pcl::transformPointCloud(*cloudl, *cloudl, transform_1);
#endif
		std::chrono::_V2::steady_clock::time_point t4 = std::chrono::_V2::steady_clock::now();

		printf("our time: %lld\n", std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count());

#endif // camera extrinsic parameters to transform

#if 0 // LUM algorithm
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudtmp (new pcl::PointCloud<pcl::PointXYZRGB>(cloudl[i])); 
		lum.addPointCould(cloudtmp);

		std::cout << transform_1 << std::endl;
		pcl::transformPointCloud(*cloudl, *cloudt, transform_1);
#endif // LUM algorithm



#else
		pcl::copyPointCloud(*cloudl, *cloudt);
#endif // global registration

		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
		ne.setInputCloud (cloudt);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		//pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
		ne.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (0.03);

		// Compute the features
		ne.compute (*cloud_normals);
		pcl::copyPointCloud(*cloudt, *cloud_normals);

		// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
		
		char cloud_data[256];
#if 0 //save single frame pcd (removal nan)
		
		sprintf(&cloud_data[0], "%strans_cloud%d.pcd", data_path.c_str(), data_num);
		//pcl::io::savePCDFile(cloud_data, *cloud_normals);
		pcl::io::savePCDFile(cloud_data, *cloudt);
#endif //save single frame pcd (removal nan)

#if 1 // save single frame obj
		pcl::PolygonMesh mesh;
		//pcl::toPCLPointCloud2(*cloud_normals, mesh.cloud);
		pcl::toPCLPointCloud2(*cloud_normals, mesh.cloud);
		sprintf(&cloud_data[0], "%scloud%d.obj", data_path.c_str(), data_num);
		pcl::io::saveOBJFile(cloud_data, mesh);
#endif // save single frame obj

		pcl::registration::LUM<pcl::PointXYZRGB> lum;

		vector<Point3D> set1, set2;
		vector<cv::Point2f> tex_coords1, tex_coords2;
		vector<cv::Point3f> normals1, normals2;
		vector<tripple> tris1, tris2;
		vector<std::string> mtls1, mtls2;

		//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudgicp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		if (data_num > 0)
		{
			//std::string input1 = data_path
			char input1[256];
			char input2[256];
#if 1 //registration method
/*
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

			sprintf(&input1[0], "%strans_cloud%d.pcd", data_path.c_str(), data_num-1);
			sprintf(&input2[0], "%strans_cloud%d.pcd", data_path.c_str(), data_num);

			if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (input1, *cloud_temp) == -1)
			{
				PCL_ERROR("Couldn't read file %s", cloud_data);
				return -1;
			}

			pcl::copyPointCloud(*cloud_temp, *cloud_in);

			if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (input2, *cloud_temp) == -1)
			{
				PCL_ERROR("Couldn't read file %s", cloud_data);
				return -1;
			}

			pcl::copyPointCloud(*cloud_temp, *cloud_out);*/
/*
			for (unsigned int i = 0; i < cloudt->size(); i++)
			{
				if(!isFiniteNumber((*cloudt)[i].x) || !isFiniteNumber((*cloudt)[i].y) || !isFiniteNumber((*cloudt)[i].z))
				{
					cloudt->erase(cloudt->begin() + i);
					i--;
				}
			}*/

#if 0 //erase points
#if 1 //direct erase
			cloud_in->erase(cloud_in->begin());
			cloud_out->erase(cloud_out->begin());
#else
			unsigned int nCount1 = 0, nCount2 = 0;
			for (unsigned int i = 0; i < cloud_in->size(); i++)
			{
				if(!isFiniteNumber((*cloud_in)[i].x) || !isFiniteNumber((*cloud_in)[i].y) || !isFiniteNumber((*cloud_in)[i].z))
				{
					cout << i << "not nuber 1: " << (*cloud_in)[i].x << ", " << (*cloud_in)[i].y << ", " << (*cloud_in)[i].z << endl;
					nCount1++;
					cloud_in->erase(cloud_in->begin() + i);
				}
			}
			for (unsigned int i = 0; i < cloud_out->size(); i++)
			{
				if(!isFiniteNumber((*cloud_out)[i].x) || !isFiniteNumber((*cloud_out)[i].y) || !isFiniteNumber((*cloud_out)[i].z))
				{
					cout << i << "not nuber 2: " << (*cloud_out)[i].x << ", " << (*cloud_out)[i].y << ", " << (*cloud_out)[i].z << endl;
					nCount2++;
					cloud_out->erase(cloud_out->begin() + i);
				}
			}
			cout << "not number size: " << nCount1 << ", " << nCount2 << endl;
#endif
#endif //erase points

			//cout << "pre frame size: " << cloud_in->size() << endl;
			//cout << "cur frame size: " << cloud_out->size() << endl;

#if 1 // global algorithm

#if 0 // LUM algorithm
			corrs = pcl::someAlgo(lum.getPointCloud(data_num-1), lum.getPointCloud(data_num));
			lum.setCorrespondences (data_num-1, data_num, corrs);
#endif // LUM algorithm

#if 0 // 4PCS algorithm
			float score = 0;
#if 1 // with ICP
			sprintf(&input1[0], "%sicp_trans_cloud%d.obj", data_path.c_str(), data_num-1);
#else
			sprintf(&input1[0], "%strans_cloud%d.obj", data_path.c_str(), data_num-1);
#endif // with ICP
			sprintf(&input2[0], "%scloud%d.obj", data_path.c_str(), data_num);

			IOManager iomananger;
			iomananger.ReadObject(&input1[0], set1, tex_coords1, normals1, tris1, mtls1);
			iomananger.ReadObject(&input2[0], set2, tex_coords2, normals2, tris2, mtls2);

			printf("num 1: %lu %lu %lu %lu %lu\n", set1.size(), tex_coords1.size(), normals1.size(), tris1.size(), mtls1.size());
			printf("num 2: %lu %lu %lu %lu %lu\n", set2.size(), tex_coords2.size(), normals2.size(), tris2.size(), mtls2.size());

			cv::Mat mat = cv::Mat::eye(4, 4, CV_64F);

			if (tris1.size() == 0)
				CleanInvalidNormals(set1, normals1);
			if (tris2.size() == 0)
				CleanInvalidNormals(set2, normals2);

			std::chrono::_V2::steady_clock::time_point t1 = std::chrono::_V2::steady_clock::now();

			Match4PCSOptions options;
#if 0 // 4PCS parameters setting
			options.overlap_estimation = 0.45;
			options.sample_size = 2000;
			options.max_normal_difference = 90.0;
			options.max_color_distance = 150;
			options.max_time_seconds = 1000;
			options.delta = 0.01;
#else
			options.overlap_estimation = 0.45;
			options.sample_size = 2000;
			options.max_normal_difference = 90.0;
			options.max_color_distance = 150;
			options.max_time_seconds = 1000;
			options.delta = 0.01;
#endif // 4PCS parameters setting

#if 1 //seletion Super4PCS or 4PCS
			MatchSuper4PCS matcher(options);
			//score = matcher.ComputeTransformation(set1, &set2, &mat);
			score = matcher.ComputeTransformation(set1, &set2, &mat);
#else
			Match4PCS matcher(options);
			score = matcher.ComputeTransformation(set1, &set2, &mat);
#endif //seletion Super4PCS or 4PCS

			Eigen::Matrix4f transform_2;// = Eigen::Matrix4f::Identity();

			transform_2 << mat.at<double>(0, 0), mat.at<double>(0, 1), mat.at<double>(0, 2), mat.at<double>(0, 3),
			mat.at<double>(1, 0), mat.at<double>(1, 1), mat.at<double>(1, 2), mat.at<double>(1, 3),
			mat.at<double>(2, 0), mat.at<double>(2, 1), mat.at<double>(2, 2), mat.at<double>(2, 3),
			0, 0, 0, 1;

			pcl::transformPointCloud(*cloud_normals, *cloud_normals, transform_2);
			std::chrono::_V2::steady_clock::time_point t2 = std::chrono::_V2::steady_clock::now();
			printf("4PCS time: %lld\n", std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());


			pcl::transformPointCloud(*cloudt, *cloudt, transform_2);

			cout << "score: " << score << endl;
			cout << "super4pcs: " << endl << mat << endl;

#if 1 // save single frame obj
			pcl::PolygonMesh meshw;
			pcl::toPCLPointCloud2(*cloud_normals, meshw.cloud);
			sprintf(&cloud_data[0], "%strans_cloud%d.obj", data_path.c_str(), data_num);
			pcl::io::saveOBJFile(cloud_data, meshw);
#endif // save single frame obj

#endif // 4PCS algorithm

#endif // global algorithm

#if 1 // ICP
			std::cout << "ICP begin " << data_num << endl;

			std::chrono::_V2::steady_clock::time_point t5 = std::chrono::_V2::steady_clock::now();

			pcl::IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
			/*icpn.setMaxCorrespondenceDistance(0.01);
			icpn.setTransformationEpsilon(1e-8);
			icpn.setEuclideanFitnessEpsilon(0.01);
			icpn.setMaximumIterations (100);
			icpn.setRANSACOutlierRejectionThreshold(1.5);*/
			icp.setMaxCorrespondenceDistance(0.03);
			icp.setTransformationEpsilon(1e-10);
			icp.setEuclideanFitnessEpsilon(0.001);
			icp.setMaximumIterations (500);
			icp.setRANSACOutlierRejectionThreshold(1.5);

			// set the input point cloud to align
			icp.setInputCloud(cloudt);
			// set the input reference point cloud
			icp.setInputTarget(cloudw);

			// compte the point cloud registration
			pcl::PointCloud<pcl::PointXYZRGB> Final;
			icp.align(Final);

			std::chrono::_V2::steady_clock::time_point t6 = std::chrono::_V2::steady_clock::now();
			printf("ICP time: %lld\n", std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5).count());

			// print if it the algorithm converged and its fitness score
			std::cout << "has converged:" << icp.hasConverged()
				<< " score: "
				<< icp.getFitnessScore() << std::endl;
			// print the output transformation
			std::cout << icp.getFinalTransformation() << std::endl;
			
			//*cloud_normals = Final;
			*cloudt = Final;


			std::chrono::_V2::steady_clock::time_point t7 = std::chrono::_V2::steady_clock::now();
#if 0
			transform_tmp = icp.getFinalTransformation() * transform_tmp;
			pcl::transformPointCloud(*cloud_normals, *cloud_normals, transform_tmp);
#else
			pcl::transformPointCloud(*cloud_normals, *cloud_normals, icp.getFinalTransformation());
#endif
			std::chrono::_V2::steady_clock::time_point t8 = std::chrono::_V2::steady_clock::now();
			printf("ICP transformation time: %lld\n", std::chrono::duration_cast<std::chrono::nanoseconds>(t8 - t7).count());

			//pcl::transformPointCloud(*cloudt, *cloudt, icp.getFinalTransformation());

#if 0 //save single frame pcd (removal nan)
			char cloud_data_re[256];
			sprintf(&cloud_data_re[0], "%strans_cloud%d_re.pcd", data_path.c_str(), data_num);
			pcl::io::savePCDFile(cloud_data_re, Final);
#endif //save single frame pcd (removal nan)

#if 1 // save single frame obj
			pcl::PolygonMesh meshicp;
			pcl::toPCLPointCloud2(*cloud_normals, meshicp.cloud);
			sprintf(&cloud_data[0], "%sicp_trans_cloud%d.obj", data_path.c_str(), data_num);
			pcl::io::saveOBJFile(cloud_data, meshicp);
#endif // save single frame obj

			std::cout << "ICP end " << data_num << endl;
#endif // ICP

#if 0 // GICP method
			std::cout << "GICP begin " << data_num << endl;
			// create the object implementing ICP algorithm
			pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> gicp;

/*
			gicp.setMaxCorrespondenceDistance(0.05);cpptools
			gicp.setTransformationEpsilon(1e-8);
			gicp.setEuclideanFitnessEpsilon(1);
			gicp.setMaximumIterations (50);
			//gicp.setRANSACOutlierRejectionThreshold(1.5);
*/

			gicp.setMaxCorrespondenceDistance(0.01);
			gicp.setTransformationEpsilon(1e-10);
			gicp.setEuclideanFitnessEpsilon(0.0001);
			gicp.setMaximumIterations (500);
			gicp.setRANSACOutlierRejectionThreshold(1.5);

			// set the input point cloud to align
			gicp.setInputCloud(cloud_normals);
			// set the input reference point cloud
			gicp.setInputTarget(cloudwn);

			// compte the point cloud registration
			pcl::PointCloud<pcl::PointXYZRGBNormal> Final;
			gicp.align(Final);

			// print if it the algorithm converged and its fitness score
			std::cout << "has converged:" << gicp.hasConverged()
				<< " score: "
				<< gicp.getFitnessScore() << std::endl;
			// print the output transformation
			std::cout << gicp.getFinalTransformation() << std::endl;
			
			//*cloud_normals = Final;
			//*cloudt = Final;

			
			pcl::transformPointCloud(*cloudt, *cloudt, gicp.getFinalTransformation());

#if 1 //save single frame pcd (removal nan)
			char cloud_data_re[256];
			sprintf(&cloud_data_re[0], "%strans_cloud%d_re.pcd", data_path.c_str(), data_num);
			pcl::io::savePCDFile(cloud_data_re, Final);
#endif //save single frame pcd (removal nan)
			std::cout << "GICP end " << data_num << endl;
#endif // GICP method

#if 0 // ICPNL method
			std::cout << "ICPNL begin " << data_num << endl;
			pcl::IterativeClosestPointNonLinear<PointXYZRGB, PointXYZRGB> icpn;

			/*icpn.setMaxCorrespondenceDistance(0.01);
			icpn.setTransformationEpsilon(1e-8);
			icpn.setEuclideanFitnessEpsilon(0.01);
			icpn.setMaximumIterations (100);
			icpn.setRANSACOutlierRejectionThreshold(1.5);*/
			icpn.setMaxCorrespondenceDistance(0.03);
			icpn.setTransformationEpsilon(1e-10);
			icpn.setEuclideanFitnessEpsilon(0.001);
			icpn.setMaximumIterations (500);
			icpn.setRANSACOutlierRejectionThreshold(1.5);

			// set the input point cloud to align
			icpn.setInputCloud(cloudt);
			// set the input reference point cloud
			icpn.setInputTarget(cloudw);

			// compte the point cloud registration
			pcl::PointCloud<pcl::PointXYZRGB> Final;
			icpn.align(Final);

			// print if it the algorithm converged and its fitness score
			std::cout << "has converged:" << icpn.hasConverged()
				<< " score: "
				<< icpn.getFitnessScore() << std::endl;
			// print the output transformation
			std::cout << icpn.getFinalTransformation() << std::endl;
			
			//*cloud_normals = Final;
			*cloudt = Final;

			
			//pcl::transformPointCloud(*cloudt, *cloudt, gicp.getFinalTransformation());

#if 1 //save single frame pcd (removal nan)
			char cloud_data_re[256];
			sprintf(&cloud_data_re[0], "%strans_cloud%d_re.pcd", data_path.c_str(), data_num);
			pcl::io::savePCDFile(cloud_data_re, Final);
#endif //save single frame pcd (removal nan)
			std::cout << "ICPNL end " << data_num << endl;
#endif // ICPNL method

#endif //registration method

		}
		else
		{
#if 1 // save single frame obj
			pcl::PolygonMesh meshw;
			pcl::toPCLPointCloud2(*cloud_normals, meshw.cloud);
			sprintf(&cloud_data[0], "%sicp_trans_cloud%d.obj", data_path.c_str(), data_num);
			pcl::io::saveOBJFile(cloud_data, meshw);
#endif // save single frame obj	
		}

		*cloudwn += *cloud_normals;
		*cloudw += *cloudt;

#if 0 // save single frame obj
		pcl::PolygonMesh meshw;
		pcl::toPCLPointCloud2(*cloudwn, meshw.cloud);
		sprintf(&cloud_data[0], "%scloud0.obj", data_path.c_str());
		pcl::io::saveOBJFile(cloud_data, meshw);
#endif // save single frame obj



		data_num++;
	}
	else
	{
		data_num = -1;
	}
	return data_num;
}


//void cloudCB(const sensor_msgs::PointCloud2& input)
void cloudCB(const sensor_msgs::ImageConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGBA> cloud; // With color

    //pcl::fromROSMsg(input, cloud); // sensor_msgs::PointCloud2 ----> pcl::PointCloud<T>

    //if(! viewer->wasStopped()) viewer->showCloud(g_cloud.makeShared());
    if(! viewer->wasStopped() && !loadCloud) viewer->showCloud(cloudw->makeShared());

    if(saveCloud)
    {
        stringstream stream;
        stream << "inputCloud"<< filesNum<< ".pcd";
        string filename = stream.str();

        if(io::savePCDFile(filename, cloud, true) == 0)
        {
           //registration method  filesNum++;
            cout << filename<<" Saved."<<endl;
        }
        else PCL_ERROR("Problem saving %s.\n", filename.c_str());

        saveCloud = false;

    }

    if(loadCloud)
    {
		LoadPCDandCombineCloud();
		if (data_num >= 10)
			loadCloud = false;
    }

    if(saveAll)
    {
		ROS_INFO("Save Obj file.");
		char obj_data[256] = "/home/kuku/catkin_ws/src/my_pcl_tutorial/data/finalcloud.obj";
		pcl::PolygonMesh mesh;
		pcl::toPCLPointCloud2(*cloudw, mesh.cloud);
		pcl::io::saveOBJFile(obj_data, mesh);

		char pcd_data[256] = "/home/kuku/catkin_ws/src/my_pcl_tutorial/data/finalcloud.pcd";
		pcl::io::savePCDFile(pcd_data, *cloudw);
		saveAll = false;
		ROS_INFO("end Obj file.");
    }
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
    if((event.getKeySym() == "a" || event.getKeySym() == "A") && event.keyDown())
        saveAll = true;
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

    //ros::Subscriber pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, cloudCB);
    ros::Subscriber pcl_sub = nh.subscribe("usb_cam/image_raw", 1, cloudCB);

    ros::Rate rate(30.0);

    while (ros::ok() && ! viewer->wasStopped())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

