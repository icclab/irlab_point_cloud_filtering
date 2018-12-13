#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "sensor_msgs/PointCloud2.h"
#include "gpd/CloudIndexed.h"
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iterator>
//#include "gpd/filter_pointcloud.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "typedefs.h"
//using namespace sensor_msgs;



char* camera_depth_optical_frame = "arm_camera_depth_optical_frame";

class ROSController {
  protected:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::ServiceServer service;
  public:
  
  bool trigger_filter(std_srvs::Trigger::Request  &req,
           std_srvs::Trigger::Response &res)
  {
    ROS_INFO("request: %s", req);
    this->sub = this->n.subscribe("/summit_xl/arm_camera/depth/points", 1000, &ROSController::filter_pointcloud, this);
    res.success = true;
    res.message = "Service triggered";
    ROS_INFO("sending back response: [%s]", res.message);
    return true;
  }

  void filter_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    ROS_INFO("Point cloud message received" );
    this->filtering(*msg); 
    delete this->sub;
  }

  ROSController(ros::NodeHandle n);
  
  bool filtering(sensor_msgs::PointCloud2 req) //, //PointCloud2 msg
//			   std_msgs::Empty::Request &res) //CloudIndexed msg
{
	CloudType::Ptr filtered_cloud	(new CloudType);
	CloudType::Ptr plane_cloud		(new CloudType);
	

	PointType p;
//	for (int i = 0; i < req.width; i+=3) {
//		p.x = static_cast<float>(req.data[i]);
//		p.y = static_cast<float>(req.data[i+1]);
//		p.z = static_cast<float>(req.data[i+2]);
//		filtered_cloud->push_back(p);
//	}
  pcl::fromROSMsg(req, *filtered_cloud);
	std::cout << "Loaded pointcloud with " << filtered_cloud->size() << " points." << std::endl;

	pcl::PassThrough<PointType> pass;
	pass.setInputCloud(filtered_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*filtered_cloud);
	std::cout << "Pointcloud after max range filtering has " << filtered_cloud->size()
			  << " points." << std::endl;

	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud(filtered_cloud);
	sor.setLeafSize(0.005f, 0.005f, 0.005f);
	sor.filter(*filtered_cloud);
	std::cout << "Downsampled pointcloud has " << filtered_cloud->size()
			  << " points." << std::endl;

	pcl::StatisticalOutlierRemoval<PointType> stat_fil;
	stat_fil.setInputCloud(filtered_cloud);
	stat_fil.setMeanK(50);
	stat_fil.setStddevMulThresh(1.0);
	stat_fil.filter(*filtered_cloud);
	std::cout << "Pointcloud after outlier filtering has " << filtered_cloud->size()
			  << " points." << std::endl;

	//Followed this tutorial http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
	pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::PointIndices::Ptr outliers(new pcl::PointIndices);
	pcl::SACSegmentation<PointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setMaxIterations(100);
	
	int i = 0, nr_points = (int) filtered_cloud->size();
	while (filtered_cloud->size() > 0.3 * nr_points)
	{
		seg.setInputCloud(filtered_cloud);
		seg.segment(*inliers, *coeffs);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		//Extract the planr inliers from the input cloud
		pcl::ExtractIndices<PointType> extract;
		extract.setInputCloud(filtered_cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		//Get the points associated with the planar surface
		extract.filter(*plane_cloud);
		std::cout << "PointCloud representing the planar component: " << plane_cloud->size()
				  << " data points." << std::endl;

		//Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*filtered_cloud);
	}
	
	//Creating the kdTree object for the search method of the extraction
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
	tree->setInputCloud(filtered_cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointType> ec;
	ec.setClusterTolerance(0.02);
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(2500);
	ec.setSearchMethod(tree);
	ec.setInputCloud(filtered_cloud);
	ec.extract(cluster_indices);

	//Save the clusters (object clouds) into pcd files
	// int j = 0;
	// for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
	// 	 it != cluster_indices.end(); ++it)
	// {
	// 	pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);
	// 	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	// 		cloud_cluster->points.push_back(filtered_cloud->points[*pit]);
	// 	cloud_cluster->width = cloud_cluster->points.size();
	// 	cloud_cluster->height = 1;
	// 	cloud_cluster->is_dense = true;

	// 	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() 
	// 			  << " data points." << std::endl;

	// 	std::stringstream ss;
	// 	ss << "cloud_cluster_" << j << ".pcd";
	// 	writer.write<PointType> (ss.str(), *cloud_cluster, false);
	// 	j++;
	// }

	// std::vector<int> range(filtered_cloud->size());
	// std::iota(range.begin(), range.end(), 0);
	// std::set_difference(range.begin(), range.end(),
	// 					inliers->indices.begin(), inliers->indices.end(),
	// 					std::inserter(outliers->indices, outliers->indices.begin()));

	// for (int i = 0; i < inliers->indices.size(); ++i)
	// 	planes_cloud->push_back(filtered_cloud->points[inliers->indices[i]]);
	// for (int i = 0 ; i < outliers->indices.size(); ++i)
	// 	objects_cloud->push_back(filtered_cloud->points[outliers->indices[i]]);

	//std::cout << "Objects cloud has: " << objects_cloud.size() << " points." << std:endl;

	//int numpoints = std::accumulate(cluster_indices.begin(), cluster_indices.end(),
	//				 				0, [](int a, pcl::PointIndices b) {return a + (int)b.indices.size();})

	sensor_msgs::PointCloud2 res_cloud;
	res_cloud.header.frame_id = camera_depth_optical_frame;
	res_cloud.header.stamp = ros::Time::now();
	res_cloud.height = 1;
	//Can we just send the entire pointcloud including multiple objects??
	res_cloud.width = cluster_indices[0].indices.size();
	res_cloud.is_bigendian = false;
	res_cloud.is_dense = true;

	sensor_msgs::PointCloud2Modifier modifier(res_cloud);
	modifier.setPointCloud2FieldsByString(1, "xyz");
	modifier.resize(cluster_indices[0].indices.size());

	sensor_msgs::PointCloud2Iterator<float> out_x(res_cloud, "x");
	sensor_msgs::PointCloud2Iterator<float> out_y(res_cloud, "y");
	sensor_msgs::PointCloud2Iterator<float> out_z(res_cloud, "z");

	// for (std::vector<pcl::PointIndices>::iterator it; it != cluster_indices.end(); ++it) {
	// 	for (int i = 0; i < it->indices.size(); ++i) {
	// 		*out_x = it->indices[i].x; 	++out_x;
	// 		*out_y = it->indices[i].y;	++out_y;
	// 		*out_z = it->indices[i].z;	++out_z;
	// 	}
	// }
	for (int i = 0; i < cluster_indices[0].indices.size(); ++i) {
		p = filtered_cloud->points[cluster_indices[0].indices[i]];
		*out_x = p.x; ++out_x;
		*out_y = p.y; ++out_y;
		*out_z = p.z; ++out_z;
	}

	//Make a single vector with the indices to pass into the cloudindexed msg
	// std::vector<int> indices_acc;
	// for (int i = 0; i < cluster_indices.size(); ++i) {
	// 	indices_acc.insert(indices_acc.end(), cluster_indices[i].indices.begin(), cluster_indices[i].indices.end());
	// }

  gpd::CloudIndexed res = *(new gpd::CloudIndexed);
	res.cloud_sources.cloud = res_cloud;
//	res.indices = indices_acc;
//	for (int i = 0; i < cluster_indices[0].indices.size(); ++i) {
//		res.indices.push_back(cluster_indices[0].indices[i]);
//	}

  this->pub.publish(res);
  
}
  
};

ROSController::ROSController(ros::NodeHandle n) {
  this->n = n;
  this->service = n.advertiseService("filter_pointcloud", &ROSController::trigger_filter, this);
//  pub = rospy.Publisher('cloud_indexed', CloudIndexed, queue_size=1, latch=True) 
  this->pub = n.advertise<gpd::CloudIndexed>("/cloud_indexed", 10, true); 
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_filtering_server");
  ros::NodeHandle n;
//  ros::ServiceServer service = n.advertiseService("filter_pointcloud", trigger_filter);
//  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1000, filter_pointcloud);

//  ros::spin();
  ROSController* c = new ROSController(n);
  ROS_INFO("Ready to filter point clouds.");
  ros::spin();
  return 0;
}

