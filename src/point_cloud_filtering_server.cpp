#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "sensor_msgs/PointCloud2.h"
#include "gpd/CloudIndexed.h"
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "typedefs.h"
//using namespace sensor_msgs;


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

class ROSController {
  protected:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Subscriber sub_2;
    ros::Publisher pub;
    ros::Publisher pub_pc;

    ros::Publisher pub_3;
    tf::TransformListener *listener;
    tf::TransformListener *listener2;
    sensor_msgs::PointCloud2 output, output1, output2, input2_filtered;
    pcl::PointCloud<pcl::PointXYZ> output_pcl, output1_pcl, output_pcl_NaNs, output2_pcl;


    ros::ServiceServer service;
    sensor_msgs::PointCloud2 raw_cloud;
    sensor_msgs::PointCloud2 raw_cloud_temp;
    char* pointcloud_topic;
    char* pointcloud_topic_2;
    int tries;
    int messages;
    int max_messages = 10;

    std::vector<int> indicies;

  public:
  
  bool trigger_filter(std_srvs::Trigger::Request  &req,
           std_srvs::Trigger::Response &res)
  {
    ROS_INFO("Received point cloud filtering request");
    tries = 0;
    messages =0;
    this->sub = this->n.subscribe(this->pointcloud_topic, 1, &ROSController::filter_pointcloud, this);
    this->sub_2 = this->n.subscribe(this->pointcloud_topic_2, 1, &ROSController::filter_pointcloud_2, this);
    res.success = true;
    res.message = "Service triggered";
    ROS_INFO("sending back response: OK");
    return true;
  }

  void filter_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& input1)
  {


    if (messages<max_messages)
    {
        ROS_INFO("Point cloud message received from ARM CAMERA" );
        ROS_INFO_STREAM((*input1).header.frame_id);
        pcl::fromROSMsg(*input1, output1_pcl);

        output_pcl = output1_pcl;
        output_pcl += output2_pcl;

        pcl::removeNaNFromPointCloud(output_pcl, output_pcl_NaNs, indicies);
        pcl::toROSMsg(output_pcl_NaNs, output);
        pub_3.publish(output);
        messages++;
        raw_cloud=output;
    ROS_INFO("The number of pointcloud messages received is: %d ", messages);
    }
    else
    {
        tries++;
        if ( this->filtering(raw_cloud) || tries > 3){
           this->sub.shutdown();
           this->sub_2.shutdown();
        }
    }
  }


 void filter_pointcloud_2(const sensor_msgs::PointCloud2::ConstPtr& input2)
  {

    if (messages<max_messages)
    {
        ROS_INFO("Point cloud message received from FRONT CAMERA" );
        ROS_INFO_STREAM((*input2).header.frame_id);


        CloudType::Ptr cloud2filter	(new CloudType);
        CloudType::Ptr filtered_cloud2	(new CloudType);
        pcl::fromROSMsg(*input2, *cloud2filter);

	    pcl::PassThrough<PointType> pass2;
	    pass2.setInputCloud(cloud2filter);
	    pass2.setFilterFieldName("z");
	    pass2.setFilterLimits(0.0, 1.5);
	    pass2.filter(*filtered_cloud2);
	    pcl::removeNaNFromPointCloud(*filtered_cloud2, output_pcl_NaNs, indicies);


        listener2->waitForTransform("/arm_camera_depth_optical_frame", (*input2).header.frame_id, (*input2).header.stamp, ros::Duration(5.0));
        pcl_ros::transformPointCloud("/arm_camera_depth_optical_frame", output_pcl_NaNs, output2_pcl, *listener2);

        output_pcl = output2_pcl;
        output_pcl += output1_pcl;
        pcl::toROSMsg(output_pcl, output);
        pub_3.publish(output);
        messages++;
        raw_cloud=output;
    ROS_INFO("The number of pointcloud messages received is: %d ", messages);
    }
    else
    {
        tries++;
        if ( this->filtering(raw_cloud) || tries > 3){
           this->sub.shutdown();
           this->sub_2.shutdown();
        }
    }


  }

  ROSController(ros::NodeHandle n, char* pointcloud_topic, char* pointcloud_topic_2);
  //ROSController(ros::NodeHandle n, char* pointcloud_topic);

  bool filtering(sensor_msgs::PointCloud2 req)
{

    ROS_INFO("The number of pointcloud messages received is: %d ", messages);
	tries = 0;
    messages=0;
	CloudType::Ptr filtered_cloud	(new CloudType);
	CloudType::Ptr plane_cloud		(new CloudType);

    pcl::fromROSMsg(req, *filtered_cloud);
	ROS_INFO("Loaded pointcloud with %lu points.", filtered_cloud->size());

	pcl::PassThrough<PointType> pass;
	pass.setInputCloud(filtered_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*filtered_cloud);
	ROS_INFO("Pointcloud after max range filtering has %lu points.", filtered_cloud->size());

/*	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud(filtered_cloud);
	sor.setLeafSize(0.001f, 0.001f, 0.001f);
	sor.filter(*filtered_cloud);
	ROS_INFO("Downsampled pointcloud has %lu points.", filtered_cloud->size());*/

	pcl::StatisticalOutlierRemoval<PointType> stat_fil;
	stat_fil.setInputCloud(filtered_cloud);
	stat_fil.setMeanK(50);
	stat_fil.setStddevMulThresh(1.0);
	stat_fil.filter(*filtered_cloud);
	ROS_INFO("Pointcloud after outlier filtering has %lu points.", filtered_cloud->size());

	//Followed this tutorial http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
	pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::PointIndices::Ptr outliers(new pcl::PointIndices);
	pcl::SACSegmentation<PointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setMaxIterations(1000);


	
	int i = 0, nr_points = (int) filtered_cloud->size();
//	while (filtered_cloud->size() > 0.3 * nr_points)
//	{
		seg.setInputCloud(filtered_cloud);
		seg.segment(*inliers, *coeffs);
		if (inliers->indices.size() == 0)
		{
			ROS_WARN("Could not estimate a planar model for the given dataset.");
//			break;
		}

		//Extract the planr inliers from the input cloud
		pcl::ExtractIndices<PointType> extract;
		extract.setInputCloud(filtered_cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		//Get the points associated with the planar surface
		extract.filter(*plane_cloud);
		ROS_INFO("PointCloud representing the planar component: %lu data points.", plane_cloud->size());

		//Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*filtered_cloud);
//	}
	
	//Creating the kdTree object for the search method of the extraction
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
	tree->setInputCloud(filtered_cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointType> ec;
	ec.setClusterTolerance(0.02);
	ec.setMinClusterSize(20);
	ec.setMaxClusterSize(20000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(filtered_cloud);
	ec.extract(cluster_indices);



/* To separate each cluster out of the vector<PointIndices> we have to
 * iterate through cluster_indices, create a new PointCloud for each
 * entry and write all points of the current cluster in the PointCloud.
 */
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_to_publish (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2::Ptr res_cloud (new sensor_msgs::PointCloud2);
    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    int j=0;
    pcl::PCDWriter writer;
    //Save the clusters (object clouds) into pcd files
    for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

        for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
        //push_back: add a point to the end of the existing vector
                cloud_cluster->points.push_back(filtered_cloud->points[*pit]);

                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                if (it == cluster_indices.begin())
                     *object_cloud_to_publish = *cloud_cluster;

        }

    std::stringstream ss;

    ss << "/home/milt/catkin_ws_kinetic/src/gpd/scripts/cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;

        //Merge current clusters to whole point cloud
    *clustered_cloud += *cloud_cluster;
    }
    std::stringstream ss_2;
    ss_2 << "/home/milt/catkin_ws_kinetic/src/gpd/scripts/merged_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss_2.str (), *clustered_cloud, false);



    //pcl::toROSMsg (*clustered_cloud , *res_cloud);
    pcl::toROSMsg (*object_cloud_to_publish, *res_cloud);
    res_cloud->header.frame_id = req.header.frame_id;
    res_cloud->header.stamp = ros::Time::now();
    res_cloud->height = 1;
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



    //Can we just send the entire pointcloud including multiple objects??
	ROS_INFO("cluster_indices size: %lu", cluster_indices.size());
	if (cluster_indices.size() == 0){
	  ROS_ERROR("Cannot find objects after clustering");
	  return false;
	}	
/*	res_cloud->width = cluster_indices[0].indices.size();
	res_cloud->is_bigendian = false;
	res_cloud->is_dense = true;

	sensor_msgs::PointCloud2Modifier modifier(*res_cloud);
	modifier.setPointCloud2FieldsByString(1, "xyz");
	modifier.resize(cluster_indices[0].indices.size());

	sensor_msgs::PointCloud2Iterator<float> out_x(*res_cloud, "x");
	sensor_msgs::PointCloud2Iterator<float> out_y(*res_cloud, "y");
	sensor_msgs::PointCloud2Iterator<float> out_z(*res_cloud, "z");*/

/*	for (std::vector<pcl::PointIndices>::iterator it; it != cluster_indices.end(); ++it) {
	 	for (int i = 0; i < it->indices.size(); ++i) {
	 		*out_x = it->indices[i].x; 	++out_x;
	 		*out_y = it->indices[i].y;	++out_y;
	 		*out_z = it->indices[i].z;	++out_z;
	 	}
	 }*/
/*	for (int i = 0; i < cluster_indices[0].indices.size(); ++i) {
		p = filtered_cloud->points[cluster_indices[0].indices[i]];
		*out_x = p.x; ++out_x;
		*out_y = p.y; ++out_y;
		*out_z = p.z; ++out_z;
	}*/

  gpd::CloudIndexed res;
	res.cloud_sources.cloud = *res_cloud;
	std_msgs::Int64 val;
	val.data = 0;
	std::vector<geometry_msgs::Point> vec;
	vec.push_back(geometry_msgs::Point());
	res.cloud_sources.view_points = vec;
	for (int i = 0; i < cluster_indices[0].indices.size(); ++i) {
	  std_msgs::Int64 index;
	  index.data = cluster_indices[0].indices[i];
		res.indices.push_back(index);
		res.cloud_sources.camera_source.push_back(val);
	}

  ROS_INFO("Publishing indexed pointCloud"); 
  this->pub.publish(res);
 // this->pub_pc.publish(res.cloud_sources.cloud);
  this->pub_pc.publish(*res_cloud);
  ROS_INFO("Published indexed pointCloud");
  return true;
}
  
};

ROSController::ROSController(ros::NodeHandle n, char* pointcloud_topic, char* pointcloud_topic_2) {
//ROSController::ROSController(ros::NodeHandle n, char* pointcloud_topic) {
  this->n = n;
  this->service = n.advertiseService("filter_pointcloud", &ROSController::trigger_filter, this);
//  pub = rospy.Publisher('cloud_indexed', CloudIndexed, queue_size=1, latch=True) 
  this->pub = n.advertise<gpd::CloudIndexed>("/cloud_indexed", 100, false);
  this->pub_pc = n.advertise<sensor_msgs::PointCloud2>("/cloud_indexed_pc_only", 100, false);

  this->pub_3 = n.advertise<sensor_msgs::PointCloud2>("/cloud_merged", 100, false);
  this->listener = new tf::TransformListener(ros::Duration(10));
  this->listener2 = new tf::TransformListener(ros::Duration(10));
  this->pointcloud_topic = pointcloud_topic;
  this->pointcloud_topic_2 = pointcloud_topic_2;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_filtering_server");
  char* pointcloud_topic = "/summit_xl/arm_camera/depth_registered/points";
  if (argc == 2)
  {
    pointcloud_topic = argv[1];
  }
  ROS_INFO("Will register to pointcloud2 topic: %s", pointcloud_topic);


char* pointcloud_topic_2 = "/summit_xl/front_rgbd_camera/depth/points"; //on robot it is depth_registered
  if (argc == 3)
  {
    pointcloud_topic_2 = argv[2];
  }
  ROS_INFO("Will register to pointcloud2 topic: %s", pointcloud_topic_2);

  ros::NodeHandle n;
//  ros::ServiceServer service = n.advertiseService("filter_pointcloud", trigger_filter);
//  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1000, filter_pointcloud);

//  ros::spin();

  ROSController* c = new ROSController(n, pointcloud_topic, pointcloud_topic_2);
  //ROSController* c = new ROSController(n, pointcloud_topic);
  ROS_INFO("Ready to filter point clouds.");
  ros::spin();
  return 0;
}

