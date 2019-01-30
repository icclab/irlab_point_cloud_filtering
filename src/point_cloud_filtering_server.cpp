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

class ROSController {
  protected:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub_pc;
    ros::ServiceServer service;
    sensor_msgs::PointCloud2 raw_cloud;
    sensor_msgs::PointCloud2 raw_cloud_temp;
   // sensor_msgs::PointCloud2 res_cloud; //cambio
    char* pointcloud_topic;
    int tries;
    int messages;
    int max_messages = 4;

  public:
  
  bool trigger_filter(std_srvs::Trigger::Request  &req,
           std_srvs::Trigger::Response &res)
  {
    ROS_INFO("Received point cloud filtering request");
    tries = 0;
    messages =0;
    this->sub = this->n.subscribe(this->pointcloud_topic, 1, &ROSController::filter_pointcloud, this);
    res.success = true;
    res.message = "Service triggered";
    ROS_INFO("sending back response: OK");
    return true;
  }

  void filter_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    ROS_INFO("Point cloud message received" );
    if (messages<max_messages)
    {
        raw_cloud_temp = *msg;
        if (messages>1)
        {
            bool result;
            result = pcl::concatenatePointCloud(raw_cloud, raw_cloud_temp, raw_cloud);
            if (result)
            {
                messages++;
            }
            else
            {
                ROS_INFO("Pointcloud dropped as concatenation failed");
            }
        }
        else
        {
            messages++;
            raw_cloud=raw_cloud_temp;
        }
    }
    else
    {
        tries++;
        if ( this->filtering(raw_cloud) || tries > 3){
           this->sub.shutdown();
         //  this->meshing_poisson_reconstruct(res_cloud); //commented, but working!
        //   this->filtering_cylindric(raw_cloud); 
        }
    }

  }

  ROSController(ros::NodeHandle n, char* pointcloud_topic);

  bool meshing_poisson_reconstruct(sensor_msgs::PointCloud2 res_cloud_meshing)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(res_cloud_meshing,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    //do stuff with temp_cloud here
    pcl::io::savePCDFile("/home/milt/catkin_ws_kinetic/src/gpd/scripts/original_pcd.pcd", *temp_cloud);
    std::cerr << "Saved " << temp_cloud->points.size() << " data points to original_pcd.pcd." << std::endl;

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud (temp_cloud);
    mls.setSearchRadius (0.01);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius (0.005);
    mls.setUpsamplingStepSize (0.003);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
    mls.process (*cloud_smoothed);

    pcl::io::savePCDFile ("/home/milt/catkin_ws_kinetic/src/gpd/scripts/smoothed-mls.pcd", *cloud_smoothed);
    std::cerr << "Saved " << cloud_smoothed->points.size() << " data points to smoothed-mls.pcd." << std::endl;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads (8);
    ne.setInputCloud (cloud_smoothed);
    ne.setRadiusSearch (0.01);
    Eigen::Vector4f centroid;
    compute3DCentroid (*cloud_smoothed, centroid);
    ne.setViewPoint (centroid[0], centroid[1], centroid[2]);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    ne.compute (*cloud_normals);

    for (size_t i = 0; i < cloud_normals->size (); ++i)
    {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
    concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth (9);
    poisson.setInputCloud(cloud_smoothed_normals);
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);
    //pcl::io::saveOBJFile("/home/milt/catkin_ws_kinetic/src/gpd/scripts/polygonmesh.ply", mesh);
    pcl::io::savePolygonFileSTL("/home/milt/catkin_ws_kinetic/src/gpd/scripts/polygonmesh.stl", mesh);
    std::cerr << "Saved polygon mesh after Poisson reconstruction to polygonmesh.stl." << std::endl;

    return true;

}


    bool filtering_cylindric(sensor_msgs::PointCloud2 req)
{
    sensor_msgs::PointCloud2::Ptr res_cloud (new sensor_msgs::PointCloud2);
    ROS_INFO("The number of pointcloud messages received is: %d ", messages);
	CloudType::Ptr filtered_cloud	(new CloudType);
	CloudType::Ptr plane_cloud		(new CloudType);
	pcl::fromROSMsg(req, *filtered_cloud);
	ROS_INFO("Loaded pointcloud with %lu points.", filtered_cloud->size());

	// Build a passthrough filter to remove spurious NaNs
	pcl::PassThrough<PointType> pass;
	pass.setInputCloud(filtered_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*filtered_cloud);
	ROS_INFO("Pointcloud after max range filtering has %lu points.", filtered_cloud->size());

/*	For downsampling uncomment here
    pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud(filtered_cloud);
	sor.setLeafSize(0.001f, 0.001f, 0.001f);
	sor.filter(*filtered_cloud);
	ROS_INFO("Downsampled pointcloud has %lu points.", filtered_cloud->size());*/

/*  For outlier filtering uncomment here
    pcl::StatisticalOutlierRemoval<PointType> stat_fil;
	stat_fil.setInputCloud(filtered_cloud);
	stat_fil.setMeanK(50);
	stat_fil.setStddevMulThresh(1.0);
	stat_fil.filter(*filtered_cloud);
	ROS_INFO("Pointcloud after outlier filtering has %lu points.", filtered_cloud->size());*/

	// Estimate point normals
	pcl::NormalEstimation<PointType, pcl::Normal> ne;
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setSearchMethod (tree);
    ne.setInputCloud (filtered_cloud);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

      // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (filtered_cloud);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

      // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (filtered_cloud);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

      // Write the planar inliers to disk
    pcl::PCDWriter writer;
    pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType> ());
    extract.filter (*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    writer.write ("/home/milt/catkin_ws_kinetic/src/gpd/scripts/table_scene_stereo_textured_plane.pcd", *cloud_plane, false);

      // Remove the planar inliers, extract the rest
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::PointCloud<PointType>::Ptr cloud_filtered2 (new pcl::PointCloud<PointType>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

      // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);
    seg.setRadiusLimits (0, 0.01);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

      // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointType>::Ptr cloud_cylinder (new pcl::PointCloud<PointType> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ())
    {
        std::cerr << "Can't find the cylindrical component." << std::endl;
        return false;
    }
    else
    {
	    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
	    writer.write ("/home/milt/catkin_ws_kinetic/src/gpd/scripts/scene_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }

    //until here from http://pointclouds.org/documentation/tutorials/cylinder_egmentation.php + voxel and outlier filtering

    //next smoothing out the pointcloud
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud (cloud_cylinder);
    mls.setSearchRadius (0.01);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius (0.005);
    mls.setUpsamplingStepSize (0.003);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
    mls.process (*cloud_cylinder_smoothed);

    pcl::io::savePCDFile ("/home/milt/catkin_ws_kinetic/src/gpd/scripts/smoothed-cylindric-mls.pcd", *cloud_cylinder_smoothed);
    std::cerr << "Saved " << cloud_cylinder_smoothed->points.size() << " data points to smoothed-cylindric-mls.pcd." << std::endl;


	//Creating the kdTree object for the search method of the extraction
	pcl::search::KdTree<PointType>::Ptr tree_two(new pcl::search::KdTree<PointType>);
	tree_two->setInputCloud(cloud_cylinder_smoothed);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointType> ec;
	ec.setClusterTolerance(0.02);
	ec.setMinClusterSize(30);
	ec.setMaxClusterSize(20000);
	ec.setSearchMethod(tree_two);
	ec.setInputCloud(cloud_cylinder_smoothed);
	ec.extract(cluster_indices);

	res_cloud->header.frame_id = req.header.frame_id;
	res_cloud->header.stamp = ros::Time::now();
	res_cloud->height = 1;
	//Can we just send the entire pointcloud including multiple objects??
	ROS_INFO("cluster_indices size: %lu", cluster_indices.size());
	if (cluster_indices.size() == 0){
	  ROS_ERROR("Cannot find objects after clustering");
	  return false;
	}
	res_cloud->width = cluster_indices[0].indices.size();
	res_cloud->is_bigendian = false;
	res_cloud->is_dense = true;

	sensor_msgs::PointCloud2Modifier modifier(*res_cloud);
	modifier.setPointCloud2FieldsByString(1, "xyz");
	modifier.resize(cluster_indices[0].indices.size());

	sensor_msgs::PointCloud2Iterator<float> out_x(*res_cloud, "x");
	sensor_msgs::PointCloud2Iterator<float> out_y(*res_cloud, "y");
	sensor_msgs::PointCloud2Iterator<float> out_z(*res_cloud, "z");

	pcl::toROSMsg(*cloud_cylinder_smoothed, *res_cloud);
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
    this->pub_pc.publish(res.cloud_sources.cloud);
    ROS_INFO("Published indexed pointCloud");

    return true;

}
  bool filtering(sensor_msgs::PointCloud2 req)
{

    ROS_INFO("The number of pointcloud messages received is: %d ", messages);
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

ROSController::ROSController(ros::NodeHandle n, char* pointcloud_topic) {
  this->n = n;
  this->service = n.advertiseService("filter_pointcloud", &ROSController::trigger_filter, this);
//  pub = rospy.Publisher('cloud_indexed', CloudIndexed, queue_size=1, latch=True) 
  this->pub = n.advertise<gpd::CloudIndexed>("/cloud_indexed", 100, true);
  this->pub_pc = n.advertise<sensor_msgs::PointCloud2>("/cloud_indexed_pc_only", 100, true);
  this->pointcloud_topic = pointcloud_topic;
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
  ros::NodeHandle n;
//  ros::ServiceServer service = n.advertiseService("filter_pointcloud", trigger_filter);
//  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1000, filter_pointcloud);

//  ros::spin();
  ROSController* c = new ROSController(n, pointcloud_topic);
  ROS_INFO("Ready to filter point clouds.");
  ros::spin();
  return 0;
}

