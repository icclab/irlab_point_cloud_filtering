#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "sensor_msgs/PointCloud2.h"
#include <gpd_ros/CloudIndexed.h>
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
#include <pcl/pcl_config.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/conversions.h>
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
    char* pointcloud_topic;
    int tries;
    int messages;
    int max_messages;
    


  public:
  
    ROSController(ros::NodeHandle n, char* pointcloud_topic);
  
  bool trigger_clustering(std_srvs::Trigger::Request  &req,
           std_srvs::Trigger::Response &res)
  {
    ROS_INFO("Received point cloud clustering request");
    this->sub = this->n.subscribe(this->pointcloud_topic, 1, &ROSController::get_filtered_pointcloud, this);
    res.success = true;
    res.message = "Service triggered";
    ROS_INFO("sending back response: OK");
    return true;
  }


  void get_filtered_pointcloud(sensor_msgs::PointCloud2 req)
{
  CloudType::Ptr filtered_cloud	(new CloudType);
  pcl::fromROSMsg(req, *filtered_cloud);
	ROS_INFO("Loaded pointcloud with %lu points.", filtered_cloud->size());
	CloudType::Ptr plane_cloud_ptr		(new CloudType);

	
	//Creating the kdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(filtered_cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ> ());

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

  ss << "/home/ros/catkin_ws/src/cloud_cluster_" << j << ".pcd";
  writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
  j++;

        //Merge current clusters to whole point cloud
    *clustered_cloud += *cloud_cluster;
    }

    pcl::toROSMsg (*object_cloud_to_publish, *res_cloud);
    res_cloud->header.frame_id = req.header.frame_id;
    res_cloud->header.stamp = ros::Time::now();
    res_cloud->height = 1;


    //Can we just send the entire pointcloud including multiple objects??
	ROS_INFO("cluster_indices size: %lu", cluster_indices.size());
	if (cluster_indices.size() == 0){
	  ROS_ERROR("Cannot find objects after clustering");

	}	

  gpd_ros::CloudIndexed res;
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

  this->pub.publish(res);
  this->pub_pc.publish(*res_cloud);
  ROS_INFO("Published indexed pointCloud");

  ROS_INFO("Unsubscribing from topic");
  this->sub.shutdown();

}
  
};

ROSController::ROSController(ros::NodeHandle n, char* pointcloud_topic) {
  this->n = n;
  this->service = n.advertiseService("cluster_pointcloud", &ROSController::trigger_clustering, this);
  this->pub = n.advertise<gpd_ros::CloudIndexed>("/cloud_indexed", 100);
  this->pub_pc = n.advertise<sensor_msgs::PointCloud2>("/cloud_indexed_pc_only", 100);
  this->pointcloud_topic = pointcloud_topic;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_filtering_server");
  char* pointcloud_topic = "/extract_plane_indices/output";
  if (argc == 2)
  {
    pointcloud_topic = argv[1];
  }
  ROS_INFO("Will register to pointcloud2 topic: %s", pointcloud_topic);
  ros::NodeHandle n;
  ROSController* c = new ROSController(n, pointcloud_topic);
  ROS_INFO("Ready to filter point clouds.");
  ros::spin();
  return 0;
}

