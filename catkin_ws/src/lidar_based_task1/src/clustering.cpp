#include <ros/ros.h>
#include <cmath>        // std::abs
#include <limits>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>


#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <lidar_based_task1/ObstaclePose.h>
#include <lidar_based_task1/ObstaclePoseList.h>


//define point cloud type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

//declare point cloud
PointCloudXYZ::Ptr cloud_inXYZ (new PointCloudXYZ);
PointCloudXYZRGB::Ptr cloud_in (new PointCloudXYZRGB); 
PointCloudXYZRGB::Ptr cloud_filtered (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_f (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_plane (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);
sensor_msgs::PointCloud2 ros_out;

//declare ROS publisher
ros::Publisher pub_result;
ros::Publisher pub_marker;
ros::Publisher pub_obstacle;

//declare global variable
bool lock = false;

//declare function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&); //point cloud subscriber call back function
void cluster_pointcloud(void); //point cloud clustering
int point_cloud_color(int input); //set point cloud RGB color
void drawRviz(lidar_based_task1::ObstaclePoseList); //draw marker in Rviz


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!lock){
    lock = true;
    //covert from ros type to pcl type
    pcl::fromROSMsg (*input, *cloud_inXYZ);
    copyPointCloud(*cloud_inXYZ, *cloud_in);

    //set color for point cloud
    for (size_t i = 0; i < cloud_in->points.size(); i++){
      cloud_in->points[i].r = 0;
      cloud_in->points[i].g = 0;
      cloud_in->points[i].b = 0;

      const float bad_point = std::numeric_limits<float>::quiet_NaN();
      float distance = sqrt(cloud_in->points[i].x*cloud_in->points[i].x + cloud_in->points[i].y*cloud_in->points[i].y + cloud_in->points[i].z*cloud_in->points[i].z);
      if (distance>=10){
        cloud_in->points[i].x = bad_point;
        cloud_in->points[i].y = bad_point;
        cloud_in->points[i].z = bad_point;
      }
    }
    //point cloud clustering
    cluster_pointcloud();
  }
  else{
    std::cout << "lock" << std::endl;
  }
}


//void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
void cluster_pointcloud()
{
  //========== Remove NaN point ==========
  std::vector<int> indices;
  for (int j=0;j<2;j++){
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
  }

  //========== Downsample ==========
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud_in);
  vg.setLeafSize (0.01f, 0.01f, 0.01f); //unit:cetimeter
  vg.filter (*cloud_filtered);
  std::cout << "Filtering successfully" << std::endl;
  
  /*
  //========== Planar filter ==========
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.06);
  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.2 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }
  */
  
  //========== Point Cloud Clustering ==========

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  // Create cluster object
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (1); // unit: meter
  ec.setMinClusterSize (1.5);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int num_cluster = 0; // number of cluster
  int set_r=0, set_g=0, set_b=0; // declare cluster point cloud color
  int start_index = 0;
  lidar_based_task1::ObstaclePoseList ob_list;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    num_cluster++;
    lidar_based_task1::ObstaclePose ob_pose;
    Eigen::Vector4f centroid;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      result->points.push_back(cloud_filtered->points[*pit]);
    }
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    //std::cout << centroid << std::endl;
    ob_pose.header.stamp = ros::Time::now();
    ob_pose.header.frame_id = cloud_cluster->header.frame_id;
    ob_pose.x = centroid[0];
    ob_pose.y = centroid[1];
    ob_pose.z = centroid[2];
    ob_pose.r = 1;

    set_r = point_cloud_color(int(255 - std::abs(30*centroid[1])));
    set_g = point_cloud_color(int(std::abs(30*centroid[1])));
    set_b = 0;

    // give every cluster a color
    for (int i = start_index; i < result->points.size(); ++i)
    {
      result->points[i].r = set_r;
      result->points[i].g = set_g;
      result->points[i].b = set_b;
    }
    if ((ob_pose.x-2)<0.2 and (std::abs(ob_pose.y)-1)<0.01){
      continue;
    }
    else{
      ob_list.list.push_back(ob_pose);
      start_index = result->points.size();
    }
  }

  //set obstacle list
  ob_list.header.stamp = ros::Time::now();
  ob_list.header.frame_id = cloud_in->header.frame_id;
  ob_list.size = num_cluster - 2;
  pub_obstacle.publish(ob_list);
  drawRviz(ob_list);

  result->header.frame_id = cloud_in->header.frame_id;
  //writer.write<pcl::PointXYZRGB> ("result.pcd", *cloud_filtered, false);
  std::cout << "Finish" << std::endl << std::endl;
  pcl::toROSMsg(*result, ros_out);
  pub_result.publish(ros_out);
  lock = false;
  result->clear();
}


int point_cloud_color(int input)
{
  if(input < 0){
    return 0;
  }
  else if(input > 255){
    return 255;
  }
  else{
    return input;
  }
}


void drawRviz(lidar_based_task1::ObstaclePoseList ob_list)
{
      visualization_msgs::Marker  marker;
      marker.header.frame_id = "velodyne";
      marker.header.stamp = ros::Time::now();
      marker.type = marker.SPHERE_LIST;
      marker.action = marker.ADD;
      marker.pose.orientation.w = 1;
      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.4;
      // set marker color
      std_msgs::ColorRGBA c;
      for (int i = 0; i < ob_list.size;i++)
      {
        float dis = sqrt(ob_list.list[i].x*ob_list.list[i].x+ob_list.list[i].y*ob_list.list[i].y);
        if (dis <= 3.0)
        {
          c.r = 1.0;
          c.g = 0.0;
          c.a = 1.0;
        }
        else
        {
          c.r = 0.0;
          c.g = 1.0;
          c.a = 1.0;
        }
        geometry_msgs::Point p;
        p.x = ob_list.list[i].x;
        p.y = ob_list.list[i].y;
        p.z = ob_list.list[i].z;
        marker.colors.push_back(c);  
        marker.points.push_back(p);
      }
      pub_marker.publish(marker);
}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_extraction");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);
  
  // Create a ROS publisher for the output point cloud
  pub_obstacle = nh.advertise< lidar_based_task1::ObstaclePoseList > ("/obstacle_list", 1);
  pub_marker = nh.advertise< visualization_msgs::Marker >("/obstacle_marker", 1);
  pub_result = nh.advertise<sensor_msgs::PointCloud2> ("/cluster_result", 1);
  
  // Spin
  ros::spin ();
}
