#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <Eigen/Dense>

typedef pcl::PointXYZ PointXYZ;

//declare global variable
bool lock = false;

//All the objects needed
pcl::PassThrough<PointXYZ> pass;
pcl::NormalEstimation<PointXYZ, pcl::Normal> ne;
pcl::SACSegmentationFromNormals<PointXYZ, pcl::Normal> seg;
pcl::ExtractIndices<PointXYZ> extract;
pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ> ());

//Datasets
pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
pcl::PointCloud<PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
pcl::PointCloud<PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<PointXYZ> ());


class PointCloud
{
private:

	ros::NodeHandle nh;

	ros::Subscriber pcd_sub; //Read in point cloud data from sensor
	ros::Publisher cloud_pub; //Publish the point cloud ROSMsg


public:

	PointCloud (ros::NodeHandle& nh): nh("~")
	{
		//Subscribe to pointcloud
		pcd_sub = nh.subscribe("/velodyne_points", 1, &PointCloud::callback, this);

		//Publish to rviz
		cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cylinder_cloud", 1);
	}


	//Process the pointcloud
	void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		ROS_INFO_STREAM("Received callback");

		if(!lock){
			lock = true;
			//Convert from ROS Msg to pcd
			pcl::fromROSMsg(*msg, *cloud);
			pointcloud_processing();
		}
	}


	void pointcloud_processing()
	{
		//Build a passthrough filter to remove spurious NaNs
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (-10, 10);
		pass.filter (*cloud_filtered);

		//Estimate point normals
		ne.setSearchMethod (tree);
		ne.setInputCloud (cloud_filtered);
		ne.setKSearch (100);
		ne.compute (*cloud_normals);

		//Create the segmentation object for cylinder segmentation and set all the parameters
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_CYLINDER);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight (0.1);
		seg.setMaxIterations (5000);
		seg.setDistanceThreshold (0.5);
		seg.setRadiusLimits (0, 1);
		seg.setInputCloud (cloud_filtered);
		seg.setInputNormals (cloud_normals);

		//Obtain the cylinder inliers and coefficients
		seg.segment (*inliers_cylinder, *coefficients_cylinder);

		//Extract cylinders
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers_cylinder);
		extract.setNegative (false);
		extract.filter (*cloud_cylinder);


		// Convert pcd data type to ROS message type
		sensor_msgs::PointCloud2 map_cloud;
		pcl::toROSMsg(*cloud_cylinder, map_cloud);

		// Set PointCloud2 parameters
		map_cloud.header.frame_id = "/velodyne";

		// Publish the point cloud ROSMsg
		cloud_pub.publish(map_cloud);

		lock = false;
	}


	void spin ()
	{
		ros::Rate rate (30);
		while (ros::ok ())
		{
		ros::spinOnce ();
		rate.sleep ();
		}
	}


};



int main(int argc, char **argv)
{
	ROS_INFO_STREAM("Starting Point Cloud node");

	//Initialize ROS
	ros::init(argc, argv, "pointcloud_node");

	ros::NodeHandle nh;

	PointCloud* node = 0;

	node = new PointCloud(nh);

	node->spin (); 
	return 0;

}