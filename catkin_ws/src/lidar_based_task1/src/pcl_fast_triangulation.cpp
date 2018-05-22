#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

ros::Publisher pub_mesh_triangulation;
ros::Publisher pub_cloud_voxel;
ros::Publisher pub_marker;

//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//bool first_Set_Viewer = true;
geometry_msgs::Point PCLPoint_To_RosPoint(pcl::PointXYZ pcl_point)
{
    geometry_msgs::Point point;
    point.x = pcl_point.x;
    point.y = pcl_point.y;
    point.z = pcl_point.z;
    return point;
}

void polygonShow(pcl::PolygonMesh triangles)
{      
    std::vector< pcl::Vertices> polygons = triangles.polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_triangulation(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(triangles.cloud, *cloud_triangulation);

    //Initial Marker
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = cloud_triangulation->header.frame_id;
    line_list.header.stamp = pcl_conversions::fromPCL(cloud_triangulation->header.stamp);
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.scale.x = 0.01;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    for (int i = 0 ; i < polygons.size() ; i++ )
    {
        std::vector<uint32_t> vertices = polygons[i].vertices;
        geometry_msgs::Point p_st, p_ed;
        p_st = PCLPoint_To_RosPoint(cloud_triangulation->points[ vertices[0] ] );
        p_ed = PCLPoint_To_RosPoint(cloud_triangulation->points[ vertices[1] ] );
        line_list.points.push_back(p_st);
        line_list.points.push_back(p_ed);
        p_ed = PCLPoint_To_RosPoint(cloud_triangulation->points[ vertices[2] ] );
        line_list.points.push_back(p_st);
        line_list.points.push_back(p_ed);
        p_st = PCLPoint_To_RosPoint(cloud_triangulation->points[ vertices[0] ] );
        line_list.points.push_back(p_st);
        line_list.points.push_back(p_ed);        
    }
    //cout << "cloud data size = " << cloud_triangulation->points.size() << endl;

    pub_marker.publish(line_list);
    
}
void cbPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    std::cout<<"start"<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // copy msg to cloud
    pcl::fromROSMsg(*cloud_msg, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*cloud, *cloud_voxel);

    // Calcualte the normal of cloud
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz(new pcl::search::KdTree<pcl::PointXYZ>);
    tree_xyz->setInputCloud(cloud_voxel);
    ne.setInputCloud(cloud_voxel);
    ne.setSearchMethod(tree_xyz);
    ne.setRadiusSearch(0.1);
    ne.compute(*cloud_normals);
    
    //combine normal and cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields( *cloud_voxel, *cloud_normals, *cloud_with_normals );


    pcl::search::KdTree<pcl::PointNormal>::Ptr tree_point_normal (new pcl::search::KdTree<pcl::PointNormal>);
    tree_point_normal->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius (0.5);
    std::cout<<"middle"<<std::endl;
    // Set typical values for the parameters
    gp3.setMu (3.0);
    gp3.setMaximumNearestNeighbors (200);
    gp3.setMaximumSurfaceAngle(M_PI/5); // 36 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree_point_normal);
    gp3.reconstruct (triangles);
    
    polygonShow(triangles);

    std::cout<<"publish"<<std::endl;
    pub_cloud_voxel.publish(*cloud_voxel);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_fast_triangulation_node");
	std::string name = ros::this_node::getName();
	ROS_INFO("[%s] Initializing ", name.c_str());

    ros::NodeHandle nh("~");
    // Subscriber
    ros::Subscriber sub_cloud = nh.subscribe("/cluster_result", 1, cbPointCloud);
    
    //Publisher
    pub_cloud_voxel = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("cloud_voxel", 1);
    pub_marker = nh.advertise<visualization_msgs::Marker>("marker_triangulation", 1);

    ros::spin();
    return 0 ;
}