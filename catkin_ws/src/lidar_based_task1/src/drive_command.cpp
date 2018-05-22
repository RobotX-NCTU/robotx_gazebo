#include <ros/ros.h>
#include <cmath>        // std::abs
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <lidar_based_task1/ObstaclePose.h>
#include <lidar_based_task1/ObstaclePoseList.h>
#include <lidar_based_task1/UsvDrive.h>

#define PI 3.14159

//define ROS publisher
ros::Publisher pub_drive;

//define global variable
lidar_based_task1::UsvDrive drive;
bool Entrance = true;
bool Exit = false;

//define function
void obstacle_cb(lidar_based_task1::ObstaclePoseList);
void drive_to_entrance(lidar_based_task1::ObstaclePoseList, float, float, float, float);
void drive_to_trans(lidar_based_task1::ObstaclePoseList, float, float, float, float);
void drive_to_exit(lidar_based_task1::ObstaclePoseList, float, float);


//Callback 
void obstacle_cb(lidar_based_task1::ObstaclePoseList pos_list)
{
  std::cout << "Receive callback" << std::endl;

  float width = 100;        //Width of the rec area
  float length = 0;         //Length of the rec area
  float long_distance = 0;  //Diagonal distance of the rec area
  float entrance_x = 0;
  float entrance_y = 0;
  float exit_x = 0;
  float exit_y = 0;
  float entrance_stop_x = 0;
  float entrance_stop_y = 0;
  float exit_stop_x = 0;
  float exit_stop_y = 0;
  float trans_x = 0;
  float trans_y = 0;
  float group_1 [2][2];     //Entrance group 
  float group_2 [2][2];     //Exit group
  float diag [2][2];        //Diagonal group
  float bot_to_closest = 0;
  float bot_to_width = 0;
  float bot_to_length = 0;
  float length_angle = 0;
  float width_angle = 0;

  //Four points are detected
  if (pos_list.size == 4){
    std::cout << "Four points are detected" << std::endl;
    float dis_array[pos_list.size];
    float distance = 0;

    //Store the distance between the first point and the remaining
    for (int i=0;i<=pos_list.size-1;i++){
      dis_array[i] = sqrt((pos_list.list[0].x - pos_list.list[i].x)*(pos_list.list[0].x - pos_list.list[i].x)  + (pos_list.list[0].y - pos_list.list[i].y)*(pos_list.list[0].y - pos_list.list[i].y)); 
    }
    
    //Grouping points
    for (int i=1;i<=pos_list.size-1;i++){
      distance = dis_array[i];
      
      if (distance<=width){
        width = distance;
        group_1[0][0] = pos_list.list[0].x;
        group_1[0][1] = pos_list.list[0].y;
        group_1[1][0] = pos_list.list[i].x;
        group_1[1][1] = pos_list.list[i].y;

        if (i==1){
          group_2[0][0] = pos_list.list[2].x;
          group_2[0][1] = pos_list.list[2].y;
          group_2[1][0] = pos_list.list[3].x;
          group_2[1][1] = pos_list.list[3].y;
        }
        else if (i==2){
          group_2[0][0] = pos_list.list[1].x;
          group_2[0][1] = pos_list.list[1].y;
          group_2[1][0] = pos_list.list[3].x;
          group_2[1][1] = pos_list.list[3].y;
        }
        else{
          group_2[0][0] = pos_list.list[1].x;
          group_2[0][1] = pos_list.list[1].y;
          group_2[1][0] = pos_list.list[2].x;
          group_2[1][1] = pos_list.list[2].y;
        }
      }

      if (distance>=long_distance){
        long_distance = distance;
        diag[0][0] = pos_list.list[0].x;
        diag[0][1] = pos_list.list[0].y;
        diag[1][0] = pos_list.list[i].x;
        diag[1][1] = pos_list.list[i].y;
      }
    }
    length = sqrt(long_distance*long_distance - width*width); 

    //====== Detect which side the robot is ======
    bot_to_closest = sqrt(group_1[0][0]*group_1[0][0] + group_1[0][1]*group_1[0][1]);
    bot_to_width = sqrt(group_1[1][0]*group_1[1][0] + group_1[1][1]*group_1[1][1]);
    bot_to_length = sqrt(group_2[0][0]*group_2[0][0] + group_2[0][1]*group_2[0][1]);

    width_angle = acos((bot_to_closest*bot_to_closest + width*width - bot_to_width*bot_to_width) / (2*bot_to_closest*width)) * 180 / PI;
    length_angle = acos((bot_to_closest*bot_to_closest + length*length - bot_to_length*bot_to_length) / (2*bot_to_closest*length)) * 180 / PI;

    if (length_angle >= 90)
      Entrance = true;  //Entrance side
    else if (width_angle >= 90 && length_angle <= 90)
      Entrance = false; //Length side

    //Calculate middle points between two entrance and exit points, respectively
    entrance_x = (group_1[0][0] + group_1[1][0]) / 2;
    entrance_y = (group_1[0][1] + group_1[1][1]) / 2;
    exit_x = (group_2[0][0] + group_2[1][0]) / 2;
    exit_y = (group_2[0][1] + group_2[1][1]) / 2;

    //Calculate points that are in front of two middle points
    entrance_stop_x = entrance_x - (exit_x - entrance_x) / 7;
    entrance_stop_y = entrance_y - (exit_y - entrance_y) / 7;
    exit_stop_x = exit_x + (exit_x - entrance_x) / 7;
    exit_stop_y = exit_y + (exit_y - entrance_y) / 7;

    //Calculate transition point
    trans_x = diag[0][0] - (diag[1][0] - diag[0][0]) / 5;
    trans_y = diag[0][1] - (diag[1][1] - diag[0][1]) / 5;

    if (Entrance){
      std::cout << width_angle << "/" << length_angle << std::endl;
      drive_to_entrance(pos_list, entrance_stop_x, entrance_stop_y, exit_stop_x, exit_stop_y);
    }
    else
      drive_to_trans(pos_list, trans_x, trans_y, entrance_stop_x, entrance_stop_y);
  }
  //If three points are detected
  else if (pos_list.size == 3){
    std::cout << "Three points are detected" << std::endl;
    if (Entrance && Exit){
      exit_stop_x = (pos_list.list[0].x + pos_list.list[1].x) / 2;
      exit_stop_y = (pos_list.list[0].y + pos_list.list[1].y) / 2;

      drive_to_exit(pos_list, exit_stop_x, exit_stop_y);
    }
    else if (Entrance){
      //entrance_stop_x = (pos_list.list[0].x + pos_list.list[1].x) / 2;
      //entrance_stop_y = (pos_list.list[0].y + pos_list.list[1].y) / 2;

      //entrance_stop_x = entrance_stop_x - (pos_list.list[2].x - entrance_stop_x) / 9;
      //entrance_stop_y = entrance_stop_y - (pos_list.list[2].y - entrance_stop_y) / 9;
      float point1 [2];
      float point2 [2];
      float s = -(pos_list.list[0].x - pos_list.list[1].x) / (pos_list.list[0].y - pos_list.list[1].y); //slope of the normal

      point1[0] = ((pos_list.list[0].x + pos_list.list[1].x) / 2) - 1;
      point1[1] = ((pos_list.list[0].y + pos_list.list[1].y) / 2) - 1 * s;

      point2[0] = ((pos_list.list[0].x + pos_list.list[1].x) / 2) + 1;
      point2[1] = ((pos_list.list[0].y + pos_list.list[1].y) / 2) + 1 * s;

      //Find closer point
      if (point1[0] > point2[0]){
        entrance_stop_x = point1[0];
        entrance_stop_y = point1[1];
        exit_stop_x = point2[0];
        exit_stop_y = point2[1];
      }
      else{
        entrance_stop_x = point2[0];
        entrance_stop_y = point2[1];
        exit_stop_x = point1[0];
        exit_stop_y = point1[1];
      }

      drive_to_entrance(pos_list, entrance_stop_x, entrance_stop_y, exit_stop_x, exit_stop_y);
      //drive_to_entrance(pos_list, entrance_stop_x, entrance_stop_y, 0, 0);
    }
    else{
      entrance_stop_x = (pos_list.list[0].x + pos_list.list[1].x) / 2;
      entrance_stop_y = (pos_list.list[0].y + pos_list.list[1].y) / 2;

      trans_x = pos_list.list[0].x - (pos_list.list[2].x - pos_list.list[0].x) / 7;
      trans_y = pos_list.list[0].y - (pos_list.list[2].y - pos_list.list[0].y) / 7;

      drive_to_trans(pos_list, trans_x, trans_y, entrance_stop_x, entrance_stop_y);
    }
  }
  //Less than four but more than two points are detected
  else if (pos_list.size == 2){
    std::cout << "Two points are detected" << std::endl;
    if (Entrance && Exit){
      exit_stop_x = (pos_list.list[0].x + pos_list.list[1].x) / 2;
      exit_stop_y = (pos_list.list[0].y + pos_list.list[1].y) / 2;

      drive_to_exit(pos_list, exit_stop_x, exit_stop_y);
    }
    else if (Entrance){
      float point1 [2];
      float point2 [2];
      float s = -(pos_list.list[0].x - pos_list.list[1].x) / (pos_list.list[0].y - pos_list.list[1].y); //slope of the normal

      point1[0] = ((pos_list.list[0].x + pos_list.list[1].x) / 2) - 1;
      point1[1] = ((pos_list.list[0].y + pos_list.list[1].y) / 2) - 1 * s;

      point2[0] = ((pos_list.list[0].x + pos_list.list[1].x) / 2) + 1;
      point2[1] = ((pos_list.list[0].y + pos_list.list[1].y) / 2) + 1 * s;

      //Find closer point
      if (point1[0] > point2[0]){
        entrance_stop_x = point1[0];
        entrance_stop_y = point1[1];
        exit_stop_x = point2[0];
        exit_stop_y = point2[1];
      }
      else{
        entrance_stop_x = point2[0];
        entrance_stop_y = point2[1];
        exit_stop_x = point1[0];
        exit_stop_y = point1[1];
      }

      std::cout << "entrance x position : " << entrance_stop_x << std::endl;
      std::cout << "entrance y position : " << entrance_stop_y << std::endl;
      drive_to_entrance(pos_list, entrance_stop_x, entrance_stop_y, exit_stop_x, exit_stop_y);
    }
    else{
      entrance_stop_x = (pos_list.list[0].x + pos_list.list[1].x) / 2;
      entrance_stop_y = (pos_list.list[0].y + pos_list.list[1].y) / 2;

      trans_x = pos_list.list[0].x - (pos_list.list[1].x - pos_list.list[0].x) / 2;
      trans_y = pos_list.list[0].y - (pos_list.list[1].y - pos_list.list[0].y) / 2;

      drive_to_trans(pos_list, trans_x, trans_y, entrance_stop_x, entrance_stop_y);
    }
  }
  //If less than two points or more than four points are detected
  else{
    std::cout << "Stop" << std::endl;
    drive.right = 0;
    drive.left  = 0;
    pub_drive.publish(drive);
  }
}


//Drive to entrance point
void drive_to_entrance(lidar_based_task1::ObstaclePoseList pos_list, float entrance_stop_x, float entrance_stop_y, float exit_stop_x, float exit_stop_y)
{
  std::cout << "Receive entrance func" << std::endl;

  if ((entrance_stop_x < 1.5) and (std::abs(entrance_stop_y) < 0.5)){
    std::cout << "Drive to exit" << std::endl;
    Exit = true;
    drive_to_exit(pos_list, exit_stop_x, exit_stop_y);
  }
  else if ((entrance_stop_x > 1.5) and (std::abs(entrance_stop_y) < 0.5)){
    std::cout << "Go straight" << std::endl;
    drive.right = 0.4;
    drive.left  = 0.4;
    pub_drive.publish(drive);
  }
  else if ((entrance_stop_x > 0) and ((entrance_stop_y) > 0)){
    std::cout << "Left turn" << std::endl;
    drive.right = 0.3;
    drive.left  = 0.1;
    pub_drive.publish(drive);
  }
  else if ((entrance_stop_x > 0) and ((entrance_stop_y) < 0)){
    std::cout << "Right turn" << std::endl;
    drive.right = 0.1;
    drive.left  = 0.3;
    pub_drive.publish(drive);
  }
  else{
    Exit = true;
    drive.right = 0.4;
    drive.left  = 0.4;
    pub_drive.publish(drive);
  }

  std::cout << std::endl;
}


//Drive to transition point
void drive_to_trans(lidar_based_task1::ObstaclePoseList pos_list, float trans_x, float trans_y, float entrance_stop_x, float entrance_stop_y)
{
  std::cout << "Receive transition func" << std::endl;

  if ((trans_x < 1.5) and (std::abs(trans_y) < 0.5)){
    std::cout << "Drive to entrance" << std::endl;
    Entrance = true;
    drive_to_entrance(pos_list, entrance_stop_x, entrance_stop_y, 0, 0);
  }
  else if ((trans_x > 1.5) and (std::abs(trans_y) < 0.5)){
    std::cout << "Go straight" << std::endl;
    drive.right = 0.4;
    drive.left  = 0.4;
    pub_drive.publish(drive);
  }
  else if ((trans_x > 0) and ((trans_y) > 0)){
    std::cout << "Left turn" << std::endl;
    drive.right = 0.3;
    drive.left  = 0.1;
    pub_drive.publish(drive);
  }
  else if ((trans_x > 0) and ((trans_y) < 0)){
    std::cout << "Right turn" << std::endl;
    drive.right = 0.1;
    drive.left  = 0.3;
    pub_drive.publish(drive);
  }
  else
    Entrance = true;
  
  std::cout << std::endl;

}


//Drive to exit point
void drive_to_exit(lidar_based_task1::ObstaclePoseList pos_list, float exit_stop_x, float exit_stop_y)
{
  std::cout << "Receive exit func" << std::endl;

  if ((exit_stop_x < 0.5) and (std::abs(exit_stop_y) < 0.5)){
    std::cout << "Arrive" << std::endl;
    drive.right = 0.4;
    drive.left  = 0.4;
  }
  else if ((exit_stop_x > 0.5) and (std::abs(exit_stop_y) < 0.5)){
    std::cout << "Go straight" << std::endl;
    drive.right = 0.4;
    drive.left  = 0.4;
  }
  else if ((exit_stop_x > 1.5) and ((exit_stop_y) > 0)){
    std::cout << "Left turn" << std::endl;
    drive.right = 0.3;
    drive.left  = 0.1;
  }
  else if ((exit_stop_x > 1.5) and ((exit_stop_y) < 0)){
    std::cout << "Right turn" << std::endl;
    drive.right = 0.1;
    drive.left  = 0.3;
  }

  pub_drive.publish(drive);
  std::cout << std::endl;

}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "obstacle_position");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<lidar_based_task1::ObstaclePoseList> ("/obstacle_list", 1, obstacle_cb);
  // Create a ROS publisher for the command drive
  pub_drive = nh.advertise<lidar_based_task1::UsvDrive> ("/cmd_drive", 1);
  // Spin
  ros::spin ();
}
