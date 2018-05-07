#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <sl/Camera.hpp>

// Point Cloud Libs
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/median_filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <time.h>
#include <iostream>

/**
*  This function convert a RGBA color packed into a packed RGBA PCL compatible format
**/
inline float convertColor(float colorIn){
    uint32_t color_uint = *(uint32_t *)&colorIn;
    unsigned char *color_uchar = (unsigned char *)&color_uint;
    color_uint = ((uint32_t)color_uchar[0] << 16 | (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
    return *reinterpret_cast<float *>(&color_uint);
}

int main(int argc,char** argv){

  std::cout << "Starting up ZED node...\n";

  ros::init(argc,argv,"ZED");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("zed_out",1000);

  // Point Cloud publisher
  ros::Publisher pub_cloud = n.advertise<sensor_msgs::PointCloud2> ("point_cloud", 1);

  // Tranform Publisher
  static tf::TransformBroadcaster br;

  ros::Rate loopRate(200);

  sl::ERROR_CODE err;
  sl::Camera zed;

  // Set configuration parameters
  sl::InitParameters init_params;
  // Use HD720 video mode (default fps: 60)
  //init_params.camera_resolution = sl::RESOLUTION_HD720;
  init_params.camera_resolution = sl::RESOLUTION_VGA;
  // Use a right-handed Y-up coordinate system
  init_params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP;
  // Set units in meters
  init_params.coordinate_units = sl::UNIT_METER; 

  err = zed.open(init_params);
  if(err != sl::SUCCESS){
    exit(-1);
  }
  std::cout << "opened ZED camera successfully\n";

  sl::TrackingParameters tracking_parameters;
  err = zed.enableTracking(tracking_parameters);
  if(err != sl::SUCCESS){
    exit(-1);
  }
  std::cout << "camera tracking enabled\n";

  // Zed Point Cloud Data
  sl::Mat cloud;

  // Transform Stamped
  geometry_msgs::TransformStamped trans_out;

  sl::Pose zed_pose;
  geometry_msgs::PoseStamped pose_out;
  pose_out.header.frame_id = "map";
  std_msgs::String msg;
  char msg_data[255];
  int count = 0;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZRGB> med_filtered;
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> med_filtered;
  pcl::MedianFilter<pcl::PointXYZRGB> mf;
  p_pcl_point_cloud->points.resize(zed.getResolution().area());
  //zed.setConfidenceThreshold(99);
  zed.setDepthMaxRangeValue(2.8);

  while(ros::ok()){
    if(zed.grab() == sl::SUCCESS){
      sl::TRACKING_STATE state =
        zed.getPosition(zed_pose, sl::REFERENCE_FRAME_WORLD);

      // Get Point Cloud Data and store it into Mat cloud
      zed.retrieveMeasure(cloud, sl::MEASURE_XYZRGBA);

      // Dimensions of zed camera
      int height = zed.getResolution().height;
      int width = zed.getResolution().width;
      p_pcl_point_cloud->height = height;
      p_pcl_point_cloud->width = width;
      int size = width*height;

      // point_cloud object to store data from zed
      //pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
      //point_cloud.width = width;
      //point_cloud.height = height;
      //point_cloud.points.resize(size);

      float *p_data_cloud = cloud.getPtr<float>();
      int index = 0;
      // Check and adjust points for PCL format
      for (auto &it : p_pcl_point_cloud->points) {
          float X = p_data_cloud[index];
          if (!isValidMeasure(X)) { // Checking if it's a valid point
              it.x = it.y = it.z = 0;
              //it.x = it.y = it.z = it.rgb = 0;
          } else {
              it.x = X;
              it.y = p_data_cloud[index + 1];
              it.z = p_data_cloud[index + 2];
              //it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
          }
          index += 4;
      }
      //mf.setInputCloud(p_pcl_point_cloud);
      //mf.applyFilter(med_filtered);

      // Point cloud data converted to ros msg
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*p_pcl_point_cloud, output); // Convert the point cloud to a ROS message
      output.header.frame_id = "camera_link"; // Set the header values of the ROS message
      //utput.height = height;
      //output.width = width;
      output.is_bigendian = true;
      output.is_dense = false;
      pub_cloud.publish(output);


      //std::cout << "time: " << zed_pose.timestamp << "\n";
      pose_out.pose.position.x = zed_pose.getTranslation().tx;
      pose_out.pose.position.y = zed_pose.getTranslation().ty;
      pose_out.pose.position.z = zed_pose.getTranslation().tz;

      pose_out.pose.orientation.x = zed_pose.getOrientation().ox;
      pose_out.pose.orientation.y = zed_pose.getOrientation().oy;
      pose_out.pose.orientation.z = zed_pose.getOrientation().oz;
      pose_out.pose.orientation.w = zed_pose.getOrientation().ow;


      tf::Transform transform;
      transform.setOrigin(tf::Vector3(
            pose_out.pose.position.x,
            pose_out.pose.position.y,
            pose_out.pose.position.z));
      tf::Quaternion qr (
          pose_out.pose.orientation.x,
          pose_out.pose.orientation.y,
          pose_out.pose.orientation.z,
          pose_out.pose.orientation.w);
      transform.setRotation(qr);
      br.sendTransform(
          tf::StampedTransform(
            transform,
            ros::Time::now(),
            "map",
            "camera_link"));

      pub.publish(pose_out);
    }

    count++;
    ros::spinOnce();
    loopRate.sleep();
  }

  std::cout << "Closing camera...\n";
  zed.close();
  return 0;
}
