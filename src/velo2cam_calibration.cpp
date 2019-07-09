/*
  velo2cam_calibration - Automatic calibration algorithm for extrinsic parameters of a stereo camera and a velodyne
  Copyright (C) 2017-2018 Jorge Beltran, Carlos Guindel

  This file is part of velo2cam_calibration.

  velo2cam_calibration is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  velo2cam_calibration is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with velo2cam_calibration.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  velo2cam_calibration: Perform the registration step
*/

#define PCL_NO_PRECOMPILE

#define DEBUG

#include "velo2cam_calibration/ClusterCentroids.h"

#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <ctime>
#include "tinyxml.h"

#ifdef TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#else
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#endif


#include "velo2cam_utils.h"

using namespace std;
using namespace sensor_msgs;

ros::Publisher t_pub;
ros::Publisher clusters_sensor2_pub, clusters_sensor1_pub;
int nFrames;
bool sensor1Received, sensor2Received;

pcl::PointCloud<pcl::PointXYZ>::Ptr sensor1_cloud, sensor2_cloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr isensor1_cloud, isensor2_cloud;
std::vector<pcl::PointXYZ> sensor1_vector(4), sensor2_vector(4);

tf::StampedTransform tf_sensor1_sensor2;

bool is_sensor1_cam, is_sensor2_cam;
string sensor1_frame_id = "";
string sensor1_rotated_frame_id = "";
string sensor2_frame_id = "";
string sensor2_rotated_frame_id = "";

typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;

tf::Transform transf;

std::vector< std::tuple<int,int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> > > sensor1_buffer;
std::vector< std::tuple<int,int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> > > sensor2_buffer;

bool sync_iterations;
bool save_to_file_;
bool publish_tf_;

long int sensor1_count, sensor2_count;

std::ofstream savefile;

const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

  return buf;
}

void calibrateExtrinsics(int seek_iter = -1){
//   ROS_INFO("Hey man, keep calm... I'm calibrating your sensors...");

  std::vector<pcl::PointXYZ> local_sensor1_vector, local_sensor2_vector;
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_sensor1_cloud, local_sensor2_cloud;
  pcl::PointCloud<pcl::PointXYZ> local_l_cloud, local_c_cloud;

  int used_sensor2, used_sensor1;

  if (seek_iter>0){
    if(DEBUG) ROS_INFO("Seeking %d iterations", seek_iter);
    if(DEBUG) ROS_INFO("Last sensor2: %d, last sensor1: %d", std::get<0>(sensor2_buffer.back()),std::get<0>(sensor1_buffer.back()));
    auto it = std::find_if(sensor2_buffer.begin(), sensor2_buffer.end(), [&seek_iter](const std::tuple<int,int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> >& e) {return std::get<0>(e) == seek_iter;});
    if (it == sensor2_buffer.end()) {
      ROS_WARN("Could not sync sensor2");
      return;
    }

    auto it2 = std::find_if(sensor1_buffer.begin(), sensor1_buffer.end(), [&seek_iter](const std::tuple<int,int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> >& e) {return std::get<0>(e) == seek_iter;});
    if (it2 == sensor1_buffer.end()) {
      ROS_WARN("Could not sync sensor1");
      return;
    }

    used_sensor2 = std::get<1>(*it);
    used_sensor1 = std::get<1>(*it2);

    local_sensor2_vector = std::get<3>(*it);
    local_c_cloud = std::get<2>(*it);
    local_sensor2_cloud = local_c_cloud.makeShared();

    local_sensor1_vector = std::get<3>(*it2);
    local_l_cloud = std::get<2>(*it2);
    local_sensor1_cloud = local_l_cloud.makeShared();
    ROS_INFO("Synchronizing cluster centroids");
  }else{
    local_sensor1_vector = sensor1_vector;
    local_sensor2_vector = sensor2_vector;
    local_sensor1_cloud = sensor1_cloud;
    local_sensor2_cloud = sensor2_cloud;
  }

  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*local_sensor2_cloud, ros_cloud);
  ros_cloud.header.frame_id = sensor2_rotated_frame_id;
  clusters_sensor2_pub.publish(ros_cloud);

  pcl::toROSMsg(*local_sensor1_cloud, ros_cloud);
  ros_cloud.header.frame_id = sensor1_frame_id;
  clusters_sensor1_pub.publish(ros_cloud);

  Vector12d sensor1_12;
  sensor1_12 <<   local_sensor1_vector[0].x,
  local_sensor1_vector[0].y,
  local_sensor1_vector[0].z,
  local_sensor1_vector[1].x,
  local_sensor1_vector[1].y,
  local_sensor1_vector[1].z,
  local_sensor1_vector[2].x,
  local_sensor1_vector[2].y,
  local_sensor1_vector[2].z,
  local_sensor1_vector[3].x,
  local_sensor1_vector[3].y,
  local_sensor1_vector[3].z;

  Vector12d sensor2_12;
  sensor2_12 <<   local_sensor2_vector[0].x,
  local_sensor2_vector[0].y,
  local_sensor2_vector[0].z,
  local_sensor2_vector[1].x,
  local_sensor2_vector[1].y,
  local_sensor2_vector[1].z,
  local_sensor2_vector[2].x,
  local_sensor2_vector[2].y,
  local_sensor2_vector[2].z,
  local_sensor2_vector[3].x,
  local_sensor2_vector[3].y,
  local_sensor2_vector[3].z;

  Vector12d diff_12;
  diff_12 = sensor1_12-sensor2_12;

  Eigen::MatrixXd matrix_transl(12,3);
  matrix_transl <<    1, 0, 0, 0, 1, 0, 0, 0, 1,
  1, 0, 0, 0, 1, 0, 0, 0, 1,
  1, 0, 0, 0, 1, 0, 0, 0, 1,
  1, 0, 0, 0, 1, 0, 0, 0, 1;

  Eigen::Vector3d x;
  x = matrix_transl.colPivHouseholderQr().solve(diff_12);

  // cout << "The least-squares solution is:\n"
  //     << x << endl;

  Eigen::Matrix4f Tm;
  Tm <<   1, 0, 0, x[0],
  0, 1, 0, x[1],
  0, 0, 1, x[2],
  0, 0, 0, 1;

  if(DEBUG) ROS_INFO("Step 1: Translation");
  if(DEBUG) cout << Tm << endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr translated_pc (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud(*local_sensor2_cloud, *translated_pc, Tm);

  pcl::toROSMsg(*translated_pc, ros_cloud);
  ros_cloud.header.frame_id = sensor1_frame_id;
  t_pub.publish(ros_cloud);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(translated_pc);
  icp.setInputTarget(local_sensor1_cloud);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  icp.setMaxCorrespondenceDistance(0.2);
  icp.setMaximumIterations(1000);
  if (icp.hasConverged()){
    if(DEBUG) ROS_INFO("ICP Converged. Score: %lf", icp.getFitnessScore());
  }else{
    ROS_WARN("ICP failed to converge");
    return;
  }
  if(DEBUG) ROS_INFO("Step 2. ICP Transformation:");
  if(DEBUG) cout << icp.getFinalTransformation() << std::endl;

  Eigen::Matrix4f transformation = icp.getFinalTransformation ();
  Eigen::Matrix4f final_trans = transformation * Tm;

  tf::Matrix3x3 tf3d;
  tf3d.setValue(final_trans(0,0), final_trans(0,1), final_trans(0,2),
  final_trans(1,0), final_trans(1,1), final_trans(1,2),
  final_trans(2,0), final_trans(2,1), final_trans(2,2));

  if(DEBUG) ROS_INFO("Final Transformation");
  if(DEBUG) cout << final_trans << endl;

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  #ifdef TF2

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = sensor1_frame_id;
  transformStamped.child_frame_id = sensor2_rotated_frame_id;
  transformStamped.transform.translation.x = final_trans(0,3);
  transformStamped.transform.translation.y = final_trans(1,3);
  transformStamped.transform.translation.z = final_trans(2,3);
  transformStamped.transform.rotation.x = tfqt.x();
  transformStamped.transform.rotation.y = tfqt.y();
  transformStamped.transform.rotation.z = tfqt.z();
  transformStamped.transform.rotation.w = tfqt.w();

  br.sendTransform(transformStamped);

  #else

  tf::Vector3 origin;
  origin.setValue(final_trans(0,3),final_trans(1,3),final_trans(2,3));

  transf.setOrigin(origin);
  transf.setRotation(tfqt);

  #endif

  static tf::TransformBroadcaster br;
  tf_sensor1_sensor2 = tf::StampedTransform(transf, ros::Time::now(), sensor1_frame_id, sensor2_rotated_frame_id);
  if (publish_tf_) br.sendTransform(tf_sensor1_sensor2);

  tf::Transform inverse = tf_sensor1_sensor2.inverse();
  double roll, pitch, yaw;
  double xt = inverse.getOrigin().getX(), yt = inverse.getOrigin().getY(), zt = inverse.getOrigin().getZ();
  inverse.getBasis().getRPY(roll, pitch, yaw);

  if (save_to_file_){
    savefile << seek_iter << ", " << xt << ", " << yt << ", " << zt << ", " << roll << ", " << pitch << ", " << yaw << ", " << used_sensor1 << ", " << used_sensor2 << endl;
  }

  ROS_INFO("[V2C] Calibration result:");
  ROS_INFO("x=%.4f y=%.4f z=%.4f",xt,yt,zt);
  ROS_INFO("roll=%.4f, pitch=%.4f, yaw=%.4f",roll,pitch,yaw);
  // ROS_INFO("Translation matrix");
  // ROS_INFO("%.4f, %.4f, %.4f",inverse.getBasis()[0][0],inverse.getBasis()[0][1],inverse.getBasis()[0][2]);
  // ROS_INFO("%.4f, %.4f, %.4f",inverse.getBasis()[1][0],inverse.getBasis()[1][1],inverse.getBasis()[1][2]);
  // ROS_INFO("%.4f, %.4f, %.4f",inverse.getBasis()[2][0],inverse.getBasis()[2][1],inverse.getBasis()[2][2]);

  sensor1Received = false;
  sensor2Received = false;
}

void sensor1_callback(const velo2cam_calibration::ClusterCentroids::ConstPtr sensor1_centroids){
  if(DEBUG) ROS_INFO("sensor1 pattern ready!");
  ROS_INFO("sensor1 pattern ready!");
  sensor1_frame_id = sensor1_centroids->header.frame_id;

  if (is_sensor1_cam){
    std::ostringstream sstream;
    sstream << "rotated_" << sensor1_frame_id;
    sensor1_rotated_frame_id = sstream.str();
#ifdef TF2

    //TODO: adapt to ClusterCentroids

    PointCloud2 xy_sensor1_cloud;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(sensor1_rotated_frame_id, sensor1_frame_id,
                             ros::Time(0), ros::Duration(20));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    fromROSMsg(sensor1_centroids->cloud, *xy_sensor1_cloud);
    tf2::doTransform (*xy_sensor1_cloud, *sensor1_cloud, transformStamped);

#else

    pcl::PointCloud<pcl::PointXYZ>::Ptr xy_sensor1_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    fromROSMsg(sensor1_centroids->cloud, *xy_sensor1_cloud);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
      listener.waitForTransform(sensor1_rotated_frame_id, sensor1_frame_id, ros::Time(0), ros::Duration(20.0));
      listener.lookupTransform (sensor1_rotated_frame_id, sensor1_frame_id, ros::Time(0), transform);
    }catch (tf::TransformException& ex) {
      ROS_WARN("TF exception:\n%s", ex.what());
      return;
    }

    tf::Transform inverse = transform.inverse();
    double roll, pitch, yaw;
    inverse.getBasis().getRPY(roll, pitch, yaw);

    pcl_ros::transformPointCloud (*xy_sensor1_cloud, *sensor1_cloud, transform);

#endif
  }else{
    fromROSMsg(sensor1_centroids->cloud, *sensor1_cloud);
  }

  sensor1Received = true;

  sortPatternCentersYZ(sensor1_cloud, sensor1_vector);
  colourCenters(sensor1_cloud, isensor1_cloud);

  sensor1_buffer.push_back(std::tuple<int,int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> >(sensor1_centroids->total_iterations, sensor1_centroids->cluster_iterations, *sensor1_cloud,sensor1_vector));
  sensor1_count = sensor1_centroids->total_iterations;

  if(DEBUG) ROS_INFO("[V2C] sensor1");

  for(vector<pcl::PointXYZ>::iterator it=sensor1_vector.begin(); it<sensor1_vector.end(); ++it){
    if (DEBUG) cout << "l" << it - sensor1_vector.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << endl;
  }

  if (sync_iterations){
    if(sensor2_count >= sensor1_count){
      calibrateExtrinsics(sensor1_count);
      return;
    }else{
      if (tf_sensor1_sensor2.frame_id_ != "" && tf_sensor1_sensor2.child_frame_id_ != ""){
        static tf::TransformBroadcaster br;
        tf_sensor1_sensor2.stamp_ = ros::Time::now();
        if (publish_tf_) br.sendTransform(tf_sensor1_sensor2);
        return;
      }
    }
  }

  if(sensor1Received && sensor2Received){
    calibrateExtrinsics();
  }else{
    if (tf_sensor1_sensor2.frame_id_ != "" && tf_sensor1_sensor2.child_frame_id_ != ""){
      static tf::TransformBroadcaster br;
      tf_sensor1_sensor2.stamp_ = ros::Time::now();
      if (publish_tf_) br.sendTransform(tf_sensor1_sensor2);
    }
  }
}

void sensor2_callback(velo2cam_calibration::ClusterCentroids::ConstPtr sensor2_centroids){
   if(DEBUG) ROS_INFO("sensor2 pattern ready!");
   ROS_INFO("sensor2 pattern ready!");
  sensor2_frame_id = sensor2_centroids->header.frame_id;

  if (is_sensor2_cam){
    std::ostringstream sstream;
    sstream << "rotated_" << sensor2_frame_id;
    sensor2_rotated_frame_id = sstream.str();
  #ifdef TF2

    //TODO: adapt to ClusterCentroids

    PointCloud2 xy_sensor2_cloud;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(sensor2_rotated_frame_id, sensor2_frame_id,
                             ros::Time(0), ros::Duration(20));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    fromROSMsg(sensor2_centroids->cloud, *xy_sensor2_cloud);
    tf2::doTransform (*xy_sensor2_cloud, sensor2_cloud, transformStamped);

  #else

    pcl::PointCloud<pcl::PointXYZ>::Ptr xy_sensor2_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    fromROSMsg(sensor2_centroids->cloud, *xy_sensor2_cloud);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
      listener.waitForTransform(sensor2_rotated_frame_id, sensor2_frame_id, ros::Time(0), ros::Duration(20.0));
      listener.lookupTransform (sensor2_rotated_frame_id, sensor2_frame_id, ros::Time(0), transform);
    }catch (tf::TransformException& ex) {
      ROS_WARN("TF exception:\n%s", ex.what());
      return;
    }

    tf::Transform inverse = transform.inverse();
    double roll, pitch, yaw;
    inverse.getBasis().getRPY(roll, pitch, yaw);

    pcl_ros::transformPointCloud (*xy_sensor2_cloud, *sensor2_cloud, transform);

  #endif
  }else{
    fromROSMsg(sensor2_centroids->cloud, *sensor2_cloud);
  }

  sensor2Received = true;

  sortPatternCentersYZ(sensor2_cloud, sensor2_vector);
  colourCenters(sensor2_cloud, isensor2_cloud);

  sensor2_buffer.push_back(std::tuple<int, int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> >(sensor2_centroids->total_iterations,sensor2_centroids->cluster_iterations,*sensor2_cloud,sensor2_vector));
  sensor2_count = sensor2_centroids->total_iterations;

  if(DEBUG) ROS_INFO("[V2C] sensor2");

  for(vector<pcl::PointXYZ>::iterator it=sensor2_vector.begin(); it<sensor2_vector.end(); ++it){
    if (DEBUG) cout << "c" << it - sensor2_vector.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]"<<endl;
  }

  if (sync_iterations){
    if(sensor1_count >= sensor2_count){
      calibrateExtrinsics(sensor2_count);
      return;
    }else{
      if (tf_sensor1_sensor2.frame_id_ != "" && tf_sensor1_sensor2.child_frame_id_ != ""){
        static tf::TransformBroadcaster br;
        tf_sensor1_sensor2.stamp_ = ros::Time::now();
        if (publish_tf_) br.sendTransform(tf_sensor1_sensor2);
        return;
      }
    }
  }

  if(sensor1Received && sensor2Received){
    if(DEBUG) ROS_INFO("[V2C] Calibrating...");
    calibrateExtrinsics();
  }else{
    if (tf_sensor1_sensor2.frame_id_ != "" && tf_sensor1_sensor2.child_frame_id_ != ""){
      static tf::TransformBroadcaster br;
      tf_sensor1_sensor2.stamp_ = ros::Time::now();
      if (publish_tf_) br.sendTransform(tf_sensor1_sensor2);
    }
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "velo2cam_calibration");
  ros::NodeHandle nh_("~"); // LOCAL

  nh_.param<bool>("sync_iterations", sync_iterations, false);
  nh_.param<bool>("save_to_file", save_to_file_, false);
  nh_.param<bool>("publish_tf", publish_tf_, true);
  nh_.param<bool>("is_sensor2_cam", is_sensor2_cam, false);
  nh_.param<bool>("is_sensor1_cam", is_sensor1_cam, false);

  sensor1Received = false;
  sensor1_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  isensor1_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  sensor2Received = false;
  sensor2_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  isensor2_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  ros::Subscriber sensor1_sub = nh_.subscribe<velo2cam_calibration::ClusterCentroids>("cloud1", 1, sensor1_callback);
  ros::Subscriber sensor2_sub = nh_.subscribe<velo2cam_calibration::ClusterCentroids>("cloud2", 1, sensor2_callback);

  t_pub = nh_.advertise<sensor_msgs::PointCloud2>("translated_cloud", 1);
  clusters_sensor2_pub = nh_.advertise<sensor_msgs::PointCloud2>("clusters_sensor2", 1);
  clusters_sensor1_pub = nh_.advertise<sensor_msgs::PointCloud2>("clusters_sensor1", 1);

  if (save_to_file_){
    ostringstream os;
    os << getenv("HOME") << "/results_" << currentDateTime() << ".csv" ;
    if (save_to_file_){
      if(DEBUG) ROS_INFO("Opening %s", os.str().c_str());
      savefile.open (os.str().c_str());
      savefile << "it, x, y, z, r, p, y, used_l, used_c" << endl;
    }
  }

  ros::Rate loop_rate(30);
  while(ros::ok()){
    ros::spinOnce();
  }

  if (save_to_file_) savefile.close();

  // Save calibration params to launch file for testing

  // Get time
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,80,"%Y-%m-%d-%H-%M-%S", timeinfo);
  std::string str(buffer);

  // Get tf data
  tf::Transform inverse = tf_sensor1_sensor2.inverse();
  double roll, pitch, yaw;
  double xt = inverse.getOrigin().getX(), yt = inverse.getOrigin().getY(), zt = inverse.getOrigin().getZ();
  inverse.getBasis().getRPY(roll, pitch, yaw);

  ROS_INFO("Calibration finished succesfully...");
  ROS_INFO("x=%.4f y=%.4f z=%.4f",xt,yt,zt);
  ROS_INFO("roll=%.4f, pitch=%.4f, yaw=%.4f", roll, pitch, yaw);

  std::string path = ros::package::getPath("velo2cam_calibration");
  string backuppath = path + "/launch/calibrated_tf_"+ str +".launch";
  path = path + "/launch/calibrated_tf.launch";

  cout << endl << "Creating .launch file with calibrated TF in: "<< endl << path.c_str() << endl;
  // Create .launch file with calibrated TF
  TiXmlDocument doc;
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "utf-8", "");
  doc.LinkEndChild( decl );
  TiXmlElement * root = new TiXmlElement( "launch" );
  doc.LinkEndChild( root );

  TiXmlElement * arg = new TiXmlElement( "arg" );
  arg->SetAttribute("name","stdout");
  arg->SetAttribute("default","screen");
  root->LinkEndChild( arg );

  string sensor2_final_transformation_frame = sensor2_frame_id;
  if(is_sensor2_cam){
    sensor2_final_transformation_frame = sensor2_rotated_frame_id;
    std::ostringstream sensor2_rot_stream_pub;
    sensor2_rot_stream_pub << "0 0 0 -1.57079632679 0 -1.57079632679 " << sensor2_rotated_frame_id << " " << sensor2_frame_id << " 10";
    string sensor2_rotation  = sensor2_rot_stream_pub.str();

    TiXmlElement * sensor2_rotation_node = new TiXmlElement( "node" );
    sensor2_rotation_node->SetAttribute("pkg","tf");
    sensor2_rotation_node->SetAttribute("type","static_transform_publisher");
    sensor2_rotation_node->SetAttribute("name","sensor2_rot_tf");
    sensor2_rotation_node->SetAttribute("args", sensor2_rotation);
    root->LinkEndChild( sensor2_rotation_node );
  }

  string sensor1_final_transformation_frame = sensor1_frame_id;
  if(is_sensor1_cam){
    sensor1_final_transformation_frame = sensor1_rotated_frame_id;
    std::ostringstream sensor1_rot_stream_pub;
    sensor1_rot_stream_pub << "0 0 0 -1.57079632679 0 -1.57079632679 " << sensor1_rotated_frame_id << " " << sensor1_frame_id << " 10";
    string sensor1_rotation  = sensor1_rot_stream_pub.str();

    TiXmlElement * sensor1_rotation_node = new TiXmlElement( "node" );
    sensor1_rotation_node->SetAttribute("pkg","tf");
    sensor1_rotation_node->SetAttribute("type","static_transform_publisher");
    sensor1_rotation_node->SetAttribute("name","sensor1_rot_tf");
    sensor1_rotation_node->SetAttribute("args", sensor1_rotation);
    root->LinkEndChild( sensor1_rotation_node );
  }


  std::ostringstream sstream;
  sstream << xt << " " << yt << " " << zt << " " << yaw << " " <<pitch<< " " << roll << " " << sensor2_final_transformation_frame << " " << sensor1_final_transformation_frame << " 100";
  string tf_args = sstream.str();
  cout << tf_args << endl;

  TiXmlElement * node = new TiXmlElement( "node" );
  node->SetAttribute("pkg","tf");
  node->SetAttribute("type","static_transform_publisher");
  node->SetAttribute("name","velo2cam_tf");
  node->SetAttribute("args", tf_args);
  root->LinkEndChild( node );

  // Save XML file and copy
  doc.SaveFile(path);
  doc.SaveFile(backuppath);

  if(DEBUG) cout << "Calibration process finished." << endl;

  return 0;
}
