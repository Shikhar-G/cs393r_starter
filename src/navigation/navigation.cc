//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  // Set the navigation goal.
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  nav_complete_ = false;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // Get last odom and point cloud #### needs fixing
  // RobotState last_odom(odom_loc_, odom_angle_, robot_vel_, robot_omega_);
  std::vector<Eigen::Vector2f> last_point_cloud_ = point_cloud_;

  


  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // float distance_to_goal = (robot_state.loc - nav_goal_loc_).norm();
  // float steering_angle = AngleMod(atan2(nav_goal_loc_.y() - robot_state.loc.y(),
  //                                      nav_goal_loc_.x() - robot_state.loc.x()) -
  //                                 robot_state.angle);
  // float steering_angle = AngleMod(atan2(nav_goal_loc_.y() - robot_loc_.y(),
  //                                      nav_goal_loc_.x() - robot_loc_.x()) -
  //                                 robot_angle_);
  float curvature = 0.5;//1 / WHEELBASE * tan(steering_angle);
  float distance_to_goal = FreePathLength(robot_loc_,curvature,point_cloud_);
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  drive_msg_.velocity = TimeOptimalControl(distance_to_goal);
  // Get curvature from steering angle
  drive_msg_.curvature = curvature;
  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

// private helper functions go here

Eigen::Vector2f Navigation::PolarToCartesian(float r, float theta) {
  // Convert polar to cartesian coordinates
  return Eigen::Vector2f(r * cos(theta), r * sin(theta));
}

Eigen::Vector2f Navigation::CartesianToPolar(float x, float y) {
  // Convert cartesian to polar coordinates
  return Eigen::Vector2f(sqrt(x * x + y * y), atan2(y, x));
}

float Navigation::TimeOptimalControl(float distance) {
  // Calculate how far we would go at the current velocity in 1 time step.
  float distance_at_current_velocity = robot_vel_.norm() * TIME_STEP;
  float distance_to_decel = ((math_util::Sq(robot_vel_.x()) + math_util::Sq(robot_vel_.y())) / (2 * MAX_ACCELERATION));
  float drive_velocity = 0;
  // Accelerate if we are far from the goal.
  if (distance > distance_at_current_velocity && distance > distance_to_decel + STOP_DISTANCE) {
   // Pass in max velocity in terms of the current theta
    drive_velocity = 1;
  }
  // Decelerate if we are close to the goal.
  return drive_velocity;
}

float Navigation::FreePathLength(Eigen::Vector2f current_position, float curvature, std::vector<Eigen::Vector2f> point_cloud) {
  float radius = 1/curvature;
  Eigen::Vector2f min_point;
  float car_w = CAR_WIDTH + MARGIN;
  float car_l = CAR_LENGTH + MARGIN;
  float range_side_l = radius - car_w/2;
  float range_side_r = sqrt(pow(range_side_l,2) + pow((WHEELBASE + car_l)/2,2));
  float range_front_b = sqrt(pow(radius + car_w/2,2) + pow((WHEELBASE + car_l)/2,2));
  // ROS_INFO("%f\t %f\t %f",range_side_l,range_side_r, range_front_b);
  float Min_Theta = 1;
  Eigen::Vector2f min_point_here(0,0);
  // float Min_Theta1 = 360;
  if(curvature == 0){
    float Straight_Free_Path_Length = 10;
    for(unsigned long point = 0; point < point_cloud.size(); point++){
      float point_x = point_cloud[point].x();
      float point_y = point_cloud[point].y();
      if(point_y <= car_w/2 && point_y >= -car_w/2){
        bool is_min = point_x < Straight_Free_Path_Length ;
        Straight_Free_Path_Length = Straight_Free_Path_Length*(!is_min) + point_x*is_min;
      }
    }
    return Straight_Free_Path_Length;
  }else{
    for(unsigned long point = 0; point < point_cloud.size(); point++){
      //convert point cloud to Polar
      //Eigen::Vector2f(radius, Theta)
      float point_x = point_cloud[point].x();
      float point_y = point_cloud[point].y();
      Eigen::Vector2f polar_point = Eigen::Vector2f(sqrt(pow(point_x,2) + pow(point_y - radius,2)), atan2(point_x, point_y - radius) );
      // ROS_INFO("%f \t %f",point_x,point_y);
      if(polar_point.x() >= range_side_l && polar_point.x() <= range_side_r){
        float Theta = acos(range_side_l/polar_point.x());
        float Diff_Theta = abs(polar_point.y() - Theta);
        bool is_min = Diff_Theta < Min_Theta;
        //assign smallest value to min theta.
        Min_Theta = Min_Theta*(!is_min) + Diff_Theta*is_min;
        if(is_min){
          min_point_here = point_cloud[point];
        }
        
      } 
      else if(polar_point.x() <= range_front_b && polar_point.x() > range_side_r){
        float Theta = asin((WHEELBASE + car_l)/2/polar_point.x());
        float Diff_Theta = abs(polar_point.y() - Theta);
        bool is_min = Diff_Theta < Min_Theta;
        //assign smallest value to min theta.
        Min_Theta = Min_Theta*(!is_min) + Diff_Theta*is_min;
        // Min_Theta1 = Min_Theta;
        if(is_min){
          min_point_here = point_cloud[point];
        }
      }

    }
    // ROS_INFO("%f \t %f",min_point_here.x(),min_point_here.y());
  }
  //change this if they are actually in radians
  float Min_Free_Path = (Min_Theta)*abs(radius) - 0.3;
  
  return Min_Free_Path;
  
}
}


  // namespace navigation
