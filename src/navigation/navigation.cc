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

using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using Eigen::Vector2f;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace
{
  ros::Publisher drive_pub_;
  ros::Publisher viz_pub_;
  VisualizationMsg local_viz_msg_;
  VisualizationMsg global_viz_msg_;
  AckermannCurvatureDriveMsg drive_msg_;
  // Epsilon value for handling limited numerical precision.
  const float kEpsilon = 1e-5;
} // namespace

namespace navigation
{

  string GetMapFileFromName(const string &map)
  {
    string maps_dir_ = ros::package::getPath("amrl_maps");
    return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
  }

  Navigation::Navigation(const string &map_name, ros::NodeHandle *n) : odom_initialized_(false),
                                                                       localization_initialized_(false),
                                                                       robot_loc_(0, 0),
                                                                       robot_angle_(0),
                                                                       robot_vel_(0, 0),
                                                                       robot_omega_(0),
                                                                       nav_complete_(true),
                                                                       nav_goal_loc_(0, 0),
                                                                       nav_goal_angle_(0)
  {
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

  void Navigation::SetNavGoal(const Vector2f &loc, float angle)
  {
    // Set the navigation goal.
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
    nav_complete_ = false;
  }

  void Navigation::UpdateLocation(const Eigen::Vector2f &loc, float angle)
  {
    localization_initialized_ = true;
    robot_loc_ = loc;
    robot_angle_ = angle;
  }

  void Navigation::UpdateOdometry(const Vector2f &loc,
                                  float angle,
                                  const Vector2f &vel,
                                  float ang_vel)
  {
    robot_omega_ = ang_vel;
    robot_vel_ = vel;
    if (!odom_initialized_)
    {
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

  void Navigation::ObservePointCloud(const vector<Vector2f> &cloud,
                                     double time)
  {
    point_cloud_ = cloud;
  }

  void Navigation::Run()
  {
    // This function gets called 20 times a second to form the control loop.

    // Clear previous visualizations.
    visualization::ClearVisualizationMsg(local_viz_msg_);
    visualization::ClearVisualizationMsg(global_viz_msg_);

    // If odometry has not been initialized, we can't do anything.
    if (!odom_initialized_)
      return;

    // std::vector<Eigen::Vector2f> last_point_cloud_ = point_cloud_;
    // The control iteration goes here.
    // Feel free to make helper functions to structure the control appropriately.
    std::vector<Eigen::Vector2f> transformed_cloud = TransformPointCloud(point_cloud_, ForwardPredictedLocationChange());
    vector<float> curvatures;
    for(float i = -1; i <= 1; i += 0.1){
      curvatures.push_back(i);
    }
    float curvature = 0;
    float distance_to_goal = 0;
    float approach = 0;
    float score = 0;
    unsigned long best_score;
    for(unsigned long i = 0; i < curvatures.size(); i++){
      float dis = FreePathLength(curvatures[i], transformed_cloud);
      approach = ClosestPointApproach(curvatures[i],transformed_cloud);
      float curr_score = ScorePaths(approach,dis,0.25);
      if(score < curr_score){
        score = curr_score;
        best_score = i;
        distance_to_goal = dis;
      }
    }
    curvature = curvatures[best_score];
    // float distance_to_goal = FreePathLength(curvature, point_cloud_);
    // float approach = ClosestPointApproach(curvature,point_cloud_);
    // float score = ScorePaths(approach,distance_to_goal,0.25);
    // ROS_INFO("%f\t%f",approach,score);
    // The latest observed point cloud is accessible via "point_cloud_"
    drive_msg_.velocity = TimeOptimalControl(distance_to_goal);
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

  Eigen::Vector2f Navigation::PolarToCartesian(float r, float theta)
  {
    // Convert polar to cartesian coordinates
    return Eigen::Vector2f(r * cos(theta), r * sin(theta));
  }

  Eigen::Vector2f Navigation::CartesianToPolar(float x, float y)
  {
    // Convert cartesian to polar coordinates
    return Eigen::Vector2f(sqrt(x * x + y * y), atan2(y, x));
  }

  float Navigation::TimeOptimalControl(float distance)
  {
    // Calculate how far we would go at the current velocity in 1 time step.
    float distance_at_current_velocity = robot_vel_.norm() * TIME_STEP;
    float distance_to_decel = ((math_util::Sq(robot_vel_.x()) + math_util::Sq(robot_vel_.y())) / (2 * MAX_ACCELERATION));
    float drive_velocity = 0;
    // Accelerate if we are far from the goal.
    if (distance > distance_at_current_velocity && distance > distance_to_decel + STOP_DISTANCE)
    {
      // Pass in max velocity in terms of the current theta
      drive_velocity = 1;
    }
    // Decelerate if we are close to the goal.
    return drive_velocity;
  }

  float Navigation::FreePathLength(float curvature, const std::vector<Eigen::Vector2f> &point_cloud)
  {
    float radius;
    if (curvature == 0)
    {
      radius = 0;
    }
    else
    {
      radius = abs(1 / curvature);
    }
    Eigen::Vector2f min_point;
    float car_w = CAR_WIDTH + 2 * MARGIN;
    float car_l = CAR_LENGTH + 2 * MARGIN;
    float range_side_l = radius - car_w / 2;
    float range_side_r = sqrt(pow(range_side_l, 2) + pow((WHEELBASE + car_l) / 2, 2));
    float range_front_b = sqrt(pow(radius + car_w / 2, 2) + pow((WHEELBASE + car_l) / 2, 2));

    float min_theta = M_PI / 2;

    if (curvature == 0)
    {
      float straight_fpl = 10;
      for (unsigned long point = 0; point < point_cloud.size(); point++)
      {
        float point_x = point_cloud[point].x();
        float point_y = point_cloud[point].y();
        if (point_y <= car_w / 2 && point_y >= -car_w / 2)
        {
          bool is_min = point_x < straight_fpl;
          straight_fpl = straight_fpl * (!is_min) + point_x * is_min;
        }
      }
      //subtract by the distance to the front of the vehicle before return
      return straight_fpl - (WHEELBASE + car_l)/2;
    }
    else
    {
      for (unsigned long point = 0; point < point_cloud.size(); point++)
      {
        // convert point cloud to Polar
        // Eigen::Vector2f(radius, theta)
        float point_x = point_cloud[point].x();
        float point_y = point_cloud[point].y();
        Eigen::Vector2f polar_point;
        if (curvature > 0) {
          polar_point = Eigen::Vector2f(sqrt(pow(point_x, 2) + pow(point_y - radius, 2)), atan2(point_y - radius, point_x) + M_PI / 2);
        }
        else {
          polar_point = Eigen::Vector2f(sqrt(pow(point_x, 2) + pow(point_y + radius, 2)), M_PI/2 - atan2(point_y + radius, point_x) );
        }
        // discard points that are behind the car
        if (polar_point.y() < -M_PI / 2 || polar_point.y() > M_PI / 2)
        {
          continue;
        }

        if (polar_point.x() >= range_side_l && polar_point.x() <= range_side_r)
        {
          float theta = acos(range_side_l / polar_point.x());
          float diff_theta = abs(polar_point.y() - theta);
          bool is_min = diff_theta < min_theta;
          // assign smallest value to min theta.
          min_theta = min_theta * (!is_min) + diff_theta * is_min;
        }
        else if (polar_point.x() <= range_front_b && polar_point.x() > range_side_r)
        {
          float theta = asin((WHEELBASE + car_l) / 2 / polar_point.x());
          float diff_theta = abs(polar_point.y() - theta);
          bool is_min = diff_theta < min_theta;
          // assign smallest value to min theta.
          min_theta = min_theta * (!is_min) + diff_theta * is_min;
        }
      }
    }
    float min_fpl = (min_theta)*abs(radius);

    return min_fpl;
  }

  float Navigation::ClosestPointApproach(float curvature, const std::vector<Eigen::Vector2f> &point_cloud){
    float radius;
    float closest_point = 3;
    float max_range = 5;
    float car_w = CAR_WIDTH + 2 * MARGIN;
    if (curvature == 0)
    {
      radius = 0;
    }
    else
    {
      radius = 1 / curvature;
    }

    if(radius == 0){
      for(unsigned long point = 0; point < point_cloud.size(); point++){
        if(point_cloud[point].y() < closest_point && point_cloud[point].x() <= max_range){
          closest_point = abs(point_cloud[point].y());
        }
      }
    } else {
      Eigen::Vector2f offset(0,-radius);
      for(unsigned long point = 0; point < point_cloud.size(); point++){
        //skip points that are outside 90 degree curve max
        if((radius > 0) && (point_cloud[point].x() < 0 || point_cloud[point].y() < -car_w/2)) continue; //turning counterclockwise, skip points
        else if(point_cloud[point].x() < 0 || point_cloud[point].y() > car_w/2) continue; //turning clockwise, skip points

        Eigen::Vector2f pt_orig = point_cloud[point] + offset;
        float distance = abs(pt_orig.norm() - radius);
        if (distance < closest_point){
          closest_point = distance;
        }
      }
    }
    return closest_point;
  }

  float Navigation::ScorePaths(float closest_approach, float free_path_length, float w1) {
    return free_path_length + closest_approach*w1;
  }

  Eigen::Vector2f Navigation::ForwardPredictedLocationChange() {
    float dx = robot_vel_.x() * LATENCY * cos(odom_angle_);
    float dy = robot_vel_.y() * LATENCY * sin(odom_angle_);
    return Eigen::Vector2f(dx, dy);
  }

  std::vector<Eigen::Vector2f> Navigation::TransformPointCloud(const std::vector<Eigen::Vector2f> &cloud, Eigen::Vector2f locChange) {
    std::vector<Eigen::Vector2f> transformed_cloud;
    transformed_cloud.resize(0);
    for (unsigned long i = 0; i < cloud.size(); i++) {
      transformed_cloud.push_back(cloud[i] + locChange);
    }
    return transformed_cloud;
  }

}

// namespace navigation
