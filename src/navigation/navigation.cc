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

using visualization::ClearVisualizationMsg;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::DrawLine;
using visualization::DrawParticle;

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
    //init global planner using map
    global_planner_ = planner::Informed_RRT_Star(&map_);
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
    // Reset any existing navigation goal and reset indicator variables.
    // Set the navigation goal.
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
    // Get planned path (another callback).
    Eigen::Vector2f start_loc(robot_loc_.x() + cos(robot_angle_), robot_loc_.y() + sin(robot_angle_));
    global_planner_.SetStart(start_loc);
    global_planner_.SetGoal(nav_goal_loc_);
    global_planner_.SetPointCloud(point_cloud_global_);
    bool path_found = global_planner_.Plan();
    if (path_found)
    {
      path_.clear();
      path_ = global_planner_.GetPath();
      nonsmooth_path_ = global_planner_.GetPath(false);
      path_index_ = 0;
      nav_complete_ = false;
      ROS_INFO("Path size: %ld", path_.size());
      SetNextLocalGoal();
    }
    else
    {
      ROS_INFO("No path found. Please try again.");
    }
  }

  void Navigation::SetNextLocalGoal() 
  {
    // check each line segment from current path index to the end of the path for the first point that intersects the carrot radius
    for (size_t i = path_index_; i < path_.size() - 1; i++)
    {
      Eigen::Vector2f end = path_[i + 1];
      float dist_to_end = (end - robot_loc_).norm();
      if (dist_to_end <= CARROT_RADIUS)
      {
        local_goal_loc_ = end;
        path_index_ = i + 1;
        return;
      }
    }
    ROS_INFO("Could not find intersection");
    nav_complete_ = true;
  }

  void Navigation::RecoverFromFailure() {
    //if the robot is stuck first send all points to informed RRT then replan*
    ROS_INFO("Replanning...");
    SetNavGoal(nav_goal_loc_, nav_goal_angle_);
  }
  void Navigation::PublishGlobalPlanner() {
  const uint32_t kColor = 0x702963;

  // vector<pair<Vector2f, Vector2f>> tree = global_planner_.GetTree();
  // for (size_t i = 0; i < tree.size(); ++i) {
  //   DrawLine(tree[i].first,
  //            tree[i].second,
  //            kColor,
  //            global_viz_msg_);
  // }
    for (size_t i = 0; i < path_.size() - 1; ++i)
    {
      DrawLine(path_[i],
               path_[i + 1],
               kColor,
               global_viz_msg_);
    }
    // for (size_t i = 0; i < nonsmooth_path_.size() - 1; ++i)
    // {
    //   DrawLine(nonsmooth_path_[i],
    //            nonsmooth_path_[i + 1],
    //            0x0000FF,
    //            global_viz_msg_);
    // }
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
    point_cloud_global_.clear();

    for (size_t i = 0; i < point_cloud_.size(); i++)
    {
      // transform point cloud to global frame relative to robot location and angle
      float x = point_cloud_[i].x() * cos(robot_angle_) - point_cloud_[i].y() * sin(robot_angle_) + robot_loc_.x();
      float y = point_cloud_[i].x() * sin(robot_angle_) + point_cloud_[i].y() * cos(robot_angle_) + robot_loc_.y();
      point_cloud_global_.push_back(Vector2f(x, y));
      DrawPoint(Vector2f(x, y), 0xFF0000, global_viz_msg_);
    }
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

    // Don't move if no path
  if (path_.empty() || nav_complete_)
      return;

    Eigen::Vector2f next_robot_loc = robot_loc_ + ForwardPredictedLocationChange();
    DrawPoint(local_goal_loc_, 0x0000FF, global_viz_msg_);
    // Do we need to replan?
    if ((next_robot_loc - local_goal_loc_).norm() < 1.5) {
      global_planner_.SetPointCloud(point_cloud_global_);
      bool path_valid = global_planner_.isPathValid(path_index_);
      if (!path_valid) {
        ROS_INFO("Path is now invalid");
        drive_msg_.velocity = 0;
        drive_msg_.curvature = 0;
        RecoverFromFailure();
        return;
      }
      // goal reached
      if (local_goal_loc_ == nav_goal_loc_) {
        if ((next_robot_loc - local_goal_loc_).norm() < 0.5) {
          drive_msg_.velocity = 0;
          drive_msg_.curvature = 0;
          path_.clear();
          nav_complete_ = true;
          path_index_ = 0;
          return;
        } 
      }
      else {
        path_index_++;
        local_goal_loc_ = path_[path_index_];
      }
    }
    else if ((next_robot_loc - local_goal_loc_).norm() > CARROT_RADIUS * 1.5) {
      drive_msg_.velocity = 0;
      drive_msg_.curvature = 0;
      RecoverFromFailure();
    }

    // std::vector<Eigen::Vector2f> last_point_cloud_ = point_cloud_;
    // The control iteration goes here.
    // Feel free to make helper functions to structure the control appropriately.
    std::vector<Eigen::Vector2f> transformed_cloud = TransformPointCloud(point_cloud_, ForwardPredictedLocationChange());
    float curvature_interval = 0.1;
    float best_curvature = 0;
    float distance_to_travel = 0;
    float score = -100000;
    std::vector<float> fpls;
    std::vector<float> approaches;
    std::vector<float> dtgs;
    for(int i = 0; i < 20; i++){
      float dis = FreePathLength(-1 + i*curvature_interval, transformed_cloud);
      float approach = ClosestPointApproach(-1 + i*curvature_interval,transformed_cloud);
      float dtg_score = DistanceToGoalScore(-1 + i*curvature_interval);
      fpls.push_back(dis);
      approaches.push_back(approach);
      dtgs.push_back(dtg_score);
    }

    // normalize all vectors
    float max_fpl = *std::max_element(fpls.begin(), fpls.end());
    float min_fpl = *std::min_element(fpls.begin(), fpls.end());
    float max_approach = *std::max_element(approaches.begin(), approaches.end());
    float min_approach = *std::min_element(approaches.begin(), approaches.end());
    float max_dtg = *std::max_element(dtgs.begin(), dtgs.end());
    float min_dtg = *std::min_element(dtgs.begin(), dtgs.end());
    for(int i = 0; i < 20; i++){
      // ensure that the path is valid
      float raw_fpl = fpls[i];
      if (fpls[i] < MARGIN || approaches[i] < MARGIN) continue;
      if (max_fpl - min_fpl == 0) fpls[i] = 0;
      else fpls[i] = (fpls[i] - min_fpl)/(max_fpl - min_fpl);
      if (max_approach - min_approach == 0) approaches[i] = 0;
      else approaches[i] = (approaches[i] - min_approach)/(max_approach - min_approach);
      if (max_dtg - min_dtg == 0) dtgs[i] = 0;
      else dtgs[i] = (dtgs[i] - min_dtg)/(max_dtg - min_dtg);
      float total_score = ScorePaths(approaches[i],fpls[i],dtgs[i]);
      if(total_score > score){
        score = total_score;
        best_curvature = -1 + i*curvature_interval;
        distance_to_travel = raw_fpl;
      }
    }
    // no valid paths
    if (score == -100000){
      drive_msg_.velocity = 0;
      drive_msg_.curvature = 0;
      ROS_INFO("No valid paths");
      RecoverFromFailure();
    }
    else {
      float curvature = best_curvature;
      // The latest observed point cloud is accessible via "point_cloud_"
      drive_msg_.velocity = TimeOptimalControl(distance_to_travel);
      drive_msg_.curvature = curvature;
    }
      // Add timestamps to all messages.
    ClearVisualizationMsg(global_viz_msg_);
    // for(size_t i= 0; i < point_cloud_global_.size(); i++){
    //   DrawPoint(point_cloud_global_[i], 0xFF0000, global_viz_msg_);
    // }
    PublishGlobalPlanner();
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
    if ((int)(curvature  * 100) == 0)
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

    if ((int)(curvature  * 100) == 0)
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

    return std::min(min_fpl,(float)10.0);
  }

  float Navigation::ClosestPointApproach(float curvature, const std::vector<Eigen::Vector2f> &point_cloud){
    float radius;
    float closest_point = 3;
    float max_range = 5;
    float car_w = CAR_WIDTH + 2 * MARGIN;
    if ((int) (curvature * 100) == 0)
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
  
  float Navigation::DistanceToGoalScore(float curvature) {
    float radius;
    if ((int) (curvature * 100) == 0) {
      radius = 0;
    } else {
      radius = 1 / curvature;
    }
    float curr_velocity = 1;
    float next_x;
    float next_y;
    // calculate steering angle based on curvature
    if (radius == 0) {
      next_x = robot_loc_.x() + curr_velocity * 2;
      next_y = robot_loc_.y() + curr_velocity * 2;
    }
    else {
      // new theta_swept
      float theta_swept = curr_velocity * 2 / radius;
      float dx = radius * sin(theta_swept);
      float dy = radius * (1 - cos(theta_swept));
      // translate dx and dy to robot's location on map frame
      float dx_map = dx * cos(robot_angle_) - dy * sin(robot_angle_);
      float dy_map = dx * sin(robot_angle_) + dy * cos(robot_angle_);
      // calculate next x and y based on steering angle and current velocity
      next_x = robot_loc_.x() + dx_map;
      next_y = robot_loc_.y() + dy_map;
    }
    float score = 1/(pow(local_goal_loc_.x() - next_x, 2) + pow(local_goal_loc_.y() - next_y, 2));
    return score;
  }
  
  float Navigation::ScorePaths(float closest_approach, float free_path_length, float distance_to_goal_score){
    // ROS_INFO("%f %f %f",free_path_length,closest_approach,distance_to_goal_score);
    return free_path_length*w1_ + closest_approach*w2_ + distance_to_goal_score*w3_;
  }

  Eigen::Vector2f Navigation::ForwardPredictedLocationChange() {
    float dx = robot_vel_.x() * LATENCY;
    float dy = robot_vel_.y() * LATENCY; 
    return Eigen::Vector2f(dx, dy);
  }

  std::vector<Eigen::Vector2f> Navigation::TransformPointCloud(const std::vector<Eigen::Vector2f> &cloud, Eigen::Vector2f locChange) {
    std::vector<Eigen::Vector2f> transformed_cloud;
    transformed_cloud.resize(0);
    for (unsigned long i = 0; i < cloud.size(); i++) {
      transformed_cloud.push_back(cloud[i] - locChange);
    }
    return transformed_cloud;
  }

// namespace navigation
}

