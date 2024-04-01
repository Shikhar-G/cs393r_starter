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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "vector_map/vector_map.h"

#include "simple_queue.h"

#include "global_planner.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros
{
  class NodeHandle;
} // namespace ros

namespace navigation
{

  struct PathOption
  {
    float curvature;
    float clearance;
    float free_path_length;
    Eigen::Vector2f obstruction;
    Eigen::Vector2f closest_point;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  struct RobotState
  {
    Eigen::Vector2f loc;
    float angle;
    Eigen::Vector2f vel;
    float omega;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class Navigation
  {
  public:
    const float MAX_CURVATURE = 1.0;
    // Max velocity constant
    const float MAX_VELOCITY = 1.0;
    // Max acceleration constant
    const float MAX_ACCELERATION = 4.0;
    // stop distance buffer from point of interest
    const float STOP_DISTANCE = 0.15;
    // Time step constant
    const float TIME_STEP = 0.05;
    // Wheelbase constant
    const float WHEELBASE = 0.324;
    // Car length
    const float CAR_LENGTH = 0.535;
    // car width
    const float CAR_WIDTH = 0.281;
    // obstacle safety margin
    const float MARGIN = 0.1;
    // system latency
    const float LATENCY = 0.125;
    // Constructor
    explicit Navigation(const std::string &map_file, ros::NodeHandle *n);

    // Used in callback from localization to update position.
    void UpdateLocation(const Eigen::Vector2f &loc, float angle);

    // Used in callback for odometry messages to update based on odometry.
    void UpdateOdometry(const Eigen::Vector2f &loc,
                        float angle,
                        const Eigen::Vector2f &vel,
                        float ang_vel);

    // Updates based on an observed laser scan
    void ObservePointCloud(const std::vector<Eigen::Vector2f> &cloud,
                           double time);

    // Main function called continously from main
    void Run();
    // Used to set the next target pose.
    void SetNavGoal(const Eigen::Vector2f &loc, float angle);

  private:
    // simple_
    // Whether odometry has been initialized.
    bool odom_initialized_;
    // Whether localization has been initialized.
    bool localization_initialized_;
    // Current robot location.
    Eigen::Vector2f robot_loc_;
    // Current robot orientation.
    float robot_angle_;
    // Current robot velocity.
    Eigen::Vector2f robot_vel_;
    // Current robot angular speed.
    float robot_omega_;
    // Odometry-reported robot location.
    Eigen::Vector2f odom_loc_;
    // Odometry-reported robot angle.
    float odom_angle_;
    // Odometry-reported robot starting location.
    Eigen::Vector2f odom_start_loc_;
    // Odometry-reported robot starting angle.
    float odom_start_angle_;
    // Latest observed point cloud.
    std::vector<Eigen::Vector2f> point_cloud_;
    // SimpleQueue to store the controls issued # needs to be fixed
    // SimpleQueue<std::pair<std::vector<Eigen::Vector2f, float>, float>> control_queue_;
    // Whether navigation is complete.
    bool nav_complete_;
    // Navigation goal location.
    Eigen::Vector2f nav_goal_loc_;
    // Navigation goal angle.
    float nav_goal_angle_;
    // Map of the environment.
    vector_map::VectorMap map_;
    // 1D time-optimal control path option, given distance to go, returns a velocity to execute.
    float TimeOptimalControl(float distance);
    // Convert cartesian to polar coordinates
    Eigen::Vector2f CartesianToPolar(float x, float y);
    // Convert polar to cartesian coordinates
    Eigen::Vector2f PolarToCartesian(float r, float theta);
    // calculate free path length for 1 curve
    float FreePathLength(float curvature, const std::vector<Eigen::Vector2f> &point_cloud);
    // along a curve or straight line the closest point to that curve or line.
    float ClosestPointApproach(float curvature, const std::vector<Eigen::Vector2f> &point_cloud);
    // calculate the forwarded prediction change in location of the robot
    Eigen::Vector2f ForwardPredictedLocationChange();
    // transform point cloud according to time delay
    std::vector<Eigen::Vector2f> TransformPointCloud(const std::vector<Eigen::Vector2f> &cloud, Eigen::Vector2f locChange);

    float ScorePaths(float closest_approach, float free_path_length, float distance_to_goal);

    Eigen::Vector2f DistanceToGoal(float curvature);
  
    // weights for scoring paths
    float w1_ = 1.0;
    float w2_ = 0.25;
    float w3_ = 0.5;


    Eigen::Vector2f GetNextLocalGoal();

    //path planning
    planner::RRT_Star global_planner_; 
    std::vector<Eigen::Vector2f> path_;
    // Local goal location.
    Eigen::Vector2f local_goal_loc_;
    // Current path index.
    size_t path_index_;
    // Carrot radius for the simple carrot follower.
    float carrot_radius_ = 0.5;

    void PublishGlobalPlanner();
  };

} // namespace navigation

#endif // NAVIGATION_H
