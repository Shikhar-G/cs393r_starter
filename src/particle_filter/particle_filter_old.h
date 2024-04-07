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
\file    particle-filter.h
\brief   Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"

//ros testing
#include "ros/ros.h"
#include "ros/package.h"

#ifndef SRC_PARTICLE_FILTER_H_
#define SRC_PARTICLE_FILTER_H_

namespace particle_filter {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

class ParticleFilter {
 public:
  // Default Constructor.
   ParticleFilter();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Predict particle motion based on odometry.
  void Predict(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Initialize the robot location.
  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle) const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              Particle* p);

  // Resample particles.
  void Resample();

  // For debugging: get predicted point cloud from current location.
  void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                              const float angle,
                              int num_ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              std::vector<Eigen::Vector2f>* scan,
                              bool cartesian = true);

 private:

  // List of particles being tracked.
  std::vector<Particle> particles_;

  // Map of the environment.
  vector_map::VectorMap map_;
  // Current eligible lines of the map;
  std::vector<geometry::line2f> eligible_lines;
  bool map_lines_initialized_;

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;


  // the rotation matrix from odom to map
  Eigen::Rotation2Df r_odom_map;
  bool rotation_initialized_;
  
  //particle sample, in the update step
    //CONSTANTS
  float k1 = 0.6; //distance error
  float k2 = 0.2; //rotation variance in distance error
  float k3 = 0.2;//distance variance in rotation error
  float k4 = 0.2; //rotation error
    //helper function: motion model sample. It takes in odom and a particle and outputs a prediction.
  Particle MotionModelSample(const Eigen::Vector2f& odom_loc, const float odom_angle);
  // Wheelbase constant
    const float WHEELBASE = 0.324;
  //particle filter update params:
  float std_dev_scan = 0.03;
  float std_dev_scan_sq = math_util::Sq(std_dev_scan);
  float gamma_update = 0.7;
  float d_short = 0.3;
  float d_long = 0.8;

  //resample counters
  int n_resample = 1;
  int n_resample_count = 0;

  // distance tracker
  float dist_traveled = 0;
  float set_distance = 0.1;
};
}  // namespace slam

#endif   // SRC_PARTICLE_FILTER_H_
