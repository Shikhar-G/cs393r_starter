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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
  // transform the particles to the map frame
  
}


void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(0.2, 0);
  Eigen::Rotation2Df car_angle(angle);
  float angle_step = (angle_max - angle_min)/num_ranges;
  float laser_start_angle = angle + angle_min;
  Vector2f laser_start_loc = loc + car_angle*kLaserLoc;
  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  float scan_angle = laser_start_angle;
  for (size_t i = 0; i < scan.size(); ++i) {
    
    scan[i] = Vector2f(range_max,scan_angle);
    scan_angle += angle_step;
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t j = 0; j < map_.lines.size(); ++j) {
    const line2f map_line = map_.lines[j];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    // line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    // printf("P0: %f, %f P1: %f,%f\n", 
    //        my_line.p0.x(),
    //        my_line.p0.y(),
    //        my_line.p1.x(),
    //        my_line.p1.y());
    bool prev_high = 0;
    bool curr_low = 0;
    for (size_t i = 0; i < scan.size(); ++i) {
      prev_high = curr_low;
      Vector2f laz_vect = geometry::Heading(scan[i].y());
      
      // Check for intersections:
      float distance_squared = 0;
      Vector2f intersect_point;
      bool intersects = geometry::RayIntersect<float>(laser_start_loc,laz_vect,map_line.p0,map_line.p1, &distance_squared, &intersect_point);//map_line.Intersects(my_line);
      curr_low = intersects;
      //only a period j to k in i consecutive increments of the scan will intersect with a line segment, after those individual scans pass the rest will never intersect.
      //created an edge detection algorithm to break when this occurs.
      if(prev_high && !curr_low) break;

      float distance = sqrt(distance_squared);
      // You can also simultaneously check for intersection, and return the point
      // of intersection:
      // Vector2f intersection_point; // Return variable
      // intersects = map_line.Intersection(my_line, &intersection_point);
      if (intersects) {
        if(distance < scan[i].x())
        {
          scan[i].x() = distance;
        }
        printf("Intersects at %f,%f\n", 
               intersect_point.x(),
               intersect_point.y());
      } else {
        printf("No intersection\n");
      }
    }
    
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:


  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //        x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
                  
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  //float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and "
  //        "standard deviation of 2 : %f\n", x);

  // Particle test_prt = MotionModelSample(odom_loc, odom_angle);
  // For each particle, update its location and angle based on the motion model. (GPU exeleration?)
  if (!odom_initialized_) {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }
  for (size_t i = 0; i < particles_.size(); ++i) {
    Particle temp_particle = MotionModelSample(odom_loc, odom_angle);
    // temp_particle.loc = r_odom_map * temp_particle.loc;
    // if (rotation_initialized_) {
    //   temp_particle.loc = r_odom_map * temp_particle.loc;
    // }
    particles_[i].loc.x() = particles_[i].loc.x() + temp_particle.loc.x();
    particles_[i].loc.y() = particles_[i].loc.y() + temp_particle.loc.y();
    particles_[i].angle = math_util::AngleMod(particles_[i].angle + temp_particle.angle);
    particles_[i].weight = temp_particle.weight;
  }
  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;
  // ROS_INFO("loc: %f", particles_[0].loc.x());
}

Particle ParticleFilter::MotionModelSample(const Eigen::Vector2f& odom_loc, const float odom_angle) {
  Eigen::Vector2f transformed_odom_loc = odom_loc;
  Eigen::Vector2f transformed_prev_odom_loc = prev_odom_loc_;
  if (rotation_initialized_) {
    transformed_odom_loc = r_odom_map * odom_loc;
    transformed_prev_odom_loc = r_odom_map * prev_odom_loc_;
  }
  float dx = transformed_odom_loc.x() - transformed_prev_odom_loc.x();
  float dy = transformed_odom_loc.y() - transformed_prev_odom_loc.y();
  float dtheta = math_util::AngleDiff(odom_angle, prev_odom_angle_);

  // sample error from Gaussian distribution
  float dx_error = rng_.Gaussian(0, k1 * sqrt(dx * dx + dy * dy) + k2 * fabs(dtheta));
  float dy_error = rng_.Gaussian(0, k1 * sqrt(dx * dx + dy * dy) + k2 * fabs(dtheta));
  float dtheta_error = rng_.Gaussian(0, k3 * sqrt(dx * dx + dy * dy) + k4 * fabs(dtheta));
  
  Particle out;
  //Only output the d_ +  error , the error compounds over time however
  out.loc = Vector2f(dx + dx_error, dy + dy_error);  
  out.loc = Vector2f(dx, dy);
  out.angle = dtheta + dtheta_error;
  out.angle = dtheta;
  out.weight = 1.0 / FLAGS_num_particles;


  return out;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  // Set all particles to the provided location and angle.
  particles_.resize(FLAGS_num_particles);
  for (size_t i = 0; i < particles_.size(); ++i) {
    particles_[i].loc = loc;
    particles_[i].angle = angle;
    particles_[i].weight = 1.0 / FLAGS_num_particles;
  }
  // save transform from odom to this location and angle
  if (odom_initialized_) {
    // get rotation matrix from odom to given angle
    r_odom_map = Eigen::Rotation2Df(math_util::AngleMod(angle - prev_odom_angle_ - 0.05));
    rotation_initialized_ = true;
    ROS_INFO("rotation initialized %f", r_odom_map.angle());
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  // Vector2f& loc = *loc_ptr;
  // float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  // loc = Vector2f(0, 0);
  // angle = 0;
}


}  // namespace particle_filter
