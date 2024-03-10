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

using Eigen::Vector2f;
using Eigen::Vector2i;
using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

DEFINE_double(num_particles, 25, "Number of particles");

namespace particle_filter
{

  config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

  ParticleFilter::ParticleFilter() : prev_odom_loc_(0, 0),
                                     prev_odom_angle_(0),
                                     odom_initialized_(false) {}

  void ParticleFilter::GetParticles(vector<Particle> *particles) const
  {
    *particles = particles_;
  }

  void ParticleFilter::GetPredictedPointCloud(const Vector2f &loc,
                                              const float angle,
                                              int num_ranges,
                                              float range_min,
                                              float range_max,
                                              float angle_min,
                                              float angle_max,
                                              vector<Vector2f> *scan_ptr,
                                              bool cartesian)
  {
    vector<Vector2f> &scan = *scan_ptr;
    // Compute what the predicted point cloud would be, if the car was at the pose
    // loc, angle, with the sensor characteristics defined by the provided
    // parameters.
    // This is NOT the motion model predict step: it is the prediction of the
    // expected observations, to be used for the update step.

    // Location of the laser on the robot. Assumes the laser is forward-facing.
    const Vector2f kLaserLoc(0.2, 0);
    Eigen::Rotation2Df car_angle(angle);
    float angle_step = (angle_max - angle_min) / num_ranges;
    //this is the beginning of the predicted lidar scan which starts at angle_min! keep it this way
    float laser_start_angle = angle + angle_min;
    // float laser_end_angle = angle + angle_max;
    Vector2f laser_start_loc = loc + car_angle * kLaserLoc;
    // Note: The returned values must be set using the `scan` variable:
    scan.resize(num_ranges);
    // Fill in the entries of scan using array writes, e.g. scan[i] = ...
    float scan_angle = laser_start_angle;
    vector<float> range_ptr;
    // map_.GetPredictedScan(laser_start_loc, range_min, range_max, laser_start_angle, laser_start_angle + angle_max, num_ranges, &range_ptr);
    // for (size_t i = 0; i < range_ptr.size(); ++i)
    // {
    //   if (cartesian)
    //   {
    //     scan[i].x() = range_ptr[i] * cos(laser_start_angle + i * angle_step);
    //     scan[i].y() = range_ptr[i] * sin(laser_start_angle + i * angle_step);
    //   }
    //   else
    //   {
    //     scan[i] = Vector2f(range_ptr[i], laser_start_angle + i * angle_step);
    //   }
    
    if (dist_traveled > set_distance || eligible_lines.size() == 0)                    
    {
      map_.GetSceneLines(laser_start_loc, range_max, &eligible_lines);
    }
    // vector<line2f> local_map_lines = eligible_lines;

    for (size_t i = 0; i < scan.size(); ++i)
    {
      scan[i].x() = laser_start_loc.x() + range_max * cos(scan_angle);
      scan[i].y() = laser_start_loc.y() + range_max * sin(scan_angle);
      // find intersection with map
      Vector2f min_intersect = scan[i];
      float min_distance = range_max;
      for (size_t j = 0; j < eligible_lines.size(); ++j) {
        const line2f map_line = eligible_lines[j];
        line2f my_line(laser_start_loc, scan[i]);
        Vector2f intersect_point;
        bool intersects = map_line.Intersection(my_line, &intersect_point);
        if (intersects)
        {
          float distance = (intersect_point - laser_start_loc).norm();
          if (distance < min_distance && distance >= range_min)
          {
            min_distance = distance;
            min_intersect = intersect_point;
          }
        }
      }
      if (cartesian)
      {
        scan[i] = min_intersect;
      }
      else
      {
        scan[i] = Vector2f(min_distance, scan_angle);
      }
      scan_angle += angle_step;
    }
  }

  void ParticleFilter::Update(const vector<float> &ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              Particle *p_ptr)
  {
    // Implement the update step of the particle filter here.
    // You will have to use the `GetPredictedPointCloud` to predict the expected
    // observations for each particle, and assign weights to the particles based
    // on the observation likelihood computed by relating the observation to the
    // predicted point cloud.
    Vector2f part_loc = p_ptr->loc;
    float part_angle = p_ptr->angle;

    size_t range_size = ranges.size();
    int range_div = 2;
    vector<Vector2f> predicted_scan;
    vector<Vector2f> predicted_scan_false;
    GetPredictedPointCloud(part_loc, part_angle, range_size/range_div, range_min, range_max, angle_min, angle_max, &predicted_scan, false);
    double Sum_log_pdf = 0;
    // bool bad_particle = false;
    for (size_t i = 0; i < range_size; i+=range_div)
    {
      //
      // if (predicted_scan[i/range_div].x() <= TOO_CLOSE_RANGE){
      //   bad_particle = true;
      // }
      if (ranges[i] > range_max || ranges[i] < range_min || predicted_scan[i/range_div].x() >= range_max || predicted_scan[i/range_div].x() <= range_min){
        continue;
      } else if(ranges[i] < predicted_scan[i/range_div].x() - d_short) {
        Sum_log_pdf += math_util::Sq(d_short) / (std_dev_scan_sq);
      } else if(ranges[i] > predicted_scan[i/range_div].x() + d_long){
        Sum_log_pdf += math_util::Sq(d_long) / (std_dev_scan_sq);
      } else {
        Sum_log_pdf += math_util::Sq(predicted_scan[i/range_div].x() - ranges[i]) / (std_dev_scan_sq);
      }
      
      
      // ROS_INFO("ranges[i]: %f", ranges[i]);
      // ROS_INFO("x for true: %f", predicted_scan[i].x());
    }
    Sum_log_pdf *= -gamma_update;

    p_ptr->weight = p_ptr->weight + Sum_log_pdf;
    // if(bad_particle){
    //   // bad_particles_.push_back(p_ptr);
    // }else {
    //   // good_particles_.push_back(p_ptr);
    // }
  }

  void ParticleFilter::Resample()
  {
    // Resample the particles, proportional to their weights.
    // The current particles are in the `particles_` variable.
    // Create a variable to store the new particles, and when done, replace the
    // old set of particles:
    vector<Particle> new_particles;
    vector<double> weights;
    double weight_sum = 0;
    for (size_t i = 0; i < FLAGS_num_particles; ++i)
    {
      weight_sum += exp(particles_[i].weight);
      weights.push_back(weight_sum);
    }
    // choose a random starting location
    double start = rng_.UniformRandom(0, weight_sum);
    double step = weight_sum / FLAGS_num_particles;
    // find which particle corresponds to the random starting location
    int j = 0;
    double left_bound = 0;
    double right_bound = weights[0];
    double curr_weight = start;
    // find the starting location
    while (start > right_bound || start < left_bound)
    {
      j++;
      left_bound = right_bound;                                       
      right_bound = weights[j];
    }
    // resampling
    for (size_t i = 0; i < FLAGS_num_particles; ++i)
    {
      // particles_[j].weight = log(1.0 / FLAGS_num_particles);
      new_particles.push_back(particles_[j]);
      // go forward by step
      curr_weight += step;
      // wrap around
      if (curr_weight > weight_sum)
      {
        curr_weight -= weight_sum;
        j = 0;
        left_bound = 0;
        right_bound = weights[0];
      }
      while (curr_weight > right_bound || curr_weight < left_bound)
      {
        j++;
        left_bound = right_bound;
        right_bound = weights[j];
      }

    }
    particles_ = new_particles;

  }

  void ParticleFilter::ObserveLaser(const vector<float> &ranges,
                                    float range_min,
                                    float range_max,
                                    float angle_min,
                                    float angle_max)
  {
    // A new laser scan observation is available (in the laser frame)
    // Call the Update and Resample steps as necessary.
    if (dist_traveled < set_distance) {return;}
    float w_max = -INFINITY;
    // int w_min_index = 0;
    float w_min = INFINITY;
    //update the particles
    // printf("gets to line 268");
    vector<Particle> temp_particles = particles_;
    for (size_t i = 0; i < temp_particles.size(); ++i)
    {
      Update(ranges, range_min, range_max, angle_min, angle_max, &temp_particles[i]);
    }
    // printf("gets to line 273");
    //update has found the good and bad particles, change the bad particles to a random good particle.
    // for (size_t i = 0; i < bad_particles_.size() && good_particles_.size() > 0; ++i)
    // {
    //   Particle good_particle = *good_particles_[rng_.RandomInt(0,(int)(good_particles_.size()) - 1)];
    //   bad_particles_[i] -> loc = good_particle.loc;
    //   bad_particles_[i] -> angle = good_particle.angle;
    //   bad_particles_[i] -> weight = good_particle.weight;

    // }
    // good_particles_.clear();
    // bad_particles_.clear();
    //find the min and max values for weights
    for (size_t i = 0; i < temp_particles.size(); ++i)
    {
      if (temp_particles[i].weight > w_max)
      {
        w_max = temp_particles[i].weight;
      }
      if (temp_particles[i].weight < w_min)
      {
        w_min = temp_particles[i].weight;
        // w_min_index = i;
      }
    }

    float particles_sum = 0;
    // normalize weights
    for (size_t i = 0; i < temp_particles.size(); ++i)
    {
      temp_particles[i].weight = temp_particles[i].weight - w_max;
      particles_sum += temp_particles[i].weight;
    }
    //assign temp to actual
    particles_ = temp_particles;

    float particles_average = particles_sum/temp_particles.size();
    float range_center = (w_max + w_min) /2;
    float diff = abs(particles_average - range_center);
    float total_part_range = abs(w_max - w_min);
    float diff_over_total = diff/total_part_range;
    // printf("diff_over_total:%f\n", diff_over_total);
    if(n_resample_count > n_resample && diff_over_total > 0.3 && diff_over_total < 4) {Resample(); n_resample_count = 0;}
    n_resample_count++;
    dist_traveled = 0;

    
  }

  void ParticleFilter::Predict(const Vector2f &odom_loc,
                               const float odom_angle)
  {

    // Implement the predict step of the particle filter here.
    // A new odometry value is available (in the odom frame)
    // Implement the motion model predict step here, to propagate the particles
    // forward based on odometry.

    // You will need to use the Gaussian random number generator provided. For
    // example, to generate a random number from a Gaussian with mean 0, and
    // standard deviation 2:
    // float x = rng_.Gaussian(0.0, 2.0);
    // printf("Random number drawn from Gaussian distribution with 0 mean and "
    //        "standard deviation of 2 : %f\n", x);

    // Particle test_prt = MotionModelSample(odom_loc, odom_angle);
    // For each particle, update its location and angle based on the motion model. (GPU acceleration?)
    if (!odom_initialized_)
    {
      prev_odom_loc_ = odom_loc;
      prev_odom_angle_ = odom_angle;
      odom_initialized_ = true;
      return;
    }
    // Eigen::Vector2f transformed_odom_loc = odom_loc;
    // Eigen::Vector2f transformed_prev_odom_loc = prev_odom_loc_;
    // if (rotation_initialized_)
    // {
    //   transformed_odom_loc = r_odom_map * odom_loc;
    //   transformed_prev_odom_loc = r_odom_map * prev_odom_loc_;
    // }
    
    vector<Particle> temp_particles = particles_;
    for (size_t i = 0; i < FLAGS_num_particles; ++i)
    {
      Particle temp_particle = MotionModelSample(odom_loc, odom_angle);
      // temp_particle.loc = r_odom_map * temp_particle.loc;
      // if (rotation_initialized_) {
      //   temp_particle.loc = r_odom_map * temp_particle.loc;
      // }
      temp_particles[i].loc.x() = temp_particles[i].loc.x() + temp_particle.loc.x();
      temp_particles[i].loc.y() = temp_particles[i].loc.y() + temp_particle.loc.y();
      temp_particles[i].angle = temp_particles[i].angle + temp_particle.angle;
      temp_particles[i].weight = temp_particle.weight;
    }

    // temp_particles[FLAGS_num_particles - 1].loc.x() = transformed_odom_loc.x() - transformed_prev_odom_loc.x();
    // temp_particles[FLAGS_num_particles - 1].loc.y() = transformed_odom_loc.y() - transformed_prev_odom_loc.y();
    // temp_particles[FLAGS_num_particles - 1].angle = math_util::AngleDiff(odom_angle, prev_odom_angle_);
    // temp_particles[FLAGS_num_particles - 1].weight = log(1.0 / FLAGS_num_particles);
    
    particles_ = temp_particles;
    prev_odom_angle_ = odom_angle;
    dist_traveled += (odom_loc - prev_odom_loc_).norm();
    prev_odom_loc_ = odom_loc;
    // ROS_INFO("loc: %f", particles_[0].loc.x());
  }

  Particle ParticleFilter::MotionModelSample(const Eigen::Vector2f &odom_loc, const float odom_angle)
  {
    Eigen::Vector2f transformed_odom_loc = odom_loc;
    Eigen::Vector2f transformed_prev_odom_loc = prev_odom_loc_;
    if (rotation_initialized_)
    {
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
    // Only output the d_ +  error , the error compounds over time however
    out.loc = Vector2f(dx + dx_error, dy + dy_error);
    // out.loc = Vector2f(dx, dy);
    out.angle = dtheta + dtheta_error;
    // out.angle = dtheta;
    out.weight = log(1.0 / FLAGS_num_particles);

    return out;
  }
  // Particle ParticleFilter::MotionModelSample(const Eigen::Vector2f &odom_loc, const float odom_angle)
  // {
  //   Eigen::Vector2f transformed_odom_loc = odom_loc;
  //   Eigen::Vector2f transformed_prev_odom_loc = prev_odom_loc_;
  //   if (rotation_initialized_)
  //   {
  //     transformed_odom_loc = r_odom_map * odom_loc;
  //     transformed_prev_odom_loc = r_odom_map * prev_odom_loc_;
  //   }
  //   float dx = transformed_odom_loc.x() - transformed_prev_odom_loc.x();
  //   float dy = transformed_odom_loc.y() - transformed_prev_odom_loc.y();
  //   float dtheta = math_util::AngleDiff(odom_angle, prev_odom_angle_);
  //   if(dtheta != 0){
  //   float radius_turn = sqrt(dx * dx + dy * dy)/sin(dtheta/2)/2;
  //   float arc_length = dtheta*radius_turn/(2*M_PI);
  //   //sample theta gaussian first
  //   float steering_angle = atan2(WHEELBASE,radius_turn);
  //   float steering_angle_err = rng_.Gaussian(0, k4 * fabs(steering_angle));
  //   float out_angle = steering_angle + steering_angle_err;
  //   // sample the arc length
  //   float darc_error = rng_.Gaussian(0, k1 * arc_length);
  //   float arc_out = arc_length + darc_error;

  //   // output radius of turning and theta
  //   float out_radius = WHEELBASE/tan(out_angle);
  //   float out_theta = 2*M_PI*arc_out/out_radius;
  //   // sample error from Gaussian distribution
  //   float dtheta_error = out_theta;
  //   float dx_error = (-(dx < 0) + (dx > 0))*out_radius*sin(out_theta);
  //   float z = 2*out_radius*sin(out_theta/2);
  //   float dy_error = (-(dy < 0) + (dy > 0))*sqrt(z*z - dx_error*dx_error);
  //   ROS_INFO("radius turn:%f",out_radius);
    

  //   Particle out;
  //   // Only output the d_ +  error , the error compounds over time however
  //   out.loc = Vector2f(dx_error, dy_error);
  //   // out.loc = Vector2f(dx, dy);
  //   out.angle = dtheta_error;
  //   // out.angle = dtheta;
  //   out.weight = log(1.0 / FLAGS_num_particles);

  //   return out;
  //   } else {
  //       // sample error from Gaussian distribution
  //   float dx_error = rng_.Gaussian(0, k1 * sqrt(dx * dx + dy * dy) + k2 * fabs(dtheta));
  //   float dy_error = rng_.Gaussian(0, k1 * sqrt(dx * dx + dy * dy) + k2 * fabs(dtheta));
  //   float dtheta_error = rng_.Gaussian(0, k3 * sqrt(dx * dx + dy * dy) + k4 * fabs(dtheta));

  //   Particle out;
  //   // Only output the d_ +  error , the error compounds over time however
  //   out.loc = Vector2f(dx + dx_error, dy + dy_error);
  //   // out.loc = Vector2f(dx, dy);
  //   out.angle = dtheta + dtheta_error;
  //   // out.angle = dtheta;
  //   out.weight = log(1.0 / FLAGS_num_particles);

  //   return out;

  //   }
  // }


  void ParticleFilter::Initialize(const string &map_file,
                                  const Vector2f &loc,
                                  const float angle)
  {
    // The "set_pose" button on the GUI was clicked, or an initialization message
    // was received from the log. Initialize the particles accordingly, e.g. with
    // some distribution around the provided location and angle.
    map_.Load(map_file);

    // Set all particles to the provided location and angle.
    vector<Particle> particles;
    particles.resize(FLAGS_num_particles);
    for (size_t i = 0; i < FLAGS_num_particles; ++i)
    {
      particles[i].loc = loc;
      particles[i].angle = angle;
      // weight is log likelihood
      particles[i].weight = log(1.0 / FLAGS_num_particles);
    }
    particles_ = particles;
    dist_traveled = 0;
    // map_.GetSceneLines(loc,10,&eligible_lines);
    // save transform from odom to this location and angle
    if (odom_initialized_)
    {
      // get rotation matrix from odom to given angle
      r_odom_map = Eigen::Rotation2Df(math_util::AngleMod(angle - prev_odom_angle_));
      rotation_initialized_ = true;
      // ROS_INFO("rotation initialized %f", r_odom_map.angle());
    }
  }

  void ParticleFilter::GetLocation(Eigen::Vector2f *loc_ptr,
                                   float *angle_ptr) const
  {
    Vector2f &loc = *loc_ptr;
    float &angle = *angle_ptr;
    // Compute the best estimate of the robot's location based on the current set
    // of particles. The computed values must be set to the `loc` and `angle`
    // variables to return them. Modify the following assignments:

    // Take the weighted average of the particles to get the location and angle
    Vector2f loc_sum(0, 0);
    float angle_sum = 0;
    // Copy the particles to a temporary vect
    vector<Particle> temp_particles = particles_;
    float weight_sum = 0;
    for (size_t i = 0; i < FLAGS_num_particles; ++i)
    {
      // stored weight is a log likelihood, so we need to exponentiate it

      float weight = exp(temp_particles[i].weight);
      loc_sum.x() += temp_particles[i].loc.x() * weight;
      loc_sum.y() += temp_particles[i].loc.y() * weight;
      angle_sum += temp_particles[i].angle * weight;
      weight_sum += weight;
      // ROS_INFO("weights %f", temp_particles[i].weight);
    }
    loc = loc_sum;
    angle = angle_sum;
    // ROS_INFO("weights %f", weight_sum);
    // ROS_INFO("loc: %f %f", loc.x(), loc.y());
    // ROS_INFO("angle: %f", angle);
  }

} // namespace particle_filter
