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
#include "particle_filter.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "config_reader/config_reader.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "vector_map/vector_map.h"

using Eigen::Vector2f;
using Eigen::Vector2i;
using geometry::line2f;
using math_util::AngleMod;
using math_util::Sq;
using std::cout;
using std::endl;
using std::fabs;
using std::max;
using std::min;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

// DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

CONFIG_FLOAT(num_particles_, "num_particles");
CONFIG_FLOAT(ds_factor, "ds_factor");
CONFIG_FLOAT(k_x_, "k_x");
CONFIG_FLOAT(k_y_, "k_y");
CONFIG_FLOAT(k_theta_, "k_theta");
CONFIG_FLOAT(k_laser_loc_x_, "k_laser_loc.x");
CONFIG_FLOAT(k_laser_loc_y_, "k_laser_loc.y");
CONFIG_FLOAT(d_short, "d_short");
CONFIG_FLOAT(d_long, "d_long");
CONFIG_FLOAT(likelihood_sigma, "likelihood_sigma");
CONFIG_FLOAT(likelihood_gamma, "likelihood_gamma");
CONFIG_FLOAT(dist_threshold, "dist_threshold");
CONFIG_FLOAT(angle_threshold, "angle_threshold");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter()
    : prev_odom_loc_(0, 0), prev_odom_angle_(0), odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>& scan) {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // Group Comments: construct point cloud in world frame
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(CONFIG_k_laser_loc_x_, CONFIG_k_laser_loc_y_);

  // Construct se2 matrix for robot pose in world to transform ray line segment from
  // base_link frame to world frame se2 transformations preserve distances between
  // points
  Eigen::Affine2f T_robot = Eigen::Translation2f(loc) * Eigen::Rotation2Df(angle);

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    // define endpoint of ray line segment relative to world frame
    float theta = angle_min + i * (angle_max - angle_min) / (num_ranges - 1);
    scan[i] = T_robot * (Vector2f(range_max * cos(theta), range_max * sin(theta)) +
                         kLaserLoc);  // world frame
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  vector<float> ranges(num_ranges, range_max);
  for (size_t i = 0; i < scan.size(); ++i) {
    for (size_t j = 0; j < map_.lines.size(); ++j) {
      const line2f map_line = map_.lines[j];
      // Construct line segment in world frame
      line2f my_line(loc.x(), loc.y(), scan[i].x(), scan[i].y());

      // Check for intersection
      Vector2f intersection_point;  // Return variable
      bool intersects = map_line.Intersection(my_line, &intersection_point);
      if (intersects) {
        // NOTE: se2 transformations preserve distances between between points
        float new_range =
            (intersection_point - loc).norm();  // compute range in world frame
        if (new_range < ranges[i]) {
          scan[i] = intersection_point;  // scan in world frame
          ranges[i] = new_range;
        }
      }
    }
  }
}

void ParticleFilter::Update(const vector<float>& observed_ranges,
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
  vector<Vector2f> predicted_scan;  // world frame

  GetPredictedPointCloud(p_ptr->loc,
                         p_ptr->angle,
                         observed_ranges.size(),
                         range_min,
                         range_max,
                         angle_min,
                         angle_max,
                         predicted_scan);

  Eigen::Affine2f T_particle =  // T^{base_link}_{world}
      Eigen::Translation2f(p_ptr->loc) * Eigen::Rotation2Df(p_ptr->angle);
  const Vector2f kLaserLoc(CONFIG_k_laser_loc_x_, CONFIG_k_laser_loc_y_);

  // predicted scan is in world frame, but observed scan is in laser frame
  // Convert predicted scan from world frame to particle laser frame
  // so that we can compute ranges to compare with d_short and d_long

  vector<float> predicted_ranges(predicted_scan.size());
  for (size_t i = 0; i < predicted_scan.size(); ++i) {
    const Vector2f& p = predicted_scan[i];  // world frame
    Vector2f p_lidar = T_particle.inverse() * p -
                       kLaserLoc;  // T_{base_link}_{laser}* T^{world}_{base_link}
    predicted_ranges[i] = p_lidar.norm();
  }

  // Compute the likelihood of the particle
  p_ptr->log_likelihood =
      computeNormalLikelihood(predicted_ranges, observed_ranges, range_min, range_max);

  if (FLAGS_v > 4) {
    printf("p_loc: (%.6f, %.6f) p_angle: %.6f p_weight: %.6f\n log_likelihood: %.6f\n",
           p_ptr->loc.x(),
           p_ptr->loc.y(),
           p_ptr->angle,
           p_ptr->weight,
           p_ptr->log_likelihood);
    cout << "==============================================================\n" << endl;
  }
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
  if (!odom_initialized_) {
    return;
  }

  // // Option1. Less often resampling
  // static Eigen::Vector2f prev_resampled_odom_loc = prev_odom_loc_;
  // static float prev_resampled_odom_angle = prev_odom_angle_;
  // Eigen::Vector2f curr_odom_loc;
  // float theta = 0;
  // GetLocation(&curr_odom_loc, &theta);

  // float distance_traveled = (curr_odom_loc - prev_resampled_odom_loc).norm();
  // float angle_traveled = fabs(AngleMod(theta - prev_resampled_odom_angle));
  // if (distance_traveled < CONFIG_dist_threshold &&
  //     angle_traveled < CONFIG_angle_threshold) {
  //   return;
  // }

  // Option2. Effective number of particle resampling
  double n_eff = 0;
  double max_log_likelihood = -std::numeric_limits<double>::infinity();
  for (const Particle& p : particles_) {
    n_eff += Sq(p.weight);
    max_log_likelihood = max(max_log_likelihood, p.log_likelihood);
  }
  n_eff = 1.0 / n_eff;
  // particles' weights are distributed uniformly or log_likelihood is too low (bad
  // estimation)
  if (n_eff > static_cast<double>(particles_.size()) / 2) {
    return;
  }

  vector<Particle> new_particles(particles_.size());
  vector<double> weights(particles_.size());
  std::transform(
      particles_.begin(), particles_.end(), weights.begin(), [](const Particle& p) {
        return p.weight;
      });
  vector<double> cum_weights(particles_.size(), 0);
  std::partial_sum(weights.begin(), weights.end(), cum_weights.begin());

  // low-variance resampling
  double r = rng_.UniformRandom(0, 1) / particles_.size();
  int index = 0;
  for (size_t i = 0; i < particles_.size(); ++i) {
    double u = r + static_cast<double>(i) / particles_.size();
    while (u > cum_weights[index]) {
      index++;
    }
    new_particles[i] = particles_[index];
    new_particles[i].weight = 1.0 / particles_.size();
  }

  particles_ = new_particles;

  // 4. Save last odom location used for update step
  // GetLocation(&prev_resampled_odom_loc, &prev_resampled_odom_angle);

  if (FLAGS_v > 2) {
    cout
        << "\033[34m=============== [Particle Filter] RESAMPLED! ===============\033[0m"
        << endl;
    cout << "n_eff: " << n_eff << endl;
    // cout << "Distance traveled: " << distance_traveled
    //      << ", Angle traveled: " << angle_traveled << endl;
  }
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  static CumulativeFunctionTimer observe_function_timer_(__FUNCTION__);
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  if (!odom_initialized_) {
    return;
  }
  CumulativeFunctionTimer::Invocation invoke(&observe_function_timer_);

  // 0. Subsample the ranges by downsample factor
  size_t num_sub_ranges = size_t(ranges.size() / CONFIG_ds_factor);
  vector<float> sub_ranges(num_sub_ranges);

  float d_theta = (angle_max - angle_min) / (ranges.size() - 1);
  for (size_t i = 0; i < num_sub_ranges; ++i) {
    size_t copy_idx = i * CONFIG_ds_factor;
    sub_ranges[i] = ranges[copy_idx];
  }
  float new_angle_max = angle_min + d_theta * CONFIG_ds_factor * (num_sub_ranges - 1) ;

  if (FLAGS_v > 2) {
    cout << "============= [Particle Filter] Downsample Scans ================" << endl;
    cout << "num_sub_ranges: " << num_sub_ranges << endl;
    cout << "new_angle_max: " << new_angle_max << endl;
  }

  // 1. Predict the likelihood of each particle
  double max_log_likelihood = -std::numeric_limits<double>::infinity();
  for (Particle& p : particles_) {
    this->Update(sub_ranges, range_min, range_max, angle_min, new_angle_max, &p);
    max_log_likelihood = max(max_log_likelihood, p.log_likelihood);
  }

  // 2.1 compute likelihood of each particle
  double total_weight = 0;
  for (Particle& p : particles_) {
    p.weight *= exp(p.log_likelihood - max_log_likelihood);  // posterior
    total_weight += p.weight;
  }

  if (total_weight == 0.0f) {
    printf(
        "\033[31m======== [Particle Filter]Total weight was ZERO!!=========\033[0m\n");
  }

  // 2.2 Normalize the weights
  for (Particle& p : particles_) {
    p.weight /= total_weight;
  }

  float real_total_weight = 0;
  for (const Particle& p : particles_) {
    real_total_weight += p.weight;
  }

  if (FLAGS_v > 2) {
    cout << "\033[32m============== [Particle Filter] UPDATE STEP ================"
         << endl;
    printf("total_weight: %.6f\n", total_weight);
    printf("real_total_weight: %.6f\n", real_total_weight);
    printf("max_log_likelihood: %.6f\n", max_log_likelihood);
    for (const Particle& p : particles_) {
      printf("(%.3f, %.3f, %.3f)",
             p.weight,
             p.log_likelihood - max_log_likelihood,
             exp(p.log_likelihood - max_log_likelihood));
    }
    cout << "\n=============================================================\033[0m\n"
         << endl;
  }

  // 3. Resample the particles
  this->Resample();
}

void ParticleFilter::Predict(const Vector2f& odom_loc, const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  // Benchmark Timing Functions
  static CumulativeFunctionTimer predict_function_timer_(__FUNCTION__);

  if (!odom_initialized_) {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }
  CumulativeFunctionTimer::Invocation invoke(&predict_function_timer_);

  // U_t = X_t^-1 * X_{t+1} : transformation from prev_odom frame to current odom frame
  // Construct SE(2) matrices: rotate then translate.
  Eigen::Affine2f X_new =
      Eigen::Translation2f(odom_loc) * Eigen::Rotation2Df(odom_angle);
  Eigen::Affine2f X_old =
      Eigen::Translation2f(prev_odom_loc_) * Eigen::Rotation2Df(prev_odom_angle_);

  Eigen::Affine2f U = X_old.inverse() * X_new;  // 3x3 matrix

  // Compute Covariance of the noise
  float sigma_x = fabs(U.translation().x());
  float sigma_y = fabs(U.translation().y());
  float sigma_theta =
      fabs(atan2(U.rotation()(1, 0), U.rotation()(0, 0)));  // sin dtheta / cos dtheta

  // Predict the new pose for each particle with noise
  for (Particle& p : particles_) {
    // Convert particles to SE(2) and add noise
    // Sample from noise distributions (center at 0 by offsetting)
    float eps_x = CONFIG_k_x_ * rng_.Gaussian(0, sigma_x);
    float eps_y = CONFIG_k_y_ * rng_.Gaussian(0, sigma_y + 1e-3);
    float eps_theta = CONFIG_k_theta_ * rng_.Gaussian(0, sigma_theta);

    // printf("eps_x: %.6f eps_y: %.6f eps_theta: %.6f\n", eps_x, eps_y, eps_theta);
    Eigen::Affine2f e =
        Eigen::Translation2f(eps_x, eps_y) * Eigen::Rotation2Df(eps_theta);

    // particles are X^t_w
    Eigen::Affine2f particle =
        Eigen::Translation2f(p.loc) * Eigen::Rotation2Df(p.angle);

    // X^{\tilde {t+1}}_w = X^t_w * U^{t+1}_t * E^{\tilde{t+1}}_{t+1}
    Eigen::Affine2f new_particle = particle * U * e;

    // SE(2) -> x y theta
    p.loc = new_particle.translation();
    p.angle = atan2(new_particle.rotation()(1, 0), new_particle.rotation()(0, 0));
  }

  // Update the previous odometry
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;

  if (FLAGS_v > 2) {
    cout << "=============== [Particle Filter] PREDICT STEP ===============" << endl;
    printf("new_odom_loc: (%.6f, %.6f) new_odom_angle: %.6f\n",
           odom_loc.x(),
           odom_loc.y(),
           odom_angle);
    printf("prev_odom_loc: (%.6f, %.6f) prev_odom_angle: %.6f\n",
           prev_odom_loc_.x(),
           prev_odom_loc_.y(),
           prev_odom_angle_);
    cout << "==============================================================\n" << endl;
  }
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  // Initialize the particles
  particles_.resize(CONFIG_num_particles_);
  for (Particle& p : particles_) {
    p.loc = loc;
    p.angle = angle;
    p.weight = 1.0 / CONFIG_num_particles_;
  }

  odom_initialized_ = false;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;

  // Importance sampling
  float cos_sum = 0;
  float sin_sum = 0;
  for (const Particle& p : particles_) {
    loc += p.loc * p.weight;
    cos_sum += cos(p.angle) * p.weight;
    sin_sum += sin(p.angle) * p.weight;
  }
  angle = atan2(sin_sum, cos_sum);
}

double ParticleFilter::computeNormalLikelihood(const vector<float>& predicted_ranges,
                                               const vector<float>& observed_ranges,
                                               float range_min,
                                               float range_max) {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  double log_likelihood = 0;

  for (size_t i = 0; i < predicted_ranges.size(); ++i) {
    if ((observed_ranges[i] >= range_max) || (observed_ranges[i] <= range_min))
      continue;

    float diff = predicted_ranges[i] - observed_ranges[i];

    if (diff > CONFIG_d_short) {
      log_likelihood -= Sq(CONFIG_d_short) / Sq(CONFIG_likelihood_sigma);
    } else if (diff < -CONFIG_d_long) {
      log_likelihood -= Sq(CONFIG_d_long) / Sq(CONFIG_likelihood_sigma);
    } else {
      log_likelihood -= Sq(diff) / Sq(CONFIG_likelihood_sigma);
    }

    if (FLAGS_v > 4) {
      printf("pred_range: %.6f obs_range: %.6f diff: %.6f, log_likelihood: %.6f\n",
             predicted_ranges[i],
             observed_ranges[i],
             diff,
             log_likelihood);
    }
  }

  return CONFIG_likelihood_gamma * log_likelihood;
}

}  // namespace particle_filter

// void ParticleFilter::ObserveLaserCUDA(const vector<float>& ranges,
//                                                  float range_min,
//                                                  float range_max,
//                                                  float angle_min,
//                                                  float angle_max) {
//   static CumulativeFunctionTimer observe_function_timer_(__FUNCTION__);
//   // A new laser scan observation is available (in the laser frame)
//   // Call the Update and Resample steps as necessary.
//   if (!odom_initialized_) {
//     return;
//   }
//   CumulativeFunctionTimer::Invocation invoke(&observe_function_timer_);

//   // 1 Allocate memory on the GPU
//   Particle* d_particles_;
//   int Nparticles = particles_.size();
//   cudaMallocManaged(&d_particles_, Nparticles * sizeof(Particle));
//   cudaMemcpy(d_particles_,
//              particles_.data(),
//              Nparticles * sizeof(Particle),
//              cudaMemcpyHostToDevice);

//   // 3 Copy ranges to unified memory on GPU
//   float* d_ranges_;
//   int Nranges = ranges.size();
//   cudaMallocManaged(&d_ranges_, Nranges * sizeof(float));
//   cudaMemcpy(d_ranges_, ranges.data(), Nranges * sizeof(float),
//   cudaMemcpyHostToDevice);

//   // 5 Update particles log likelihoods and weights on GPU
//   int blockSize = 256;
//   int numBlocks = (Nparticles + blockSize - 1) / blockSize;
//   UpdateKernel<<<numBlocks, blockSize>>>(Nparticles,
//                                          Nranges,
//                                          d_ranges_,
//                                          range_min,
//                                          range_max,
//                                          angle_min,
//                                          angle_max,
//                                          d_particles_);

//   cudaDeviceSynchronize();
//   cudaFree(d_particles_);

// // // 1. Predict the likelihood of each particle
// // double max_log_likelihood = -std::numeric_limits<double>::infinity();
// // {
// //   for (Particle& p : particles_) {
// //     this->Update(ranges, range_min, range_max, angle_min, angle_max, &p);
// //     max_log_likelihood = max(max_log_likelihood, p.log_likelihood);
// //   }
// // }

// // // 2.1 compute likelihood of each particle
// // double total_weight = 0;
// // for (Particle& p : particles_) {
// //   p.weight *= exp(p.log_likelihood - max_log_likelihood);  // posterior
// //   total_weight += p.weight;
// // }
// // // 2.2 Normalize the weights
// // for (Particle& p : particles_) {
// //   p.weight /= total_weight;
// // }

// if (FLAGS_v > 2) {
//   cout << "\033[32m============== [Particle Filter] UPDATE STEP ================"
//        << endl;
//   printf("max_log_likelihood: %.6f\n", max_log_likelihood);
//   for (const Particle& p : particles_) {
//     printf("%.3f ", p.weight);
//   }
//   cout <<
//   "\n=============================================================\033[0m\n"
//        << endl;
// }

// // 3. Resample the particles
// this->Resample();
// }
