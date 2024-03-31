

#include "global_planner.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <algorithm>
#include <vector>

namespace planner {

    RRT_Star::RRT_Star() {
        // Find min/max x and y
        float min_x = std::numeric_limits<float>::infinity();
        float max_x = -std::numeric_limits<float>::infinity();
        float min_y = std::numeric_limits<float>::infinity();
        float max_y = -std::numeric_limits<float>::infinity();
        for (const auto& line : vector_map_.lines)
        {
            min_x = std::min(min_x, std::min(line.p0.x(), line.p1.x()));
            max_x = std::max(max_x, std::max(line.p0.x(), line.p1.x()));
            min_y = std::min(min_y, std::min(line.p0.y(), line.p1.y()));
            max_y = std::max(max_y, std::max(line.p0.y(), line.p1.y()));
        }
        assert (min_x != std::numeric_limits<float>::infinity());
        assert (max_x != -std::numeric_limits<float>::infinity());
        assert (min_y != std::numeric_limits<float>::infinity());
        assert (max_y != -std::numeric_limits<float>::infinity());
        min_x_ = min_x;
        max_x_ = max_x;
        min_y_ = min_y;
        max_y_ = max_y;
    }

    void RRT_Star::Plan() {
        // Reset the tree
        Clear();
        goal_index_ = -1;
        bool goal_reached = false;
        // Add the start node
        vertices_.push_back(start_);
        parents_.push_back(0);
        costs_.push_back(0);
        for (size_t i = 0; i < num_iterations_; i++)
        {
            // Sample a random point
            Eigen::Vector2f random_point = SampleRandomPoint();
            // Find the nearest vertex
            size_t nearest_vertex_index = FindNearestVertex(random_point);
            // Steer towards the random point
            Eigen::Vector2f new_vertex = Steer(nearest_vertex_index, random_point);
            // Check if the new vertex is valid
            if (!IsCollision(vertices_[nearest_vertex_index], new_vertex))
            {
                // Find the nearest vertex to the new vertex
                vector<size_t> vertices_in_radius = FindVerticesInRadius(new_vertex, radius_);
                size_t nearest_vertex_new = FindNearestVertexInRadius(new_vertex, nearest_vertex_index, vertices_in_radius);
                // Add the new vertex to the tree
                vertices_.push_back(new_vertex);
                parents_.push_back(nearest_vertex_new);
                costs_.push_back(costs_[nearest_vertex_new] + Cost(vertices_[nearest_vertex_new], new_vertex));
                // check if nearest_vertex_new is in edges_
                if (edges_.find(nearest_vertex_new) == edges_.end())
                {
                    edges_[nearest_vertex_new] = vector<size_t>();
                }
                // Add the new vertex to the edges
                edges_[nearest_vertex_new].push_back(vertices_.size() - 1);
                // Rewire the tree
                Rewire(nearest_vertex_new, new_vertex, vertices_in_radius);
                // Check if the goal is reached
                if ((new_vertex - goal_).norm() < goal_radius_)
                {
                    // Set the goal index if goal is not reached yet or the new vertex has a lower cost
                    if (!goal_reached || costs_.back() < costs_[goal_index_]) {
                        goal_index_ = vertices_.size() - 1;
                        goal_reached = true;
                    }
                }
            }
        }
    }

    vector<Eigen::Vector2f> RRT_Star::GetPath()
    {
        
        size_t curr_vertex = goal_index_;
        //push goal to path.
        vector<Eigen::Vector2f> path_out;
        path_out.push_back(vertices_[curr_vertex]);

        //starting from the goal work backwards to the start
        while (curr_vertex != 0)
        {
            size_t parent_vertex = parents_[curr_vertex];
            // Eigen::Vector2f curr = vertices_[curr_vertex];
            // Eigen::Vector2f parent = vertices_[parent_vertex];
            curr_vertex = parent_vertex;
            path_out.push_back(vertices_[curr_vertex]);
        }
        //the path goes from goal to start right now, reverse it
        std::reverse(path_out.begin(),path_out.end());
        return path_out;
    }

    float RRT_Star::Cost(const Eigen::Vector2f& start, const Eigen::Vector2f& end)
    {
        return (start - end).norm();
    }

    Eigen::Vector2f RRT_Star::SampleRandomPoint()
    {
        // Sample a random point
        double x = min_x_ + static_cast<double>(rand()) / RAND_MAX * (max_x_ - min_x_);
        double y = min_y_ + static_cast<double>(rand()) / RAND_MAX * (max_y_ - min_y_);
        return Eigen::Vector2f(x, y);
    }

    size_t RRT_Star::FindNearestVertex(const Eigen::Vector2f& point)
    {
        // Find the nearest vertex
        size_t nearest_vertex_index = vertices_.size();
        double min_cost = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < vertices_.size(); i++)
        {
            double cost = Cost(vertices_[i], point);
            if (cost < min_cost)
            {
                nearest_vertex_index = i;
                min_cost = cost;
            }
        }
        return nearest_vertex_index;
    }

    Eigen::Vector2f RRT_Star::Steer(size_t nearest_vertex_index, const Eigen::Vector2f& random_point)
    {
        // Steer towards the random point
        Eigen::Vector2f nearest = vertices_[nearest_vertex_index];
        Eigen::Vector2f direction = (random_point - nearest).normalized();
        return nearest + direction * step_size_;
    }

    bool RRT_Star::IsCollision(const Eigen::Vector2f& start, const Eigen::Vector2f& end)
    {
        return vector_map_.Intersects(start, end);
    }

    vector<size_t> RRT_Star::FindVerticesInRadius(const Eigen::Vector2f& point, double radius)
    {
        vector<size_t> vertices_in_radius;
        for (size_t i = 0; i < vertices_.size(); i++)
        {
            if ((vertices_[i] - point).norm() < radius)
            {
                vertices_in_radius.push_back(i);
            }
        }
        return vertices_in_radius;
    }

    size_t RRT_Star::FindNearestVertexInRadius(const Eigen::Vector2f& point, size_t nearest_vertex_index, const vector<size_t>& vertices_in_radius)
    {
        double min_cost = costs_[nearest_vertex_index];
        for (size_t i : vertices_in_radius)
        {
            if (!IsCollision(vertices_[i], point))
            {
                double cost = costs_[i] + Cost(vertices_[i], point);
                if (cost < min_cost)
                {
                    nearest_vertex_index = i;
                    min_cost = cost;
                }
            }
        }
        return nearest_vertex_index;
    }



    void RRT_Star::Rewire(size_t nearest_vertex_new_index, const Eigen::Vector2f& new_vertex,const vector<size_t>& vertices_in_radius)
    {
        for(auto & vertex_near : vertices_in_radius)
        {
            if(IsCollision(vertices_[vertex_near],new_vertex)) continue;

            float cost = costs_[nearest_vertex_new_index] + Cost(new_vertex, vertices_[vertex_near]);
            if(cost < costs_[vertex_near])
            {
                costs_[vertex_near] = cost;
                parents_[vertex_near] = nearest_vertex_new_index;
                edges_[nearest_vertex_new_index].push_back(vertex_near);
            }
        }
    }
    
}