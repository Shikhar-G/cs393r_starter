//this will be dedicated to the 




#include <algorithm>
#include <vector>
#include <unordered_map>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"

#include "simple_queue.h"

//ros testing
#include "ros/ros.h"
#include "ros/package.h"

#ifndef GLOBAL_PLANNER_H_
#define GLOBAL_PLANNER_H_

namespace planner {

using std::vector;
using std::unordered_map;

// General parent class planner that will be inherited by all other planners
class Planner {
    public:
        virtual ~Planner() = default;
        virtual void Plan() = 0;
        virtual vector<Eigen::Vector2f> GetPath() = 0;
        virtual void Clear() = 0;
        virtual void SetStart(Eigen::Vector2f start) = 0;
        virtual void SetGoal(Eigen::Vector2f goal) = 0;
};



class RRT_Star : public Planner{

    private:
        Eigen::Vector2f start_;
        Eigen::Vector2f goal_;
        //the goal radius is in meters, creates 
        float goal_radius_ = 0.2;
        vector<Eigen::Vector2f> vertices_;
        unordered_map<size_t, vector<size_t>> edges_;
        vector<size_t> parents_;
        vector<float> costs_;
        size_t goal_index_ = -1;
        vector_map::VectorMap *vector_map_;
        size_t num_iterations_ = 100000;
        float min_x_;
        float max_x_;
        float min_y_;
        float max_y_;
        float radius_ = 5;
        float step_size_ = 0.5;
        float safety_margin_ = 0.2;

        Eigen::Vector2f SampleRandomPoint();
        size_t FindNearestVertex(const Eigen::Vector2f& point);
        Eigen::Vector2f Steer(size_t nearest_vertex_index, const Eigen::Vector2f& random_point);
        bool IsValidVertex(const Eigen::Vector2f& vertex);
        bool IsCollision(const Eigen::Vector2f& start, const Eigen::Vector2f& end);
        void Rewire(size_t nearest_vertex_new_index, const Eigen::Vector2f& new_vertex, const vector<size_t>& nearby_vertices);
        float Cost(const Eigen::Vector2f& start, const Eigen::Vector2f& end);
        size_t FindNearestVertexInRadius(const Eigen::Vector2f& point, size_t nearest_vertex_index, const vector<size_t>& vertices_in_radius);
        vector<size_t> FindVerticesInRadius(const Eigen::Vector2f& point, double radius);
        
    public:
        //initialization
        RRT_Star();
        RRT_Star(vector_map::VectorMap* map);
        RRT_Star(Eigen::Vector2f start, Eigen::Vector2f goal): start_(start), goal_(goal){}


        
        void Clear() {vertices_.clear(); edges_.clear(); parents_.clear(); costs_.clear();}

        void SetStart(Eigen::Vector2f start) {start_ = start;}
        void SetGoal(Eigen::Vector2f goal) {goal_ = goal;}

        void Plan();

        vector<pair<Eigen::Vector2f, Eigen::Vector2f>> GetTree();

        vector<Eigen::Vector2f> GetPath();

    };
}
#endif //GLOBAL_PLANNER_H_
