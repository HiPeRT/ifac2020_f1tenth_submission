#ifndef MURETTO_NODE_H
#define MURETTO_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <muretto/Strategy.h>
#include <yaml-cpp/yaml.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

struct vector3d{
    std::vector<double> xs, ys, speed;
};

struct obstacle{
    double x, y, radius;
};

struct car_description{
    double x, y, speed;
};

class MurettoNode
{
    private:
        ros::NodeHandle node_handler;
        ros::NodeHandle private_node_handler;
        ros::Publisher muretto_pub; //publish the lane that should be followed to avoid collision
        ros::Subscriber obstacles_sub;
        ros::Subscriber ego_odom_sub;
        ros::Subscriber opp_odom_sub;
        ros::Subscriber scan_sub;
        ros::Subscriber map_sub;
        ros::Publisher strategy_pub;
        ros::Publisher map_update_pub_;

        tf2_ros::Buffer tf_buffer_;
        geometry_msgs::TransformStamped tf_laser_to_map_;
        nav_msgs::OccupancyGrid input_map_;
        std::vector<size_t > new_obstacles_;
        int clear_obstacles_count_;


        void computeDistance();
        bool checkPath(vector3d path);
        bool freePose(double x, double y);
        void update_strategy();

        float max_range;
        float min_range;
        float field_of_view = M_PI/2;

        std::vector<obstacle> _obstacles;
        car_description ego_desc;
        car_description opp_desc;

        double distance_to_opp;


    public:
        MurettoNode(){};
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr scan_msg);
        void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr obstacles);
        void egoOdomCallback(const nav_msgs::Odometry odom);
        void oppOdomCallback(const nav_msgs::Odometry odom);
        std::vector<int> get_expanded_row_major_indices(const double x_map, const double y_map);

        vector3d load_flag_path(std::string file_path);
        float trigger_distance = 2;
        float opponent_radius;
        int map_cols_;

        void start();
        vector3d path;
        bool obstacle_test = false;
        double orientation_yaw;
};

#endif //MURETTO_NODE_H
