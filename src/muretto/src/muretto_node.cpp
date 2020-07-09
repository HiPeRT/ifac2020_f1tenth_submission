#include "../include/muretto_node.h"
#include "../include/occupancy_grid.h"
#include <fstream>
#include <iostream>
#include <string>


vector3d MurettoNode::load_flag_path(std::string file_path)
{

    std::string line;
    size_t pos = 0;
    std::string temp;
    int i;
    vector3d xyspeed;

    std::cout << file_path << "\n";

    std::ifstream is(file_path);
	if (!is)
    {
        std::cout << file_path << " not found!\n";
		return xyspeed;
    }
    
    //file format:
    // - Name: Flag 0
    // X: 0.442536
    // Y: 0.320954
    // Z: 4.49081
    
    for(i=0; std::getline(is,line); i++)//Read useless line
    {
        //read x
        std::getline(is,line);
        pos = line.find(":") + 1;
        xyspeed.xs.push_back(atof(line.substr(pos, std::string::npos).c_str()));

        //read y
        std::getline(is,line);
        pos = line.find(":") + 1;
        xyspeed.ys.push_back(atof(line.substr(pos, std::string::npos).c_str()));

        //read speed
        std::getline(is,line);
        pos = line.find(":") + 1;
        xyspeed.speed.push_back(atof(line.substr(pos, std::string::npos).c_str()));
    }

    return xyspeed;

}

void MurettoNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr scan)
{
    float angle = scan->angle_min;
    int center = scan->ranges.size() / 2;
    float frontal_distance = scan->ranges[center];
    float left_distance = 0;
    float right_distance = 0;
    int n_right = 0;
    int n_left = 0;

    for (int i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment)
    {
        float actual_range = scan->ranges[i];

        //Do some checks
        if(std::isnan(actual_range) || angle < scan->angle_min || angle > scan->angle_max
            || std::abs(angle) > field_of_view || actual_range > max_range || actual_range < min_range
            || actual_range < scan->range_min || actual_range > scan->range_max)
        {
            //ignore the actual range
            continue;
        }

        if(angle > 0) //left
        {
            left_distance += actual_range;
            n_left++;
        }
        else if(angle < 0) //right
        {
            right_distance += actual_range;
            n_right++;
        }
    }

    update_strategy();
    //std::cout << "mean left distance: " << left_distance/n_left << "\n";
    //std::cout << "mean right distance: " << right_distance/n_right << "\n";
}
/// Returns the row major indeices for the map of an inflated area around a point based on inflation radius
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return row major index of the map
std::vector<int> MurettoNode::get_expanded_row_major_indices(const double x_map, const double y_map)
{
    //Inflation radius == 5 test
    std::vector<int> expanded_row_major_indices;
    const auto x_index = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);
    for(int i=-5+x_index; i<5+1+x_index; i++)
    {
        for(int j=-5+y_index; j<5+1+y_index; j++)
        {
            expanded_row_major_indices.emplace_back(j*map_cols_ + i);
        }
    }
    return expanded_row_major_indices;
}

void MurettoNode::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr obstacles)
{

    std::vector<obstacle> temp_obs;
    
    obstacle temp_ob;
    for(const obstacle_detector::CircleObstacle obs : obstacles->circles)
    {
        temp_ob.x = obs.center.x;
        temp_ob.y = obs.center.y;
        temp_ob.radius = obs.true_radius;

        temp_obs.push_back(temp_ob);
    }

    _obstacles = temp_obs;

    if(!obstacle_test && !MurettoNode::checkPath(path))
        obstacle_test = true;
        update_strategy();
}

void MurettoNode::computeDistance()
{
    distance_to_opp = std::hypot(opp_desc.x-ego_desc.x, opp_desc.y-ego_desc.y);

    //printf("distance: %f\n", distance_to_opp);
}

void MurettoNode::egoOdomCallback(const nav_msgs::Odometry odom)
{
    ego_desc.x = odom.pose.pose.position.x;
    ego_desc.y = odom.pose.pose.position.y;
    ego_desc.speed = odom.twist.twist.linear.x;
    
    tf2::Quaternion tfq;
    tf2::fromMsg(odom.pose.pose.orientation, tfq);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tfq).getRPY(roll,pitch,yaw);
    
    orientation_yaw = yaw;
    
    //printf("ego pose:\t(%f, %f)\nego speed:\t%f\n", ego_desc.x, ego_desc.y, ego_desc.speed);
    
    computeDistance();
}

void MurettoNode::oppOdomCallback(const nav_msgs::Odometry odom)
{
    opp_desc.x = odom.pose.pose.position.x;
    opp_desc.y = odom.pose.pose.position.y;
    opp_desc.speed = odom.twist.twist.linear.x;
    
    //printf("opponent pose:\t(%f, %f)\nopponent speed:\t%f\n", opp_desc.x, opp_desc.y, opp_desc.speed);
    
    computeDistance();
}

bool MurettoNode::freePose(double x, double y)
{
    double distance;
    for(const obstacle ob : _obstacles)
    {
        distance = std::abs(std::hypot(x - ob.x, y - ob.y) - ob.radius);
        if(distance < trigger_distance)
            return false;
    }

    return true;
}

bool MurettoNode::checkPath(vector3d path)
{
    for(int i=0; i<path.xs.size(); i++)
    {
        if(!freePose(path.xs[i], path.ys[i]))
            return false;
    }
    return true;
}

void MurettoNode::update_strategy(){
    //main strategy logic

    muretto::Strategy strategy_msg;
    
    //race_type = 
    //      0 => time attack
    //      1 => obstacle test
    if(obstacle_test)
        strategy_msg.race_type = 1;
    else
        strategy_msg.race_type = 0;

    //overtake_strategy = 
    //      0 => avoid overtaking
    //      1 => overtake on the left
    //      2 => overtake on the right
    strategy_msg.overtake_strategy = 0;

    //follow_local_path =
    //      true  => 
    //      false => 
    strategy_msg.follow_local_path = false;

    //std::cout << "sending strategy msg" << std::endl;

    strategy_pub.publish(strategy_msg);
}

void MurettoNode::start()
{
    std::cout << "MURETTO NODE STARTED\n";

    ros::NodeHandle private_node_handler("~");
    std::string trj_path;
    std::string param_file;
    private_node_handler.param<std::string>("trajectory_path", trj_path, "");
    private_node_handler.param<std::string>("param_file", param_file, "");
    YAML::Node conf = YAML::LoadFile(param_file);

    max_range = conf["max_range"].as<float>();
    min_range = conf["min_range"].as<float>();

    // Load Input Map from map_server
    input_map_  = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map",ros::Duration(1)));
    if(input_map_.data.empty())
    {
        std::__throw_invalid_argument("Input Map Load Unsuccessful\"");
    }
    std::cout << "Map Load Successful." << std::endl;
    // Map Data
    map_cols_ = input_map_.info.width;
    new_obstacles_ = {};
    new_obstacles_.reserve(2000);
    clear_obstacles_count_ = 0;

    std::string opt_path;
    private_node_handler.getParam("global_trajectory", opt_path);

    path = load_flag_path(opt_path); 

    for(int i=0; i<path.xs.size()-1; i++)
        if(occupancy_grid::is_xy_occupied(input_map_, path.xs[i], path.ys[i]))
            obstacle_test = true;
    //sub, pub
    //obstacles_sub = node_handler.subscribe("/raw_obstacles",10,&MurettoNode::obstacleCallback, this);
    scan_sub = node_handler.subscribe("/scan",10,&MurettoNode::scanCallback, this);
    ego_odom_sub = node_handler.subscribe("/odom",10,&MurettoNode::egoOdomCallback, this);
    opp_odom_sub = node_handler.subscribe("/opp_odom",10,&MurettoNode::oppOdomCallback, this);
    strategy_pub = node_handler.advertise<muretto::Strategy>("/strategy", 10);
    map_update_pub_ = node_handler.advertise<nav_msgs::OccupancyGrid>("/map_updated", 5);

    update_strategy();

    //START SPIN
    ros::spin();

    /*
    ros::Rate r(100);

    while(ros::ok())
    {
        // TO-DO: Behavior plannin using state-machine approach -> if-else using the data retrieved
        
        ros::spinOnce();
        r.sleep();
    }
    */
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "muretto_node");
    MurettoNode muretto;
    muretto.start();

    return 0;
}
