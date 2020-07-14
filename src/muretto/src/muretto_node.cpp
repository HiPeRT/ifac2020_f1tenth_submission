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
    left_distance = 0;
    right_distance = 0;
    int n_right = 0;
    int n_left = 0;

    for (int i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment)
    {
        float actual_range = scan->ranges[i];

        //Do some checks
        if(std::isnan(actual_range) || angle < scan->angle_min || angle > scan->angle_max
            || std::abs(angle) > field_of_view || actual_range > max_range || actual_range < min_range
            || actual_range < scan->range_min || actual_range > scan->range_max || (i > center-30 && i < center+30))
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
    left_distance = left_distance/n_left;
    right_distance = right_distance/n_right;
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

    if(!obstacle_test && !MurettoNode::freePath(path))
        obstacle_test = true;
        update_strategy();
}

void MurettoNode::computeDistance()
{
    distance_to_opp = std::hypot(opp_desc.x-ego_desc.x, opp_desc.y-ego_desc.y);

    //printf("---------------------------------------------------distance: %f\n", distance_to_opp);
}

void MurettoNode::computePlacement()
{
    ego_closest_point = 0; 
    int ego_closest_point_dist = std::hypot(path.xs[0]-ego_desc.x, path.ys[0]-ego_desc.y);
    opp_closest_point = 0; 
    int opp_closest_point_dist = std::hypot(path.xs[0]-opp_desc.x, path.ys[0]-opp_desc.y);
    
    for(int i=1; i<path.xs.size(); i++)
    {
        // Find distance from ego to current point 
        int i_dist = std::hypot(path.xs[i]-ego_desc.x, path.ys[i]-ego_desc.y);
        if(i_dist < ego_closest_point_dist)
        {
            ego_closest_point = i;
            ego_closest_point_dist = i_dist;
        }
        
        // Find distance from opp to current point
        i_dist = std::hypot(path.xs[i]-opp_desc.x, path.ys[i]-opp_desc.y);
        if(i_dist < opp_closest_point_dist)
        {
            opp_closest_point = i;
            opp_closest_point_dist = i_dist;
        }
    }

    ego_closest_point +=5;
    opp_closest_point +=5;
    ego_closest_point = ego_closest_point % path.xs.size();
    opp_closest_point = opp_closest_point % path.xs.size();

    int path_distance = ego_closest_point-opp_closest_point;

    if(std::abs(path_distance) < path.xs.size()/2)
        ahead = path_distance > 0;
    else
        ahead = path_distance < 0;
    
    if(!ahead && distance_to_opp > overtaking_range)
        ahead = true;

    
    //publish markers
    visualization_msgs::Marker ego_marker;
    ego_marker.header.frame_id = "map";
    ego_marker.header.stamp = ros::Time();
    ego_marker.id = 0;
    ego_marker.type = visualization_msgs::Marker::SPHERE;
    ego_marker.action = visualization_msgs::Marker::ADD;
    ego_marker.pose.position.x = path.xs[ego_closest_point];
    ego_marker.pose.position.y = path.ys[ego_closest_point];
    ego_marker.pose.orientation.x = 0.0;
    ego_marker.pose.orientation.y = 0.0;
    ego_marker.pose.orientation.z = 0.0;
    ego_marker.pose.orientation.w = 1.0;
    ego_marker.scale.x = 0.2;
    ego_marker.scale.y = 0.2;
    ego_marker.scale.z = 0.2;
    ego_marker.color.a = 1.0;
    ego_marker.color.r = 0.0;
    ego_marker.color.g = 0.0;
    ego_marker.color.b = 1.0;

    ego_marker_pub.publish(ego_marker);

    visualization_msgs::Marker opp_marker;
    opp_marker.header.frame_id = "map";
    opp_marker.header.stamp = ros::Time();
    opp_marker.id = 0;
    opp_marker.type = visualization_msgs::Marker::SPHERE;
    opp_marker.action = visualization_msgs::Marker::ADD;
    opp_marker.pose.position.x = path.xs[opp_closest_point];
    opp_marker.pose.position.y = path.ys[opp_closest_point];
    opp_marker.pose.orientation.x = 0.0;
    opp_marker.pose.orientation.y = 0.0;
    opp_marker.pose.orientation.z = 0.0;
    opp_marker.pose.orientation.w = 1.0;
    opp_marker.scale.x = 0.2;
    opp_marker.scale.y = 0.2;
    opp_marker.scale.z = 0.2;
    opp_marker.color.a = 1.0;
    opp_marker.color.r = 1.0;
    opp_marker.color.g = 0.0;
    opp_marker.color.b = 0.0;
    
    opp_marker_pub.publish(opp_marker);

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
    computePlacement();
}

void MurettoNode::oppOdomCallback(const nav_msgs::Odometry odom)
{
    opp_desc.x = odom.pose.pose.position.x;
    opp_desc.y = odom.pose.pose.position.y;
    opp_desc.speed = odom.twist.twist.linear.x;
    
    //printf("opponent pose:\t(%f, %f)\nopponent speed:\t%f\n", opp_desc.x, opp_desc.y, opp_desc.speed);
    
    computeDistance();
    computePlacement();
}

void MurettoNode::pathAvailableCallback(const std_msgs::Bool msg)
{
    path_available = msg.data;
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

bool MurettoNode::freePath(vector3d path)
{
    if(race_type == 1)
    {
        for(int i=0; i<path.xs.size(); i++)
        {
            if(!freePose(path.xs[i], path.ys[i]))
                return false;
        }
    } 
    else 
    {
        double distance;
        for(int i=0; i<path.xs.size(); i++)
        {
            distance = std::abs(std::hypot(path.xs[i] - opp_desc.x, path.ys[i] - opp_desc.y));
            if(distance < 1)
                return false;
        }
    }

    return true;
}

void MurettoNode::update_strategy(){
    //main strategy logic

    muretto::Strategy strategy_msg;
    
    //race_type =
    //      0 => time attack
    //      1 => obstacle test
    //      2 => head to head
    if(race_type == 2)
        strategy_msg.race_type = 2;
    else if(obstacle_test)
        strategy_msg.race_type = 1;
    else
        strategy_msg.race_type = 0;

    strategy_msg.ego_path_index = ego_closest_point;

    //overtake_strategy =
    //     -1 => no overtake (vado a bomba)
    //      0 => avoid overtaking (sono dietro ma non è safe sorpassare)
    //      1 => overtake on the left
    //      2 => overtake on the right
    strategy_msg.overtake_strategy = overtake_strategy;

    strategy_msg.right_distance = right_distance;
    strategy_msg.left_distance = left_distance;

    //ahead =
    //      true  => ahead
    //      false => behind
    strategy_msg.ahead = ahead;

    strategy_msg.follow_local_path = !freePath(path);
    if(overtake_strategy == -1)
        strategy_msg.follow_local_path = false;

    strategy_msg.speed_limit = speed_limit;

    //std::cout << "sending strategy msg" << std::endl;

    strategy_pub.publish(strategy_msg);
}

int MurettoNode::path_dist()
{
    int opp_i = opp_closest_point;
    if(opp_closest_point < ego_closest_point)
    {
        opp_i = opp_closest_point + path.xs.size();
    }

    return opp_i - ego_closest_point;
    
}

bool MurettoNode::can_overtake()
{
    for(int i=ego_closest_point; i < ego_closest_point + path_point_horizon; ++i)
    {
        if(path.speed[i] < no_overtake_speed)
            return false;
    }

    return true;
}

void MurettoNode::start()
{
    std::cout << "MURETTO NODE STARTED\n";

    ros::NodeHandle private_node_handler("~");
    std::string param_file;

    private_node_handler.param<std::string>("param_file", param_file, "");

    YAML::Node conf = YAML::LoadFile(param_file);

    max_range = conf["max_range"].as<float>();
    min_range = conf["min_range"].as<float>();
    race_type = conf["race_type"].as<int>();
    safe_distance_after_overtake = conf["safe_dist_overtake"].as<float>();
    overtaking_range = conf["overtaking_range"].as<float>();
    safe_path_dist = conf["safe_path_dist"].as<float>();
    no_overtake_speed = conf["no_overtake_speed"].as<float>();
    path_point_horizon = conf["path_point_horizon"].as<float>();

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
    std::string scan_topic;
    std::string odom_topic;
    std::string opp_odom_topic;
    std::string strategy_topic;
    std::string map_updated_topic;
    std::string ego_marker_topic;
    std::string path_available_topic;
    std::string opp_marker_topic;

    private_node_handler.param<std::string>("global_trajectory", opt_path, "");
    private_node_handler.param<std::string>("scan_topic", scan_topic, "");
    private_node_handler.param<std::string>("odom_topic", odom_topic, "");
    private_node_handler.param<std::string>("opp_odom_topic", opp_odom_topic, "");
    private_node_handler.param<std::string>("strategy_topic", strategy_topic, "");
    private_node_handler.param<std::string>("path_available_topic", path_available_topic, "");
    private_node_handler.param<std::string>("map_updated_topic", map_updated_topic, "");
    private_node_handler.param<std::string>("ego_marker_topic", ego_marker_topic, "");
    private_node_handler.param<std::string>("opp_marker_topic", opp_marker_topic, "");

    path = load_flag_path(opt_path); 

    for(int i=0; i<path.xs.size()-1; i++)
        if(occupancy_grid::is_xy_occupied(input_map_, path.xs[i], path.ys[i]))
            obstacle_test = true;
    //sub, pub
    //obstacles_sub = node_handler.subscribe("/raw_obstacles",10,&MurettoNode::obstacleCallback, this);
    scan_sub = node_handler.subscribe(scan_topic,10,&MurettoNode::scanCallback, this);
    ego_odom_sub = node_handler.subscribe(odom_topic,10,&MurettoNode::egoOdomCallback, this);
    opp_odom_sub = node_handler.subscribe(opp_odom_topic,10,&MurettoNode::oppOdomCallback, this);
    path_available_sub = node_handler.subscribe(path_available_topic,10,&MurettoNode::pathAvailableCallback, this);
    
    strategy_pub = node_handler.advertise<muretto::Strategy>(strategy_topic, 10);
    map_update_pub_ = node_handler.advertise<nav_msgs::OccupancyGrid>(map_updated_topic, 5);
    ego_marker_pub = node_handler.advertise<visualization_msgs::Marker>(ego_marker_topic, 10);
    opp_marker_pub = node_handler.advertise<visualization_msgs::Marker>(opp_marker_topic, 10);

    update_strategy();
    //START SPIN
    //ros::spin();

    int overtake_threshold = 2;
    ros::Rate r(40);

    while(ros::ok())
    {
        // TO-DO: Behavior plannin using state-machine approach -> if-else using the data retrieved
        ros::spinOnce();
        
	    //printf("---------------------------------------------------distance: %f\n", distance_to_opp);
	    //std::cout << "-----------------------------ahead: " << ahead << " \n";

        if(ahead && distance_to_opp > safe_distance_after_overtake)
            overtake_strategy = -1;
        else if(!can_overtake())
            if(path_dist() > safe_path_dist)
                overtake_strategy = -1;
            else
                overtake_strategy = 0;
        else if(!path_available)
            overtake_strategy = -1;
        else if(right_distance > overtake_threshold || overtake_strategy == 2)
            overtake_strategy = 2;
        else if(left_distance > overtake_threshold || overtake_strategy == 1)
            overtake_strategy = 1;
        else
            overtake_strategy = 0;
    
        speed_limit = 20;
        if(overtake_strategy == 0)
            speed_limit = opp_desc.speed;

        /*if(ahead && distance_to_opp > 1)
            overtake_strategy = -1;

        if(overtake_strategy < 1)
        {
            if(ahead)
                overtake_strategy = -1;
            else if(left_distance > overtake_threshold)
                overtake_strategy = 1;
            else if(right_distance > overtake_threshold)
                overtake_strategy = 2;
            else
                overtake_strategy = 0;
        }
        */
        update_strategy();

        r.sleep();
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "muretto_node");
    MurettoNode muretto;
    muretto.start();

    return 0;
}
