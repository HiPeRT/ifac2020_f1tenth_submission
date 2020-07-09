#include "../include/frenet_optimal_trajectory.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <utility>
#include <ros/console.h>
#include <fstream>
#include <tf/transform_listener.h>
#include <chrono>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "obstacle_detector/Obstacles.h"
#include "obstacle_detector/CircleObstacle.h"
#include <muretto/Strategy.h>

geometry_msgs::Pose pose;
geometry_msgs::Twist twist;

std::vector<obstacle> _obstacles;

void odom_callback(const nav_msgs::Odometry::ConstPtr& data)
{
    pose = data->pose.pose;
    twist = data->twist.twist;
}

void raw_obstacles_callback(const obstacle_detector::Obstacles::ConstPtr obstacles)
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
}

void load_flag_path(vecD *vec_x, vecD *vec_y, vecD *vec_z, std::string trj_file)
{

    std::string line;
    size_t pos = 0;
    std::string temp;
    int i;


    std::ifstream is(trj_file);
	  if (!is)
    {
        std::cout << "File not found!\n";
        return;
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
        vec_x->push_back(atof(line.substr(pos, std::string::npos).c_str()));

        //read y
        std::getline(is,line);
        pos = line.find(":") + 1;
        vec_y->push_back(atof(line.substr(pos, std::string::npos).c_str()));

        //read speed
        std::getline(is,line);
        pos = line.find(":") + 1;
        vec_z->push_back(atof(line.substr(pos, std::string::npos).c_str()));
    }

}

double calc_dis(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}


void find_nearest_in_global_path(vecD global_x, vecD global_y, double &min_x, double &min_y, double &min_dis, int &min_id)
{
	double bot_x = pose.position.x;
	double bot_y = pose.position.y;
	
	min_dis = FLT_MAX;
	for(int i = 0; i < global_x.size(); i++)
	{
		double dis = calc_dis(global_x[i], global_y[i], bot_x, bot_y);
		if(dis < min_dis)
		{
			min_dis = dis;
			min_x = global_x[i];
			min_y = global_y[i];
			min_id = i;
		}
	}
}


double calc_s(double ptx, double pty, vecD global_x, vecD global_y)
{
	double s = 0;
	if(global_x[0] == ptx && global_y[0] == pty)
		return s;

	for(int i = 1; i < global_x.size(); i++)
	{
		double dis = calc_dis(global_x[i], global_y[i], global_x[i - 1], global_y[i - 1]);
		s = s + dis;

		if(global_x[i] == ptx && global_y[i] == pty)
			break;
	}
    //cout<<"HO TROVATO S: "<<s<<endl;
	return s;
}


double get_bot_yaw()
{
	//geometry_msgs::Pose p = pose.pose;
	
	//tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	
	tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	tf::Matrix3x3 m(q);

	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
    
	return yaw;
}


vecD global_path_yaw(Spline2D csp, vecD gx, vecD gy)
{
	vecD yaw;
	vecD t = csp.calc_s(gx, gy);
	for(int i = 0; i < t.size(); i++)
	{
		yaw.push_back(csp.calc_yaw(t[i]));
	}
	return yaw;
}

double find_s(vecD global_x, vecD global_y)
{
    double min_x, min_y;
	int min_id;
    double c_d;
	// getting d
	find_nearest_in_global_path(global_x, global_y, min_x, min_y, c_d, min_id); // in realta' e' in_local_path, magari rinominare funzione per creare meno confusione..
	return calc_s(min_x, min_y, global_x, global_y);
}

void initial_conditions(Spline2D csp, vecD global_x, vecD global_y, vecD ryaw, double &s0, double &c_speed, double &c_d, double &c_d_d, double &c_d_dd)
{
	double vx = twist.linear.x;
	double vy = twist.linear.y;
	double v = sqrt(vx*vx + vy*vy);
	
	
	double min_x, min_y;
	int min_id;

	// getting d
	find_nearest_in_global_path(global_x, global_y, min_x, min_y, c_d, min_id);

	// deciding the sign for d 
	pair<double, double> vec1, vec2;
	vec1.first = pose.position.x - global_x[min_id];
	vec1.second = pose.position.y - global_y[min_id];

	vec2.first = global_x[min_id] - global_x[min_id + 1];
	vec2.second = global_y[min_id] - global_y[min_id + 1];
	double curl2D = vec1.first*vec2.second - vec2.first*vec1.second;
	if(curl2D < 0) 
		c_d *= -1;
    
	s0 = calc_s(min_x, min_y, global_x, global_y);
	//cout<<"CALCOLATO S0: "<<s0<<endl;

	double bot_yaw = get_bot_yaw();
	
	
	vecD theta = global_path_yaw(csp, global_x, global_y);
	double g_path_yaw = theta[min_id];	
    
    //cout<<"BOT_YAW: "<<bot_yaw<<"G_PATH_YAW: "<<g_path_yaw<<endl;
    
	double delta_theta = bot_yaw - g_path_yaw;
    
	//c_d_d = v*sin(delta_theta);   // e' settato a 0 da loro perche' deve essere "aggiornato"
    
	double k_r = csp.calc_curvature(s0);

	c_speed = v*cos(delta_theta) / (1 - k_r*c_d);

	c_d_dd = 0; // For the time being. Need to be updated
}

void publishPath(nav_msgs::Path &path_msg, FrenetPath path, vecD rk, vecD ryaw, double &c_speed, double &c_d, double &c_d_d)
{
    //cout<<"X SIZE: "<<path.x.size()<<endl;
	for(int i = 0; i < path.x.size(); i++)
		{
			geometry_msgs::PoseStamped loc;
			loc.pose.position.x = path.x[i];
			loc.pose.position.y = path.y[i];

			double delta_theta = atan(c_d_d / ((1 - rk[i]*c_d)*c_speed));       // c_d_d fisso a 0 mi pare, quindi boh, verificare effettivamente se yaw ha senso...
			double yaw = delta_theta + ryaw[i];

			tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw); // roll , pitch = 0
			q.normalize();
			quaternionTFToMsg(q, loc.pose.orientation);

			path_msg.poses.push_back(loc);
		}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "frenet_planner");
	ros::NodeHandle n;
	
  ros::NodeHandle privateNodeHandler("~");
  std::string trj_file;
  privateNodeHandler.param<std::string>("trj", trj_file, "");

	ros::Publisher frenet_path = n.advertise<nav_msgs::Path>("/local_path", 1);		//Publish frenet path
	ros::Publisher pub_first_path = n.advertise<nav_msgs::Path>("/first_path", 1);		//Publish frenet path
	ros::Publisher pub_last_path = n.advertise<nav_msgs::Path>("/last_path", 1);		//Publish frenet path
	ros::Publisher global_path = n.advertise<nav_msgs::Path>("/global_path", 1);		//Publish global path
	
	ros::Publisher pub_target_pose = n.advertise<geometry_msgs::PoseStamped>("/frenet_cpp_target", 10);			//Publish target pose //serve implementato il pp
	ros::Publisher pub_frenet_coo  = n.advertise<geometry_msgs::PoseArray>("/frenet_cpp_coo", 10);			//Publish frenet path coordinates
  
	//ros::Subscriber raw_obstacles_sub = n.subscribe("/raw_obstacles", 40, raw_obstacles_callback);  //statici
	ros::Subscriber raw_obstacles_sub = n.subscribe("/obstacles", 40, raw_obstacles_callback);  //dinamici
    
	ros::Subscriber odom_sub = n.subscribe("odom", 40, odom_callback);		//Subscribe the initial conditions
	
	ros::Publisher ready_pub = n.advertise<std_msgs::Int8>("/ready", 1);		//Publish global path
	
	
	FrenetPath first_path, last_path;
	
	
	ros::Rate rate(80);
    //ROS_INFO("Getting params");
    // get params
    /*n.param("/frenet_planner/path/max_speed", MAX_SPEED, 100.0);
    n.param("/frenet_planner/path/max_accel", MAX_ACCEL, 100.0);
    n.param("/frenet_planner/path/max_curvature", MAX_CURVATURE, 100.0);
    n.param("/frenet_planner/path/max_road_width", MAX_ROAD_WIDTH, 1.2);
    n.param("/frenet_planner/path/d_road_w", D_ROAD_W, 0.1);
    n.param("/frenet_planner/path/dt", DT, 0.3);
    n.param("/frenet_planner/path/maxt", MAXT, 1.5);
    n.param("/frenet_planner/path/mint", MINT, 1.4);    //1.2
    n.param("/frenet_planner/path/target_speed", TARGET_SPEED, 3.0);
    n.param("/frenet_planner/path/d_t_s", D_T_S, 1.0);
    n.param("/frenet_planner/path/n_s_sample", N_S_SAMPLE, 0.5);
    n.param("/frenet_planner/path/robot_radius", ROBOT_RADIUS, 0.3);
    n.param("/frenet_planner/cost/kj", KJ, 0.03);   //0.03  // piu' e' alto, piu' mantiene la stessa traiettoria
    n.param("/frenet_planner/cost/kt", KT, 0.0);           // preferisci traiettorie che richiedono piu' tempo, quindi quelle piu' lunghe
    n.param("/frenet_planner/cost/kd", KD, 2.0);    //2.0
    n.param("/frenet_planner/cost/klon", KLON, 1.0);
    n.param("/frenet_planner/cost/klat", KLAT, 1.0);*/
    // cout << "param " << MAX_SPEED << endl;	

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	vecD wx, wy, speeds;

	load_flag_path(&wx, &wy, &speeds, trj_file);
	vecD rx, ry, ryaw, rk;

	double ds = 0.1;	//ds represents the step size for cubic_spline
	//Global path is made using the waypoints
	Spline2D csp = calc_spline_course(wx, wy, rx, ry, ryaw, rk, ds);
	//ROS_INFO("Spline is made");
	
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "tempo spline = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

	double s0, c_d, c_d_d, c_d_dd, c_speed ;
	
  
  geometry_msgs::PoseStamped target_pose;
  FrenetPath path;
	while(ros::ok())
	{
		std_msgs::Int8 ready;
		ready.data = 1;
	    ready_pub.publish(ready);
	    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	    
		//Specifing initial conditions for the frenet planner using ODOMETRY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		initial_conditions(csp, rx, ry, ryaw, s0, c_speed, c_d, c_d_d, c_d_dd);
		//ROS_INFO("Initial conditions set");
		
		//std::cout<<"c_speed: "<<c_speed<<endl;
		double actual_speed = std::min(10.0, sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y));
		//c_speed = std::min(0.3, sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y));
		c_speed = std::min(3.0, actual_speed);

		//std::cout<<"c_speed: "<<c_speed<<endl;
		for (auto const& ob : _obstacles)
		{
		    //cout<<"ostacoli\n";
		    c_speed = 0.5; //1.0
		    break;
		}

		//std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, _obstacles, first_path, last_path);
    //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //std::cout << "tempo iterazione = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
		
		if (path.s.size() == 0)
		{
		    cout<<"NO PATH"<<endl;
		    //fermare macchina
		    continue;
		}
		
		//ROS_INFO("Frenet path created");

		nav_msgs::Path path_msg;
		nav_msgs::Path first_msg;
		nav_msgs::Path last_msg;
		nav_msgs::Path global_path_msg;

		// paths are published in map frame
		path_msg.header.frame_id = "map";
		first_msg.header.frame_id = "map";
		last_msg.header.frame_id = "map";
		global_path_msg.header.frame_id = "map";
        
		//Global path pushed into the message TOGLIERE DAL CICLO WHILE, BASTA FARLO UNA SOLA VOLTA ALL'INIZIO
		for(int i = 0; i < rx.size(); i++)
		{
			geometry_msgs::PoseStamped loc;
			loc.pose.position.x = rx[i];
			loc.pose.position.y = ry[i];
			global_path_msg.poses.push_back(loc);
		}

		//Required tranformations on the Frenet path are made and pushed into message
		// non fa trasformazioni vere e proprie, ma aggiunge le pos calcolate a path_msg e calcola le orientazioni, quest'ultime da verificare
		    publishPath(path_msg, path, rk, ryaw, c_d, c_speed, c_d_d);	
        publishPath(first_msg, first_path, rk, ryaw, c_d, c_speed, c_d_d);
        publishPath(last_msg, last_path, rk, ryaw, c_d, c_speed, c_d_d);
        

        ///double target_x, target_y;
        //vecD lrx, lry, lryaw, lrk;
        //Spline2D csp_local = calc_spline_course(path.x, path.y, lrx, lry, lryaw, lrk, ds);


        vector<geometry_msgs::Pose> vecP;
        for (int i=0; i<path.x.size(); i++)
        {
            geometry_msgs::Pose px;
            px.position.x = path.x[i];
            px.position.y = path.y[i];
            vecP.push_back(px);
        }
        
        geometry_msgs::PoseArray pArray;
        pArray.header.frame_id = "map";
        pArray.poses = vecP;
        
		frenet_path.publish(path_msg);
		pub_first_path.publish(first_msg);
		pub_last_path.publish(last_msg);
		global_path.publish(global_path_msg);
		
		pub_frenet_coo.publish(pArray);
		
		
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "tempo iterazione = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
		//std::cout<< std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
		//return 0;
		//ROS_INFO("Path published");
		ros::spinOnce();
		rate.sleep();
	}
}

