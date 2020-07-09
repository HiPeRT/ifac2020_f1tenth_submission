#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <stdlib.h>

std::string odom_topic;

double x = 0;
double y = 0;
double yaw = 0;
double w = 0;
double last_x, last_y;
double vx, vy, vx_from_sim;

ros::Subscriber odom_sub;
ros::Subscriber drive_sub;
ros::Publisher velocities_pub;

double wheelbase = 0.33;
double dt = 1.0/50;
double steering_angle = 0;

void odom_callback (const nav_msgs::Odometry::ConstPtr &odom_msg) 
{

    last_x = x;
    last_y = y;

    x = odom_msg->pose.pose.position.x;
    y = odom_msg->pose.pose.position.y;

    vx_from_sim = odom_msg->twist.twist.linear.x;

    yaw = odom_msg->twist.twist.angular.z;

}

void drive_callback (const ackermann_msgs::AckermannDriveStamped::ConstPtr &drive_msg) 
{

    steering_angle = drive_msg->drive.steering_angle;

}

void update_velocities() 
{


    vx              = vx_from_sim;
    vy              = 0;
    w               = 0;

    double new_vx = vx;
    double new_vy = 0;

    // compute dots
    double dt      = 1.0/50;

    double R = 0;
    double angular_distance = 0;

    if (steering_angle != 0)
    {
        R = wheelbase / sin(steering_angle);
        angular_distance = vx_from_sim*dt/R;

        double dx = R * sin(angular_distance);
        double dy = R-R*cos(angular_distance);;

        if(steering_angle < 0)
        {
            dy = -(R-R*cos(angular_distance));
        }

        double new_vx = dx / dt;
        double new_vy = dy / dt;
    }

    /*if(new_vx > 1)
    {
        std::cout << "vx: " << new_vx << " vy: " << new_vy << "\n";
        std::cout << "vx_from_sim: " << vx_from_sim << "\n";
    }*/

    geometry_msgs::Point32 msg;
    msg.x = new_vx;
    msg.y = new_vy;
    velocities_pub.publish(msg);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocities_node");
    ros::NodeHandle nh;

    // read param from launch file
    nh.param<std::string>("odometry_topic", odom_topic, "/odom");

    velocities_pub = nh.advertise<geometry_msgs::Point32>("velocities", 1);
    odom_sub = nh.subscribe(odom_topic, 1, &odom_callback); 
    drive_sub = nh.subscribe("/drive", 1, &drive_callback); 
    
    ros::Rate loop_rate(50);
    while (ros::ok()) 
    {

        update_velocities();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


