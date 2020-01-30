#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include "wavefront_detector.h"
#include "navigator.h"
#include "visualizer.h"

// defintiions and global constants
#define NUM_BUMPER (3) // LEFT, CENTER, RIGHT
#define RAD2DEG(rad) ((rad) * 180./M_PI)
#define DEG2RAD(deg) ((deg) * M_PI/180.)
#define TIME_LIMIT 480 // seconds
#define addydontbelate(time_elapsed) ((time_elapsed <= TIME_LIMIT) ? true : false) 

// fsm states
#define _INIT_ 0
#define _RECOVERY_ 1
#define _GET_NEW_FRONTIER_ 2
#define _NAV_TO_FRONTIER_ 3

// global variables
int rob_state = _INIT_;
float rob_pos_x = 0.0;
float rob_pos_y = 0.0;
float goal_pos_x = rob_pos_x;
float goal_pos_y = rob_pos_y;
float yaw = 0.0;
float min_laser_dist = std::numeric_limits<float>::infinity(); 
int32_t n_lasers = 0;
int32_t desired_n_lasers = 0; 
int32_t view_angle = 5; // +-5 deg from heading angle
bool detect_frontier = false; // wfd enable flag
bool bumper_hit = false; // recovery mode flag
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, 
                    kobuki_msgs::BumperEvent::RELEASED, 
                    kobuki_msgs::BumperEvent::RELEASED};

// objects
Wavefront_Detector wfd;
Visualizer viz;
Navigator nav;

// callback functions
void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper] = msg->state;
    
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
        ROS_INFO("%s bumper hit!", (msg->bumper == kobuki_msgs::BumperEvent::LEFT) ? "LEFT" : 
            (msg->bumper == kobuki_msgs::BumperEvent::CENTER) ? "CENTER" : "RIGHT" );
        bumper_hit = true; // set flag
    }
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	min_laser_dist = std::numeric_limits<float>::infinity(); 
    n_lasers = (msg->angle_max - msg->angle_min)/msg->angle_increment; 
    desired_n_lasers = DEG2RAD(view_angle)/msg->angle_increment;

    if (DEG2RAD(view_angle) < msg->angle_max && -DEG2RAD(view_angle) > msg->angle_min) 
    {
        // find min_laser_dist over view_angle
        for (uint32_t laser_idx = (n_lasers/2 - desired_n_lasers); laser_idx < (n_lasers/2 + desired_n_lasers); ++laser_idx)
            min_laser_dist = std::min(min_laser_dist, msg->ranges[laser_idx]);
    } 
    else 
    {
        // use full range if view angle > angle range
        for (uint32_t laser_idx = 0; laser_idx < n_lasers; ++laser_idx) 
            min_laser_dist = std::min(min_laser_dist, msg->ranges[laser_idx]);
    }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    rob_pos_x = msg->pose.pose.position.x; 
    rob_pos_y = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

void map_callback(const nav_msgs::OccupancyGrid& map)
{
    float resolution = map.info.resolution;
    float map_x = map.info.origin.position.x/resolution;
    float map_y = map.info.origin.position.y/resolution;
    float x = 0. - map_x;
    float y = 0. - map_y;

    if (detect_frontier)
    {
        // get frontiers
        std::vector<std::vector<int>> frontiers = wfd.frontiers(map, map.info.height, 
            map.info.width, x + (y * map.info.width));
        ROS_INFO("Found %d frontiers", static_cast<int>(frontiers.size()));

        // visualize frontiers
        // std::vector<std::vector<int>> map_2d(map->info.height, std::vector<int>(map->info.width, 0)); // print to file

        // get frontier medians
        std::vector<int> frontier_median;
        for (int i = 0; i < frontiers.size(); i++)
            frontier_median.push_back(wfd.frontier_median(frontiers[i]));

        // corresponding position of median on map
        std::vector<geometry_msgs::Point> frontier_pos(frontier_median.size());
        
        for (int i = 0; i < frontier_median.size(); i++)
        {
            frontier_pos[i].x = ((frontier_median[i] % map.info.width) + map_x)*resolution;
            frontier_pos[i].y = ((frontier_median[i] / map.info.width) + map_y)*resolution;
            frontier_pos[i].z = 0;
        }

        // get nearest frontier
        int nf_idx = wfd.nearest_frontier_idx(frontier_pos);
        ROS_INFO("Nearest frontier index is: %d", nf_idx);

        //  update goal position
        goal_pos_x = frontier_pos[nf_idx].x;
        goal_pos_y = frontier_pos[nf_idx].y;
        ROS_INFO("Goal position set to: (%f, %f)", goal_pos_x, goal_pos_y);

        detect_frontier = false; // set to true when needing a new frontier
    }
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "auto_explorer");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumper_callback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laser_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odom_callback);
    ros::Subscriber map_sub = nh.subscribe("map", 1, &map_callback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t seconds_elapsed = 0;

    while(ros::ok() && addydontbelate(seconds_elapsed)) 
    {
        ros::spinOnce();
        ROS_INFO("Position: (%f, %f); Orientation: %f deg; Min Laser Dist: %f;", 
            rob_pos_x, rob_pos_y, RAD2DEG(yaw), min_laser_dist);

        // unexpected hit
        if (bumper_hit)
        {
            ; // initiate recovery mode.

            rob_state = _RECOVERY_;
            bumper_hit = false; // reset flag
        }

        // robot fsm
        if (rob_state == _INIT_)
        {
            ROS_INFO("Robot in INIT state");
            ; // rot 360;
            detect_frontier = true;
        }
        else if (rob_state == _RECOVERY_)
        {
            ROS_INFO("Robot in RECOVERY state");
            rob_state = _INIT_; // also make robot move away from obst
        }
        else if (rob_state == _GET_NEW_FRONTIER_)
        {
            ROS_INFO("Robot in GET_NEW_FRONTIER state");
            ; //
        }
        else if (rob_state == _NAV_TO_FRONTIER_)
        {
            ROS_INFO("Robot in NAV_TO_FRONTIER state");
            ; //
        }
        else // invalid state stored
        {
            ROS_INFO("Robot in INVALID state! Starting RECOVERY");
            rob_state = _RECOVERY_;
            nav.stop(); // stop robot movement
        }

        
        // publish next move
        vel.angular.z = nav.get_angular_vel();
        vel.linear.x = nav.get_linear_vel();
        vel_pub.publish(vel);

        // update the timer.
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}