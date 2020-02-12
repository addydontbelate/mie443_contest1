#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include "csrw.h"

// timer macros
#define TIME std::chrono::time_point<std::chrono::system_clock>
#define CLOCK std::chrono::system_clock
#define TIME_S std::chrono::duration_cast<std::chrono::seconds>
#define TIME_LIMIT 480 // seconds
#define addydontbelate(time_elapsed) ((time_elapsed <= TIME_LIMIT) ? true : false) 

// fsm states
#define _INIT_ 0             // initialize
#define _RECOVERY_ 1         // recovery
#define _GET_NEW_CRNR_ 2     // get new corner
#define _NAV_TO_CRNR_ 3      // navigate to corner
#define _NAV_TO_CNTR_ 4      // navigate to center
#define _DO_RW_ 5            // do random walk

// global robot state variables
uint8_t rob_state = _INIT_;
uint8_t prev_rob_state = _INIT_; 
float rob_yaw = 0.0;
float rob_pos_x = 0.0;
float rob_pos_y = 0.0;
float min_pos_x = std::numeric_limits<float>::infinity();
float max_pos_x = -std::numeric_limits<float>::infinity();
float min_pos_y = std::numeric_limits<float>::infinity();
float max_pos_y = -std::numeric_limits<float>::infinity();
float front_laser_dist = std::numeric_limits<float>::infinity();
float left_laser_dist = std::numeric_limits<float>::infinity();
float right_laser_dist = std::numeric_limits<float>::infinity();
int32_t n_lasers = 0;
int32_t desired_n_lasers = 0;
int32_t view_angle = 10;        // +-10 deg from heading angle
bool bumper_hit = false;        // recovery mode flag
uint8_t bumper[NUM_BUMPER] = {kobuki_msgs::BumperEvent::RELEASED, 
    kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

// global wall clock timer
uint64_t seconds_elapsed = 0.0;
TIME start;

/**
 * ROS callback to record bumper hit.
 */
void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper] = msg->state;
    
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
        ROS_INFO("%s bumper hit!", (msg->bumper == kobuki_msgs::BumperEvent::LEFT) ? "LEFT" : 
            (msg->bumper == kobuki_msgs::BumperEvent::CENTER) ? "CENTER" : "RIGHT" );
        bumper_hit = true; // set flag
    }

    // update global timer
    seconds_elapsed = TIME_S(CLOCK::now()-start).count();
}

/**
 * ROS callback to set minimum distance from laser over view angle.
 */
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	front_laser_dist = std::numeric_limits<float>::infinity();
    left_laser_dist = std::numeric_limits<float>::infinity();
    right_laser_dist = std::numeric_limits<float>::infinity();
    n_lasers = (msg->angle_max - msg->angle_min)/msg->angle_increment; 
    desired_n_lasers = DEG2RAD(view_angle)/msg->angle_increment;

    // find front_laser_dist over view_angle
    for (uint32_t laser_idx = (n_lasers/2 - desired_n_lasers); laser_idx < (n_lasers/2 + desired_n_lasers); ++laser_idx)
        front_laser_dist = std::min(front_laser_dist, msg->ranges[laser_idx]);
    
    // find right_laser_dist
    for (uint32_t laser_idx = 0; laser_idx < (n_lasers/2 - desired_n_lasers) - 1; ++laser_idx)
        right_laser_dist = std::min(right_laser_dist, msg->ranges[laser_idx]);
    
    // find left_laser_dist
    for (uint32_t laser_idx = (n_lasers/2 + desired_n_lasers) + 1; laser_idx < msg->ranges.size(); ++laser_idx)
        left_laser_dist = std::min(left_laser_dist, msg->ranges[laser_idx]);

    // update global timer
    seconds_elapsed = TIME_S(CLOCK::now()-start).count();
}

/**
 * ROS callback to update robot pose.
 */
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    rob_pos_x = msg->pose.pose.position.x; 
    rob_pos_y = msg->pose.pose.position.y;
    rob_yaw = tf::getYaw(msg->pose.pose.orientation);

    // update global timer
    seconds_elapsed = TIME_S(CLOCK::now()-start).count();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "addydontbelate_explorer");
    ros::NodeHandle nh;
    
    // init subscribers
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumper_callback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laser_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odom_callback);

    // init csrw object
    CSRW csrw(&nh);

    // init loop rate
    ros::Rate loop_rate(10);

    // contest count down timer
    start = CLOCK::now();
    srand(time(0)); // init w time seed for random walk

    // robot loop
    while(ros::ok() && addydontbelate(seconds_elapsed)) 
    {
        // update robot state vars
        ros::spinOnce();
        ROS_INFO("[MAIN] Position: (%f, %f) || Yaw: %f deg || Laser Distance: (Front: %f; Right: %f; Left: %f)", 
            rob_pos_x, rob_pos_y, RAD2DEG(rob_yaw), front_laser_dist, right_laser_dist, left_laser_dist);

        // unexpected hit: move away from hit
        if (bumper_hit)
        {   
            bumper_hit = false; // reset flag
            csrw.respond_to_bump();

            // initiate recovery mode
            rob_state = _RECOVERY_;
            prev_rob_state = prev_rob_state; // keep track of state coming from
        }

        // robot fsm
        if (rob_state == _INIT_)
        {
            ROS_INFO("[MAIN] Robot in INITIAL state");
            csrw.rotate();

            if (prev_rob_state == _DO_RW_)
            {
                rob_state = _DO_RW_;
                prev_rob_state = _INIT_;
            }
            else
            {
                rob_state = _GET_NEW_CRNR_;
                prev_rob_state = _INIT_;   
            }
        }
        else if (rob_state == _RECOVERY_)
        {
            ROS_INFO("[MAIN] Robot in RECOVERY state");
            rob_state = _INIT_;
            prev_rob_state = prev_rob_state; // keep track of where coming from
        }
        else if (rob_state == _GET_NEW_CRNR_)
        {
            ROS_INFO("[MAIN] Robot in GET_NEW_CORNER state");
            csrw.set_goal(); 

            if (!csrw.crnrs_complete())
            {
                rob_state = _NAV_TO_CRNR_;
                prev_rob_state = _GET_NEW_CRNR_;
            }
            else
            {
                rob_state = _NAV_TO_CNTR_;
                prev_rob_state = _GET_NEW_CRNR_;
            }
        }
        else if (rob_state == _NAV_TO_CRNR_)
        {
            ROS_INFO("[MAIN] Robot in NAVIGATE_TO_CORNER state");
            csrw.go_to_goal();
            rob_state = _GET_NEW_CRNR_;
            prev_rob_state = _NAV_TO_CRNR_;
        }
        else if (rob_state == _NAV_TO_CNTR_)
        {
            ROS_INFO("[MAIN] Robot in NAVIGATE_TO_CENTER state");
            csrw.go_to_goal();
            rob_state = _DO_RW_;
            prev_rob_state = _NAV_TO_CNTR_;
        }
        else if (_DO_RW_)
        {
            ROS_INFO("[MAIN] Robot in RANDOM_WALK state");
            csrw.do_rw();
            rob_state = _INIT_;
            prev_rob_state = _DO_RW_;
        }
        else // invalid state stored
        {
            ROS_INFO("[MAIN] Robot in INVALID state! Starting RECOVERY");
            rob_state = _RECOVERY_;
            csrw.pause(); // stop robot movement
        }

        // update the timer
        seconds_elapsed = TIME_S(CLOCK::now()-start).count();
        loop_rate.sleep();
    }

    // time up: stop
    ROS_INFO("[MAIN] Time up! Stopping.");
    csrw.pause();
    return 0;
}