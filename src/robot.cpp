#include "robot.h"
#include <stdio.h>

void Robot::init(ros::NodeHandle* nh)
{
    rob_vel.angular.z = angular_vel;
    rob_vel.linear.x = linear_vel;
    
    // init velocity publisher
    vel_pub = nh->advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/teleop", 1);
}

void Robot::rotate(float rad, bool clockwise)
{
    float initial_yaw = yaw;

    // Set robot vars
    linear_vel = 0.0;
    if (clockwise)
        angular_vel = -fabs(MAX_ANG_VEL);
    else
        angular_vel = fabs(MAX_ANG_VEL);

	float angle_turned = 0.0;
    while (angle_turned < rad && ros::ok())
    {
        initial_yaw = yaw;
        publish_move();
        loop_rate.sleep();
        angle_turned += fabs(yaw - initial_yaw);
    }
    
    stop();
}

void Robot::move_straight(float dist, float linear_speed, bool forward)
{
	// initial pose before moving
	float initial_pos[2] = {pos[0], pos[1]};

    // Set Nav vars 
    angular_vel = 0.0;
	if (forward)
		linear_vel = fabs(linear_speed); // positive for forward
	else
		linear_vel = -fabs(linear_speed); // nagtive for backwards
    
    // Keep moving as long as distance travelled is under limit
	float dist_moved = 0.0;
	while (dist_moved < dist && ros::ok())
    {
        publish_move();
		ros::spinOnce();
		loop_rate.sleep();		
        dist_moved = sqrt(pow((pos[0] - initial_pos[0]), 2) +
			pow((pos[1] - initial_pos[1]), 2));
	}
	stop();
}

void Robot::move_right(float dist, float linear_speed)
{
    rotate_right();
    move_straight(dist, linear_speed, true);
}

void Robot::move_left(float dist, float linear_speed)
{
    rotate_left();
    move_straight(dist, linear_speed, true);
}

void Robot::rotate_right()
{
    rotate(DEG2RAD(90), true);
}

void Robot::rotate_left()
{
    rotate(DEG2RAD(90), false);
}

void Robot::stop()
{
    rob_vel.angular.z = 0.0;
    rob_vel.linear.x = 0.0;
    publish_move();
}

void Robot::publish_move() 
{   
    ros::spinOnce();
    rob_vel.angular.z = angular_vel;
    rob_vel.linear.x = linear_vel;
    vel_pub.publish(rob_vel);
}

// Callbacks 

void Robot::bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper] = msg->state;
    
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
        ROS_INFO("%s bumper hit!", (msg->bumper == kobuki_msgs::BumperEvent::LEFT) ? "LEFT" : 
            (msg->bumper == kobuki_msgs::BumperEvent::CENTER) ? "CENTER" : "RIGHT" );
        bumper_hit = true; // set flag
    }
}

void Robot::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    float n_lasers = (msg->angle_max - msg->angle_min)/msg->angle_increment; 
    float desired_n_lasers = DEG2RAD(view_angle)/msg->angle_increment;

    // find front_laser_dist over view_angle
    for (uint32_t laser_idx = (n_lasers/2 - desired_n_lasers); laser_idx < (n_lasers/2 + desired_n_lasers); ++laser_idx)
        front_laser_dist = std::min(front_laser_dist, msg->ranges[laser_idx]);
    
    // find left_laser_dist
    for (uint32_t laser_idx = 0; laser_idx < (n_lasers/2 - desired_n_lasers) - 1; ++laser_idx)
        left_laser_dist = std::min(left_laser_dist, msg->ranges[laser_idx]);
    
    // find right_laser_dist
    for (uint32_t laser_idx = (n_lasers/2 + desired_n_lasers) + 1; laser_idx < msg->ranges.size(); ++laser_idx)
        right_laser_dist = std::min(right_laser_dist, msg->ranges[laser_idx]);
}

void Robot::pos_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pos = {msg->pose.pose.position.x, msg->pose.pose.position.y};
    yaw = tf::getYaw(msg->pose.pose.orientation);
}


// void Robot::respond_to_bump()
// {
//     // decrement available responses
//     (num_obst_response > 0) ? (num_obst_response--) : (num_obst_response = 0);

//     // get number of hits
//     uint8_t num_hits = 0;
//     for (uint8_t i = 0; i < NUM_BUMPER; ++i)
//         if (bumper[i] == kobuki_msgs::BumperEvent::PRESSED)
//             num_hits++;
    
//     ROS_INFO("[BUMP_HIT] Detected %d hits!", num_hits);
//     if (num_obst_response > 0)
//     {
//         if (num_hits == 1) // one hit only
//         {
//             // left: move right
//             if (bumper[0] == kobuki_msgs::BumperEvent::PRESSED)
//             {
//                 ROS_INFO("[BUMP_HIT] Left bumper hit. Moving right!");
//                 move_right(SF*OBST_DIST_THRESH, OBST_DET_VEL, MAX_ANG_VEL);
//             }
//             // center: move back
//             else if (bumper[1] == kobuki_msgs::BumperEvent::PRESSED)
//             {
//                 ROS_INFO("[BUMP_HIT] Center bumper hit. Moving back!");
//                 move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, BCK);
//             }
//             // right: move left
//             else
//             {
//                 ROS_INFO("[BUMP_HIT] Right bumper hit. Moving left!");
//                 move_left(SF*OBST_DIST_THRESH, OBST_DET_VEL, MAX_ANG_VEL);
//             }
//         }
//         else if (num_hits == 2) // two hits
//         {
//             // left + center: move back towards right
//             if (bumper[0] == kobuki_msgs::BumperEvent::PRESSED  &&
//                 bumper[1] == kobuki_msgs::BumperEvent::PRESSED)
//             {
//                 ROS_INFO("[BUMP_HIT] Left + Center bumpers hit. Moving back towards right!");
//                 rotate(DEG2RAD(45), MAX_ANG_VEL, CCW);
//                 move_straight(SF*OBST_DIST_THRESH/2, OBST_DET_VEL, BCK);
//                 rotate(DEG2RAD(45), MAX_ANG_VEL, CCW);
//                 move_straight(SF*OBST_DIST_THRESH/2, OBST_DET_VEL, BCK);
//             }
//             // right + center: move back towards left
//             else if (bumper[2] == kobuki_msgs::BumperEvent::PRESSED  &&
//                     bumper[1] == kobuki_msgs::BumperEvent::PRESSED)
//             {
//                 ROS_INFO("[BUMP_HIT] Right + Center bumpers hit. Moving back towards left!");
//                 rotate(DEG2RAD(45), MAX_ANG_VEL, CW);
//                 move_straight(SF*OBST_DIST_THRESH/2, OBST_DET_VEL, BCK);
//                 rotate(DEG2RAD(45), MAX_ANG_VEL, CW);
//                 move_straight(SF*OBST_DIST_THRESH/2, OBST_DET_VEL, BCK);
//             }
//             // right + left (unlikely): move back
//             else
//             {
//                 ROS_INFO("[BUMP_HIT] Left + Right bumpers hit. Moving back!");
//                 move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, BCK);
//             }
//         }
//         else // all bumpers are hit: move back
//         {
//             ROS_INFO("[BUMP_HIT] All bumpers hit. Moving back!");
//             move_straight(SF*OBST_DIST_THRESH, OBST_HIT_DIST, BCK);
//         }
//     }
// }

// void Robot::respond_to_obst()
// {
//     // decrement available responses
//     (num_obst_response > 0) ? (num_obst_response--) : (num_obst_response = 0);

//     ROS_INFO("[NAV_OBST] Front Dist: %f; Right Dist: %f; Left Dist: %f;", front_laser_dist, right_laser_dist, left_laser_dist);

//     if (num_obst_response > 0)
//     {
//         if (front_laser_dist > OBST_DIST_THRESH && left_laser_dist > OBST_DIST_THRESH && 
//             right_laser_dist > OBST_DIST_THRESH)
//         {
//             // no obstacle (unlikely)
//             ROS_INFO("[NAV_OBST] No obstacles around!");
//         }
//         else if (front_laser_dist < OBST_DIST_THRESH && left_laser_dist < OBST_DIST_THRESH && 
//             right_laser_dist < OBST_DIST_THRESH)
//         {
//             // obstacles on all sides: blocked
//             ROS_INFO("[NAV_OBST] Blocked, rotating and moving back!");
//             rotate(DEG2RAD(180), MAX_ANG_VEL, CW);
//             move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
//         }
//         else if (front_laser_dist < OBST_DIST_THRESH && left_laser_dist > OBST_DIST_THRESH && 
//             right_laser_dist > OBST_DIST_THRESH)
//         {
//             // obstacle at the front only
//             ROS_INFO("[NAV_OBST] Obstacle in front, moving around!");
            
//             // obstacle to right farther than left 
//             if (left_laser_dist < right_laser_dist)
//             {
//                 // rotate right relative to the distance from the obstacle
//                 rotate(DEG2RAD((45/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - front_laser_dist) + 45),
//                     MAX_ANG_VEL, CW);
//                 move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
//             }
//             else
//             {
//                 // rotate left relative to the distance from the obstacle
//                 rotate(DEG2RAD((45/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - front_laser_dist) + 45),
//                     MAX_ANG_VEL, CCW);
//                 move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
//             }
//         }
//         else if (front_laser_dist > OBST_DIST_THRESH && left_laser_dist > OBST_DIST_THRESH && 
//             right_laser_dist < OBST_DIST_THRESH)
//         {
//             // obstacle on the right only
//             ROS_INFO("[NAV_OBST] Obstacle at the right, moving towards left!");
            
//             // obstacle to front farther than left 
//             if (left_laser_dist < front_laser_dist)
//             {
//                 // rotate left relative to the distance from the obstacle
//                 rotate(DEG2RAD((15/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - right_laser_dist) + 30),
//                     MAX_ANG_VEL, CCW);
//                 move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
//             }
//             else
//             {
//                 // rotate left relative to the distance from the obstacle
//                 rotate(DEG2RAD((45/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - right_laser_dist) + 45),
//                     MAX_ANG_VEL, CCW);
//                 move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
//             }
//         }
//         else if (front_laser_dist > OBST_DIST_THRESH && left_laser_dist < OBST_DIST_THRESH && 
//             right_laser_dist > OBST_DIST_THRESH)
//         {
//             // obstacle on the left only
//             ROS_INFO("[NAV_OBST] Obstacle at the left, moving towards right!");

//             // obstacle to front farther than right 
//             if (right_laser_dist < front_laser_dist)
//             {
//                 // rotate right relative to the distance from the obstacle
//                 rotate(DEG2RAD((15/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - left_laser_dist) + 30),
//                     MAX_ANG_VEL, CW);
//                 move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
//             }
//             else
//             {
//                 // rotate right relative to the distance from the obstacle
//                 rotate(DEG2RAD((45/(OBST_DIST_THRESH - OBST_HIT_DIST))*(OBST_DIST_THRESH - left_laser_dist) + 45),
//                     MAX_ANG_VEL, CW);
//                 move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
//             }
//         }
//         else if (front_laser_dist < OBST_DIST_THRESH && left_laser_dist > OBST_DIST_THRESH && 
//             right_laser_dist < OBST_DIST_THRESH)
//         {
//             // obstacles on the front and right only
//             ROS_INFO("[NAV_OBST] Obstacles on front and right, moving towards left!");
//             move_left(SF*OBST_DIST_THRESH, OBST_DET_VEL, MAX_ANG_VEL);
//         }
//         else if (front_laser_dist < OBST_DIST_THRESH && left_laser_dist < OBST_DIST_THRESH && 
//             right_laser_dist > OBST_DIST_THRESH)
//         {
//             // obstacles on the front and left only
//             ROS_INFO("[NAV_OBST] Obstacles on front and left, moving towards right!");
//             move_right(SF*OBST_DIST_THRESH, OBST_DET_VEL, MAX_ANG_VEL);
//         }
//         else if (front_laser_dist > OBST_DIST_THRESH && left_laser_dist < OBST_DIST_THRESH && 
//             right_laser_dist < OBST_DIST_THRESH)
//         {
//             // obstacles on the right and left only
//             ROS_INFO("[NAV_OBST] Obstacles on front and left, moving towards right!");
//             move_straight(SF*OBST_DIST_THRESH, OBST_DET_VEL, FWD);
//         }
//         else 
//         {
//             // unknown state
//             stop();   
//         }
//     }
// }

// void Robot::move_to(float goal_x, float goal_y) 
// {
//     ROS_INFO("[NAV] Currently at (%f, %f); Moving to (%f, %f);", pos[0], pos[1], goal_x, goal_y);
//     float m_angle = 0.0;
//     uint8_t num_tries = 0;

//     while ((fabs(pos[0] - goal_x) > GOAL_REACH_DIST || fabs(pos[1] - goal_y) > GOAL_REACH_DIST) && 
//         num_tries < NUM_REPLANS) 
//     {
//         num_obst_response = OBST_RESPONSE_LIM;

//         // rotate towards goal
//         m_angle = atan2f(goal_y - pos[1], goal_x - pos[0]);

//         if (goal_y > 0) // goal ccw
//         {  
//             if (yaw > 0)
//                 (yaw - m_angle > 0) ? rotate(yaw - m_angle, MAX_ANG_VEL, CW) : 
//                     rotate(m_angle - yaw, MAX_ANG_VEL, CCW);
//             else
//                 rotate(m_angle + yaw, MAX_ANG_VEL, CCW);
//         }
//         else // goal cw
//         {
//             if (yaw < 0)
//                 (yaw - m_angle < 0) ? rotate(fabs(yaw - m_angle), MAX_ANG_VEL, CCW) : 
//                     rotate(fabs(m_angle - yaw), MAX_ANG_VEL, CW);
//             else
//                 rotate(m_angle + yaw, MAX_ANG_VEL, CW);
//         }

//         // move straight to goal
//         float dist = sqrt(pow((pos[0] - goal_x), 2) + pow((pos[1] - goal_y), 2));
//         move_straight(dist, FREE_ENV_VEL, FWD);

//         // increment move
//         ROS_INFO("[NAV] Required %d replans so far", num_tries);
//         num_tries++;
//     }
    
//     ROS_INFO("[NAV] Moved to (%f, %f);", pos[0], pos[1]);
// }