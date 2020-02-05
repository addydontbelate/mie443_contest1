#include "navigator.h"

Navigator::Navigator(ros::NodeHandle* nh)
{
    // init velocities
    angular_vel = 0.0; 
    linear_vel = 0.0;
    rob_vel.angular.z = angular_vel;
    rob_vel.linear.x = linear_vel;
    
    // init velocity publisher
    vel_pub = nh->advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/teleop", 1);
}

void Navigator::rotate(float rad, float angular_speed, bool clockwise)
{
	ROS_INFO("Currently at (%f, %f) @ %f deg;", rob_pos_x, rob_pos_y, RAD2DEG(rob_yaw));
    
    float initial_yaw;

    if (clockwise)
        angular_vel = -fabs(angular_speed);
    else
        angular_vel = fabs(angular_speed);

    linear_vel = 0.0;

	float angle_turned = 0.0;
    ros::Rate loop_rate(10);
	
    if (rad < M_PI)
        while (angle_turned < rad && ros::ok())
        {
            initial_yaw = rob_yaw;
            ros::spinOnce();
            publish_move();
            loop_rate.sleep();
            
            angle_turned += fabs(rob_yaw - initial_yaw);
        }
    else
    {
        while (angle_turned < M_PI && ros::ok())
        {
            initial_yaw = rob_yaw;
            ros::spinOnce();
            publish_move();
            loop_rate.sleep();
            
            angle_turned += fabs(rob_yaw - initial_yaw);
        }

        rotate(rad-M_PI, angular_speed, clockwise);
    }
    
    stop();

    ROS_INFO("Rotated to (%f, %f) @ %f deg;", rob_pos_x, rob_pos_y, RAD2DEG(rob_yaw));
}

void Navigator::move_straight(float dist, float linear_speed, bool forward)
{
	//initial pose before moving
	float initial_pos_x = rob_pos_x;
    float initial_pos_y = rob_pos_y;

	if (forward)
		linear_vel = fabs(linear_speed); // positive for forward
	else
		linear_vel = -fabs(linear_speed); // nagtive for backwards
	
    angular_vel = 0.0;

	float dist_moved = 0.0;
	ros::Rate loop_rate(10);

	while (dist_moved < dist && ros::ok())
    {
		ros::spinOnce();

        if (bumper_hit)
        {   
            respond_to_bump();
            bumper_hit = false; // reset flag
            return; // recalculate move
        }

        publish_move();
		loop_rate.sleep();
		
        dist_moved = sqrt(pow((rob_pos_x - initial_pos_x), 2) +
			pow((rob_pos_y - initial_pos_y), 2));
	}

	stop();
}

void Navigator::move_to(float goal_x, float goal_y) 
{
    ROS_INFO("Currently at (%f, %f);\t Moving to (%f, %f);", rob_pos_x, rob_pos_y, goal_x, goal_y);
    float m_angle;
    uint8_t num_tries = 0;
    
    // while not at goal: replan and get to goal (< NUM_REPLANS)
    while (fabs(rob_pos_x - goal_x) < GOAL_REACH_DIST && fabs(rob_pos_y - goal_y) < GOAL_REACH_DIST && 
        num_tries < NUM_REPLANS)
    {
        ros::spinOnce;
        // rotate towards goal
        float m_angle = atan2f(goal_y - rob_pos_y, goal_x - rob_pos_x);

        if (goal_y > 0) // goal ccw
        {  
            if (rob_yaw > 0)
                (rob_yaw - m_angle > 0) ? rotate(rob_yaw - m_angle, MAX_ANG_VEL, CW) : 
                    rotate(m_angle - rob_yaw, MAX_ANG_VEL, CCW);
            else
                rotate(m_angle + rob_yaw, MAX_ANG_VEL, CCW);
        }
        else // goal cw
        {
            if (rob_yaw < 0)
                (rob_yaw - m_angle < 0) ? rotate(fabs(rob_yaw - m_angle), MAX_ANG_VEL, CCW) : 
                    rotate(fabs(m_angle - rob_yaw), MAX_ANG_VEL, CW);
            else
                rotate(m_angle + rob_yaw, MAX_ANG_VEL, CW);
        }

        // move straight to goal
        float dist = sqrt(pow((rob_pos_x - goal_x), 2) + pow((rob_pos_y - goal_y), 2));
        move_straight(dist, FREE_ENV_VEL, FWD);
    }

    ROS_INFO("Moved to (%f, %f);", rob_pos_x, rob_pos_y);
}

void Navigator::move_right(float dist, float linear_speed, float angular_speed)
{
    // TODO: while not at goal: try this over and over till num_try = 5?

    ROS_INFO("Currently at (%f, %f);", rob_pos_x, rob_pos_y);
    rotate_right(angular_speed);
    move_straight(dist, linear_speed, FWD);
    ROS_INFO("Moved to (%f, %f);", rob_pos_x, rob_pos_y);
}

void Navigator::move_left(float dist, float linear_speed, float angular_speed)
{
    // TODO: while not at goal: try this over and over till num_try = 5?

    ROS_INFO("Currently at (%f, %f);", rob_pos_x, rob_pos_y);
    rotate_left(angular_speed);
    move_straight(dist, linear_speed, FWD);
    ROS_INFO("Moved to (%f, %f);", rob_pos_x, rob_pos_y);
}

void Navigator::rotate_right(float angular_speed)
{
    rotate(DEG2RAD(90), angular_speed, CW);
}

void Navigator::rotate_left(float angular_speed)
{
    rotate(DEG2RAD(90), angular_speed, CCW);
}

void Navigator::respond_to_bump()
{
    // get number of hits
    uint8_t num_hits = 0;
    for (uint8_t i = 0; i < NUM_BUMPER; ++i)
        if (bumper[i] == kobuki_msgs::BumperEvent::PRESSED)
            num_hits++;
    
    if (num_hits == 1) // one hit only
    {
        // left: move right
        if (bumper[0] == kobuki_msgs::BumperEvent::PRESSED)
            move_right(OBST_HIT_DIST, OBST_DET_VEL, MAX_ANG_VEL);
        // center: move back
        else if (bumper[1] == kobuki_msgs::BumperEvent::PRESSED)
            move_straight(OBST_HIT_DIST, OBST_DET_VEL, BCK);
        // right: move left
        else
            move_left(OBST_HIT_DIST, OBST_DET_VEL, MAX_ANG_VEL);;
    }
    else if (num_hits == 2) // two hits
    {
        // left + center: move back towards right
        if (bumper[0] == kobuki_msgs::BumperEvent::PRESSED  &&
            bumper[1] == kobuki_msgs::BumperEvent::PRESSED)
        {
            move_straight(OBST_HIT_DIST/2, OBST_DET_VEL, BCK);
            rotate(DEG2RAD(45), MAX_ANG_VEL, CCW);
            move_straight(OBST_HIT_DIST/2, OBST_DET_VEL, BCK);
        }
        // right + center: move back towards left
        else if (bumper[2] == kobuki_msgs::BumperEvent::PRESSED  &&
                bumper[1] == kobuki_msgs::BumperEvent::PRESSED)
        {
            move_straight(OBST_HIT_DIST/2, OBST_DET_VEL, BCK);
            rotate(DEG2RAD(45), MAX_ANG_VEL, CW);
            move_straight(OBST_HIT_DIST/2, OBST_DET_VEL, BCK);
        }
        // right + left (unlikely): move back
        else
            move_straight(OBST_HIT_DIST, OBST_DET_VEL, BCK);
    }
    else // all bumpers are hit: move back
    {
        move_straight(OBST_HIT_DIST, OBST_HIT_DIST, BCK);
    }
}

void Navigator::respond_to_obst()
{
    ;
}

void Navigator::stop()
{
    rob_vel.angular.z = 0.0;
    rob_vel.linear.x = 0.0;
    publish_move();
}

void Navigator::publish_move() 
{
    rob_vel.angular.z = angular_vel;
    rob_vel.linear.x = linear_vel;
    vel_pub.publish(rob_vel);
}