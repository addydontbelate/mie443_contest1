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
            publish_move();
            ros::spinOnce();
            loop_rate.sleep();
            
            angle_turned += fabs(rob_yaw - initial_yaw);
        }
    else
    {
        while (angle_turned < M_PI && ros::ok())
        {
            initial_yaw = rob_yaw;
            publish_move();
            ros::spinOnce();
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
		publish_move();
		ros::spinOnce();
		loop_rate.sleep();
		
        dist_moved = sqrt(pow((rob_pos_x - initial_pos_x), 2) +
			pow((rob_pos_y - initial_pos_y), 2));
	}

	stop();
}

void Navigator::move_to(float goal_x, float goal_y) 
{
    ROS_INFO("Currently at (%f, %f);\t Moving to (%f, %f);", rob_pos_x, rob_pos_y, goal_x, goal_y);

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
    
    ROS_INFO("Moved to (%f, %f);", rob_pos_x, rob_pos_y);
}

void Navigator::move_right(float dist, float linear_speed, float angular_speed)
{
    ROS_INFO("Currently at (%f, %f);", rob_pos_x, rob_pos_y);
    rotate_right(angular_speed);
    move_straight(dist, linear_speed, FWD);
    ROS_INFO("Moved to (%f, %f);", rob_pos_x, rob_pos_y);
}

void Navigator::move_left(float dist, float linear_speed, float angular_speed)
{
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