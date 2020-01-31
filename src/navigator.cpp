#include "navigator.h"

void Navigator::rotate(float rad, float angular_speed, bool clockwise)
{
	nav_msgs::Odometry initial_pose = rob_pose;

    if (clockwise)
        angular_vel = -fabs(angular_vel);
    else
        angular_vel = fabs(angular_vel);

	float angle_turned = 0.0;
    ros::Rate loop_rate(10);
	
    while (angle_turned < rad && ros::ok())
    {
		publish_move();
		ros::spinOnce();
		loop_rate.sleep();
		
        angle_turned = abs(rob_yaw - tf::getYaw(rob_pose.pose.pose.orientation));
	}
    
    stop();
}

void Navigator::move_straight(float dist, float linear_speed, bool forward)
{
	//initial pose before moving
	nav_msgs::Odometry initial_pose = rob_pose;

	if (forward)
		linear_vel = fabs(linear_speed); // positive for forward
	else
		linear_vel = -fabs(linear_speed); // nagtive for backwards
	
	float dist_moved = 0.0;
	ros::Rate loop_rate(10);

	while (dist_moved < dist && ros::ok())
    {
		publish_move();
		ros::spinOnce();
		loop_rate.sleep();
		
        dist_moved = sqrt(pow((rob_pos_x - initial_pose.pose.pose.position.x), 2) +
			pow((rob_pos_y - initial_pose.pose.pose.position.y), 2));
	}

	stop();
}

void Navigator::move_to(float goal_x, float goal_y) 
{
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