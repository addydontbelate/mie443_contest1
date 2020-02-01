#ifndef INCLUDE_VISUALIZER_H
#define INCLUDE_VISUALIZER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

class Visualizer
{
 private:
    // markers?
    // publisher

 public:
    void init(ros::NodeHandle* nh);
    void visualize_frontier();
    void visualize_point();
    void clear_markers();
    ~Visualizer() {};
};

#endif  // INCLUDE_VISUALIZER_H