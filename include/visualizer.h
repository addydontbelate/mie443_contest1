#ifndef INCLUDE_VISUALIZER_H
#define INCLUDE_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class Visualizer
{
 private:
    // markers?

 public:
    void visualize_frontier();
    void visualize_point();
    void clear_markers();
    ~Visualizer() {};
};

#endif  // INCLUDE_VISUALIZER_H