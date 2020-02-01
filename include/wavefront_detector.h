#ifndef INCLUDE_WAVEFRONT_DETECTOR_H
#define INCLUDE_WAVEFRONT_DETECTOR_H

#include <vector>
#include <queue>
#include <map>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// definitions
#define UNKNOWN -1 // occupancy unknown
#define OCCUPIED 100
#define UNOCCUPIED 0
#define OCC_THRESHOLD 10  // occupancy threshold value
#define MAP_OPEN_LIST 1
#define MAP_CLOSE_LIST 2
#define FRONTIER_OPEN_LIST 3
#define FRONTIER_CLOSE_LIST 4
#define MIN_FRONTIER_SIZE 5
#define NUM_NBORS 8 // # neighbors of each cell

class Wavefront_Detector 
{
 private:
    bool is_frontier_point(const nav_msgs::OccupancyGrid& map, 
        int pos, int map_size, int map_width);
    void pos_neighbors(int nbor[], int pos, int map_width);
    float points_dist(float x_1, float x_2, float y_1, float y_2);

 public:
    std::vector<std::vector<int>> frontiers(const nav_msgs::OccupancyGrid& map, 
        int map_height, int map_width, int pose);
    int nearest_frontier_idx(const std::vector<geometry_msgs::Point>& frontier_pos);
    int frontier_median(std::vector<int> frontier);
    
    // destructor
    ~Wavefront_Detector() {};
};

#endif  // INCLUDE_WAVEFRONT_DETECTOR_H