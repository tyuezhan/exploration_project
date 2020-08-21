 
#ifndef FRONTIERPLANNER_H_
#define FRONTIERPLANNER_H_

// #include <grid_map_ros/grid_map_ros.hpp>
#include <ddk_nav_2d/GridMap.h>
#include <angles/angles.h>

#define EXPL_TARGET_SET 1
#define EXPL_FINISHED   2
#define EXPL_WAITING    3
#define EXPL_FAILED     4

class FrontierPlanner {
  public:
    FrontierPlanner();
    ~FrontierPlanner();
    
    int findExplorationTarget(GridMap* map, double current_yaw, unsigned int start, unsigned int &goal);
    void setObstacleScanRange(double range);
    void setGoalFrontierThreshold(int threshold);
    void setFrontierDistanceThreshold(double distance);
    void setFovRange(double fov);
    // int findExplorationTarget(grid_map::GridMap* map, grid_map::Position start, grid_map::Position &goal);

  private:
    double euclidean(double x1, double y1, double x2, double y2);
    double scan_distance_ = 0.8;
    double scan_cell_distance_;
    int goal_frontier_threshold_ = 30;
    double frontier_distance_threshold_ = 0.5;
    double fov_range_ = 120;
};

#endif //