#include <ddk_nav_2d/frontier_planner.h>
#include <math.h>

#define PI 3.14159265

typedef std::pair<double,unsigned int> Entry; // Euclidean + index
typedef std::multimap<double, Entry> Queue; // Manhatan, Euclidean, index

FrontierPlanner::FrontierPlanner(std::string map_frame) {
  map_frame_ = map_frame;
  frontier_costs_pub_ = nh_.advertise<visualization_msgs::Marker>("frontier_cost", 5);
  fov_frontier_pub_ = nh_.advertise<visualization_msgs::Marker>("fov_frontier", 5);
  penalize_factor_ = 2;
  obstacle_penalize_dis_ = 0.2;
  close_obstacle_penalize_factor_ = 8;
  fov_cost_ = 15;
}

FrontierPlanner::~FrontierPlanner() {}


int FrontierPlanner::findExplorationTarget(GridMap* map, double current_yaw, unsigned int start, unsigned int &goal) {
  // Create some workspace for the wavefront algorithm
  unsigned int mapSize = map->getSize();
  double* plan = new double[mapSize];
  for(unsigned int i = 0; i < mapSize; i++)
  {
    plan[i] = -1;
  }
  
  // Initialize the queue with the robot position
  Queue queue;
  Entry startPoint(0.0, start);
  queue.insert(std::make_pair(0.0, startPoint));
  plan[start] = 0;
  
  Queue::iterator next;
  double distance;
  double resolution = map->getResolution();
  bool foundFrontier = false;
  int cellCount = 0;
  double obstacle_dis;

  // Init priority queue for frontier
  Queue frontier_queue;

  visualization_msgs::Marker fov_frontier_marker;
  fov_frontier_marker.scale.x = .05;
  fov_frontier_marker.scale.y = .05;
  fov_frontier_marker.scale.z = .05;
  fov_frontier_marker.color.a = 1;
  fov_frontier_marker.color.r = 1;
  fov_frontier_marker.pose.orientation.w = 1.0;
  fov_frontier_marker.header.frame_id = map_frame_;
  fov_frontier_marker.header.stamp = ros::Time::now();
  fov_frontier_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  std_msgs::ColorRGBA fov_frontier_color;
  fov_frontier_color.a = 1;
  
  geometry_msgs::Point fov_frontier_pt;

  // Do full search with weightless Dijkstra-Algorithm
  while(!queue.empty())
  {
    cellCount++;
    // Get the nearest cell from the queue
    next = queue.begin();
    distance = next->first;
    double euclidean_path_distance = std::get<0>(next->second);
    unsigned int index = std::get<1>(next->second);
    queue.erase(next);
    
    if(map->isFrontier(index)) {
      // We reached the border of the map, which is unexplored terrain as well:
      foundFrontier = true;
      fov_frontier_color.r = 0;
      fov_frontier_color.g = 0.5;
      fov_frontier_color.b = 0;

      // Calculate frontier fov
      unsigned int start_x = 0, start_y = 0, goal_x = 0, goal_y = 0;
      map->getCoordinates(start_x, start_y, start);
      map->getCoordinates(goal_x, goal_y, index);
      double world_start_x = 0, world_start_y = 0, world_goal_x = 0, world_goal_y = 0;

      getWorldCoordinate(map, start_x, start_y, world_start_x, world_start_y);
      getWorldCoordinate(map, goal_x, goal_y, world_goal_x, world_goal_y);
      double fov_cost = 0;
      double goal_yaw = atan2(world_goal_y-world_start_y, world_goal_x-world_start_x);

      double diff = std::abs(angles::shortest_angular_distance(current_yaw, goal_yaw));
      // ROS_INFO("start pos: %f, %f, goal pos: %f, %f, current_yaw: %f, goal_yaw: %f, diff: %f", world_start_x, world_start_y, world_goal_x, world_goal_y, current_yaw, goal_yaw, diff);
      if (diff > angles::from_degrees(fov_range_ / 2)) {
        fov_cost = fov_cost_;
        fov_frontier_color.r = 1.0;
        fov_frontier_color.g = 0;
        // ROS_INFO("penalize ouside fov");
      }
      fov_frontier_pt.x = world_goal_x;
      fov_frontier_pt.y = world_goal_y;
      fov_frontier_pt.z = 0.5;
      fov_frontier_marker.points.push_back(fov_frontier_pt);
      fov_frontier_marker.colors.push_back(fov_frontier_color);


      // scan nearby obstacle
      Queue scan_quque;
      // scan_cell_distance_ = scan_distance_ / resolution;
      Entry frontier_cell(0.0, index);
      scan_quque.insert(std::make_pair(0.0, frontier_cell));
      mapSize = map->getSize();
      double* frontier_plan = new double[mapSize];
      for(unsigned int i = 0; i < mapSize; i++)
      {
        frontier_plan[i] = -1;
      }
      frontier_plan[index] = 0;
      Queue::iterator scan_next;
      double distance_to_frontier;
      bool found_obstacle = false;

      while(!scan_quque.empty()){
        scan_next = scan_quque.begin();
        distance_to_frontier = scan_next->first;
        // double euclidean_distance_to_frontier = std::get<0>(scan_next->second);
        unsigned int curr_index = std::get<1>(scan_next->second);
        scan_quque.erase(scan_next);
        if (!map->isObstacle(curr_index)){
          unsigned int neighbor[4];
          neighbor[0] = curr_index - 1;               // left
          neighbor[1] = curr_index + 1;               // right
          neighbor[2] = curr_index - map->getWidth(); // up
          neighbor[3] = curr_index + map->getWidth(); // down
          for(unsigned int it = 0; it < 4; it++){
            unsigned int i = neighbor[it];
            if (i < 0 || i > mapSize) continue;
            if (frontier_plan[i] == -1) {
              // check if within scan distance
              unsigned int check_x, check_y;
              if(map->getCoordinates(check_x, check_y, i)) {
                double world_check_x = 0, world_check_y = 0;
                getWorldCoordinate(map, check_x, check_y, world_check_x, world_check_y);
                double check_dis = euclidean(world_check_x, world_check_y, world_goal_x, world_goal_y);
                if (check_dis <= scan_distance_) {
                  scan_quque.insert(std::make_pair(distance_to_frontier+resolution, Entry(check_dis, i)));
                  frontier_plan[i] = distance_to_frontier+resolution;
                }
              }
            }
          }
        } else {
          // found obstacle
          found_obstacle = true;
          unsigned int obs_x, obs_y;
          double world_obs_x = 0, world_obs_y = 0;
          map->getCoordinates(obs_x, obs_y, curr_index);
          getWorldCoordinate(map, obs_x, obs_y, world_obs_x, world_obs_y);
          obstacle_dis = euclidean(world_obs_x, world_obs_y, world_goal_x, world_goal_y);
          break;
        }
      }
      if (!found_obstacle){
        // ROS_INFO("dont find a obstacle for this frontier");
        // obstacle_dis = scan_cell_distance_;
        obstacle_dis = scan_distance_;
      }
      // calculate total cost and put frontier into priority queue
      double penalize_factor = penalize_factor_;
      if (obstacle_dis < obstacle_penalize_dis_) penalize_factor = close_obstacle_penalize_factor_;
      double total_cost = distance/std::sqrt(2) + (scan_distance_ - obstacle_dis) * penalize_factor + fov_cost;
      // double total_cost = goal_dis + (scan_cell_distance_ - obstacle_dis) * penalize_factor + fov_cost;
      // double total_cost = (distance / resolution) + (scan_cell_distance_ - obstacle_dis) * penalize_factor + fov_cost;
      ROS_INFO("Total cost for curr frontier: %f, path cost(man): %f, path_cost: %f, obs cost: %f, fov_cost: %f", total_cost, distance/std::sqrt(2), euclidean_path_distance, (scan_distance_ - obstacle_dis) * penalize_factor, fov_cost);
      frontier_queue.insert(std::make_pair(total_cost, Entry(euclidean_path_distance, index)));
      delete[] frontier_plan;

    } else {
      unsigned int ind[4];

      ind[0] = index - 1;               // left
      ind[1] = index + 1;               // right
      ind[2] = index - map->getWidth(); // up
      ind[3] = index + map->getWidth(); // down
      
      for(unsigned int it = 0; it < 4; it++) {
        unsigned int i = ind[it];

        if(map->isFree(i) && plan[i] == -1) {
          // calculate euclidean distance
          // unsigned int neighbor_x = 0, neighbor_y = 0, start_x = 0, start_y = 0;
          // double world_start_x = 0, world_start_y = 0;
          // map->getCoordinates(start_x, start_y, start);
          // getWorldCoordinate(map, start_x, start_y, world_start_x, world_start_y);
          // double world_neighbor_x = 0, world_neighbor_y = 0;
          // map->getCoordinates(neighbor_x, neighbor_y, i);
          // getWorldCoordinate(map, neighbor_x, neighbor_y, world_neighbor_x, world_neighbor_y);
          // double neighbor_dis = euclidean(world_neighbor_x, world_neighbor_y, world_start_x, world_start_y);
          // queue.insert(std::make_pair(distance+resolution, Entry(neighbor_dis, i)));

          // Insert distance as 0, not used for now.
          queue.insert(std::make_pair(distance+resolution, Entry(0, i)));
          plan[i] = distance+resolution;
        }
      }
    }
  }

  fov_frontier_pub_.publish(fov_frontier_marker);

  ROS_DEBUG("Checked %d cells.", cellCount);	
  delete[] plan;
  
  if (!frontier_queue.empty()){
    // visualization
    visualization_msgs::Marker frontier_marker;
    frontier_marker.scale.x = .05;
    frontier_marker.scale.y = .05;
    frontier_marker.scale.z = .05;
    frontier_marker.color.a = 1;
    frontier_marker.color.r = 1;
    frontier_marker.pose.orientation.w = 1.0;
    frontier_marker.header.frame_id = map_frame_;
    frontier_marker.header.stamp = ros::Time::now();
    frontier_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    std_msgs::ColorRGBA color;
    color.a = 1;
    geometry_msgs::Point frontier_pt;

    for (Queue::iterator it=frontier_queue.begin(); it != frontier_queue.end(); it++) {
      double frontier_cost = it->first;
      // double frontier_euclidean_distance = std::get<0>(it->second);
      unsigned int frontier_ind = std::get<1>(it->second);
      unsigned int frontier_x = 0, frontier_y = 0;
      map->getCoordinates(frontier_x, frontier_y, frontier_ind);
      if (frontier_cost < fov_cost_/2){
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
      } else if (frontier_cost < fov_cost_){
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        
      } else {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
      }
      float map_frontier_x = map->getOriginX() + (((double)frontier_x + 0.5) * resolution);
      float map_frontier_y = map->getOriginY() + (((double)frontier_y + 0.5) * resolution);
      float map_frontier_z = 0.5;

      frontier_pt.x = map_frontier_x;
      frontier_pt.y = map_frontier_y;
      frontier_pt.z = map_frontier_z;
      // Debug usage, delete later
      // ROS_INFO("frontier: x: %f, y: %f, z: %f", map_frontier_x, map_frontier_y, map_frontier_z);
      frontier_marker.colors.push_back(color);
      frontier_marker.points.push_back(frontier_pt);
    }
    frontier_costs_pub_.publish(frontier_marker);

    if (frontier_queue.size() <= goal_frontier_threshold_) {
      ROS_INFO("Less than %d frontiers. Return exploration finished.", goal_frontier_threshold_);
      return EXPL_FINISHED;
    } else {
      for (Queue::iterator it = frontier_queue.begin(); it != frontier_queue.end(); it++){
        unsigned int check_start_x = 0, check_start_y = 0, check_goal_x = 0, check_goal_y = 0;
        map->getCoordinates(check_start_x, check_start_y, start);
        map->getCoordinates(check_goal_x, check_goal_y, std::get<1>(it->second));
        double check_goal_dis = euclidean((double)check_start_x, (double)check_start_y, (double)check_goal_x, (double)check_goal_y);
        if (check_goal_dis > (frontier_distance_threshold_ / resolution)) {
          double frontier_dis = it->first;
          goal = std::get<1>(it->second);
          ROS_INFO("found %d fontiers, current frontier cost: %f", frontier_queue.size(), frontier_dis);
          return EXPL_TARGET_SET;
        }
        // double frontier_euclidean_distance = std::get<0>(it->second);
        // if (frontier_euclidean_distance > frontier_distance_threshold_) {
        //   goal = std::get<1>(it->second);
        //   return EXPL_TARGET_SET;
        // }
        if (it == frontier_queue.end()) {
          ROS_ERROR("No frontier out of %f meters", frontier_distance_threshold_);
          return EXPL_FAILED;
        }
      }
      return EXPL_FAILED;
    }
    
  } else {
    if (cellCount > 50) {
      return EXPL_FINISHED;
    } else {
      ROS_INFO("frontier queue empty, checked less than 50 cells.");
      return EXPL_FAILED;
    }
  }

  return EXPL_FAILED;
}

double FrontierPlanner::euclidean(double x1, double y1, double x2, double y2) {
  double x = x1 - x2;
  double y = y1 - y2;
  double dist;
  dist = pow(x, 2) + pow(y, 2);      
  dist = sqrt(dist);
  return dist;
}

void FrontierPlanner::setObstacleScanRange(double range) {
  scan_distance_ = range;
}

void FrontierPlanner::setGoalFrontierThreshold(int threshold) {
  goal_frontier_threshold_ = threshold;
}

void FrontierPlanner::setFrontierDistanceThreshold(double distance) {
  frontier_distance_threshold_ = distance;
}

void FrontierPlanner::setFovRange(double fov){
  fov_range_ = fov;
}

void FrontierPlanner::getWorldCoordinate(GridMap* map, unsigned int cell_x, unsigned int cell_y, double &world_x, double &world_y) {
  world_x = map->getOriginX() + (((double)cell_x + 0.5) * map->getResolution());
  world_y = map->getOriginY() + (((double)cell_y + 0.5) * map->getResolution());
}
