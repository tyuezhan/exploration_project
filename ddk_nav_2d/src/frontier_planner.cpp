#include <ddk_nav_2d/frontier_planner.h>
#include <math.h>
typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

FrontierPlanner::FrontierPlanner() {
}

FrontierPlanner::~FrontierPlanner() {}


int FrontierPlanner::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{
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
	queue.insert(startPoint);
	plan[start] = 0;
	
	Queue::iterator next;
	double distance;
	double linear = map->getResolution();
	bool foundFrontier = false;
	int cellCount = 0;
	double obstacle_dis;

  // Init priority queue for frontier
  Queue frontier_queue;

	// Do full search with weightless Dijkstra-Algorithm
	while(!queue.empty())
	{
		cellCount++;
		// Get the nearest cell from the queue
		next = queue.begin();
		distance = next->first;
		unsigned int index = next->second;
		queue.erase(next);
		
		// Add all adjacent cells
		if(map->isFrontier(index))
		{
			// We reached the border of the map, which is unexplored terrain as well:
			foundFrontier = true;
      // Calculate two distance.
      unsigned int start_x = 0, start_y = 0, goal_x = 0, goal_y = 0;
      map->getCoordinates(start_x, start_y, start);
      map->getCoordinates(goal_x, goal_y, index);
      double goal_dis = euclidean((double)start_x, (double)start_y, (double)goal_x, (double)goal_y);
      // double euclidean_dis = std::sqrt(std::pow((double)start_x - (double)goal_x, 2) + std::pow((double)start_y - (double)goal_y, 2));
      // ROS_INFO("Euclidean dis: %f, start_x: %u, goal_x: %u, start_y: %u, goal_y: %u", goal_dis, start_x, goal_x, start_y, goal_y);
      
      // scan nearby obstacle
      Queue scan_quque;
      scan_cell_distance_ = scan_distance_ / linear; 
      Entry frontier_cell(0.0, index);
      scan_quque.insert(frontier_cell);
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
        unsigned int curr_index = scan_next->second;
        scan_quque.erase(scan_next);
        // ROS_INFO("Current cell value: %c", map->getData(curr_index));
        if (!map->isObstacle(curr_index)){
          // ROS_INFO("Enter is free");
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
                double check_dis = euclidean((double)check_x, (double)check_y, (double)goal_x, (double)goal_y);
                if (check_dis <= scan_cell_distance_) {
                  // ROS_INFO("Check dis: %f, scan dis: %f", check_dis, scan_cell_distance_);
                  scan_quque.insert(Entry(distance_to_frontier+linear, i));
                  frontier_plan[i] = distance_to_frontier+linear;
                }
              }
            }
          }
        } else {
          // found obstacle
          found_obstacle = true;
          unsigned int obs_x, obs_y;
          map->getCoordinates(obs_x, obs_y, curr_index);
          // obstacle_dis = std::sqrt(std::pow((double)obs_x - (double)goal_x, 2) + std::pow((double)obs_x - (double)goal_y, 2));
          obstacle_dis = euclidean((double)obs_x, (double)obs_y, (double)goal_x, (double)goal_y);
          // ROS_INFO("Obs dis: %f, obs_x: %u, obs_y:%u, goal_x:%u, goal_y:%u, double value: obs_x: %f, obs_y:%f, goal_x:%f, goal_y:%f", obstacle_dis, obs_x, obs_y, goal_x, goal_y, (double)obs_x, (double)obs_y, (double)goal_x, (double)goal_y);
          break;
        }
      }
      if (!found_obstacle){
        // ROS_INFO("dont find a obstacle for this frontier");
        obstacle_dis = scan_cell_distance_;
      }
      // calculate total cost and put frontier into priority queue
      double total_cost = goal_dis + (scan_cell_distance_ - obstacle_dis) * 40;
      // ROS_INFO("Total cost for curr frontier: %f, obstacle dis: %f, euclidean: %f, scan dis: %f", total_cost, obstacle_dis, goal_dis, scan_cell_distance_);
      frontier_queue.insert(Entry(total_cost, index));
      delete[] frontier_plan;
		} else
		{
			unsigned int ind[4];

			ind[0] = index - 1;               // left
			ind[1] = index + 1;               // right
			ind[2] = index - map->getWidth(); // up
			ind[3] = index + map->getWidth(); // down
			
			for(unsigned int it = 0; it < 4; it++)
			{
				unsigned int i = ind[it];
				if(map->isFree(i) && plan[i] == -1)
				{
					queue.insert(Entry(distance+linear, i));
					plan[i] = distance+linear;
				}
			}
		}
	}

	ROS_DEBUG("Checked %d cells.", cellCount);	
	delete[] plan;
	
  if (!frontier_queue.empty()){
    if (frontier_queue.size() <= 30) {
      ROS_INFO("Less than 30 frontiers. Return exploration finished.");
      return EXPL_FINISHED;
    }
    Queue::iterator iter;
    iter = frontier_queue.begin();
    goal = iter->second;
    ROS_INFO("found %d fontiers, current frontier cost: %f", frontier_queue.size(), iter->first);
    return EXPL_TARGET_SET;
  } else {
    if (cellCount > 50) {
      return EXPL_FINISHED;
    } else {
      return EXPL_FAILED;
    }
  }
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

// int FrontierPlanner::findExplorationTarget(grid_map::GridMap* map, grid_map::Position start, grid_map::Position &goal) {
  
//   grid_map::Matrix& map_copy = (*map)["test"];
//   for (grid_map::GridMapIterator iterator(*map); !iterator.isPastEnd(); ++iterator) {
//     const int i = iterator.getLinearIndex();
//     const float value = map_copy(i);
//     if (value < 1) {
//       // < 1 free
//       const grid_map::Index index(*iterator);
//       if ((map_copy(index(0)-1, index(1)-1) == -1) || (map_copy(index(0), index(1)-1) == -1) ||
//           (map_copy(index(0)-1, index(1)) == -1)   || (map_copy(index(0)-1, index(1)+1) == -1) ||
//           (map_copy(index(0)+1, index(1)-1) == -1) || (map_copy(index(0), index(1)+1) == -1) ||
//           (map_copy(index(0)+1, index(1)) == -1)   || (map_copy(index(0)+1, index(1)+1) == -1)) {
//             // find frontier, calculate two costs

//         }
      
//     }
//     // cout << "The value at index " << index.transpose() << " is " << map_copy(index(0), index(1)) << endl;
//     // if (isFrontier(index)) 
//   } 

//   grid_map::Position center(0.0, -0.15);
//   double radius = 20.0;
//   for (grid_map::CircleIterator iterator(*map, center, radius); !iterator.isPastEnd(); ++iterator) {
//     float data = map->at("test", *iterator); // NAN or not
//     ROS_INFO("The value of iter: %f", data);
//   }
// }
