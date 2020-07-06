#include <Eigen/Geometry>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

typedef Eigen::Vector3f Vec3;

#define EXPL_TARGET_SET 1
#define EXPL_FINISHED 2
#define EXPL_WAITING 3
#define EXPL_FAILED 4

class ddkPlanner {
public:
  ddkPlanner();
  ~ddkPlanner();

  Vec3 posInMap(Vec3 pos);
  bool isFree(Vec3 pos);
  bool isFrontier(Vec3 pos);
  bool isInMap(Vec3 pos);
  bool updateOctomap(octomap_msgs::Octomap msg);
  void displayData();
  int findFrontierNN(Vec3 startPos, Vec3 &goalPos);
  void getNeighbors(Vec3 currentPos, Vec3 *neighbors);

private:
  octomap::OcTree *octoTree;
  double resolution;
  double minX, minY, minZ;
  double maxX, maxY, maxZ;
};
