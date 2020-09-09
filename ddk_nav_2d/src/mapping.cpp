#include <plan_env/sdf_map.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapping");
  ros::NodeHandle pnh_("~");

  SDFMap::Ptr sdf_map_;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(pnh_);

  ros::spin();
  return 0;
}
