#include "ros/ros.h"
#include "gnc_experimentation/map_name_in.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "save_map_client");
  if (argc != 2){
    ROS_INFO("usage: single name, no . (period)");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gnc_experimentation::map_name_in>("save_map");
  gnc_experimentation::map_name_in srv_file;
  srv_file.request.file_name = argv[1];
  if (client.call(srv_file)){
    ROS_INFO("Map saved to disk");
  }
  else{
    ROS_ERROR("Failed to save");
    return 1;
  }

  return 0;
}
