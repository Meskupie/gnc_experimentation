#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "gnc_experimentation/map_name_in.h"

#include <iostream>
#include <fstream>
#include <string> 

#define MAP_RESOLUTION 0.1 // meters per pixle
#define MAP_BASE_WIDTH 2.0 // meters
#define MAP_BASE_HEIGHT 2.0 // meters
#define MAP_MARGINS 1.0 // meters

#define MAP_ITERATE_HIT_AMMOUNT 0.05
#define MAP_ITERATE_MISS_AMMOUNT 0.02
#define MAP_SATURATE_AMMOUNT 6.0

#define MAPS_FILE_LOCATION "/home/meskupie/catkin_ws/src/gnc_experimentation/maps/"

struct map_index{
    int x;
    int y;

    inline bool operator==(map_index a) {
       if (a.x==x && a.y== y)
          return true;
       else
          return false;
    }
    inline bool operator!=(map_index a) {
       if (a.x==x && a.y== y)
          return false;
       else
          return true;
    }
};

class MappingClass{
public:

    MappingClass(ros::NodeHandle _nh);
    void publishOccupencyGrid();

private:
    // transmition related veriables
    ros::NodeHandle n;
    ros::Publisher pose_publisher;
    ros::Publisher marker_publisher;
    ros::Publisher velocity_publisher;
    ros::Publisher map_publisher;
    ros::Publisher scan_publisher;
    ros::Subscriber pose_sub;
    ros::Subscriber scan_sub;
    ros::ServiceServer save_map_service;
    ros::ServiceServer build_map_service;
    ros::ServiceServer find_robot_service;

    // tf listener to keep track of frames
    tf::TransformListener listener;

    // global map
    nav_msgs::OccupancyGrid global_map_stamped;
    std::deque<std::deque<double> > global_map;
    double global_map_offset_x;
    double global_map_offset_y;
    int global_map_width;
    int global_map_height;
    int global_map_add_p_x;
    int global_map_add_n_x;
    int global_map_add_p_y;
    int global_map_add_n_y;
    int global_map_margin_integer;

    // global robot pos
    ros::Time curpos_time_stamp;
    geometry_msgs::Pose curpose;
    geometry_msgs::Pose oldpose;
    geometry_msgs::Twist curtwist;
    double curyaw;

    // private functions
    bool saveMapService(gnc_experimentation::map_name_in::Request &req, gnc_experimentation::map_name_in::Response &res);
    
    void poseCallback(const gazebo_msgs::ModelStates& msg);
    void scanCallback(const sensor_msgs::LaserScan& raw_scan);

    void findIndexesToChange(sensor_msgs::LaserScan scan, tf::StampedTransform scan_transform, std::vector<map_index> &hit, std::vector<map_index> &miss);
    void rayTraceLine(map_index start, map_index end, std::vector<map_index> &miss);
    void refreshMap(std::vector<map_index> &hit, std::vector<map_index> &miss);
    void updateMapExtrema(map_index point);
};


MappingClass::MappingClass(ros::NodeHandle _nh){
    // grab node handle
    n = _nh;

    //Subscribe to the desired topics and assign callbacks
    pose_sub = n.subscribe("/gazebo/model_states", 1, &MappingClass::poseCallback, this);
    scan_sub = n.subscribe("/scan", 1, &MappingClass::scanCallback, this);

    // setup services
    save_map_service = n.advertiseService("save_map", &MappingClass::saveMapService, this);

    //Setup topics to Publish from this node
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/occupancy_map", 1, true);
    scan_publisher = n.advertise<sensor_msgs::LaserScan>("/new_scan", 1, true);

    // set map dimentions
    global_map_offset_x = -(MAP_BASE_WIDTH/2.0+MAP_MARGINS);
    global_map_offset_y = -(MAP_BASE_WIDTH/2.0+MAP_MARGINS);
    global_map_width = round(MAP_BASE_WIDTH+(2*MAP_MARGINS)/MAP_RESOLUTION);
    global_map_height = round(MAP_BASE_HEIGHT+(2*MAP_MARGINS)/MAP_RESOLUTION);
    global_map_margin_integer = round(MAP_MARGINS/MAP_RESOLUTION);

    // reset all enlargement veriables
    global_map_add_p_x = 0;
    global_map_add_n_x = 0;
    global_map_add_p_y = 0;
    global_map_add_n_y = 0;

    global_map.resize(global_map_width);
    for(int i = 0; i < global_map_width; i++){
        global_map.at(i).resize(global_map_height,0);
    }
}


//Callback function for the map
bool MappingClass::saveMapService(gnc_experimentation::map_name_in::Request &req, gnc_experimentation::map_name_in::Response &res){
    std::fstream map_file;
    map_file.open((MAPS_FILE_LOCATION+req.file_name+".txt").c_str(), std::ifstream::out | std::ifstream::trunc);
    if(map_file.is_open()){
        
        map_file << global_map_stamped.header.frame_id << std::endl;
        map_file << global_map_stamped.info.resolution << '\n';
        map_file << global_map_stamped.info.width << '\n';
        map_file << global_map_stamped.info.height << '\n';
        map_file << global_map_stamped.info.origin.position.x << '\n';
        map_file << global_map_stamped.info.origin.position.y << '\n';
        map_file << global_map_stamped.info.origin.position.z << '\n';
        map_file << global_map_stamped.info.origin.orientation.x << '\n';
        map_file << global_map_stamped.info.origin.orientation.y << '\n';
        map_file << global_map_stamped.info.origin.orientation.z << '\n';
        map_file << global_map_stamped.info.origin.orientation.w << '\n';
        
        for(int i = 0; i < global_map_stamped.data.size(); i++){
            map_file << '\n' << int(global_map_stamped.data.at(i));
        }
        
        map_file.close(); 
        ROS_INFO("Map has been saved");
    }  
    else{
        ROS_ERROR("Map name invalid");
    }
    return true;
}


//Callback function for the Position topic
void MappingClass::poseCallback(const gazebo_msgs::ModelStates& msg){
    int robot_index;
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // look for desired name in the message and grab the index
    for (int i = 0;i<100;i++){
        if (msg.name[i] == "mobile_base"){
            robot_index = i;
            break;
        }
    }

    curpos_time_stamp = ros::Time::now();

    // re-format infomation
    curpose = msg.pose[robot_index];
    curtwist = msg.twist[robot_index];

    // get euler yaw
    tf::Quaternion q(curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    curyaw = yaw;

    // set robot tf position based off current map and gazebo robot locations
    transform.setOrigin( tf::Vector3(curpose.position.x, curpose.position.y, 0.0) );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, curpos_time_stamp, "/map", "/base_link"));
}


void MappingClass::updateMapExtrema(map_index point){
    // if the point lies outside of the current map bounds, record by how much for use when enlarging the map
    global_map_add_p_x = std::max(global_map_add_p_x, point.x+global_map_margin_integer-(global_map_width-1));
    global_map_add_n_x = std::max(global_map_add_n_x, -point.x+global_map_margin_integer);
    global_map_add_p_y = std::max(global_map_add_p_y, point.y+global_map_margin_integer-(global_map_height-1));
    global_map_add_n_y = std::max(global_map_add_n_y, -point.y+global_map_margin_integer);
}


void MappingClass::rayTraceLine(map_index start, map_index end, std::vector<map_index> &miss){
    map_index point;
    int x = start.x;
    int y = start.y;
    int x_inc = 1;
    int y_inc = 1;
    int dx = end.x-start.x;
    int dy = end.y-start.y;
    if(dx < 0){
        x_inc = -1;
        dx = -dx;
    }
    if(dy < 0){
        y_inc = -1;
        dy = -dy;
    }

    if(dx > dy){
        int D = 2*dy-dx;
        for(int i = 0; i<dx; i++){
            point.x = x;
            point.y = y;
            miss.push_back(point);
            this->updateMapExtrema(point);
            if(D > 0){
                y = y+y_inc;
                D -= 2*dx;
            }
            D += 2*dy;
            x = x+x_inc;
        } 
    }
    else{
        int D = 2*dx-dy;
        for(int i = 0; i<dy; i++){
            point.x = x;
            point.y = y;
            miss.push_back(point);
            this->updateMapExtrema(point);
            if(D > 0){
                x = x+x_inc;
                D -= 2*dy;
            }
            D += 2*dx;
            y = y+y_inc;
        } 
    }
}


void MappingClass::findIndexesToChange(sensor_msgs::LaserScan scan, tf::StampedTransform scan_transform, std::vector<map_index> &hit, std::vector<map_index> &miss){

    // find rounded sensor location
    map_index sensor_location;
    sensor_location.x = round((scan_transform.getOrigin().x()-global_map_offset_x)/MAP_RESOLUTION);
    sensor_location.y = round((scan_transform.getOrigin().y()-global_map_offset_y)/MAP_RESOLUTION);

    // build list of hits and misses
    for(int laser_index = 0; laser_index < 640; laser_index++){
        if(scan.ranges[laser_index] > scan.range_min && scan.ranges[laser_index] < scan.range_max){
            map_index point;
            double laser_yaw = curyaw+scan.angle_min+(laser_index*scan.angle_increment);
            point.x = round((scan_transform.getOrigin().x()-global_map_offset_x+(scan.ranges[laser_index]*cos(laser_yaw)))/MAP_RESOLUTION);
            point.y = round((scan_transform.getOrigin().y()-global_map_offset_y+(scan.ranges[laser_index]*sin(laser_yaw)))/MAP_RESOLUTION);
            hit.push_back(point);
            this->updateMapExtrema(point);

            // run a ray trace to find all the boxes that the laser passed through
            this->rayTraceLine(sensor_location, point, miss);
        }
    }
}


void MappingClass::refreshMap(std::vector<map_index> &hit, std::vector<map_index> &miss){
    // enlarge width and fill with empty ques of the old length
    for(int i = 0; i < global_map_add_n_x; i++){
        global_map.push_front(std::deque<double>(global_map_height,0));
    }
    for(int i = 0; i < global_map_add_p_x; i++){
        global_map.push_back(std::deque<double>(global_map_height,0));
    }
    // enlagre each nested que
    for(int j = 0; j < global_map.size(); j++){
        for(int i = 0; i < global_map_add_n_y; i++){
            global_map.at(j).push_front(0);
        }
        for(int i = 0; i < global_map_add_p_y; i++){
            global_map.at(j).push_back(0);
        }
    }
    // update map dimentions
    global_map_width = global_map.size();
    global_map_height = global_map.at(0).size();
    global_map_offset_x -= global_map_add_n_x*MAP_RESOLUTION;
    global_map_offset_y -= global_map_add_n_y*MAP_RESOLUTION;

    for(int i = 0; i < hit.size(); i++){
        hit.at(i).x += global_map_add_n_x;
        hit.at(i).y += global_map_add_n_y;
        global_map.at(hit.at(i).x).at(hit.at(i).y) = std::min(MAP_SATURATE_AMMOUNT,global_map.at(hit.at(i).x).at(hit.at(i).y)+MAP_ITERATE_HIT_AMMOUNT);
    }
    for(int i = 0; i < miss.size(); i++){
        miss.at(i).x += global_map_add_n_x;
        miss.at(i).y += global_map_add_n_y;
        global_map.at(miss.at(i).x).at(miss.at(i).y) = std::max(-MAP_SATURATE_AMMOUNT,global_map.at(miss.at(i).x).at(miss.at(i).y)-MAP_ITERATE_MISS_AMMOUNT);  
    }
    // reset all enlargement veriables
    global_map_add_p_x = 0;
    global_map_add_n_x = 0;
    global_map_add_p_y = 0;
    global_map_add_n_y = 0;

    nav_msgs::OccupancyGrid occupancy_map;
    occupancy_map.header.frame_id = "/map";
    occupancy_map.info.map_load_time = ros::Time::now();
    occupancy_map.info.resolution = MAP_RESOLUTION;
    occupancy_map.info.width = global_map_width;
    occupancy_map.info.height = global_map_height;
    occupancy_map.info.origin.position.x = global_map_offset_x;
    occupancy_map.info.origin.position.y = global_map_offset_y;
    occupancy_map.info.origin.position.z = 0;
    occupancy_map.info.origin.orientation.x = 0;
    occupancy_map.info.origin.orientation.y = 0;
    occupancy_map.info.origin.orientation.z = 0;
    occupancy_map.info.origin.orientation.w = 1;
    occupancy_map.data.resize(global_map_width*global_map_height,50);

    for(int i = 0; i < global_map_width; i++){
        for(int j = 0; j < global_map_height; j++){
            occupancy_map.data.at(i+j*global_map_width) = round(100*(1-(1/(1+exp(global_map.at(i).at(j))))));
        }
    }

    map_publisher.publish(occupancy_map);
    global_map_stamped = occupancy_map;
}


void MappingClass::scanCallback(const sensor_msgs::LaserScan& _scan){
    sensor_msgs::LaserScan curscan = _scan;

    // TODO only do stuff if we are moving slowly and we have moved a bit since last time
    bool scan_stable = true;

    // if statement is to fix the unknown scans that have no time stamp
    if (curscan.header.stamp.sec != 0  && scan_stable){
        oldpose = curpose;
        // get sensor position transform from /map;
        tf::StampedTransform scan_transform;
        try{
            listener.lookupTransform("/map", "/camera_depth_frame", ros::Time(0), scan_transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }
        
        // vectors of indexes affected
        std::vector<map_index> miss;
        std::vector<map_index> hit;
        this->findIndexesToChange(curscan, scan_transform, hit, miss);
        this->refreshMap(hit, miss);
    }
}


int main(int argc, char **argv){
	//Initialize the ROS framework
    ros::init(argc,argv,"mapping_node");
    ros::NodeHandle n;
    MappingClass turtle(n);

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
    }

    return 0;
}
