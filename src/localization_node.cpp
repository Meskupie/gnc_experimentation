#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>

#include "gnc_experimentation/map_name_in.h"

#include "common/common_functions.cpp"
#include "common/statistics.cpp"

#include <fstream>
#include <boost/lexical_cast.hpp>
#include <Eigen/Geometry>


#define MAPS_FILE_LOCATION "/home/meskupie/catkin_ws/src/gnc_experimentation/maps/"
#define MAP_HIT_THRESHOLD 80

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

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


class LocalizationClass{
public:
    LocalizationClass(ros::NodeHandle _nh, std::string file_name);

private:
    ros::NodeHandle n;
    ros::Publisher pose_publisher;
    ros::Publisher marker_publisher;
    ros::Publisher map_publisher;
    ros::Publisher map_hit_points_publisher;
    ros::Publisher base_scan_publisher;
    ros::Publisher scan_match_publisher;
    ros::Publisher bad_scan_publisher;

    ros::Subscriber pose_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber scan_sub;
    ros::Subscriber cmd_sub;

    // tf listener to keep track of frames
    tf::TransformListener listener;

    // State control;
    bool first_pose_callback;
    double odom_time;

    // global_map
    nav_msgs::OccupancyGrid global_map_stamped;
    PointCloudXYZ::Ptr map_hit_points;
    pcl::KdTreeFLANN<PointXYZ> map_hit_kdtree;

    // Exact robot measurments
    ros::Time curpos_time_stamp;
    Eigen::Matrix<double, 3, 1> exact_x;
    Eigen::Matrix<double, 3, 1> last_scan_exact_x;
    Eigen::Matrix<double, 2, 1> exact_velocities;

    // robot input
    Eigen::Matrix<double, 2, 1> cur_input;

    // EKF state
    Eigen::Matrix<double, 3, 1> prev_mean;
    Eigen::Matrix<double, 3, 1> cur_mean;
    Eigen::Matrix<double, 3, 1> pred_mean;
    Eigen::Matrix<double, 3, 3> prev_cov;
    Eigen::Matrix<double, 3, 3> cur_cov;
    Eigen::Matrix<double, 3, 3> pred_cov;
    // EKF matricies
    Eigen::Matrix<double, 3, 3> ekf_G;
    Eigen::Matrix<double, 3, 3> ekf_C;
    Eigen::Matrix<double, 3, 3> ekf_K_gps;
    Eigen::Matrix<double, 2, 3> ekf_H;
    Eigen::Matrix<double, 3, 2> ekf_K_enc;
    // EKF noise
    Eigen::Matrix<double, 3, 3> ekf_R;
    Eigen::Matrix<double, 3, 3> ekf_Q_gps;
    Eigen::Matrix<double, 2, 2> ekf_Q_enc;
    // Sensor Measurments
    int measurment_counter;
    Eigen::Matrix<double, 3, 1> y_gps;
    Eigen::Matrix<double, 2, 1> y_enc;
    // Sensor noise
    Eigen::Matrix<double, 3, 3> Q_gps;
    Eigen::Matrix<double, 2, 2> Q_enc;

    void loadMapFile(std::string file_name);
    void cmdCallback(const geometry_msgs::Twist cmd);
    void poseCallback(const gazebo_msgs::ModelStates& msg);
    void odomCallback(const nav_msgs::Odometry enc);
    void scanCallback(const sensor_msgs::LaserScan& raw_scan);
    void scanMatchToMap(PointCloudXYZ::Ptr _base_scan);

};

LocalizationClass::LocalizationClass(ros::NodeHandle _nh, std::string file_name)
    :
    map_hit_points(new PointCloudXYZ())
    {
        // grab node handle
        n = _nh;

        //Subscribe to the desired topics and assign callbacks
        pose_sub = n.subscribe("/gazebo/model_states", 1, &LocalizationClass::poseCallback, this);
        odom_sub = n.subscribe("/odom", 1, &LocalizationClass::odomCallback, this);
        scan_sub = n.subscribe("/scan", 1, &LocalizationClass::scanCallback, this);
        cmd_sub = n.subscribe("/cmd_vel_mux/input/teleop", 1, &LocalizationClass::cmdCallback, this);

        //Setup topics to Publish from this node
        pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
        marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
        map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/occupancy_map", 1, true);
        map_hit_points_publisher = n.advertise<sensor_msgs::PointCloud2>("/map_hit_points", 1, true);
        base_scan_publisher = n.advertise<sensor_msgs::PointCloud2>("/base_scan", 1, true);
        scan_match_publisher = n.advertise<sensor_msgs::PointCloud2>("/matched_scan", 1, true);
        bad_scan_publisher = n.advertise<sensor_msgs::PointCloud2>("/bad_scan", 1, true);

        // Set initial states
        prev_mean  << 0,0,0;
        prev_cov   << 1,0,0,
                      0,1,0,
                      0,0,1;
        cur_input  << 0,0;

        // Set matricies
        ekf_C = Eigen::Matrix<double, 3, 3>::Identity();

        // Set model and sensor coveriances
        ekf_R     << 0.02 ,0    ,0     ,
                     0    ,0.1  ,0     ,
                     0    ,0    ,0.05  ;

        ekf_Q_gps << 0.5  ,0    ,0     ,
                     0    ,0.5  ,0     ,
                     0    ,0    ,0.02  ;

        ekf_Q_enc << 0.02 ,0    ,
                     0    ,0.02 ;
        Q_gps = ekf_Q_gps;
        Q_enc = ekf_Q_enc;

        measurment_counter = 0;
        first_pose_callback = true;


        loadMapFile(file_name);
    }


// Function to load map file
void LocalizationClass::loadMapFile(std::string file_name){
    std::fstream map_file((MAPS_FILE_LOCATION+file_name+".txt").c_str());
    std::string line;
    if(map_file.is_open()){
        ROS_INFO("%s",("Loading map: "+file_name+".txt").c_str() );
        // Read in header info
        map_file >> global_map_stamped.header.frame_id;
        map_file >> global_map_stamped.info.resolution;
        map_file >> global_map_stamped.info.width;
        map_file >> global_map_stamped.info.height;
        map_file >> global_map_stamped.info.origin.position.x;
        map_file >> global_map_stamped.info.origin.position.y;
        map_file >> global_map_stamped.info.origin.position.z;
        map_file >> global_map_stamped.info.origin.orientation.x;
        map_file >> global_map_stamped.info.origin.orientation.y;
        map_file >> global_map_stamped.info.origin.orientation.z;
        map_file >> global_map_stamped.info.origin.orientation.w;

        // Now read in the data and build kdtree of occupied cells
        PointXYZ hit_point;
        int map_data;
        int map_length =  global_map_stamped.info.width*global_map_stamped.info.height;
        global_map_stamped.data.resize(map_length);
        for(int i = 0; i < map_length; i++){
            map_file >> map_data;
            if(map_data > MAP_HIT_THRESHOLD){
                global_map_stamped.data.at(i) = 100;
                hit_point.x = (floor(i%global_map_stamped.info.width)*global_map_stamped.info.resolution)+global_map_stamped.info.origin.position.x; //floor is precautionary
                hit_point.y = (floor(i/global_map_stamped.info.width)*global_map_stamped.info.resolution)+global_map_stamped.info.origin.position.y; //floor is precautionary
                hit_point.z = 0;
                map_hit_points->points.push_back(hit_point);
            }
            else{global_map_stamped.data.at(i) = 0;}
        }
        map_file.close();

        // create kd tree
        map_hit_points->height = 1;
        map_hit_points->width = map_hit_points->points.size();
        map_hit_points->is_dense = true;
        map_hit_points->header.frame_id = "/map";
        map_hit_kdtree.setInputCloud(map_hit_points);

        // publish map
        map_publisher.publish(global_map_stamped);
        map_hit_points_publisher.publish(map_hit_points);
    }  
    else{
        ROS_ERROR("Map failed to load");
    }
}


void LocalizationClass::cmdCallback(const geometry_msgs::Twist cmd){
    cur_input(0) = cmd.linear.x;
    cur_input(1) = cmd.angular.z;
}


void LocalizationClass::odomCallback(const nav_msgs::Odometry enc){
    odom_time = ros::Time::now().toSec();
}


//Callback function for the Position topic
void LocalizationClass::poseCallback(const gazebo_msgs::ModelStates& msg){
    ROS_INFO("%f",ros::Time::now().toSec()-odom_time);

    //---------------------------------------
    // Extract exact robot state from message
    //---------------------------------------
    geometry_msgs::Pose cur_pose;
    geometry_msgs::Twist cur_twist;
    int robot_index;
    curpos_time_stamp = ros::Time::now();
    // look for desired name in the message and grab the index
    for (int i = 0;i<200;i++){
        if (msg.name[i] == "mobile_base"){
            robot_index = i;
            break;
        }
    }
    // re-format infomation
    cur_pose = msg.pose[robot_index];
    cur_twist = msg.twist[robot_index];
    // get euler angles
    tf::Quaternion q(cur_pose.orientation.x,cur_pose.orientation.y,cur_pose.orientation.z,cur_pose.orientation.w);
    tf::Matrix3x3 m(q);
    double exact_roll, exact_pitch, exact_yaw;
    m.getRPY(exact_roll, exact_pitch, exact_yaw);
    // Set exact state
    exact_x(0) = cur_pose.position.x;
    exact_x(1) = cur_pose.position.y;
    exact_x(2) = exact_yaw;
    exact_velocities(0) = cur_twist.linear.x;
    exact_velocities(1) = cur_twist.angular.z;
    // set robot tf position based off current map and gazebo robot locations
    tf::TransformBroadcaster br; // this used to be static
    tf::Transform exact_robot_pose;
    exact_robot_pose.setOrigin( tf::Vector3(cur_pose.position.x, cur_pose.position.y, 0.0) );
    exact_robot_pose.setRotation(q);
    br.sendTransform(tf::StampedTransform(exact_robot_pose, curpos_time_stamp, "/map", "/base_link"));

    //---------------------------------------
    // Gernerate simulated sensor measurments
    //---------------------------------------
    measurment_counter ++;
    // GPS
    Eigen::Matrix<double, 3, 1> gps_e;
    gps_e(0) = randNormal(Q_gps(0,0));
    gps_e(1) = randNormal(Q_gps(1,1));
    gps_e(2) = randNormal(Q_gps(2,2));
    y_gps = exact_x+gps_e;
    // Encoders
    Eigen::Matrix<double, 2, 1> enc_e;
    gps_e(0) = randNormal(Q_enc(0,0));
    gps_e(1) = randNormal(Q_enc(1,1));
    y_enc = exact_velocities+enc_e;

    //---------------------------------------
    // EKF
    //---------------------------------------
    // Initialize previous_mean to current_gps at the beginning
    if(first_pose_callback){
        first_pose_callback = false;
        prev_mean = y_gps;
    }

    // ------Build predicted state------
    double ekf_dt = 0.01;
    // predU = g(prevU,u)
    pred_mean(0) = prev_mean(0)+cur_input(0)*cos(prev_mean(2))*ekf_dt;
    pred_mean(1) = prev_mean(1)+cur_input(0)*sin(prev_mean(2))*ekf_dt;
    pred_mean(2) = prev_mean(2)+cur_input(1)*ekf_dt;
    // G = d/dx( g(prevU,u) )
    ekf_G = Eigen::Matrix<double, 3, 3>::Identity();
    ekf_G(0,2) = -cur_input(0)*ekf_dt*sin(prev_mean(2));
    ekf_G(1,2) =  cur_input(0)*ekf_dt*cos(prev_mean(2));
    // predE = G*prevE*G^T+R
    pred_cov = ekf_G*prev_cov*ekf_G.transpose()+ekf_R;

    // ------Measurment update------
    // H = d/dx(h(x))
    ekf_H = Eigen::Matrix<double, 2, 3>::Zero();
    ekf_H(0,0) = (pred_mean(0)-prev_mean(0))/(ekf_dt*sqrt(pow(pred_mean(0)-prev_mean(0),2)+pow(pred_mean(1)-prev_mean(1),2)));
    ekf_H(1,0) = (pred_mean(1)-prev_mean(1))/(ekf_dt*sqrt(pow(pred_mean(0)-prev_mean(0),2)+pow(pred_mean(1)-prev_mean(1),2)));
    ekf_H(0,2) = 1/ekf_dt;
    // K = predE*H^T*(H*predE*H^T+Q)^-1
    ekf_K_enc = pred_cov*ekf_H.transpose()*(ekf_H*pred_cov*ekf_H.transpose()+ekf_Q_enc).inverse();
    // pre-calculate h(predU)
    Eigen::Matrix<double, 2, 1> H_predU;
    H_predU(0) = sqrt(pow(pred_mean(0)-prev_mean(0),2)+pow(pred_mean(1)-prev_mean(1),2))/ekf_dt;
    H_predU(1) = (pred_mean(2)-prev_mean(2))/ekf_dt;
    // K = predE*C^T*(C*predE*C^T+Q)^-1
    ekf_K_gps = pred_cov*ekf_C.transpose()*(ekf_C*pred_cov*ekf_C.transpose()+ekf_Q_gps).inverse();
    // resolve U and E basaed on how many sensors available
    if(measurment_counter >= 100){ // run the EKF with both sensors
        measurment_counter = 0;
        // curU = predU+K*(y-h(predU))+K*(y-C*predU)
        cur_mean = pred_mean+ekf_K_enc*(y_enc-H_predU)+ekf_K_gps*(y_gps-ekf_C*pred_mean);
        // curE = (I-K*C)*((I-K*H)*predE)
        cur_cov = (Eigen::Matrix<double, 3, 3>::Identity()-ekf_K_gps*ekf_C)*((Eigen::Matrix<double, 3, 3>::Identity()-ekf_K_enc*ekf_H)*pred_cov);
    }
    else{ // run the EKF with encoders only
        // curU = predU+K*(y-h(predU))
        cur_mean = pred_mean+ekf_K_enc*(y_enc-H_predU);
        // curE = (I-K*H)*predE
        cur_cov = (Eigen::Matrix<double, 3, 3>::Identity()-ekf_K_enc*ekf_H)*pred_cov;
    }

    std::cout << cur_mean << std::endl;
    std::cout << std::endl;
}


void LocalizationClass::scanMatchToMap(PointCloudXYZ::Ptr _base_scan){
    PointCloudXYZ::Ptr base_scan(new PointCloudXYZ());
    *base_scan = *_base_scan;

    pcl::IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setInputSource(base_scan);
    icp.setInputTarget(this->map_hit_points);

    PointCloudXYZ::Ptr Final(new PointCloudXYZ());
    icp.align(*Final);
    Final->header.frame_id = "/map";
    scan_match_publisher.publish(Final);

    //icp.hasConverged();
    //icp.getFitnessScore();
    //std::cout << icp.getFinalTransformation() << std::endl;
}


void LocalizationClass::scanCallback(const sensor_msgs::LaserScan& _scan){
    sensor_msgs::LaserScan curscan = _scan;
    PointCloudXYZ::Ptr base_scan(new PointCloudXYZ());
    PointCloudXYZ::Ptr bad_scan(new PointCloudXYZ());

    // TODO only do stuff if we are moving slowly and we have moved a bit since last time
    bool scan_stable = true;
    /*
    // if statement is to fix the unknown scans that have no time stamp
    if (curscan.header.stamp.sec != 0  && scan_stable){
        last_scan_exact_x = exact_x;
        // get sensor position transform from /map;
        tf::StampedTransform scan_frame_transform;
        try{
            listener.lookupTransform("/map", "/camera_depth_frame", ros::Time(0), scan_frame_transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }
        // Get Translation
        double rand_angle = 2*M_PI*((rand()%1000000)/1000000.0f);
        double rand_dist = 0.5*((rand()%1000000)/1000000.0f); // ==========change for more or less noise
        double offset_x = rand_dist*cos(rand_angle);
        double offset_y = rand_dist*sin(rand_angle);

        gps_pose.position.x = scan_frame_transform.getOrigin().x()+offset_x;
        gps_pose.position.y = scan_frame_transform.getOrigin().y()+offset_y;

        // Get Yaw
        double offset_yaw = 0.01*((rand()%1000000)/1000000); // ==========change for more or less noise
        tf::Matrix3x3 m(scan_frame_transform.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        bad_cur_yaw = yaw+offset_yaw;

        // Build point cloud
        PointXYZ hit_point;
        for(int laser_index = 0; laser_index < 640; laser_index++){
            if(curscan.ranges[laser_index] > curscan.range_min && curscan.ranges[laser_index] < curscan.range_max){
                double laser_yaw = curscan.angle_min+(laser_index*curscan.angle_increment);
                hit_point.x = curscan.ranges[laser_index]*cos(laser_yaw);
                hit_point.y = curscan.ranges[laser_index]*sin(laser_yaw);
                hit_point.z = 0;
                base_scan->points.push_back(hit_point);
            }
        }
        base_scan->height = 1;
        base_scan->width = base_scan->points.size();
        base_scan->is_dense = true;

        // Transform scanned pointcloud to correct frame;
        Eigen::Matrix4f scan_transform = Eigen::Matrix4f::Identity();
        // (row, column)
        // Rotation
        scan_transform (0,0) = cos (bad_cur_yaw);
        scan_transform (0,1) = -sin(bad_cur_yaw);
        scan_transform (1,0) = sin (bad_cur_yaw);
        scan_transform (1,1) = cos (bad_cur_yaw);
        // Translation
        scan_transform (0,3) = gps_pose.position.x;
        scan_transform (1,3) = gps_pose.position.y;

        pcl::transformPointCloud (*base_scan, *bad_scan, scan_transform);

        base_scan->header.frame_id = "/camera_depth_frame";
        bad_scan->header.frame_id = "/map";
        base_scan_publisher.publish(base_scan);
        bad_scan_publisher.publish(bad_scan);

        this->scanMatchToMap(bad_scan);
    }
    */
}


int main(int argc, char **argv){
	//Initialize the ROS framework
    ros::init(argc,argv,"localization_node");
    if (argc != 2){
        ROS_INFO("Please specify a map file name to load");
        return 1;
    }

    ros::NodeHandle n;
    LocalizationClass turtle(n,argv[1]);

    ros::spin();   //Check for new messages

    return 0;
}
