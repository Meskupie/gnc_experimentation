//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// 
// Author: Michael Skupien
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

#include <math.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <vector>

#define LOOP_PERIOD 0.05



class GeneratePath{

public:

    //public veriables
    struct path_point{
        double x;
        double y;
    };

    std::vector<path_point> path;

    // Constructor
    GeneratePath(ros::NodeHandle _nh);

    // Load maunally inputed path
    void loadManualPath();

    // Generate path by driving around and returning back to the start point
    void createPath();

    // Publish data
    void publishPathMarkers();

private:

    // ros related publish/subscribe veriables
    ros::NodeHandle n;
    ros::Publisher path_marker;

    visualization_msgs::Marker path_line;

    // current pose information
    path_point current_location;
    double current_yaw;

    // used to determine when the first pose message has been processed
    bool callback_made;

    // Calculate the squared distance between two point objects
    double squaredDist(path_point one, path_point two);

    // Callback function to recieve subscribed pose data
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    // Returns false after the first callback has been made, use to wait for amcl_pose message before starting to use current location information
    bool waitForCallback();
};



class VehicleControl {

public:

    typedef GeneratePath::path_point path_point;

    visualization_msgs::Marker wagon_line;

    // Constructor
    VehicleControl(ros::NodeHandle _nh);

    // load and find location on path
    void initializePath(std::vector<path_point> _path);

    // loop called function, update the robot position going forward a max amount, 
    // find new lookahed distance and calculate desired commands
    void traversePath();

private:

    ros::NodeHandle n;
    ros::Subscriber pose_sub;
    ros::Publisher velocity_publisher; 
    ros::Publisher wagon_marker;

    std::vector<path_point> path;
    int path_length;

    path_point current_location;
    double current_yaw;

    //Velocity control variable
    geometry_msgs::Twist command;

    double step_length;
    double lookahed_dist;

    double desired_velocity;
    double max_iterate_distance;

    // veriables to hold the vehicle path location by point index
    int path_location_index;
    int interpolated_location_index;

    // verialbes for path interpolation
    int interval_count;
    double real_step_length;
    double current_dist_from_path;
    double iterate_dist_from_path;
    double iterate_lookahed_dist;

    path_point step_update;
    path_point iterate_point;
    path_point path_location;
    path_point lookahed_point;

    bool callback_made;

    double yaw_integral_windup;
    double yaw_previous_error;

    // Calculate the squared distance between two point objects
    double squaredDist(path_point one, path_point two);

    // Callback function to recieve subscribed pose data
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    // Returns false after the first callback has been made
    bool waitForCallback();

    // Calculate the step amount in x and y and number of steps for a given path point index
    void calculateIterateStep(int i);

    // Find the closest point on the path to your location to use as an initial path location
    bool searchPathLocation(int i, int j, double max_distance);

    // update marker object corresponding to the forward lookahed point
    void updateWagonMarker();

    // look forward for a point down the path a distance lookahed_dist away
    void findLookahedPoint();

    // set the output commands
    double calculateYawRate(double error);

    // set the output commands
    geometry_msgs::Twist calculateVehicleCommands();
};



// ==========================
// Constructor
// ==========================
GeneratePath::GeneratePath(ros::NodeHandle _nh){
    n = _nh;
    callback_made = false;

    //Advertze path marker message
    path_marker = n.advertise<visualization_msgs::Marker>("/path_marker", 1); 
}


// ---------------------------------------
// Returns false after the first callback has been made, use to wait for 
// amcl_pose message before starting to use current location information
// ---------------------------------------
bool GeneratePath::waitForCallback(){
    return(!callback_made);
}


// ---------------------------------------
// Load maunally inputed path
// ---------------------------------------
void GeneratePath::loadManualPath(){

    path_point p;
    path.clear();

    // manually build path_array and transfer array length to the for loops cutoff
    double path_array[8] = {-0.7, 1.6, 1.2, 1.4, 1, -0.8, -0.9, -0.6};
    for (int i = 0; i<8/2; i++){
        p.x = path_array[i];
        p.y = path_array[i+1];
        path.push_back(p);
    }

    int path_length = path.size();

    path_line.type = visualization_msgs::Marker::LINE_STRIP;
    path_line.header.frame_id = "/map";

    path_line.scale.x = 0.1;
    path_line.scale.y = 0.1;
    path_line.scale.z = 0.1;
    path_line.color.a = 1.0;
    path_line.color.r = 0.0;        
    path_line.color.g = 1.0;  
    path_line.color.b = 0.0;        

    for (int i = 0; i <= path_length; i++){
        geometry_msgs::Point p;
        p.x = path.at(i%path_length).x;
        p.y = path.at(i%path_length).y;
        p.z = 0.0;
        path_line.points.push_back(p);
    }
}


// ---------------------------------------
// Generate path by driving around and returning back to the start point
// ---------------------------------------
void GeneratePath::createPath(){
    ros::Rate loop_rate(20);

    //Subscribe to pose updates
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, &GeneratePath::poseCallback, this);

    // spin until we get the first pose update
    while (ros::ok() && waitForCallback()){
        loop_rate.sleep();
        ros::spinOnce();
    }

    bool ready_for_reentry = false;
    double point_separation_dist = 0.25;
    double depart_dist_threshold = 0.75;
    double reentry_dist_threshold = 0.5; 

    // Initialize marker object
    path_line.type = visualization_msgs::Marker::LINE_STRIP;
    path_line.header.frame_id = "/map";
    path_line.scale.x = 0.05;
    path_line.scale.y = 0.05;
    path_line.scale.z = 0.05;
    path_line.color.a = 1.0;
    path_line.color.r = 0.0;        
    path_line.color.g = 1.0;  
    path_line.color.b = 0.0;

    geometry_msgs::Point p;

    path.clear();
    path.push_back(current_location);
    p.x = current_location.x;
    p.y = current_location.y;
    p.z = 0.0;
    path_line.points.push_back(p);

    while (ros::ok() && (squaredDist(path.front(),current_location) > reentry_dist_threshold*reentry_dist_threshold || !ready_for_reentry)){
        // check if the distance from the last point is large enough, stamp a new point if it is.
        if(squaredDist(current_location,path.back()) > point_separation_dist*point_separation_dist){
            path.push_back(current_location);
            p.x = current_location.x;
            p.y = current_location.y;
            p.z = 0.0;
            path_line.points.push_back(p);
        }

        // once the distance from the start point is large enough, flag so that the main loop can break when the vehicle gets close to the start again
        if(!ready_for_reentry){
            if(squaredDist(path.front(),current_location) > depart_dist_threshold*depart_dist_threshold){
                ready_for_reentry = true;
            }
        }

        path_marker.publish(path_line);

        loop_rate.sleep();
        ros::spinOnce();
    }

    p.x = path.front().x;
    p.y = path.front().y;
    p.z = 0.0;
    path_line.points.push_back(p);
}

// ---------------------------------------
// Callback function to recieve subscribed pose data
// ---------------------------------------
void GeneratePath::publishPathMarkers(){
    path_marker.publish(path_line);
}

// ---------------------------------------
// Calculate the squared distance between two point objects
// ---------------------------------------
double GeneratePath::squaredDist(path_point one, path_point two){
    return(pow(one.x-two.x,2)+pow(one.y-two.y,2));
}

// ---------------------------------------
// Callback function to recieve subscribed pose data
// ---------------------------------------
void GeneratePath::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    callback_made = true;
    current_location.x = msg->pose.pose.position.x; // Robot X psotition
    current_location.y = msg->pose.pose.position.y; // Robot Y psotition
    current_yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
}




// ---------------------------------------
// Calculate the squared distance between two point objects
// ---------------------------------------
double VehicleControl::squaredDist(path_point one, path_point two){
    return(pow(one.x-two.x,2)+pow(one.y-two.y,2));
}

// =======================================
// Constructors / Deconstructors
// =======================================
VehicleControl::VehicleControl(ros::NodeHandle _nh){
    //Assumptions:
    //=======
    //step_length is smaller than the distance between points
    //if 2*step_length > distance between points, divide by zero on interval_count
    //error in lookahed distance will wined up if step_length is not considerably smaller
    //=======
    //lookahed_dist is not a significant lenght of the total path (<50%)
    //=======

    n = _nh;

    callback_made = false;
    step_length = 0.03;
    desired_velocity = 0.4;
    lookahed_dist = 1.5*desired_velocity;
    max_iterate_distance = desired_velocity*LOOP_PERIOD*3;

    yaw_integral_windup = 0;
    yaw_previous_error = 0;

    //Subscribe to the desired topics and assign callbacks
    pose_sub = n.subscribe("/amcl_pose", 1, &VehicleControl::poseCallback, this);

    //Setup topics to Publish from this node
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    wagon_marker = n.advertise<visualization_msgs::Marker>("/wagon_marker", 1);
}

// ---------------------------------------
// Callback function to recieve subscribed pose data
// ---------------------------------------
void VehicleControl::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    callback_made = true;
    current_location.x = msg->pose.pose.position.x; // Robot X psotition
    current_location.y = msg->pose.pose.position.y; // Robot Y psotition
    current_yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
}

// ---------------------------------------
// Returns false after the first callback has been made
// ---------------------------------------
bool VehicleControl::waitForCallback(){
    return(!callback_made);
}

// ---------------------------------------
// Calculate the step amount in x and y and number of steps for a given path point index
// ---------------------------------------
void VehicleControl::calculateIterateStep(int i){
    // calculate the dx and dy to step by based on the desired step length and path
    interval_count = round(sqrt(squaredDist(path.at(i%path_length),path.at((i+1)%path_length)))/step_length);

    // ****************** TODO fix devide by zero error ******************** 
    step_update.x = (path.at((i+1)%path_length).x-path.at(i%path_length).x)/double(interval_count);
    step_update.y = (path.at((i+1)%path_length).y-path.at(i%path_length).y)/double(interval_count);

    // Find the actuall step length based off an integer breakup, used for continuous path distance summation
    real_step_length = sqrt(step_update.x*step_update.x+step_update.y*step_update.y);
}

// ---------------------------------------
// Find the closest point on the path to your location to use as an initial path location
// ---------------------------------------
bool VehicleControl::searchPathLocation(int i, int j, double max_distance){
    for(j; j<interval_count; j++){
        iterate_point.x = path.at(i%path_length).x+(step_update.x*j);
        iterate_point.y = path.at(i%path_length).y+(step_update.y*j);

        iterate_dist_from_path = squaredDist(current_location,iterate_point);

        if (iterate_dist_from_path < current_dist_from_path){
            path_location = iterate_point;
            current_dist_from_path = iterate_dist_from_path;
            path_location_index = i;
            interpolated_location_index = j;
        }

        if(max_distance > 0){
            // add to path distance from closest point
            iterate_lookahed_dist += real_step_length;
            if (iterate_lookahed_dist >= max_distance){
                return(true);
            }
        }
    }
    return(false);
}

// --------------------------------------
// load and find location on path
// --------------------------------------
void VehicleControl::initializePath(std::vector<path_point> _path){
    path = _path;
    path_length = path.size();

    ros::Rate temp_rate(20);

    // spin until we get the first pose update
    while (ros::ok() && waitForCallback()){
        temp_rate.sleep();
        ros::spinOnce();
    }

    int i = 0;
    int initial_j = 0;

    //initialize this to be very large
    current_dist_from_path = 100000000;

    for(i; i<path_length; i++){
        calculateIterateStep(i);
        
        // itterate down path segment
        searchPathLocation(i,initial_j,-1);
    }
}

// --------------------------------------
// update marker object corresponding to the forward lookahed point
// --------------------------------------
void VehicleControl::updateWagonMarker(){
    visualization_msgs::Marker new_wagon_line;

    new_wagon_line.action = 2;
    new_wagon_line.action = 0;

    new_wagon_line.type = visualization_msgs::Marker::LINE_STRIP;
    new_wagon_line.header.frame_id = "/map";

    new_wagon_line.scale.x = 0.05;
    new_wagon_line.scale.y = 0.05;
    new_wagon_line.scale.z = 0.05;
    new_wagon_line.color.a = 1.0;
    new_wagon_line.color.r = 1.0;        
    new_wagon_line.color.g = 0.0;  
    new_wagon_line.color.b = 0.0;        

    geometry_msgs::Point p;
    p.x = current_location.x;
    p.y = current_location.y;
    p.z = 0.0;
    new_wagon_line.points.push_back(p);
    p.x = lookahed_point.x;
    p.y = lookahed_point.y;
    p.z = 0.0;
    new_wagon_line.points.push_back(p);

    wagon_line = new_wagon_line;
}

// --------------------------------------
// look forward for a point down the path a distance lookahed_dist away
// --------------------------------------
void VehicleControl::findLookahedPoint(){
    int i = path_location_index;
    int j = interpolated_location_index;

    iterate_lookahed_dist = 0;

    for(i; i<path_location_index+path_length; i++){            
        calculateIterateStep(i);
    
        // itterate down path segment
        for(j; j<interval_count; j++)
        {
            // add to path distance from closest point
            iterate_lookahed_dist += real_step_length;

            // check if we are at the correct path distance away
            iterate_point.x = path.at(i%path_length).x+(step_update.x*j);
            iterate_point.y = path.at(i%path_length).y+(step_update.y*j);
            if (iterate_lookahed_dist > lookahed_dist){
                lookahed_point = iterate_point;
                break;
            }
        }

        // nested break
        if (iterate_lookahed_dist > lookahed_dist){
            break;
        }

        // make sure to now start at the beginning of the interpolation
        j = 0;
    } 
    updateWagonMarker();
}

// --------------------------------------
// set the output commands
// --------------------------------------
double VehicleControl::calculateYawRate(double error){
    double kp = 1.5;
    double ki = 0.0; //0.05
    double kd = -0.75;
    double PID;

    //loop rate is 20hz
    yaw_integral_windup =+ error*LOOP_PERIOD;
    PID = error*kp+yaw_integral_windup*ki+((yaw_previous_error-error)/LOOP_PERIOD)*kd;
    yaw_previous_error = error;

    return(PID);
}

// --------------------------------------
// set the output commands
// --------------------------------------
geometry_msgs::Twist VehicleControl::calculateVehicleCommands(){
    double kv = 5;

    double d_angle = atan2(lookahed_point.y-current_location.y,lookahed_point.x-current_location.x)-current_yaw;

    if (d_angle > M_PI){
        d_angle = d_angle - 2*M_PI;
    }
    else if (d_angle < -M_PI){
        d_angle = d_angle + 2*M_PI;
    }

    //ROS_INFO("delta:%0.3f",d_angle);

    // set command outputs, 
    // forward velocity is scaled down with angular error
    command.linear.x = desired_velocity*std::max(1-(kv*abs(d_angle)/M_PI),0.0); // set linear speed
    command.angular.z = calculateYawRate(d_angle); // set angular speed

    return command;
}

// --------------------------------------
// loop called function, update the robot position going forward a max amount, 
// find new lookahed distance and calculate desired commands
// --------------------------------------
void VehicleControl::traversePath(){
    int i = path_location_index;
    int j = interpolated_location_index;
    
    iterate_lookahed_dist = 0;

    //initialize this to be very large
    current_dist_from_path = 1000000;

    for(i; i<path_location_index+path_length; i++){
        calculateIterateStep(i);
        // itterate down path segment
        if(searchPathLocation(i,j,max_iterate_distance)){
            break;
        }
        // make sure to now start at the beginning of the interpolation
        j = 0;
    }  

    findLookahedPoint();
    
    velocity_publisher.publish(calculateVehicleCommands());
    wagon_marker.publish(wagon_line);
}




int main(int argc, char **argv){
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;
    GeneratePath trajectory(n);
    VehicleControl turtle(n);

    //Set the loop rate to 20Hz
    ros::Rate loop_rate(20);

    trajectory.createPath();
    turtle.initializePath(trajectory.path);   

    while (ros::ok())
    {
    	ros::spinOnce();   //Check for new messages
    
        turtle.traversePath();
        trajectory.publishPathMarkers();

        loop_rate.sleep(); //Maintain the loop rate
    }

    return 0;
}
