#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "common/markers.cpp"
#include "common/common_functions.cpp"
#include "common/statistics.cpp"
#include "gnc_experimentation/vehicle_state.h"

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>


#define TF_FIXED_FRAME "/absolute"


class LocalizationClass{
public:
    LocalizationClass(ros::NodeHandle _nh);
    void cmdCallback(const geometry_msgs::Twist cmd);
    void poseCallback(const gazebo_msgs::ModelStates& msg);
    void odomCallback(const nav_msgs::Odometry enc);
    void updateEKF();

private:
    ros::NodeHandle n;
    ros::Publisher ekf_pose_pub;

    // tf listener to keep track of frames
    tf::TransformListener listener;

    // Visualization
    PoseMarker exact_pose_marker;
    PoseMarker gps_pose_marker;
    PoseMarker ekf_pose_marker;
    PoseMarker ekf_pred_pose_marker;

    // State control;
    bool enc_meas_ready;
    bool gps_meas_ready;
    bool gps_meas_made;
    double ekf_last_time;
    double enc_last_time;
    double gps_last_time;

    // Exact robot measurments
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

};

LocalizationClass::LocalizationClass(ros::NodeHandle _nh)
:
exact_pose_marker(_nh,"/marker_absolute",0,1,0),
gps_pose_marker(_nh,"/marker_gps",1,0,0),
ekf_pose_marker(_nh,"/marker_ekf",0,0,1),
ekf_pred_pose_marker(_nh,"/marker_ekf_pred",0,0,1)
{
    // grab node handle
    n = _nh;

    // Build publishers
    ekf_pose_pub = n.advertise<gnc_experimentation::vehicle_state>("/ekf_pose",5);

    enc_last_time = 0;
    gps_last_time = 0;
    ekf_last_time = 0;
    enc_meas_ready = false;
    gps_meas_ready = false;
    gps_meas_made = false;

    // Set initial states
    prev_mean  << 0    ,0    ,0    ;
    prev_cov   << 0.5  ,0    ,0    ,
                  0    ,0.5  ,0    ,
                  0    ,0    ,0.01 ;
    cur_input  << 0    ,0    ;

    // Set matricies
    ekf_C = Eigen::Matrix<double, 3, 3>::Identity();

    // Set model and sensor coveriances
    // ekf_R is defined per second
    ekf_R     << 0.05 ,0    ,0     ,
                 0    ,0.05 ,0     ,
                 0    ,0    ,0.005 ; 

    ekf_Q_gps << 0.15 ,0    ,0     ,
                 0    ,0.15 ,0     ,
                 0    ,0    ,0.002 ;

    ekf_Q_enc << 0.05,0    ,
                 0    ,0.001;
    Q_gps = ekf_Q_gps;
    Q_enc = ekf_Q_enc;
}


void LocalizationClass::cmdCallback(const geometry_msgs::Twist cmd){
    cur_input(0) = cmd.linear.x;
    cur_input(1) = cmd.angular.z;
}


void LocalizationClass::odomCallback(const nav_msgs::Odometry enc){
    // Produce encoder measurments
    double enc_rate = 20; //Hz
    double enc_cur_time = ros::Time::now().toSec();
    if((enc_cur_time-enc_last_time) >= 1.0/enc_rate-0.00001){ // -0.00001 to account for double rounding
        if(enc_meas_ready){ROS_ERROR("Encoder measurment was not processed in time");}
        enc_last_time = (floor(enc_cur_time*enc_rate))/enc_rate;
        // Set exact state
        exact_velocities(0) = enc.twist.twist.linear.x;
        exact_velocities(1) = enc.twist.twist.angular.z;
        // Gernerate simulated encoder measurments
        Eigen::Matrix<double, 2, 1> enc_e;
        enc_e(0) = randNormal(Q_enc(0,0));
        enc_e(1) = randNormal(Q_enc(1,1));
        y_enc = exact_velocities+enc_e;
        enc_meas_ready = true;
    } 
}


//Callback function for the Position topic
void LocalizationClass::poseCallback(const gazebo_msgs::ModelStates& msg){
    // Extract exact robot state from message
    int robot_index;
    for (int i = 0;i<200;i++){ // look for desired name in the message and grab the index
        if (msg.name[i] == "mobile_base"){
            robot_index = i;
            break;
        }
    }
    geometry_msgs::Pose robot_pose = msg.pose[robot_index];
    geometry_msgs::Twist robot_twist = msg.twist[robot_index];
    // get euler angles
    double exact_roll, exact_pitch, exact_yaw;
    tf::Matrix3x3 m( tf::Quaternion(robot_pose.orientation.x,robot_pose.orientation.y,robot_pose.orientation.z,robot_pose.orientation.w) );
    m.getRPY(exact_roll, exact_pitch, exact_yaw);
    // Set exact state
    exact_x(0) = robot_pose.position.x;
    exact_x(1) = robot_pose.position.y;
    exact_x(2) = exact_yaw;
    // Build absolute tf state;
    tf::Transform exact_robot_pose;
    exact_robot_pose.setOrigin( tf::Vector3(robot_pose.position.x, robot_pose.position.y, 0.0) );
    exact_robot_pose.setRotation( tf::Quaternion(robot_pose.orientation.x,robot_pose.orientation.y,robot_pose.orientation.z,robot_pose.orientation.w) );
    tf::TransformBroadcaster tf_br;
    tf_br.sendTransform(tf::StampedTransform(exact_robot_pose, ros::Time::now(), TF_FIXED_FRAME, "/base_link"));
    // update visualization
    exact_pose_marker.updatePoseStateVector(exact_x);
    // Produce GPS measurments
    double gps_rate = 1; //Hz
    double gps_cur_time = ros::Time::now().toSec();
    if((gps_cur_time-gps_last_time) >= 1.0/gps_rate-0.00001){ // -0.00001 to account for double rounding
        if(gps_meas_ready){ROS_ERROR("GPS measurment was not processed in time");}
        gps_last_time = (floor(gps_cur_time*gps_rate))/gps_rate;
        // Gernerate simulated gps noise
        Eigen::Matrix<double,3,1> rand_e;
        rand_e(0) = randNormal(1);
        rand_e(1) = randNormal(1);
        rand_e(2) = randNormal(1);
        Eigen::EigenSolver<Eigen::Matrix<double,3,3> > es(Q_gps);
        Eigen::Matrix<double,3,3> Ev;
        Ev << es.eigenvectors()(0,0).real(),es.eigenvectors()(0,1).real(),es.eigenvectors()(0,2).real(),
              es.eigenvectors()(1,0).real(),es.eigenvectors()(1,1).real(),es.eigenvectors()(1,2).real(),
              es.eigenvectors()(2,0).real(),es.eigenvectors()(2,1).real(),es.eigenvectors()(2,2).real();
        Eigen::Matrix<double,3,3> ev;
        ev = Eigen::Matrix<double,3,3>::Zero();
        ev(0,0) = sqrt(es.eigenvalues()(0).real());
        ev(1,1) = sqrt(es.eigenvalues()(1).real());
        ev(2,2) = sqrt(es.eigenvalues()(2).real());
        // Calculate gps measurment
        y_gps = exact_x+Ev*ev*rand_e;
        gps_meas_ready = true;
        // Initialize previous_mean to current_gps at the beginning
        if(gps_meas_made = false){
            prev_mean = y_gps;
        }
        gps_meas_made = true;
        // update visualization
        gps_pose_marker.updatePoseStateVector(y_gps);
        gps_pose_marker.updateEllipseStateVector(exact_x, Q_gps, 3);
        gps_pose_marker.updateArcStateVector(exact_x, y_gps, Q_gps, 3);
    }
}


void LocalizationClass::updateEKF(){
    if(enc_meas_ready && gps_meas_made){
        enc_meas_ready = false;
        // ------Build predicted state------
        // set timing
        double ekf_cur_time = ros::Time::now().toSec();
        double ekf_dt = std::min(std::max(ekf_cur_time-ekf_last_time,0.01),0.1);
        ekf_last_time = ekf_cur_time;
        // predU = g(prevU,u)
        pred_mean(0) = prev_mean(0)+y_enc(0)*cos(prev_mean(2))*ekf_dt;
        pred_mean(1) = prev_mean(1)+y_enc(0)*sin(prev_mean(2))*ekf_dt;
        pred_mean(2) = prev_mean(2)+y_enc(1)*ekf_dt;
        // G = d/dx( g(prevU,u) )
        ekf_G = Eigen::Matrix<double, 3, 3>::Identity();
        ekf_G(0,2) = -y_enc(0)*ekf_dt*sin(prev_mean(2));
        ekf_G(1,2) =  y_enc(0)*ekf_dt*cos(prev_mean(2));
        // predE = G*prevE*G^T+R
        pred_cov = ekf_G*prev_cov*ekf_G.transpose()+ekf_R*ekf_dt;

        // ------Measurment update------
        if(gps_meas_ready){ // run the EKF with both sensors
            gps_meas_ready = false;
            // K = predE*C^T*(C*predE*C^T+Q)^-1
            ekf_K_gps = pred_cov*ekf_C.transpose()*(ekf_C*pred_cov*ekf_C.transpose()+ekf_Q_gps).inverse();
            //calculate innovation and fix angle rollover
            Eigen::Matrix<double,3,1> gps_innovation = y_gps-ekf_C*pred_mean;
            gps_innovation(2) += (gps_innovation(2)>M_PI) ? -2*M_PI : (gps_innovation(2)<-M_PI) ? 2*M_PI : 0; // fix angle rollover
            // curU = predU+K*(y-C*predU)
            cur_mean = pred_mean+ekf_K_gps*(gps_innovation);
            // curE = (I-K*C)*predE
            cur_cov = (Eigen::Matrix<double, 3, 3>::Identity()-ekf_K_gps*ekf_C)*pred_cov;
        }
        else{ // run the EKF with encoders only
            cur_mean = pred_mean;
            cur_cov = pred_cov;
        }
        prev_mean = cur_mean;
        prev_cov  =  cur_cov;

        // Publish pose
        gnc_experimentation::vehicle_state ekf_pose;
        for(int i = 0; i < 3; i++){
            ekf_pose.state.push_back(cur_mean(i));
        }
        for(int i = 0; i < 9; i++){
            ekf_pose.covariance.push_back(cur_cov(i));
        }
        ekf_pose_pub.publish(ekf_pose);

        ekf_pred_pose_marker.updatePoseStateVector(pred_mean);
        ekf_pred_pose_marker.updateEllipseStateVector(pred_mean, pred_cov, 3);
        ekf_pred_pose_marker.updateArcStateVector(pred_mean, pred_mean, pred_cov, 3);

        ekf_pose_marker.updatePoseStateVector(cur_mean);
        ekf_pose_marker.updateEllipseStateVector(cur_mean, cur_cov, 3);
        ekf_pose_marker.updateArcStateVector(cur_mean, cur_mean, cur_cov, 3);
    }
}


int main(int argc, char **argv){
	//Initialize the ROS framework
    ros::init(argc,argv,"vehicle_localization_node");
    ros::NodeHandle n;
    ros::Subscriber pose_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber cmd_sub;

    LocalizationClass turtle(n);

    //Subscribe to the desired topics and assign callbacks
    pose_sub = n.subscribe("/gazebo/model_states", 1, &LocalizationClass::poseCallback, &turtle, ros::TransportHints().tcpNoDelay());
    odom_sub = n.subscribe("/odom", 10, &LocalizationClass::odomCallback, &turtle, ros::TransportHints().tcpNoDelay());
    cmd_sub = n.subscribe("/cmd_vel_mux/input/teleop", 1, &LocalizationClass::cmdCallback, &turtle);

    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();

    while(ros::ok()){
        turtle.updateEKF();
    }

    return 0;
}
