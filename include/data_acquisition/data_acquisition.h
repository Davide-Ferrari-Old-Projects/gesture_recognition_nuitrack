#ifndef DATA_ACQUISITION_H
#define DATA_ACQUISITION_H

#include <fstream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <librealsense2/rs.hpp> 
#include <librealsense2/rsutil.h>

#include <ros/ros.h>
#include <ros/package.h>

#include "nuitrack_msgs/SkeletonDataArray.h"
#include "nuitrack_msgs/SkeletonData.h"
#include "gesture_recognition/String.h"
#include <sensor_msgs/Image.h>

#include <std_srvs/Trigger.h>

// Define Joint Names
#define HEAD "joint_head"
#define NECK "joint_neck"
#define TORSO "joint_torso"
#define WAIST "joint_waist"
#define LEFT_COLLAR "joint_left_collar"
#define LEFT_SHOULDER "joint_left_shoulder"
#define LEFT_ELBOW "joint_left_elbow"
#define LEFT_WRIST "joint_left_wrist"
#define LEFT_HAND "joint_left_hand"
#define RIGHT_COLLAR "joint_right_collar"
#define RIGHT_SHOULDER "joint_right_shoulder"
#define RIGHT_ELBOW "joint_right_elbow"
#define RIGHT_WRIST "joint_right_wrist"
#define RIGHT_HAND "joint_right_hand"
#define LEFT_HIP "joint_left_hip"
#define LEFT_KNEE "joint_left_knee"
#define LEFT_ANKLE "joint_left_ankle"
#define RIGHT_HIP "joint_right_hip"
#define RIGHT_KNEE "joint_right_knee"
#define RIGHT_ANKLE "joint_right_ankle"

struct Skeleton_Markers {
    std::string joint_name;
    double joint_pose_2D[2];
};

class data_acquisition {

    public:

        data_acquisition();

        ~data_acquisition();

        void spinner (void);

    private:

        ros::NodeHandle nh;

        // ---- Parameters ---- //
        // bool live_mode;
        bool start_registration, shutdown_required;
        std::string save_file_name;
        sensor_msgs::Image image_raw;
        nuitrack_msgs::SkeletonData skeleton;

        // ---- OfStreams ---- //
        std::ofstream image_raw_save, skeleton_data_save;
        bool image_raw_first_row, skeleton_data_first_row;
        
        // ---- PUBLISHERS & SUBSCRIBERS ---- //
        ros::Subscriber skeleton_data_subscriber, image_raw_subscriber;

        // ---- SERVICE & CLIENT ---- //
        ros::ServiceServer start_registration_service, stop_registration_service;


        // ---- CALLBACKS ---- //
        void image_raw_Callback (const sensor_msgs::Image::ConstPtr &);
        void skeleton_data_Callback (const nuitrack_msgs::SkeletonDataArray::ConstPtr &);

        bool Start_Registration_Service_Callback (gesture_recognition::String::Request &req, gesture_recognition::String::Response &res);
        bool Stop_Registration_Service_Callback (std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

        // ---- FUNCTIONS ---- //
        void ofstream_creation (std::string ofstream_name);
        void draw_skeleton (nuitrack_msgs::SkeletonData skeleton_data, sensor_msgs::Image image);
        void draw_skeleton_between_two_markers (std::string marker1, std::string marker2, std::vector<Skeleton_Markers> skeleton, cv::Mat *img);

};

#endif /* DATA_ACQUISITION_H */