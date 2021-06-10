#ifndef SHOW_SKELETON_H
#define SHOW_SKELETON_H

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <ros/package.h>

#include "nuitrack_msgs/SkeletonDataArray.h"
#include "nuitrack_msgs/SkeletonData.h"
#include <sensor_msgs/Image.h>

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

class show_skeleton {

    public:

        show_skeleton();

        ~show_skeleton();

        void spinner (void);

    private:

        ros::NodeHandle nh;
        
        // ---- Parameters ---- //
        bool skeleton_data_received, image_row_received;
        sensor_msgs::Image image_raw;
        nuitrack_msgs::SkeletonData skeleton;

        // ---- PUBLISHERS & SUBSCRIBERS ---- //
        ros::Subscriber skeleton_data_subscriber, image_raw_subscriber;

        // ---- CALLBACKS ---- //
        void image_raw_Callback (const sensor_msgs::Image::ConstPtr &);
        void skeleton_data_Callback (const nuitrack_msgs::SkeletonDataArray::ConstPtr &);

        // ---- DRAW SKELETON FUNCTIONS ---- //
        int draw_skeleton (nuitrack_msgs::SkeletonData skeleton_data, sensor_msgs::Image image, std::string window_name, int wait_key);
        void draw_skeleton_between_two_markers (std::string marker1, std::string marker2, std::vector<Skeleton_Markers> skeleton, cv::Mat *img);

};

#endif /* SHOW_SKELETON_H */