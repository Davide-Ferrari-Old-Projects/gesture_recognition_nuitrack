#ifndef DATA_ACQUISITION_H
#define DATA_ACQUISITION_H

#include <fstream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <ros/package.h>

#include "nuitrack_msgs/SkeletonDataArray.h"
#include "nuitrack_msgs/SkeletonData.h"
#include "gesture_recognition/String.h"
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>

#include <std_srvs/Trigger.h>

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

};

#endif /* DATA_ACQUISITION_H */