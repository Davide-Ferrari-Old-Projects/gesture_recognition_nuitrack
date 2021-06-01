#include "data_acquisition/data_acquisition.h"

//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

data_acquisition::data_acquisition() {

    // ---- LOAD PARAMETERS ---- //
    if (!nh.param<bool>("/data_acquisition_Node/save_image_raw", save_image_raw, false)) {ROS_ERROR("Couldn't retrieve the Save Image Raw Parameter value.");}
    if (!nh.param<bool>("/data_acquisition_Node/save_skeleton_message", save_skeleton_message, false)) {ROS_ERROR("Couldn't retrieve the Save Skeleton Message Parameter value.");}
    if (!nh.param<bool>("/data_acquisition_Node/save_skeleton_pose", save_skeleton_pose, true)) {ROS_ERROR("Couldn't retrieve the Save Skeleton Pose Parameter value.");}
    // if (!nh.param<std::string>("/data_acquisition_Node/save_file_name", save_file_name, "new_save")) {ROS_ERROR("Couldn't retrieve the csv Save File Name.");}

    // ---- ROS SUBSCRIBERS ---- //
    image_raw_subscriber = nh.subscribe("/nuitrack/rgb/image_raw", 1, &data_acquisition::image_raw_Callback, this);
    skeleton_data_subscriber = nh.subscribe("/nuitrack/skeletons", 1, &data_acquisition::skeleton_data_Callback, this);
  
    // ---- ROS SERVICES ---- //
    start_registration_service = nh.advertiseService("/data_acquisition_Node/start_registration", &data_acquisition::Start_Registration_Service_Callback, this);
    stop_registration_service = nh.advertiseService("/data_acquisition_Node/stop_registration", &data_acquisition::Stop_Registration_Service_Callback, this);

    // Variable Initialization
    start_registration = false;
    shutdown_required = false;
    skeleton_data_received = false;
    image_row_received = false;

    // First Row Bool Initialization
    image_raw_first_row = false;
    skeleton_message_first_row = false;
    skeleton_pose_first_row = false;
    
}

data_acquisition::~data_acquisition() {}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


bool data_acquisition::Start_Registration_Service_Callback (gesture_recognition::String::Request &req, gesture_recognition::String::Response &res) {
    
    start_registration = true;
    save_file_name = req.message_data;

    // ---- OFSTREAM CREATION ---- //
    ofstream_creation (save_file_name);
    
    // sleep 2 seconds
    ros::Duration(2).sleep();
    
    res.success = true;
    return true;

}


bool data_acquisition::Stop_Registration_Service_Callback (std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

    start_registration = false;
    shutdown_required = true;

    // Close OfStreams
    if (save_image_raw) {image_raw_save.close();}
    if (save_skeleton_message) {skeleton_message_save.close();}
    if (save_skeleton_pose) {skeleton_pose_save.close();}

    ROS_WARN("Data Saved Succesfully");

    res.success = true;
    return true;

}


void data_acquisition::image_raw_Callback (const sensor_msgs::Image::ConstPtr &msg) {

    image_raw = *msg;
    image_row_received = true;
    
    /********************************************************************************************
     *                                  sensor_msgs::Image                                      *
     *                                                                                          *
     *  std_msgs/Header header                                                                  *
     *      uint32 seq                                                                          *
     *      time stamp              # timestamp should be acquisition time of image             *
     *      string frame_id         # frame_id should be optical frame of camera                *
     *                                                                                          *
     *  uint32 height               # image height, that is, number of rows                     *
     *  uint32 width                # image width, that is, number of columns                   *
     *  string encoding             # Encoding of pixels -- channel meaning, ordering, size     *
     *  uint8 is_bigendian          # is this data bigendian?                                   *
     *  uint32 step                 # Full row length in bytes                                  *
     *  uint8[] data                # actual matrix data, size is (step * rows)                 *
     *                                                                                          *
     *******************************************************************************************/

    if (!image_raw_first_row &&  save_image_raw) {

        // First, Second Row: Time Stamp, Header, Dimension, Image Parameters, Data ...
        image_raw_save << "Timestamp,Timestamp,Header,Header, ,Height,Width, ,Encoding,Is Bigendian,Step, ,Data...\n";
        image_raw_save << "sec,nsec,frame_id,seq \n";

        // Third Row: Empty
        image_raw_save << "\n";

        // Turn Up the Flag
        image_raw_first_row = true;

    }

    if (start_registration && save_image_raw) {

        // Time Stamp & Header
        image_raw_save << image_raw.header.stamp.sec << "," << image_raw.header.stamp.nsec << ",";
        image_raw_save << image_raw.header.frame_id << "," << image_raw.header.seq << ", ,";

        // Dimension
        image_raw_save << image_raw.height << "," << image_raw.width << ", ,";

        // Parameters ()
        image_raw_save << image_raw.encoding << "," << image_raw.is_bigendian << "," << image_raw.step << ", ,";

        // Data (size = row (height) * step)
        for (unsigned int i = 0; i < image_raw.data.size(); i++) {image_raw_save << int(image_raw.data[i]) << ",";}

        // New Line
        image_raw_save << "\n";

        // Show Image Raw
        cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(image_raw, "bgr8");
        cv::imshow("Image Raw",img->image);
        cv::waitKey(10);

        ROS_INFO_STREAM_THROTTLE(5, "Saving Image Raw...");
    
    }
    
}

void data_acquisition::skeleton_data_Callback (const nuitrack_msgs::SkeletonDataArray::ConstPtr &msg) {

    nuitrack_msgs::SkeletonDataArray skeleton_data = *msg;
    skeleton_data_received = true;

    if (skeleton_data.skeletons.size() > 0) {skeleton = skeleton_data.skeletons[0];}

    /********************************************************************************************
     *                              nuitrack_msgs::SkeletonDataArray                            *
     *                                                                                          *
     *  std_msgs/Header header                                                                  *
     *      uint32 seq                                                                          *
     *      time stamp              # timestamp should be acquisition time of image             *
     *      string frame_id         # frame_id should be optical frame of camera                *
     *                                                                                          *
     *  nuitrack_msgs::SkeletonData[] skeletons                                                 *
     *      uint16 id                           # user id                                       *
     *      string[] joint_names                     # joint names                                   *
     *      geometry_msgs/Point[] joint_pos_3D     # joint cartesian positions                     *
     *                                                                                          *
     *******************************************************************************************/
    
    if (!skeleton_message_first_row && save_skeleton_message) {

        // First Row: Time Stamp, Header, User ID, ...
        skeleton_message_save << "Timestamp,Timestamp,Header,Header, ,user id, ,";

        // ... Joint Names (x3 each joint), twice (3D and 2D) ...
        for (unsigned i = 0; i < skeleton_data.skeletons[0].joint_names.size(); i++) {skeleton_message_save << skeleton_data.skeletons[0].joint_names[i] << "," << skeleton_data.skeletons[0].joint_names[i] << "," << skeleton_data.skeletons[0].joint_names[i] << ",";}
        skeleton_message_save << " ,";
        for (unsigned i = 0; i < skeleton_data.skeletons[0].joint_names.size(); i++) {skeleton_message_save << skeleton_data.skeletons[0].joint_names[i] << "," << skeleton_data.skeletons[0].joint_names[i] << "," << skeleton_data.skeletons[0].joint_names[i] << ",";}

        // Second Row: sec, nsec, frame id, seq, x,y,z (x20 joint_names)
        skeleton_message_save << "\nsec,nsec,frame_id,seq, , , ,";

        // ... x,y,z (x20 joint_names), twice (3D and 2D) ...
        for (unsigned i = 0; i < skeleton_data.skeletons[0].joint_names.size(); i++) {skeleton_message_save << "x - 3D,y - 3D,z - 3D" << ",";}
        skeleton_message_save << " ,";
        for (unsigned i = 0; i < skeleton_data.skeletons[0].joint_names.size(); i++) {skeleton_message_save << "x - 2D,y - 2D,z - 2D" << ",";}

        // Third Row: Empty
        skeleton_message_save << "\n\n";

        // Turn Up the Flag
        skeleton_message_first_row = true;

    }

    if (start_registration && save_skeleton_message) {

        // Check Skeleton Joint Size (check if all 20 joints  are present)
        if (skeleton_data.skeletons[0].joint_names.size() != 20) {ROS_WARN_STREAM("Warning! Skeleton Joint Size = " << skeleton_data.skeletons[0].joint_names.size());}

        // Time Stamp & Header
        skeleton_message_save << skeleton_data.header.stamp.sec << "," << skeleton_data.header.stamp.nsec << ",";
        skeleton_message_save << skeleton_data.header.frame_id << "," << skeleton_data.header.seq << ", ,";

        // User ID
        skeleton_message_save << skeleton_data.skeletons[0].user_id << ", ,";

        // Data (3D x,y,z for each joint)
        for (unsigned int i = 0; i < skeleton_data.skeletons[0].joint_names.size(); i++) {
            skeleton_message_save << skeleton_data.skeletons[0].joint_pos_3D[i].x << ",";
            skeleton_message_save << skeleton_data.skeletons[0].joint_pos_3D[i].y << ",";
            skeleton_message_save << skeleton_data.skeletons[0].joint_pos_3D[i].z << ",";
        }

        skeleton_message_save << " ,";

        for (unsigned int i = 0; i < skeleton_data.skeletons[0].joint_names.size(); i++) {
            skeleton_message_save << skeleton_data.skeletons[0].joint_pos_2D[i].x << ",";
            skeleton_message_save << skeleton_data.skeletons[0].joint_pos_2D[i].y << ",";
            skeleton_message_save << skeleton_data.skeletons[0].joint_pos_2D[i].z << ",";
        }

        // New Line
        skeleton_message_save << "\n";
        
        ROS_INFO_STREAM_THROTTLE(5, "Saving Skeleton Message...");

    }


    if (!skeleton_pose_first_row && save_skeleton_pose) {

        // First Row: Time Stamp, Header, User ID, ...
        skeleton_pose_save << "Timestamp,Timestamp, ,";

        // ... Joint Names (x3 each joint) ...
        for (unsigned i = 0; i < skeleton_data.skeletons[0].joint_names.size(); i++) {skeleton_pose_save << skeleton_data.skeletons[0].joint_names[i] << "," << skeleton_data.skeletons[0].joint_names[i] << "," << skeleton_data.skeletons[0].joint_names[i] << ",";}

        // Second Row: sec, nsec, frame id, seq, x,y,z (x20 joint_names)
        skeleton_pose_save << "\nsec,nsec, ,";

        // ... x,y,z (x20 joint_names) ...
        for (unsigned i = 0; i < skeleton_data.skeletons[0].joint_names.size(); i++) {skeleton_pose_save << "x,y,z" << ",";}

        // Third Row: Empty
        skeleton_pose_save << "\n\n";

        // Turn Up the Flag
        skeleton_message_first_row = true;

    }

    if (start_registration && save_skeleton_pose) {

        // Check Skeleton Joint Size (check if all 20 joints  are present)
        if (skeleton_data.skeletons[0].joint_names.size() != 20) {ROS_WARN_STREAM("Warning! Skeleton Joint Size = " << skeleton_data.skeletons[0].joint_names.size());}

        // Time Stamp
        skeleton_pose_save << skeleton_data.header.stamp.sec << "," << skeleton_data.header.stamp.nsec << ", ,";

        // Data (x,y,z for each joint)
        for (unsigned int i = 0; i < skeleton_data.skeletons[0].joint_names.size(); i++) {
            skeleton_pose_save << skeleton_data.skeletons[0].joint_pos_3D[i].x << ",";
            skeleton_pose_save << skeleton_data.skeletons[0].joint_pos_3D[i].y << ",";
            skeleton_pose_save << skeleton_data.skeletons[0].joint_pos_3D[i].z << ",";
        }

        // New Line
        skeleton_pose_save << "\n";
        
        ROS_INFO_STREAM_THROTTLE(5, "Saving Skeleton Pose...");

    }
}

//------------------------------------------------------ FUNCTIONS -----------------------------------------------------//


void data_acquisition::ofstream_creation (std::string ofstream_name) {

    // Get ROS Package Path
    std::string package_path = ros::package::getPath("gesture_recognition_nuitrack");
    std::cout << std::endl;
    ROS_INFO_STREAM_ONCE("Package Path:  " << package_path);

    std::string image_raw_save_file = package_path + "/dataset/data_acquisition/" + ofstream_name + "_image_raw.csv";
    std::string skeleton_message_save_file = package_path + "/dataset/data_acquisition/" + ofstream_name + "_skeleton_message.csv";
    std::string skeleton_pose_save_file = package_path + "/dataset/data_acquisition/" + ofstream_name + "_skeleton_pose.csv";

    // OfStream Creation
    if (save_image_raw) {image_raw_save = std::ofstream(image_raw_save_file); ROS_WARN_STREAM("Image Raw File:\t\"" << ofstream_name << "_image_raw.csv\"");}
    if (save_skeleton_message) {skeleton_message_save = std::ofstream(skeleton_message_save_file); ROS_WARN_STREAM("Skeleton Message File:\t\"" << ofstream_name << "_skeleton_message.csv\"");}
    if (save_skeleton_pose) {skeleton_pose_save = std::ofstream(skeleton_pose_save_file); ROS_WARN_STREAM("Skeleton Pose File:\t\"" << ofstream_name << "_skeleton_pose.csv\"");}

}

void data_acquisition::draw_skeleton (nuitrack_msgs::SkeletonData skeleton_data, sensor_msgs::Image image) {

    // Create OpenCV Image Matrix
    cv_bridge::CvImagePtr img;
    img = cv_bridge::toCvCopy(image, image.encoding);
    cv::Mat img_mat =  img->image;

    ROS_INFO_STREAM_THROTTLE(5,"\nImage Timestamp: " << skeleton_data.header.stamp);
    ROS_INFO_STREAM_THROTTLE(5,"Skeleton Timestamp: " << image.header.stamp);

    // Create Skeleton Markers Vector
    std::vector<Skeleton_Markers> skeleton_markers;


    // Draw Single Joints:

    for (unsigned int i = 0; i < skeleton_data.joint_names.size(); i++) {

        // 2D Coords (Joint position in normalized projective coordinates (x, y from 0.0 to 1.0, z is real)).
        double normalized_coords[2] = {skeleton_data.joint_pos_2D[i].x, skeleton_data.joint_pos_2D[i].y};

        // De-Normalize in Camera Frame (image_raw 620x480)
        double de_normalized_coords[2] = {normalized_coords[0] * 640, normalized_coords[1] * 480};

        // Save Data on Markers Vector
        Skeleton_Markers temp;
        temp.joint_name = skeleton_data.joint_names[i];
        temp.joint_pose_2D[0] = de_normalized_coords[0];
        temp.joint_pose_2D[1] = de_normalized_coords[1];
        skeleton_markers.push_back(temp);

        // Drow Skeleton Joint Positions
        cv::circle(img_mat, cv::Point(skeleton_markers[i].joint_pose_2D[0],skeleton_markers[i].joint_pose_2D[1]), 1, (0, 0, 255), 10);

    }

    // Draw Skeleton Lines:

    // Head, Neck, Collar (letf collar = right collar)
    draw_skeleton_between_two_markers(HEAD, NECK, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(NECK, LEFT_COLLAR, skeleton_markers, &img_mat);

    // Left Arm
    draw_skeleton_between_two_markers(LEFT_COLLAR, LEFT_SHOULDER, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(LEFT_SHOULDER, LEFT_ELBOW, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(LEFT_ELBOW, LEFT_WRIST, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(LEFT_WRIST, LEFT_HAND, skeleton_markers, &img_mat);

    // Right Arm
    draw_skeleton_between_two_markers(RIGHT_COLLAR, RIGHT_SHOULDER, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(RIGHT_SHOULDER, RIGHT_ELBOW, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(RIGHT_ELBOW, RIGHT_WRIST, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(RIGHT_WRIST, RIGHT_HAND, skeleton_markers, &img_mat);

    // Body
    draw_skeleton_between_two_markers(RIGHT_COLLAR, TORSO, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(TORSO, WAIST, skeleton_markers, &img_mat);

    // Left Leg
    draw_skeleton_between_two_markers(WAIST, LEFT_HIP, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(LEFT_HIP, LEFT_KNEE, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(LEFT_KNEE, LEFT_ANKLE, skeleton_markers, &img_mat);

    // Right Leg
    draw_skeleton_between_two_markers(WAIST, RIGHT_HIP, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(RIGHT_HIP, RIGHT_KNEE, skeleton_markers, &img_mat);
    draw_skeleton_between_two_markers(RIGHT_KNEE, RIGHT_ANKLE, skeleton_markers, &img_mat);

    // Drow Test Point (MAX COORDS)
    // cv::circle(img_mat, cv::Point(640,480), 10, (255, 0, 0), 10);

    // Show Results
    cv::imshow("Skeleton Markers",img_mat);
    cv::waitKey(10);

}

void data_acquisition::draw_skeleton_between_two_markers (std::string marker1, std::string marker2, std::vector<Skeleton_Markers> skeleton, cv::Mat *img) {

    cv::Point point1, point2;

    for (unsigned int i = 0; i < skeleton.size(); i++) {
        
        // Find the Desired Marker Points
        if (skeleton[i].joint_name == marker1) {point1 = cv::Point(skeleton[i].joint_pose_2D[0],skeleton[i].joint_pose_2D[1]);}
        if (skeleton[i].joint_name == marker2) {point2 = cv::Point(skeleton[i].joint_pose_2D[0],skeleton[i].joint_pose_2D[1]);}

    }

    cv::line(*img,point1,point2,(0,0,255),5);

}

//-------------------------------------------------------- MAIN --------------------------------------------------------//


void data_acquisition::spinner (void) {

    ros::spinOnce();

    // Draw Skeleton On Image Raw (only if Image and Skeletons already exists)
    if (skeleton_data_received && image_row_received) {draw_skeleton (skeleton, image_raw);}

    if (shutdown_required) {ros::shutdown();}

}
