#include "data_acquisition/data_acquisition.h"

//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

data_acquisition::data_acquisition() {

     // ---- LOAD PARAMETERS ---- //
    if (!nh.param<bool>("/data_acquisition_Node/live_mode", live_mode, false)) {ROS_ERROR("Couldn't retrieve the Live Mode Parameter value.");}
    // if (!nh.param<std::string>("/data_acquisition_Node/save_file_name", save_file_name, "new_save")) {ROS_ERROR("Couldn't retrieve the csv Save File Name.");}
  
    // ---- ROS SERVICES ---- //
    start_registration_service = nh.advertiseService("/data_acquisition_Node/start_registration", &data_acquisition::Start_Registration_Service_Callback, this);
    stop_registration_service = nh.advertiseService("/data_acquisition_Node/stop_registration", &data_acquisition::Stop_Registration_Service_Callback, this);

    // Variable Initialization
    start_registration = false;
    shutdown_required = false;
    
}

data_acquisition::~data_acquisition() {}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


bool data_acquisition::Start_Registration_Service_Callback (gesture_recognition::String::Request &req, gesture_recognition::String::Response &res) {
    
    start_registration = true;
    save_file_name = req.message_data;

    // ---- INITIALIZE ROS SUBSCRIBERS ---- //
    image_raw_subscriber            = nh.subscribe("/nuitrack/rgb/image_raw", 1,        &data_acquisition::image_raw_Callback, this);
    skeleton_data_subscriber        = nh.subscribe("/nuitrack/skeletons", 1,            &data_acquisition::skeleton_data_Callback, this);
    visualization_marker_subscriber = nh.subscribe("/nuitrack/viz_skeleton_markers", 1, &data_acquisition::skeleton_marker_Callback, this);

    // ---- OFSTREAM CREATION ---- //
    ofstream_creation (save_file_name);
    
    res.success = true;
    return true;

}


bool data_acquisition::Stop_Registration_Service_Callback (std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

    start_registration = false;
    shutdown_required = true;

    // Close OfStreams
    image_raw_save.close();
    skeleton_data_save.close();
    skeleton_marker_save.close();

    ROS_WARN("Data Saved Succesfully");

    res.success = true;
    return true;

}


void data_acquisition::image_raw_Callback (const sensor_msgs::Image::ConstPtr &msg) {

    sensor_msgs::Image image_raw = *msg;
    image_temp = image_raw;

    // Change from "rgb8" to "bgr8"
    // if (image_raw.encoding == "bgr8") {image_raw.encoding == "rgb8";}
    // else if (image_raw.encoding == "rgb8") {image_raw.encoding == "brg8";};
    
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

    if (!image_raw_first_row) {

        // First, Second Row: Time Stamp, Header, Dimension, Image Parameters, Data ...
        image_raw_save << "Timestamp,Timestamp,Header,Header, ,Height,Width, ,Encoding,Is Bigendian,Step, ,Data...\n";
        image_raw_save << "sec,nsec,frame_id,seq \n";

        // Third Row: Empty
        image_raw_save << "\n";

        // Turn Up the Flag
        image_raw_first_row = true;

    }

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
    cv_bridge::CvImagePtr img;
    img = cv_bridge::toCvCopy(image_raw, image_raw.encoding);
    cv::imshow("my display",img->image);
    cv::waitKey(1);

    ROS_INFO_STREAM_THROTTLE(5, "Saving Image Raw...");
    
}

void data_acquisition::skeleton_data_Callback (const nuitrack_msgs::SkeletonDataArray::ConstPtr &msg) {

    nuitrack_msgs::SkeletonDataArray skeleton_data = *msg;

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
     *      string[] joints                     # joint names                                   *
     *      geometry_msgs/Point[] joint_pos     # joint cartesian positions                     *
     *                                                                                          *
     *******************************************************************************************/
    
    if (!skeleton_data_first_row) {

        // First Row: Time Stamp, Header, User ID, ...
        skeleton_data_save << "Timestamp,Timestamp,Header,Header, ,user id, ,";

        // ... Joint Names (x3 each joint) ...
        for (unsigned i = 0; i < skeleton_data.skeletons[0].joints.size(); i++) {skeleton_data_save << skeleton_data.skeletons[0].joints[i] << "," << skeleton_data.skeletons[0].joints[i] << "," << skeleton_data.skeletons[0].joints[i] << ",";}

        // Second Row: sec, nsec, frame id, seq, x,y,z (x20 joints)
        skeleton_data_save << "\nsec,nsec,frame_id,seq, , , ,";

        // ... x,y,z (x20 joints) ...
        for (unsigned i = 0; i < skeleton_data.skeletons[0].joints.size(); i++) {skeleton_data_save << "x,y,z" << ",";}

        // Third Row: Empty
        skeleton_data_save << "\n\n";

        // Turn Up the Flag
        skeleton_data_first_row = true;

    }

    // Check Skeleton Joint Size (check if all 20 joints are present)
    if (skeleton_data.skeletons[0].joints.size() != 20) {ROS_WARN_STREAM("Warning! Skeleton Joint Size = " << skeleton_data.skeletons[0].joints.size());}

    // Time Stamp & Header
    skeleton_data_save << skeleton_data.header.stamp.sec << "," << skeleton_data.header.stamp.nsec << ",";
    skeleton_data_save << skeleton_data.header.frame_id << "," << skeleton_data.header.seq << ", ,";

    // User ID
    skeleton_data_save << skeleton_data.skeletons[0].id << ", ,";

    // Data (x,y,z for each joint)
    for (unsigned int i = 0; i < skeleton_data.skeletons[0].joints.size(); i++) {
        skeleton_data_save << skeleton_data.skeletons[0].joint_pos[i].x << ",";
        skeleton_data_save << skeleton_data.skeletons[0].joint_pos[i].y << ",";
        skeleton_data_save << skeleton_data.skeletons[0].joint_pos[i].z << ",";
    }

    // New Line
    skeleton_data_save << "\n";

    ROS_INFO_STREAM_THROTTLE(5, "Saving Skeleton Data...");

}

void data_acquisition::skeleton_marker_Callback (const visualization_msgs::MarkerArray::ConstPtr &msg) {

    visualization_msgs::MarkerArray skeleton_marker = *msg;

    cv_bridge::CvImagePtr img;
    img = cv_bridge::toCvCopy(image_temp, image_temp.encoding);
    cv::Mat img_mat =  img->image;
    cv::circle(img_mat, cv::Point(1,1), 1, (0, 0, 255), 10);
    cv::circle(img_mat, cv::Point(10,10), 10, (0, 0, 255), 10);
    cv::circle(img_mat, cv::Point(1920,1080), 100, (0, 0, 255), 100);

    cv::imshow("skeleton marker",img_mat);
    cv::waitKey(1);

    // skeleton_marker_save << "Time Stamp,Time Stamp,Data\n\n";

}

//------------------------------------------------------ FUNCTIONS -----------------------------------------------------//


void data_acquisition::ofstream_creation (std::string ofstream_name) {

    // Get ROS Package Path
    std::string package_path = ros::package::getPath("gesture_recognition_nuitrack");
    std::cout << std::endl;
    ROS_INFO_STREAM_ONCE("Package Path:  " << package_path);

    std::string image_raw_save_file         = package_path + "/dataset/data_acquisition/" + ofstream_name + "_image_raw.csv";
    std::string skeleton_data_save_file     = package_path + "/dataset/data_acquisition/" + ofstream_name + "_skeleton_data.csv";
    std::string skeleton_marker_save_file   = package_path + "/dataset/data_acquisition/" + ofstream_name + "_skeleton_marker.csv";

    // OfStream Creation
    image_raw_save       = std::ofstream(image_raw_save_file);
    skeleton_data_save   = std::ofstream(skeleton_data_save_file);
    skeleton_marker_save = std::ofstream(skeleton_marker_save_file);

    // First Row Bool Initialization
    image_raw_first_row = false;
    skeleton_data_first_row = false;
    skeleton_marker_first_row = false;

    ROS_WARN_STREAM("Output Files:  \"" << ofstream_name << "_image_raw.csv\"" << "\t\"" << ofstream_name << "_skeleton_data.csv\"" << "\t\"" << ofstream_name << "_skeleton_marker.csv\"" << std::endl);

}


//-------------------------------------------------------- MAIN --------------------------------------------------------//


void data_acquisition::spinner (void) {

    ros::spinOnce();

    if (shutdown_required) {ros::shutdown();}

}
