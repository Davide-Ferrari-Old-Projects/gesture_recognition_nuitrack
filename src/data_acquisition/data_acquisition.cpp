#include "data_acquisition/data_acquisition.h"

//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

data_acquisition::data_acquisition() {

    // ---- LOAD PARAMETERS ---- //
    // if (!nh.param<bool>("/data_acquisition_Node/live_mode", live_mode, false)) {ROS_ERROR("Couldn't retrieve the Live Mode Parameter value.");}
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

    // First Row Bool Initialization
    image_raw_first_row = false;
    skeleton_data_first_row = false;

    // Inizialize Realsense D435i Intrinsics Parameters
    realsense_intrinsics = realsense_parameters("DEPTH");
    
}

data_acquisition::~data_acquisition() {}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


bool data_acquisition::Start_Registration_Service_Callback (gesture_recognition::String::Request &req, gesture_recognition::String::Response &res) {
    
    start_registration = true;
    save_file_name = req.message_data;

    // ---- OFSTREAM CREATION ---- //
    ofstream_creation (save_file_name);
    
    res.success = true;
    return true;

}


bool data_acquisition::Stop_Registration_Service_Callback (std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

    start_registration = false;
    shutdown_required = true;

    // First Row Bool Initialization
    image_raw_first_row = false;
    skeleton_data_first_row = false;

    // Close OfStreams
    image_raw_save.close();
    skeleton_data_save.close();

    ROS_WARN("Data Saved Succesfully");

    res.success = true;
    return true;

}


void data_acquisition::image_raw_Callback (const sensor_msgs::Image::ConstPtr &msg) {

    image_raw = *msg;
    
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
/* 
        // First, Second Row: Time Stamp, Header, Dimension, Image Parameters, Data ...
        image_raw_save << "Timestamp,Timestamp,Header,Header, ,Height,Width, ,Encoding,Is Bigendian,Step, ,Data...\n";
        image_raw_save << "sec,nsec,frame_id,seq \n";

        // Third Row: Empty
        image_raw_save << "\n"; */

        // Turn Up the Flag
        image_raw_first_row = true;

    }

    if (start_registration) {
/* 
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
        cv::waitKey(10); */

        ROS_INFO_STREAM_THROTTLE(5, "Saving Image Raw...");
    
    }
    
}

void data_acquisition::skeleton_data_Callback (const nuitrack_msgs::SkeletonDataArray::ConstPtr &msg) {

    nuitrack_msgs::SkeletonDataArray skeleton_data = *msg;
    skeleton = skeleton_data.skeletons[0];
    skeleton_array = skeleton_data;

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
     *      geometry_msgs/Point[] joint_pos_3D     # joint cartesian positions                     *
     *                                                                                          *
     *******************************************************************************************/
    
    if (!skeleton_data_first_row) {

 /*        // First Row: Time Stamp, Header, User ID, ...
        skeleton_data_save << "Timestamp,Timestamp,Header,Header, ,user id, ,";

        // ... Joint Names (x3 each joint) ...
        for (unsigned i = 0; i < skeleton_data.skeletons[0].joints.size(); i++) {skeleton_data_save << skeleton_data.skeletons[0].joints[i] << "," << skeleton_data.skeletons[0].joints[i] << "," << skeleton_data.skeletons[0].joints[i] << ",";}

        // Second Row: sec, nsec, frame id, seq, x,y,z (x20 joints)
        skeleton_data_save << "\nsec,nsec,frame_id,seq, , , ,";

        // ... x,y,z (x20 joints) ...
        for (unsigned i = 0; i < skeleton_data.skeletons[0].joints.size(); i++) {skeleton_data_save << "x,y,z" << ",";}

        // Third Row: Empty
        skeleton_data_save << "\n\n";
 */
        // Turn Up the Flag
        skeleton_data_first_row = true;

    }

    if (start_registration) {
/* 
        // Check Skeleton Joint Size (check if all 20 joints are present)
        if (skeleton_data.skeletons[0].joints.size() != 20) {ROS_WARN_STREAM("Warning! Skeleton Joint Size = " << skeleton_data.skeletons[0].joints.size());}

        // Time Stamp & Header
        skeleton_data_save << skeleton_data.header.stamp.sec << "," << skeleton_data.header.stamp.nsec << ",";
        skeleton_data_save << skeleton_data.header.frame_id << "," << skeleton_data.header.seq << ", ,";

        // User ID
        skeleton_data_save << skeleton_data.skeletons[0].id << ", ,";

        // Data (x,y,z for each joint)
        for (unsigned int i = 0; i < skeleton_data.skeletons[0].joints.size(); i++) {
            skeleton_data_save << skeleton_data.skeletons[0].joint_pos_3D[i].x << ",";
            skeleton_data_save << skeleton_data.skeletons[0].joint_pos_3D[i].y << ",";
            skeleton_data_save << skeleton_data.skeletons[0].joint_pos_3D[i].z << ",";
        }

        // New Line
        skeleton_data_save << "\n";
    */
        
        ROS_INFO_STREAM_THROTTLE(5, "Saving Skeleton Data...");

    }

}

//------------------------------------------------------ FUNCTIONS -----------------------------------------------------//


void data_acquisition::ofstream_creation (std::string ofstream_name) {

    // Get ROS Package Path
    std::string package_path = ros::package::getPath("gesture_recognition_nuitrack");
    std::cout << std::endl;
    ROS_INFO_STREAM_ONCE("Package Path:  " << package_path);

    std::string image_raw_save_file = package_path + "/dataset/data_acquisition/" + ofstream_name + "_image_raw.csv";
    std::string skeleton_data_save_file = package_path + "/dataset/data_acquisition/" + ofstream_name + "_skeleton_data.csv";

    // OfStream Creation
    image_raw_save = std::ofstream(image_raw_save_file);
    skeleton_data_save = std::ofstream(skeleton_data_save_file);

    ROS_WARN_STREAM("Output Files:  \"" << ofstream_name << "_image_raw.csv\"" << "\t\"" << ofstream_name << "_skeleton_data.csv\"" << std::endl);

}

void data_acquisition::draw_skeleton (nuitrack_msgs::SkeletonData skeleton_data, sensor_msgs::Image image) {

    // Create OpenCV Image Matrix
    cv_bridge::CvImagePtr img;
    img = cv_bridge::toCvCopy(image, "bgr8");
    cv::Mat img_mat =  img->image;

    ROS_INFO_STREAM_THROTTLE(5,"\nImage Timestamp: " << skeleton_array.header.stamp.toSec());
    ROS_INFO_STREAM_THROTTLE(5,"Skeleton Timestamp: " << image.header.stamp.toSec());


    for (unsigned int i = 0; i < skeleton_data.joints.size(); i++) {

        float pixel[2], point[3];
    
        // Get Skeleton Joint Position (x,y,z) 
        point[0] = skeleton_data.joint_pos_3D[i].x;
        point[1] = skeleton_data.joint_pos_3D[i].y;
        point[2] = skeleton_data.joint_pos_3D[i].z;

        // Convert Poit to Pixel
        rs2_project_point_to_pixel(pixel, &realsense_intrinsics, point);
    
        // TODO: No-Sense Conversion
        pixel[0] *= (620.0 / 720.0);
        pixel[1] *= (480.0 / 1080.0);

        ROS_INFO_STREAM_THROTTLE(5,"\nSkeleton:");
        ROS_INFO_STREAM_THROTTLE(5,"Joint Name: " << skeleton_data.joints[0]);
        ROS_INFO_STREAM_THROTTLE(5,"Point: \t" << point[0] << "\t" << point[1] << "\t" << point[2]);
        ROS_INFO_STREAM_THROTTLE(5,"pixels:\t" << pixel[0] << "\t" << pixel[1]);

        // Drow Skeleton Joint Positions
        cv::circle(img_mat, cv::Point(pixel[0],pixel[1]), 1, (0, 0, 255), 10);

    }

    // Drow Test Point (MAX COORDS)
    // cv::circle(img_mat, cv::Point(620,480), 10, (255, 0, 0), 10);

    // Show Results
    cv::imshow("Skeleton Markers",img_mat);
    cv::waitKey(10);

}

rs2_intrinsics data_acquisition::realsense_parameters (std::string type) { // COLOR or DEPTH

    // Create RealSense Intrinsic Parameters
    rs2_intrinsics realsense_d435i_intrinsics;

    if (type == "COLOR") {

        // RealSense D435i Camera Info (COLOR)
        int width = 1280, height = 720;
        std::string distortion_model = "plumb_bob";
        float D[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
        float K[9] = {914.9129638671875, 0.0, 646.0457763671875, 0.0, 912.9351196289062, 369.348388671875, 0.0, 0.0, 1.0};
        float R[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        float P[12] = {914.9129638671875, 0.0, 646.0457763671875, 0.0, 0.0, 912.9351196289062, 369.348388671875, 0.0, 0.0, 0.0, 1.0, 0.0};
        int binning_x = 0;
        int binning_y = 0;
        
        int roi_x_offset = 0;
        int roi_y_offset = 0;
        int roi_height = 0;
        int roi_width = 0;
        bool roi_do_rectify = false;

        /********************************************************************************************************************************************
         *                                                                                                                                          *
         *   header:                                                                                                                                *
         *   seq: 8425                                                                                                                              *
         *   stamp:                                                                                                                                 *
         *       secs: 1622037279                                                                                                                   *
         *       nsecs: 636879921                                                                                                                   *
         *   frame_id: "camera_color_optical_frame"                                                                                                 *
         *   height: 720                                                                                                                            *
         *   width: 1280                                                                                                                            *   
         *   distortion_model: "plumb_bob"                                                                                                          *
         *   D: [0.0, 0.0, 0.0, 0.0, 0.0]                                                                                                           *
         *   K: [914.9129638671875, 0.0, 646.0457763671875, 0.0, 912.9351196289062, 369.348388671875, 0.0, 0.0, 1.0]                                *
         *   R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]                                                                                       *
         *   P: [914.9129638671875, 0.0, 646.0457763671875, 0.0, 0.0, 912.9351196289062, 369.348388671875, 0.0, 0.0, 0.0, 1.0, 0.0]                 *
         *   binning_x: 0                                                                                                                           *   
         *   binning_y: 0                                                                                                                           *   
         *   roi:                                                                                                                                   *    
         *      x_offset: 0                                                                                                                         *   
         *      y_offset: 0                                                                                                                         *   
         *      height: 0                                                                                                                           *   
         *      width: 0                                                                                                                            *   
         *      do_rectify: False                                                                                                                   *   
         *                                                                                                                                          *
         *******************************************************************************************************************************************/

        realsense_d435i_intrinsics.width = width;
        realsense_d435i_intrinsics.height = height;

        realsense_d435i_intrinsics.ppx = K[2];
        realsense_d435i_intrinsics.ppy = K[5];
        realsense_d435i_intrinsics.fx = K[0];
        realsense_d435i_intrinsics.fy = K[4];

        if (distortion_model == "plumb_bob") {realsense_d435i_intrinsics.model = RS2_DISTORTION_BROWN_CONRADY;}
        else if  (distortion_model == "equidistant") {realsense_d435i_intrinsics.model = RS2_DISTORTION_KANNALA_BRANDT4;}
        for (int i = 0; i < 5; i++) {realsense_d435i_intrinsics.coeffs[i] = D[i];}

    } else if (type == "DEPTH") {
        
        // RealSense D435i Camera Info (DEPTH)
        int width = 848, height = 480;
        std::string distortion_model = "plumb_bob";
        float D[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
        float K[9] = {423.06988525390625, 0.0, 420.19415283203125, 0.0, 423.06988525390625, 238.3802032470703, 0.0, 0.0, 1.0};
        float R[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        float P[12] = {423.06988525390625, 0.0, 420.19415283203125, 0.0, 0.0, 423.06988525390625, 238.3802032470703, 0.0, 0.0, 0.0, 1.0, 0.0};
        int binning_x = 0;
        int binning_y = 0;
        
        int roi_x_offset = 0;
        int roi_y_offset = 0;
        int roi_height = 0;
        int roi_width = 0;
        bool roi_do_rectify = false;

        /********************************************************************************************************************************************
         *                                                                                                                                          *
         *   header:                                                                                                                                *
         *   seq: 2                                                                                                                                 *
         *   stamp:                                                                                                                                 *
         *       secs: 1622037263                                                                                                                   *
         *       nsecs:   3570557                                                                                                                   *
         *   frame_id: "camera_depth_optical_frame"                                                                                                 *
         *   height: 480                                                                                                                            *
         *   width: 848                                                                                                                             *
         *   distortion_model: "plumb_bob"                                                                                                          *
         *   D: [0.0, 0.0, 0.0, 0.0, 0.0]                                                                                                           *
         *   K: [423.06988525390625, 0.0, 420.19415283203125, 0.0, 423.06988525390625, 238.3802032470703, 0.0, 0.0, 1.0]                            *
         *   R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]                                                                                       *
         *   P: [423.06988525390625, 0.0, 420.19415283203125, 0.0, 0.0, 423.06988525390625, 238.3802032470703, 0.0, 0.0, 0.0, 1.0, 0.0]             *
         *   binning_x: 0                                                                                                                           *
         *   binning_y: 0                                                                                                                           *
         *   roi:                                                                                                                                   *
         *      x_offset: 0                                                                                                                         *
         *      y_offset: 0                                                                                                                         *
         *      height: 0                                                                                                                           *
         *      width: 0                                                                                                                            *
         *      do_rectify: False                                                                                                                   *   
         *                                                                                                                                          *
         *******************************************************************************************************************************************/

        realsense_d435i_intrinsics.width = width;
        realsense_d435i_intrinsics.height = height;

        realsense_d435i_intrinsics.ppx = K[2];
        realsense_d435i_intrinsics.ppy = K[5];
        realsense_d435i_intrinsics.fx = K[0];
        realsense_d435i_intrinsics.fy = K[4];

        if (distortion_model == "plumb_bob") {realsense_d435i_intrinsics.model = RS2_DISTORTION_BROWN_CONRADY;}
        else if  (distortion_model == "equidistant") {realsense_d435i_intrinsics.model = RS2_DISTORTION_KANNALA_BRANDT4;}
        for (int i = 0; i < 5; i++) {realsense_d435i_intrinsics.coeffs[i] = D[i];}
    
    }

    return realsense_d435i_intrinsics;

}



//-------------------------------------------------------- MAIN --------------------------------------------------------//


void data_acquisition::spinner (void) {

    ros::spinOnce();

    // Draw Skeleton On Image Raw (only if the Image already exist)
    if (/* start_registration && */ skeleton_data_first_row && image_raw_first_row) {draw_skeleton (skeleton, image_raw);}

    if (shutdown_required) {ros::shutdown();}

}
