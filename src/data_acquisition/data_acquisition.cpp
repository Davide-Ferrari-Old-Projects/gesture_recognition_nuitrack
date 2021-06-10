#include "data_acquisition/data_acquisition.h"

//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

data_acquisition::data_acquisition() {

    // ---- LOAD PARAMETERS ---- //
    if (!nh.param<bool>("/data_acquisition_Node/save_image_raw", save_image_raw, false)) {ROS_ERROR("Couldn't retrieve the Save Image Raw Parameter value.");}
    if (!nh.param<bool>("/data_acquisition_Node/save_skeleton_message", save_skeleton_message, false)) {ROS_ERROR("Couldn't retrieve the Save Skeleton Message Parameter value.");}
    if (!nh.param<bool>("/data_acquisition_Node/save_skeleton_pose", save_skeleton_pose, true)) {ROS_ERROR("Couldn't retrieve the Save Skeleton Pose Parameter value.");}
    if (!nh.param<bool>("/data_acquisition_Node/data_filtering", data_filtering, true)) {ROS_ERROR("Couldn't retrieve the Data Filtering Parameter value.");}
    if (!nh.param<bool>("/data_acquisition_Node/ignore_leg_markers", ignore_leg_markers, true)) {ROS_ERROR("Couldn't retrieve the Ignore Leg Markers Parameter value.");}

    // ---- ROS SUBSCRIBERS ---- //
    image_raw_subscriber = nh.subscribe("/nuitrack/rgb/image_raw", 0, &data_acquisition::image_raw_Callback, this);
    skeleton_data_subscriber = nh.subscribe("/nuitrack/skeletons", 0, &data_acquisition::skeleton_data_Callback, this);
  
    // ---- ROS SERVICES ---- //
    start_registration_service = nh.advertiseService("/data_acquisition_Node/start_registration", &data_acquisition::Start_Registration_Service_Callback, this);
    stop_registration_service = nh.advertiseService("/data_acquisition_Node/stop_registration", &data_acquisition::Stop_Registration_Service_Callback, this);

    // Variable Initialization
    start_registration = false;
    shutdown_required = false;
    queue_empty = false;
    skeleton_data_received = false;
    image_row_received = false;
    first_skeleton_callback = true;

    // First Row Bool Initialization
    image_raw_first_row = false;
    skeleton_message_first_row = false;
    skeleton_pose_first_row = false;
    
    // DEBUG
    DEBUG = false;

}

data_acquisition::~data_acquisition() {}


//------------------------------------------------------ CALLBACK -------------------------------------------------------//


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

    shutdown_required = true;

    std::cout << "\n";
    ROS_WARN("Wait For Data Saving...");

    res.success = true;
    return true;

}


void data_acquisition::image_raw_Callback (const sensor_msgs::Image::ConstPtr &msg) {

    image_raw = *msg;
    image_row_received = true;

    // Get Message Time
    new_message_time = ros::Time::now();

    // Save Image on .csv
    if (save_image_raw) {ofstream_image_raw(image_raw);}

}


void data_acquisition::skeleton_data_Callback (const nuitrack_msgs::SkeletonDataArray::ConstPtr &msg) {

    nuitrack_msgs::SkeletonDataArray skeleton_data = *msg;
    skeleton_data_received = true;

    // Get Message Time
    new_message_time = ros::Time::now();

    if (skeleton_data.skeletons.size() > 0) {

        // Skeleton Data    
        skeleton = skeleton_data.skeletons[0];

        // Header
        skeleton.header = skeleton_data.header;

    } else {ROS_ERROR("Skeleton Message Missing");}

    // Only First Cycle
    if (first_skeleton_callback) {last_skeleton_data = skeleton; first_skeleton_callback = false;}

    if (!data_filtering || check_data_validity(last_skeleton_data, skeleton)) {

        // Assign Current Message to Last Skeleton Data
        last_skeleton_data = skeleton;

        // Save Skeleton Message on .csv
        if (save_skeleton_message) {ofstream_skeleton_message(skeleton_data);}

        // Save Skeleton Pose on .csv
        if (save_skeleton_pose) {ofstream_skeleton_pose(skeleton_data);}

    } else {

        // Show The Skeleton and Ask the User if it's Good
        if (image_row_received) {

            int keycode = draw_skeleton (skeleton, image_raw, "Data is Valid ? s/n", 0);

            // ROS_WARN_STREAM(keycode);
            int s = 1048691, n = 1048686;

            // If User Answer 's'
            if (keycode == s) {

                // Assign Current Message to Last Skeleton Data
                last_skeleton_data = skeleton;

                // Save Skeleton Message on .csv
                if (save_skeleton_message) {ofstream_skeleton_message(skeleton_data);}

                // Save Skeleton Pose on .csv
                if (save_skeleton_pose) {ofstream_skeleton_pose(skeleton_data);}

            }

        }

    }

}


//-------------------------------------------------- OFSTREAM FUNCTION --------------------------------------------------//

void data_acquisition::ofstream_creation (std::string ofstream_name) {

    // Get ROS Package Path
    std::string package_path = ros::package::getPath("gesture_recognition_nuitrack");
    std::cout << std::endl;
    // ROS_INFO_STREAM_ONCE("Package Path:  " << package_path);

    std::string image_raw_save_file = package_path + "/dataset/data_acquisition/" + ofstream_name + "_image_raw.csv";
    std::string skeleton_message_save_file = package_path + "/dataset/data_acquisition/" + ofstream_name + "_skeleton_message.csv";
    std::string skeleton_pose_save_file = package_path + "/dataset/data_acquisition/" + ofstream_name + "_skeleton_pose.csv";

    // OfStream Creation
    if (save_image_raw) {image_raw_save = std::ofstream(image_raw_save_file); ROS_WARN_STREAM("Image Raw File:\t\t\"" << ofstream_name << "_image_raw.csv\"");}
    if (save_skeleton_message) {skeleton_message_save = std::ofstream(skeleton_message_save_file); ROS_WARN_STREAM("Skeleton Message File:\t\"" << ofstream_name << "_skeleton_message.csv\"");}
    if (save_skeleton_pose) {skeleton_pose_save = std::ofstream(skeleton_pose_save_file); ROS_WARN_STREAM("Skeleton Pose File:\t\"" << ofstream_name << "_skeleton_pose.csv\"");}

    // New Line
    std::cout << "\n";

}


void data_acquisition::ofstream_image_raw (sensor_msgs::Image image) {

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

    if (start_registration) {

        // Time Stamp & Header
        image_raw_save << image.header.stamp.sec << "," << image.header.stamp.nsec << ",";
        image_raw_save << image.header.frame_id << "," << image.header.seq << ", ,";

        // Dimension
        image_raw_save << image.height << "," << image.width << ", ,";

        // Parameters ()
        image_raw_save << image.encoding << "," << image.is_bigendian << "," << image.step << ", ,";

        // Data (size = row (height) * step)
        for (unsigned int i = 0; i < image.data.size(); i++) {image_raw_save << int(image.data[i]) << ",";}

        // New Line
        image_raw_save << "\n";

        // Show Image Raw
        cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(image, "bgr8");
        cv::imshow("Image Raw",img->image);
        cv::waitKey(10);

        ROS_INFO_STREAM_THROTTLE(5, "Saving Image Raw...");
    
    }

}


void data_acquisition::ofstream_skeleton_message (nuitrack_msgs::SkeletonDataArray skeleton_message) {

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
     *      string[] joint_names                # joint names                                   *
     *      geometry_msgs/Point[] joint_pos_3D  # joint cartesian positions                     *
     *                                                                                          *
     *******************************************************************************************/
    
    if (!skeleton_message_first_row) {

        // First Row: Time Stamp, Header, User ID, ...
        skeleton_message_save << "Timestamp,Timestamp,Header,Header, ,user id, ,";

        // ... Joint Names (x3 each joint), twice (3D and 2D) ...
        for (unsigned i = 0; i < skeleton_message.skeletons[0].joint_names.size(); i++) {skeleton_message_save << skeleton_message.skeletons[0].joint_names[i] << "," << skeleton_message.skeletons[0].joint_names[i] << "," << skeleton_message.skeletons[0].joint_names[i] << ",";}
        skeleton_message_save << " ,";
        for (unsigned i = 0; i < skeleton_message.skeletons[0].joint_names.size(); i++) {skeleton_message_save << skeleton_message.skeletons[0].joint_names[i] << "," << skeleton_message.skeletons[0].joint_names[i] << "," << skeleton_message.skeletons[0].joint_names[i] << ",";}

        // Second Row: sec, nsec, frame id, seq, x,y,z (x20 joint_names)
        skeleton_message_save << "\nsec,nsec,frame_id,seq, , , ,";

        // ... x,y,z (x20 joint_names), twice (3D and 2D) ...
        for (unsigned i = 0; i < skeleton_message.skeletons[0].joint_names.size(); i++) {skeleton_message_save << "x - 3D,y - 3D,z - 3D" << ",";}
        skeleton_message_save << " ,";
        for (unsigned i = 0; i < skeleton_message.skeletons[0].joint_names.size(); i++) {skeleton_message_save << "x - 2D,y - 2D,z - 2D" << ",";}

        // Third Row: Empty
        skeleton_message_save << "\n\n";

        // Turn Up the Flag
        skeleton_message_first_row = true;

    }

    if (start_registration) {

        // Check Skeleton Joint Size (check if all 20 joints  are present)
        if (skeleton_message.skeletons[0].joint_names.size() != 20) {ROS_WARN_STREAM("Warning! Skeleton Joint Size = " << skeleton_message.skeletons[0].joint_names.size());}

        // Time Stamp & Header
        skeleton_message_save << skeleton_message.header.stamp.sec << "," << skeleton_message.header.stamp.nsec << ",";
        skeleton_message_save << skeleton_message.header.frame_id << "," << skeleton_message.header.seq << ", ,";

        // User ID
        skeleton_message_save << skeleton_message.skeletons[0].user_id << ", ,";

        // Data (3D x,y,z for each joint)
        for (unsigned int i = 0; i < skeleton_message.skeletons[0].joint_names.size(); i++) {
            skeleton_message_save << skeleton_message.skeletons[0].joint_pos_3D[i].x << ",";
            skeleton_message_save << skeleton_message.skeletons[0].joint_pos_3D[i].y << ",";
            skeleton_message_save << skeleton_message.skeletons[0].joint_pos_3D[i].z << ",";
        }

        skeleton_message_save << " ,";

        for (unsigned int i = 0; i < skeleton_message.skeletons[0].joint_names.size(); i++) {
            skeleton_message_save << skeleton_message.skeletons[0].joint_pos_2D[i].x << ",";
            skeleton_message_save << skeleton_message.skeletons[0].joint_pos_2D[i].y << ",";
            skeleton_message_save << skeleton_message.skeletons[0].joint_pos_2D[i].z << ",";
        }

        // New Line
        skeleton_message_save << "\n";
        
        ROS_INFO_STREAM_THROTTLE(5, "Saving Skeleton Message...");

    }

}


void data_acquisition::ofstream_skeleton_pose (nuitrack_msgs::SkeletonDataArray skeleton_pose) {

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
     *      string[] joint_names                # joint names                                   *
     *      geometry_msgs/Point[] joint_pos_3D  # joint cartesian positions                     *
     *                                                                                          *
     *******************************************************************************************/

    if (!skeleton_pose_first_row) {

        // First Row: Time Stamp, Header, User ID, ...
        skeleton_pose_save << "Timestamp,Timestamp,Header, ,";

        // ... Joint Names (x3 each joint) ...
        for (unsigned i = 0; i < skeleton_pose.skeletons[0].joint_names.size(); i++) {skeleton_pose_save << skeleton_pose.skeletons[0].joint_names[i] << "," << skeleton_pose.skeletons[0].joint_names[i] << "," << skeleton_pose.skeletons[0].joint_names[i] << ",";}

        // Second Row: sec, nsec, seq, x,y,z (x20 joint_names)
        skeleton_pose_save << "\nsec,nsec,seq, ,";

        // ... x,y,z (x20 joint_names) ...
        for (unsigned i = 0; i < skeleton_pose.skeletons[0].joint_names.size(); i++) {skeleton_pose_save << "x,y,z" << ",";}

        // Third Row: Empty
        skeleton_pose_save << "\n\n";

        // Turn Up the Flag
        skeleton_pose_first_row = true;

    }

    if (start_registration) {

        // Check Skeleton Joint Size (check if all 20 joints  are present)
        if (skeleton_pose.skeletons[0].joint_names.size() != 20) {ROS_WARN_STREAM("Warning! Skeleton Joint Size = " << skeleton_pose.skeletons[0].joint_names.size());}

        // Time Stamp
        skeleton_pose_save << skeleton_pose.header.stamp.sec << "," << skeleton_pose.header.stamp.nsec << "," << skeleton_pose.header.seq << ", ,";

        // Data (x,y,z for each joint)
        for (unsigned int i = 0; i < skeleton_pose.skeletons[0].joint_names.size(); i++) {
            skeleton_pose_save << skeleton_pose.skeletons[0].joint_pos_3D[i].x << ",";
            skeleton_pose_save << skeleton_pose.skeletons[0].joint_pos_3D[i].y << ",";
            skeleton_pose_save << skeleton_pose.skeletons[0].joint_pos_3D[i].z << ",";
        }

        // New Line
        skeleton_pose_save << "\n";
        
        ROS_INFO_STREAM_THROTTLE(5, "Saving Skeleton Pose...");

    }

}


//-------------------------------------------------- SKELETON DRAWING ---------------------------------------------------//

int data_acquisition::draw_skeleton (nuitrack_msgs::SkeletonData skeleton_data, sensor_msgs::Image image, std::string window_name = "Skeleton Markers", int wait_key = 10) {

    // Create OpenCV Image Matrix
    cv_bridge::CvImagePtr img;
    img = cv_bridge::toCvCopy(image, image.encoding);
    cv::Mat img_mat =  img->image;

    // Print Timestamp
    // ROS_INFO_STREAM_THROTTLE(5,"\nImage Timestamp: " << image.header.stamp);
    // ROS_INFO_STREAM_THROTTLE(5,"Skeleton Timestamp: " << skeleton_data.header.stamp);

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
    cv::imshow(window_name, img_mat);
    int keycode_pressed = cv::waitKeyEx(wait_key);

    return keycode_pressed;

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


//-------------------------------------------- CHECK SKELETON DATA FUNCTION ---------------------------------------------//

bool data_acquisition::check_data_validity (nuitrack_msgs::SkeletonData last_valid_skeleton_data, nuitrack_msgs::SkeletonData new_skeleton_data) {

    nuitrack_msgs::SkeletonData last_data = last_valid_skeleton_data;
    nuitrack_msgs::SkeletonData new_data = new_skeleton_data;

    bool all_points_are_close = true;

    for (unsigned int i = 0; i < last_data.joint_names.size(); i++) {

        // Ignore Left and Right Legs
        if (ignore_leg_markers && (last_data.joint_names[i] == RIGHT_HIP || last_data.joint_names[i] == RIGHT_KNEE || last_data.joint_names[i] == RIGHT_ANKLE ||
                                   last_data.joint_names[i] == LEFT_HIP  || last_data.joint_names[i] == LEFT_KNEE  || last_data.joint_names[i] == LEFT_ANKLE)) {

            // DO NOTHING

        } else {
            
            // If One Point isn't close to the last valid ones set the flag false
            if (!points_are_close(last_data.joint_pos_3D[i], new_data.joint_pos_3D[i])) {
                
                all_points_are_close = false;
                ROS_WARN_STREAM("Joint " << last_data.joint_names[i] << " too far");

            }
        
        }

    }

    if (all_points_are_close) {return true;}
    else {std::cout << "\n"; return false;}

}


bool data_acquisition::points_are_close (geometry_msgs::Point first_point, geometry_msgs::Point second_point) {

    // Two Points are Considered Near if the Distance is lower than 30cm
    const double distance_tolerance = 30;

    double points_distance = sqrt(pow(first_point.x - second_point.x, 2) + pow(first_point.y - second_point.y, 2) + pow(first_point.z - second_point.z, 2));

    if (fabs(points_distance) < distance_tolerance) {return true;}
    else {return false;}
    
}


//-------------------------------------------------------- MAIN --------------------------------------------------------//


void data_acquisition::spinner (void) {

    ros::spinOnce();

    // Draw Skeleton On Image Raw (only if Image and Skeletons already exists)
    if (skeleton_data_received && image_row_received) {draw_skeleton (skeleton, image_raw);}

    // Shutdown Required
    if (shutdown_required) {

        // Wait for the Subscriber Queue to Empty
        while (!queue_empty) {

            ros::spinOnce();

            // Wait 2 Seconds From Last Message Received (if any queue exist)
            if ((ros::Time::now() - new_message_time).toSec() > 2) {queue_empty = true;}

            // Draw Skeleton On Image Raw (only if Image and Skeletons already exists)
            if (skeleton_data_received && image_row_received) {draw_skeleton (skeleton, image_raw);}

        }

        // Stop Registration
        start_registration = false;
        
        // Close OfStreams
        if (save_image_raw) {image_raw_save.close();}
        if (save_skeleton_message) {skeleton_message_save.close();}
        if (save_skeleton_pose) {skeleton_pose_save.close();}

        ROS_WARN("Data Saved Succesfully\n");

        ros::shutdown();
        
    }

    // DEBUG
    if (DEBUG) {
        
        if (skeleton_data_received && image_row_received) {

        int keycode = draw_skeleton (skeleton, image_raw, "Data is Valid ? s/n", 0);
        ROS_WARN_STREAM(keycode);
        
        }
    }

}
