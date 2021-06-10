#include "show_skeleton/show_skeleton.h"

//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

show_skeleton::show_skeleton() {

    // ---- ROS SUBSCRIBERS ---- //
    image_raw_subscriber = nh.subscribe("/nuitrack/rgb/image_raw", 0, &show_skeleton::image_raw_Callback, this);
    skeleton_data_subscriber = nh.subscribe("/nuitrack/skeletons", 0, &show_skeleton::skeleton_data_Callback, this);
  
    skeleton_data_received = false;
    image_row_received = false;

}

show_skeleton::~show_skeleton() {}


//------------------------------------------------------ CALLBACK -------------------------------------------------------//

void show_skeleton::image_raw_Callback (const sensor_msgs::Image::ConstPtr &msg) {

    image_raw = *msg;
    image_row_received = true;

}


void show_skeleton::skeleton_data_Callback (const nuitrack_msgs::SkeletonDataArray::ConstPtr &msg) {

    nuitrack_msgs::SkeletonDataArray skeleton_data = *msg;
    skeleton_data_received = true;

    if (skeleton_data.skeletons.size() > 0) {

        // Skeleton Data    
        skeleton = skeleton_data.skeletons[0];

        // Header
        skeleton.header = skeleton_data.header;

    } else {ROS_ERROR("Skeleton Message Missing");}

}

//-------------------------------------------------- SKELETON DRAWING ---------------------------------------------------//

int show_skeleton::draw_skeleton (nuitrack_msgs::SkeletonData skeleton_data, sensor_msgs::Image image, std::string window_name = "Skeleton Markers", int wait_key = 10) {

    // Create OpenCV Image Matrix
    cv_bridge::CvImagePtr img;
    img = cv_bridge::toCvCopy(image, image.encoding);
    cv::Mat img_mat =  img->image;
    
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


void show_skeleton::draw_skeleton_between_two_markers (std::string marker1, std::string marker2, std::vector<Skeleton_Markers> skeleton, cv::Mat *img) {

    cv::Point point1, point2;

    for (unsigned int i = 0; i < skeleton.size(); i++) {
        
        // Find the Desired Marker Points
        if (skeleton[i].joint_name == marker1) {point1 = cv::Point(skeleton[i].joint_pose_2D[0],skeleton[i].joint_pose_2D[1]);}
        if (skeleton[i].joint_name == marker2) {point2 = cv::Point(skeleton[i].joint_pose_2D[0],skeleton[i].joint_pose_2D[1]);}

    }

    cv::line(*img,point1,point2,(0,0,255),5);

}

//-------------------------------------------------------- MAIN --------------------------------------------------------//


void show_skeleton::spinner (void) {

    ros::spinOnce();

    // Draw Skeleton On Image Raw (only if Image and Skeletons already exists)
    if (skeleton_data_received && image_row_received) {draw_skeleton (skeleton, image_raw, "Skeleton Markers", 1);}

}
