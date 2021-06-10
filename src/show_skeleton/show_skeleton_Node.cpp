#include "show_skeleton/show_skeleton.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "show_skeleton_Node");

    show_skeleton ss;

    while (ros::ok()) {

        ss.spinner();

    }

return 0;

}
