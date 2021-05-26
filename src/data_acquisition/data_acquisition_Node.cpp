#include "data_acquisition/data_acquisition.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "data_acquisition_Node");

    data_acquisition da;

    while (ros::ok()) {

        da.spinner();

    }

return 0;

}
