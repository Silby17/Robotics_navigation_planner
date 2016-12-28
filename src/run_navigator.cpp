
#include "Navigator.h"


 int main(int argc, char **argv) {

    // Initiate new ROS node named "wander_bot"
     ros::init(argc, argv, "load_map");

    // Create new stopper object
     NavigatorPath navigatorPath;
     ros::NodeHandle nh;

     if(!navigatorPath.requestMap(nh))
         exit(-1);

    // Start the movement


    return 0;
};

