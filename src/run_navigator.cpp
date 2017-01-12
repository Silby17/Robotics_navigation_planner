/*
*	Yosef Silberhaft
*	210028924
*	run_navigator.cpp
*/
#include "Navigator.h"
using namespace std;

/*************************************************************************
 * This function will convert the points into locations from a string
 * that is separated by a comma
 * @param str - string to be converted
 * @return - x,y location
 *************************************************************************/
Location parseVector(string str){
    stringstream ss(str);
    vector<double> t_vector;

    double d;

    while(ss >> d){
        t_vector.push_back(d);
        if(ss.peek() == ','){
            ss.ignore();
        }
    }
    Location t_location;
    t_location.first = t_vector.at(0);
    t_location.second = t_vector.at(1);
    return t_location;
};

/**Main Function**/
 int main(int argc, char **argv) {
     //Deceleration of variables
     string tempStart;
     string tempGoal;
    double robot_size;
    double resolution;

    // Initiate new ROS node named "wander_bot"
     ros::init(argc, argv, "navigation_planner");

    // Create new stopper object
     NavigatorPath navigatorPath;
     ros::NodeHandle nh;


    //Gets Parameters from the launch file
    nh.getParam("starting_location", tempStart);
    nh.getParam("goal_location", tempGoal);
    nh.getParam("robot_size", robot_size);
    nh.getParam("map_resolution", resolution);

    //Sets the size of the Pixels
    navigatorPath.setPixelSize(robot_size, resolution);

     //Converts the start and goal locations into locations
     navigatorPath.starting_location = parseVector(tempStart);
     navigatorPath.goal_location = parseVector(tempGoal);

     if(!navigatorPath.requestMap(nh)){
         exit(-1);
     }
    navigatorPath.createTempIntGrid();

    navigatorPath.printGridToFile();

    navigatorPath.inflateObstacles();
    navigatorPath.printInflatedGridToFile();

    //navigatorPath.printNewGrid();

    return 0;
}
