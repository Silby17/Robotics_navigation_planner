/*
*	Yosef Silberhaft
*	210028924
*	run_navigator.cpp
*/
#include "Navigator.h"
#include "RunAlgorithm.cpp"
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

void WritePathToFile(vector<pair<int, int> > locations, string path) {
    //Sets the path of the output file
    string file_path = "/home/viki/grids/";
    file_path += path;
    ofstream file;
    file.open(file_path.c_str());

    typedef std::vector<std::pair<int, int> > vector_type;
    for (vector_type::const_iterator pos = locations.begin();
         pos != locations.end(); ++pos)
    {
        file << "(" << pos->first << "," << pos->second << ")" << endl;
    }
    file.close();

    ROS_INFO("Writing to File completed");
}


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

    //Creates A temp grid that's obstacles are surrounded by 2's
    navigatorPath.createTempIntGrid();
    //Inflates the obstacles in the map
    navigatorPath.inflateObstacles();

    //Reduces the matrix by the size of the robot
    navigatorPath.createRobotSizeGrid();
    int arr[] = {
            1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
            1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,1,
            1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,
            1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,
            1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,
            1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,
            1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,
            1,9,9,9,9,9,9,9,9,1,1,1,9,9,9,9,9,9,9,1,
            1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,
            1,9,1,9,9,9,9,9,9,9,1,1,9,9,9,9,9,9,9,1,
            1,9,1,1,1,1,9,1,1,9,1,1,1,1,1,1,1,1,1,1,
            1,9,9,9,9,9,1,9,1,9,1,9,9,9,9,9,1,1,1,1,
            1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,
            1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,
            1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,
            1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,
            1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,
            1,1,9,9,9,9,9,9,9,1,1,1,9,9,9,1,9,9,9,9,
            1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,
            1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    };

    vector<int> vec (arr, arr + sizeof(arr) / sizeof(arr[0]));
    AStarAlgorithm algo;
    AStarAlgorithm::MapSearchNode startingNode;
    startingNode.x = 3;
    startingNode.y = 7;

    AStarAlgorithm::MapSearchNode goalNode;
    goalNode.x =17;
    goalNode.y = 16;

    algo.Run(vec, 20, 20, startingNode, goalNode);

    //Gets The navigation path from the ASTar Algorithm run
    vector<pair<int, int> > path_cells = algo.GetLocation();
    //Writes the locations into txt file
    WritePathToFile(path_cells, "navigation_plan.txt");

    return 0;
}

