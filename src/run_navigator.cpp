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
    Location location = make_pair(t_vector.at(0), t_vector.at(1));
    return location;
};


/***************************************************************************
 * This Function will output the navigation path to a text file
 * @param locations
 * @param path
 ***************************************************************************/
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
    ROS_INFO("----------------------Writing Navigation Path to file----------------------");
}



vector<pair<int, int> > createPathCoordinates(vector<pair<int, int> > path_cells,
                                              int size_map_units,
                                              int k_row, int k_cols){
    vector<pair<int, int> > real_path;
    int size = path_cells.size();

    typedef std::vector<std::pair<int, int> > vector_type;
    for (vector_type::const_iterator pos = path_cells.begin();
         pos != path_cells.end(); ++pos)
    {
        real_path.push_back(make_pair((pos->first - (k_row/2)), (pos->second - (k_cols/2))));
    }
    return real_path;
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
    navigatorPath.given_start = parseVector(tempStart);
    navigatorPath.given_goal = parseVector(tempGoal);

    if(!navigatorPath.requestMap(nh)){
        exit(-1);
    }
    //Calculates the center of the grid
    //navigatorPath.center_coordinated = make_pair((navigatorPath.rows / 2), (navigatorPath.cols / 2));
    //Creates A temp grid that's obstacles are surrounded by 2's
    navigatorPath.createTempIntGrid();
    //Inflates the obstacles in the map
    navigatorPath.inflateObstacles();

    //Reduces the matrix by the size of the robot
    navigatorPath.createRobotSizeGrid();

    ROS_INFO("Starting Location: (%.0f,%.0f)", navigatorPath.s_start_location.first, navigatorPath.s_start_location.second);
    ROS_INFO("Goal Location: (%.0f,%.0f)", navigatorPath.s_goal_location.first, navigatorPath.s_goal_location.second);

    AStarAlgorithm algo;
    //Creates a node for the starting location
    AStarAlgorithm::MapSearchNode startingNode;
    startingNode.x = navigatorPath.s_start_location.first;
    startingNode.y = navigatorPath.s_start_location.second;

    //Creates a node for the goal location
    AStarAlgorithm::MapSearchNode goalNode;
    goalNode.x =navigatorPath.s_goal_location.first;
    goalNode.y = navigatorPath.s_goal_location.second;


    algo.Run(navigatorPath.one_dim_grid, navigatorPath.rows, navigatorPath.cols, startingNode, goalNode);

    //Gets The navigation path from the ASTar Algorithm run
    vector<pair<int, int> > path_cells = algo.GetLocation();
    //Writes the locations into txt file
    WritePathToFile(path_cells, "navigation_plan.txt");

    navigatorPath.path_coordinates = createPathCoordinates(path_cells, navigatorPath.size_map_units, navigatorPath.n_rows, navigatorPath.n_cols);

    WritePathToFile(navigatorPath.path_coordinates, "map_path.txt");
    return 0;
}



