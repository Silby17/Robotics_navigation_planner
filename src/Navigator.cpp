/*
*	Yosef Silberhaft
*	210028924
*	Navigator.cpp
*/
#include "Navigator.h"

/**Default Constructor**/
NavigatorPath::NavigatorPath() {}


/*****************************************************************************
 * This Function will request the map
 * @param nh - node handler
 * @return - bool if successful or not
 ****************************************************************************/
bool NavigatorPath::requestMap(ros::NodeHandle &nh) {
	nav_msgs::GetMap::Request req;
	nav_msgs::GetMapResponse res;

	while(!ros::service::waitForService("static_map", ros::Duration(3.0))){
		ROS_INFO("Waiting for static_map to become available");
	}

	ROS_INFO("Requesting the map...");
	ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("static_map");
	if(mapClient.call(req, res)){
		readMap(res.map);
		return true;
	}
	else{
		ROS_ERROR("Failed to call map service");
		return false;
	}
}


/*****************************************************************************
 * This will set the size of the pixels by dividing
 * the robot size by the resolution
 * @param robot - size
 * @param res - resolution
 *****************************************************************************/
void NavigatorPath::setPixelSize(double robot, double res) {
    double size_map_units = (robot/res);
    //The robot size in pixels in map units dived by 2
    pixels_size = size_map_units / 2;
    //Rounds down and converts to integer
    perimeter = (int)pixels_size;
    ROS_INFO("The perimeter size around each object will be: %d", perimeter);
}


/****************************************************************************
 * This method will read the map from the occupancy grid
 * @param map - grid to be read from
 ****************************************************************************/
void NavigatorPath::readMap(const nav_msgs::OccupancyGrid& map) {
	ROS_INFO("Received a %d X %d map @ %.3f m/px\n",
             map.info.width, map.info.height, map.info.resolution);

    //Define Rows, Cols and resolution
	rows = map.info.height;
	cols = map.info.width;
	mapResolution = map.info.resolution;

	// Dynamically resize the grid
	grid.resize(rows);
    int_grid.resize(rows);
	for (int i = 0; i < rows; i++) {
		grid[i].resize(cols);
        int_grid[i].resize(cols);
	}
    int currCell = 0;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
            if(map.data[currCell] == 0){
                grid[i][j] = false;
                int_grid[i][j] = 0;
            }
            else{
                grid[i][j] = true;
                int_grid[i][j] = 1;
            }
            currCell++;
		}
	}
}

/******************************************************************************
 * This Method will make the out perimeter of each object 2's
 *****************************************************************************/
void NavigatorPath::createTempIntGrid() {
    for (int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++) {
            if(int_grid[i][j] == 1){
                if(int_grid[i][j-1] == 0){
                    int_grid[i][j] = 2;
                }
                if(int_grid[i - 1][j] == 0){
                    int_grid[i][j] = 2;
                }
                if(int_grid[i][j + 1] == 0){
                    int_grid[i][j] = 2;
                }
                if(int_grid[i + 1][j] == 0){
                    int_grid[i][j] = 2;
                }
            }
        }
    }
}


/******************************************************************************
 * This function will inflate all obstacles according to the size of the robot
 *****************************************************************************/
void NavigatorPath::inflateObstacles() {
    ROS_INFO("----------Starting to inflate obstacles----------");
    int size = perimeter;
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            if(int_grid[i][j] == 2){
                //Check the top bounds
                for(int k = 1; k <= size; k++){
                    if(i - k >= 0){
                        int_grid[i-k][j] = 1;
                    }
                }
                //Check left
                for(int k = 1; k <= size; k++){
                    if(j + k <= cols){
                        int_grid[i][j + k] = 1;
                    }
                }
                //Check top right
                for(int k = 1; k <= size; k++){
                    if((i - k >= 0) && (j + k <= cols)){
                        int_grid[i-k][j+k] = 1;
                    }
                }
                //Check Down
                for(int k = 1; k <= size; k++){
                    if(i + k <= rows){
                        int_grid[i+k][j] = 1;
                    }
                }
                //Check down-right
                for(int k = 1; k <= size; k++){
                    if((i + k <= rows) && (j+k <= cols)){
                        int_grid[i+1][j+1] = 1;
                    }
                }
                //Check Left
                for(int k = 1; k <= size; k++){
                    if(j - k >= 0){
                        int_grid[i][j-k] = 1;
                    }
                }
                //Check Down-left
                for(int k = 1; k <= size; k++){
                    if((i + k <= rows) && (j - k >= 0)){
                        int_grid[i + k][j - k ] = 0;
                    }
                }
                //Check Up-left
                for(int k = 1; k <= size; k++){
                    if((i - k >= 0) && (j - k >= 0)){
                        int_grid[i-k][j-k] = 1;
                    }
                }
            }
        }
    }
    //Runs over the grid and any numbers '2' will be converted into 1
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            if(int_grid[i][j] == 2){int_grid[i][j] = 1;}
        }
    }
}


/*****************************************************************************
 * This Function will print the grid to a text file
 ****************************************************************************/
void NavigatorPath::printGridToFile() {
    ofstream gridFile;
    gridFile.open("/home/viki/grids/original_grid.txt");

    ofstream int_gridFile;
    int_gridFile.open("/home/viki/grids/int_grid.txt");

    for (int i =0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            gridFile << (grid[i][j] ? 1 : 0);
            int_gridFile << (grid[i][j] ? 1 : 0);
        }
        gridFile << endl;
        int_gridFile << endl;
    }
    gridFile.close();
    int_gridFile.close();
    ROS_INFO("----------Done writing the grid and integer grid to file ---------\n");
}


/*****************************************************************************
 * This function will print the Inflated grid into a txt file
 *****************************************************************************/
void NavigatorPath::printInflatedGridToFile(){
    ofstream new_gridFile;
    new_gridFile.open("/home/viki/grids/inflated_grid.txt");

    for (int i =0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            new_gridFile << (int_grid[i][j]);
        }
        new_gridFile << endl;
    }
    new_gridFile.close();
    ROS_INFO("----------Done writing the inflated grid to file---------\n");
}