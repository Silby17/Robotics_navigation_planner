/*
*	Yosef Silberhaft
*	210028924
*	NavigatorBot.cpp
*/
#include "Navigator.h"

/**Constructor**/
NavigatorPath::NavigatorPath() {}


/*******************************************************
 * This Function will request the map
 * @param nh - node handler
 * @return - bool if successful or not
 *******************************************************/
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


void NavigatorPath::readMap(const nav_msgs::OccupancyGrid& map) {
	ROS_INFO("Received a %d X %d map @ %.3f m/px\n",
             map.info.width, map.info.height, map.info.resolution);

	rows = map.info.height;
	cols = map.info.width;
	mapResolution = map.info.resolution;

	// Dynamically resize the grid
	grid.resize(rows);
	for (int i = 0; i < rows; i++) {
		grid[i].resize(cols);
	}
    int currCell = 0;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
            if(map.data[currCell] == 0){
                grid[i][j] = false;
            }
            else{
                grid[i][j] = true;
            }
            currCell++;
		}
	}
}

/*******************************************************
 * This Function will print the grid to a text file
 ******************************************************/
void NavigatorPath::printGridToFile() {
    ofstream gridFile;
    gridFile.open("/home/viki/my_grid.txt");
    for (int i =0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            gridFile << (grid[i][j] ? 1 : 0);
        }
        gridFile << endl;
    }
    gridFile.close();
    ROS_INFO("----------Done creating grid file---------\n");
}
