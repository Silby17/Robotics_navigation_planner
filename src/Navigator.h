/*
*	Yosef Silberhaft
*	210028924
*	Navigator.h
*/

#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>
#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <iomanip>

using namespace std;

//Define locations as x,y
typedef pair<double, double> Location;

class NavigatorPath{
public:
	NavigatorPath();
    bool requestMap(ros:: NodeHandle &nh);
    void readMap(const nav_msgs::OccupancyGrid& map);
    void printGridToFile();
	Location starting_location;
	Location goal_location;

private:
	double mapResolution;
	double robot_size;
	int rows;
	int cols;
	vector<vector<bool>> grid;
};


#endif