/*
*	Yosef Silberhaft
*	210028924
*	Navigator.h
*/

#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_


#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <string>

using namespace std;

//Define locations as x,y
typedef pair<double, double> Location;

class NavigatorPath{
public:
	NavigatorPath();
    bool requestMap(ros:: NodeHandle &nh);
    void readMap(const nav_msgs::OccupancyGrid& map);
    void printGridToFile();



private:
	double robot_size;
	// Grid map definition
	int rows;
	int cols;
	double mapResolution;
	vector<vector<bool> > grid;
};


#endif