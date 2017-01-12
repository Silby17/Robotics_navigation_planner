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
    void inflateObstacles();
    void printGridToFile();
    void printNewGrid();
    void printInflatedGridToFile();
    void createTempIntGrid();
    void setPixelSize(double robot, double res);
	Location starting_location;
	Location goal_location;

private:
	double mapResolution;
	double robot_size;
	double pixels_size;
    int perimeter;
    int rows;
	int cols;
	vector<vector<bool> > grid;
	vector<vector<bool> > new_grid;
	vector<vector<int> > int_grid;
};


#endif