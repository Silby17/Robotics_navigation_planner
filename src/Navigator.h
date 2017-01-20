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
typedef pair<int, int> Location;

struct Node {
    int x, y;
    struct Node *up;
    struct Node *right;
    struct Node *down;
    struct Node *left;
};

class NavigatorPath{
public:
	NavigatorPath();
    bool requestMap(ros:: NodeHandle &nh);
    void readMap(const nav_msgs::OccupancyGrid& map);
    void PrintIntegerVector(vector<vector<int> > vector, int r, int c, string path);
    void inflateObstacles();
    void createTempIntGrid();
    void setPixelSize(double robot, double res);
    void createRobotSizeGrid();
    void SetNewGridSize(int rows, int cols);
    double idiv_ceil(double numerator, double denominator);
    void CreateReducedGrid(int k_rows, int k_cols);
    bool CheckSubMatrix(int x_start, int y_start, int x_end, int y_end);
	Location starting_location;
	Location goal_location;

private:
	double mapResolution;
	double robot_size;
	double pixels_size;
	double size_map_units;
    int perimeter;
    int rows;
	int cols;
    int n_rows;
    int n_cols;
	vector<vector<int> > grid;
	vector<vector<int> > int_grid;
    vector<vector<int> > robot_size_grid;
	vector<int> single_vector;

};
#endif