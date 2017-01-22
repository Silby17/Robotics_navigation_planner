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
    size_map_units = (robot/res);
    ROS_INFO("The Size of the robot in map units is: %.2f", size_map_units);

    //The robot size in pixels in map units dived by 2
    pixels_size = size_map_units / 2;
    //Rounds down and converts to integer
    perimeter = (int)pixels_size;
    ROS_INFO("The perimeter size around each object will be: %d", perimeter);
}

Location NavigatorPath::CalculateRealPosition(double x, double y, double resolution) {
    double r_x = x / resolution;
    double r_y = y / resolution;
    Location location = make_pair(r_x, r_y);
    return location;
}


/****************************************************************************
 * This method will read the map from the occupancy grid
 * @param map - grid to be read from
 ****************************************************************************/
void NavigatorPath::readMap(const nav_msgs::OccupancyGrid& map) {
	ROS_INFO("Received a %d X %d map @ %.3f m/px\n",
             map.info.height, map.info.width, map.info.resolution);

    //Define Rows, Cols and resolution
	rows = map.info.height;
	cols = map.info.width;
	mapResolution = map.info.resolution;

    //Calculates the center of the board
    int r_start_x = (int)idiv_ceil((double)rows, (double)2);
    int r_start_y = (int)idiv_ceil((double)cols, (double)2);
    center_coordinated = make_pair(r_start_x, r_start_y);


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
                grid[i][j] = 0;
                int_grid[i][j] = 0;
            }
            else{
                grid[i][j] = 1;
                int_grid[i][j] = 1;
            }
            currCell++;
		}
	}

    this->center_coordinated = make_pair(r_start_x, r_start_y);
    this->int_grid[r_start_x][r_start_y] = 7;

    //Calculate the Start and Goal Location in Original Grid
    Location s = CalculateRealPosition(given_start.first, given_start.second, mapResolution);
    Location end = CalculateRealPosition(given_goal.first, given_goal.second, mapResolution);
    this->big_goal_location = make_pair((r_start_x + end.first), (r_start_y + end.second));
    this->big_start_location = make_pair((r_start_x + s.first), (r_start_y + s.second));

    ROS_INFO("Center of the grid is: (%d,%d)", r_start_x, r_start_y);
    int_grid[big_goal_location.first][big_goal_location.second] = 4;
    PrintIntegerVector(grid, rows, cols, "original_grid.txt");
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
    PrintIntegerVector(int_grid, rows, cols, "temp_int_grid.txt");
}


/******************************************************************************
 * This function will inflate all obstacles according to the size of the robot
 *****************************************************************************/
void NavigatorPath::inflateObstacles() {
    ROS_INFO("----------------------Starting to inflate obstacles----------------------");
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
    PrintIntegerVector(int_grid, rows, cols, "inflated_grid.txt");
}


/*****************************************************************************
 * This function will create a new grid adjucted to the size of the robot
 *****************************************************************************/
void NavigatorPath::createRobotSizeGrid() {
    //Gets the size of the new reduced matrix
    double new_rows = idiv_ceil((double)rows, size_map_units);
    double new_cols = idiv_ceil((double)cols, size_map_units);
    //Defines the size of the new grid
    SetNewGridSize((int)new_rows, (int)new_cols);
    CreateReducedGrid((int)new_rows, (int)new_cols);
}


/*****************************************************************************
 * This function will create a shrinked grid, shrinked to the calculated
 * size.
 * @param k_rows - the adjusted amount of rows
 * @param k_cols - the adjusted amount of cols
 *****************************************************************************/
void NavigatorPath::CreateReducedGrid(int k_rows, int k_cols) {
    for(int i = 0; i < k_rows; i++){
        for(int j = 0; j < k_cols; j++){
            int result = CheckSubMatrix(i * 7, (j *7) , ((i + 1)*7), ((j+1)*7));
            if(result == 1){
                robot_size_grid[i][j] = 1;
            }
            else if(result == 7){
                this->s_start_location = make_pair(i, j);
                robot_size_grid[i][j] = 0;
            }
            else if(result == 4){
                this->s_goal_location= make_pair(i, j);
                robot_size_grid[i][j] = 0;
            }
            else if(result == 0){
                robot_size_grid[i][j] = 0;
            }
        }
    }
    PrintIntegerVector(robot_size_grid, n_rows, n_cols, "shrinked_grid.txt");
    CreateAlgoGrid(robot_size_grid, k_rows, k_cols);
}


/******************************************************************************
 * This function will create a grid for the running of the algorithm
 * by replacing 1's with 9's and 0's with 1's
 * @param grid - grid to be converted
 * @param rows - size of rows
 * @param cols - size of cols
 *****************************************************************************/
void NavigatorPath::CreateAlgoGrid(vector<vector<int> > grid, int rows, int cols) {
    algo_grid.resize(rows);
    for(int i = 0; i < rows; i++){
        algo_grid[i].resize(cols);
        for(int j = 0; j < cols; j++){
            if(grid[i][j] == 1){
                algo_grid[i][j] = 9;
            }
            else{
                algo_grid[i][j] = grid[i][j];
            }
        }
    }
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++) {
            if(grid[i][j] == 0){
                algo_grid[i][j] = 1;
            }
        }
    }
    //PrintIntegerVector(algo_grid, rows, cols, "done.txt");
    CreateVectorForAlgorithm();
}


/******************************************************************************
 * This Function will create a one dimensional vector for the running
 * of the A* Algorithm
 *****************************************************************************/
void NavigatorPath::CreateVectorForAlgorithm(){
    //Resize the single vector
    this->one_dim_grid.resize(n_rows * n_cols);
    for(int i = 0; i< n_rows; i++) {
        for (int j = 0; j < n_cols; j++) {
            int temp = algo_grid[i][j];
            this->one_dim_grid[i * n_cols + j] = temp;
        }
    }
}


/*****************************************************************************
 * This function will check the sub-matrix values of the original sized grid
 * @param x_start - starting x coordinate
 * @param y_start - starting y coordinate
 * @param x_end - finishing x coordinate
 * @param y_end - finishing y coordinate
 * @return - the value of what was found in the sub-matrix defined
 *****************************************************************************/
int NavigatorPath::CheckSubMatrix(int x_start, int y_start, int x_end, int y_end){
    int flag = 0;
    //Makes sure that the values are within the bounds of the matrix
    if(x_end >= rows){ x_end = rows - 1;}
    if(y_end >= cols){ y_end = cols - 1;}


    for (int i = x_start; i < x_end; i++){
        for(int j = y_start; j < y_end; j++){
            if(int_grid[i][j] == 1){
                flag = 1;
            }
            if(int_grid[i][j] == 7){
                flag = 7;
            }
            if(int_grid[i][j] == 4){
                flag = 4;
            }
        }
    }
    return flag;
}


/*****************************************************************************
 * Sets the size of the new Grid adjusted to the robot size
 * @param rows - new row size
 * @param cols - new cols size
 *****************************************************************************/
void NavigatorPath::SetNewGridSize(int rows, int cols) {
    n_rows = rows;
    n_cols = cols;

    robot_size_grid.resize(n_rows);
    for (int i = 0; i < n_cols; i++){
        robot_size_grid[i].resize(n_cols);
    }
    //ROS_INFO("New Col size is: %d, new Rows size: %d", n_rows, n_cols);
}


/*****************************************************************************
 * This function will divide 2 double and return the ceiling value
 * @param numerator - the numerator
 * @param denominator - the denominator
 * @return - the result
 ****************************************************************************/
double NavigatorPath::idiv_ceil(double numerator, double denominator){
    double result = numerator / denominator;
    //Adding 0.5 to the number as the compiler will always truncate
    //This is done only when x > 0 and here we can assume this is true
    result += 0.5;
    return result;
}


/***************************************************************************
 * This function will output a given vector to file
 * @param vector - to be outputted
 * @param r - rows
 * @param c - cols
 * @param path - file name
 ****************************************************************************/
void NavigatorPath::PrintIntegerVector(vector<vector<int> > vector, int r, int c, string path){
    ofstream file;
    string file_path = "/home/viki/grids/";
    file_path += path;
    file.open(file_path.c_str());

    for(int i = 0; i < r; i++){
        for(int j = 0; j < c; j++){
            file << vector[i][j];
        }
        file << endl;
    }
    file.close();
    ROS_INFO("----------------------Done writing: %s to file---------------", path.c_str());
}