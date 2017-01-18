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
    int_grid[115][97] = 7;


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

    PrintIntegerVector(int_grid, rows, cols, "inflated_grid.txt");
}



void NavigatorPath::createRobotSizeGrid() {
    //Gets the size of the new reduced matrix
    double new_rows = idiv_ceil((double)rows, size_map_units);
    double new_cols = idiv_ceil((double)cols, size_map_units);
    //Defines the size of the new grid
    SetNewGridSize((int)new_rows, (int)new_cols);


    CreateReducedGrid((int)new_rows, (int)new_cols);
}



void NavigatorPath::CreateReducedGrid(int k_rows, int k_cols) {
    ROS_INFO("Starting print sum simple function");
    bool n_array[k_rows][k_cols];
    int x = 0;
    int y = 0;

    for(int i = 0; i < k_rows; i++){
        for(int j = 0; j < k_cols; j++){
            //ROS_INFO("Local i = %d, Local j = %d", i, j);
            n_array[i][j] = CheckSubMatrix(i * 7, (j *7) , ((i + 1)*7), ((j+1)*7));
        }
    }

    for(int i = 0; i < k_rows; i++){
        for(int j =0; j < k_cols; j++){
            robot_size_grid[i][j] = n_array[i][j];
        }
    }
    PrintIntegerVector(robot_size_grid, n_rows, n_cols, "shrinked_grid.txt");

/*
    int ar[k_rows * k_cols];
    for(int i = 0; i< k_rows; i++){
        for(int j = 0; j < k_cols; j++){
            ar[i * k_cols + j] = n_array[i][j];
        }
    }
    cout << std::end(ar) - std::begin(ar);
    ROS_INFO("Done converting into 1d array :");
*/

}


bool NavigatorPath::CheckSubMatrix(int x_start, int y_start, int x_end, int y_end){
    //Makes sure that the values are within the bounds of the matrix
    if(x_end >= rows){ x_end = rows - 1;}
    if(y_end >= cols){ y_end = cols - 1;}


    for (int i = x_start; i < x_end; i++){
        for(int j = y_start; j < y_end; j++){
            if(int_grid[i][j] == 1){
                return true;
            }
        }
    }
    return false;
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
    ROS_INFO("New Col size is: %d, new Rows size: %d", n_rows, n_cols);
}


double NavigatorPath::idiv_ceil(double numerator, double denominator){
    double result = numerator / denominator;
    //Adding 0.5 to the number as the compiler will always truncate
    //This is done only when x > 0 and here we can assume this is true
    result += 0.5;
    return result;
}



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
    ROS_INFO("----------------------Done writing: %s to file--------------=", path.c_str());
}