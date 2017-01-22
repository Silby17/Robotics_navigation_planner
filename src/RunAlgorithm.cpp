#include "stlastar.h"
using namespace std;
int GLOBAL_WIDTH;
int GlOBAL_HEIGHT;
vector<int> map_vector;
vector<pair<int, int> > Location_vector;

class AStarAlgorithm {
public:
    //Nested Class
    class MapSearchNode{
    public:
        int x;
        int y;
        MapSearchNode() {x = y = 0;}
        void SetSizes(int rows, int cols);
        MapSearchNode(int px, int py) { x=px; y=py; }
        float GoalDistanceEstimate( MapSearchNode &nodeGoal );
        bool IsGoal( MapSearchNode &nodeGoal );
        bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
        float GetCost( MapSearchNode &successor );
        bool IsSameState( MapSearchNode &rhs );
        void PrintNodeInfo();
        int GetFromMap(int x, int y);
        int GetX();
        int GetY();
    };

    MapSearchNode mapSearchNode;
    void Run(vector<int> map_vec, int height, int width, AStarAlgorithm::MapSearchNode start, AStarAlgorithm::MapSearchNode Goal);
    vector<pair<int, int> > GetLocation();
};

vector<pair<int, int> > AStarAlgorithm::GetLocation() {
    return Location_vector;
}

int AStarAlgorithm::MapSearchNode::GetX() {
    return this->x;
}


int AStarAlgorithm::MapSearchNode::GetY() {
    return this->y;
}

void AStarAlgorithm::MapSearchNode::SetSizes(int rows, int cols) {
    GLOBAL_WIDTH = cols;
    GlOBAL_HEIGHT = rows;
}



int AStarAlgorithm::MapSearchNode::GetFromMap(int x, int y){
    if(x < 0 || x >= GLOBAL_WIDTH || y < 0 || y >= GlOBAL_HEIGHT) {
        return 9;
    }
    return map_vector[(y * GLOBAL_WIDTH)+x];
}


bool AStarAlgorithm::MapSearchNode::IsSameState( MapSearchNode &rhs ) {
    // same state in a maze search is simply when (x,y) are the same
    return (x == rhs.x) &&
           (y == rhs.y);
}

void AStarAlgorithm::MapSearchNode::PrintNodeInfo() {
    char str[100];
    //sprintf( str, "Node position : (%d,%d)\n", x,y );
    Location_vector.push_back(make_pair(x, y));
    //cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal.
float AStarAlgorithm::MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal ) {
    return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);
}


bool AStarAlgorithm::MapSearchNode::IsGoal( MapSearchNode &nodeGoal ) {
    return (x == nodeGoal.x) && (y == nodeGoal.y);
}


// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool AStarAlgorithm::MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node ) {
    int parent_x = -1;
    int parent_y = -1;

    if(parent_node) {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }
    MapSearchNode NewNode;
    NewNode.SetSizes(GlOBAL_HEIGHT, GLOBAL_WIDTH);

    // push each possible move except allowing the search to go backwards
    if((GetFromMap(x - 1, y) < 9) && !((parent_x == x-1) && (parent_y == y))) {
        NewNode = MapSearchNode(x - 1, y);
        NewNode.SetSizes(GlOBAL_HEIGHT, GLOBAL_WIDTH);
        astarsearch->AddSuccessor(NewNode);
    }

    if((GetFromMap(x, y - 1) < 9) && !((parent_x == x) && (parent_y == y-1))) {
        NewNode = MapSearchNode( x, y-1 );
        NewNode.SetSizes(GlOBAL_HEIGHT, GLOBAL_WIDTH);
        astarsearch->AddSuccessor( NewNode );
    }
    if((GetFromMap( x+1, y ) < 9) && !((parent_x == x+1) && (parent_y == y))) {
        NewNode = MapSearchNode( x+1, y );
        NewNode.SetSizes(GlOBAL_HEIGHT, GLOBAL_WIDTH);
        astarsearch->AddSuccessor( NewNode );
    }
    if((GetFromMap( x, y+1 ) < 9) && !((parent_x == x) && (parent_y == y+1))) {
        NewNode = MapSearchNode( x, y+1 );
        NewNode.SetSizes(GlOBAL_HEIGHT, GLOBAL_WIDTH);
        astarsearch->AddSuccessor( NewNode );
    }
    return true;
}


float AStarAlgorithm::MapSearchNode::GetCost( MapSearchNode &successor ) {
    return (float) GetFromMap( x, y );
}


void AStarAlgorithm::Run(vector<int> map_vec, int height, int width, AStarAlgorithm::MapSearchNode start_node, AStarAlgorithm::MapSearchNode goal_node){
    map_vector = map_vec;
    GLOBAL_WIDTH = width;
    GlOBAL_HEIGHT = height;

    cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";
    AStarSearch<AStarAlgorithm::MapSearchNode> astarsearch;
    unsigned int SearchCount = 0;

    const unsigned int NumSearches = 1;
    AStarAlgorithm::MapSearchNode mapSearch;
    mapSearch.SetSizes(GlOBAL_HEIGHT, GLOBAL_WIDTH);
    while(SearchCount < NumSearches)
    {
        astarsearch.SetStartAndGoalStates( start_node, goal_node);

        unsigned int SearchState;
        unsigned int SearchSteps = 0;
        do {
            SearchState = astarsearch.SearchStep();
            SearchSteps++;
        }
        while( SearchState == AStarSearch<AStarAlgorithm::MapSearchNode>::SEARCH_STATE_SEARCHING );

        if( SearchState == AStarSearch<AStarAlgorithm::MapSearchNode>::SEARCH_STATE_SUCCEEDED ) {
            cout << "Search found goal state" << endl;

            AStarAlgorithm::MapSearchNode *node = astarsearch.GetSolutionStart();
            node->SetSizes(GlOBAL_HEIGHT, GLOBAL_WIDTH);
            int steps = 0;

            node->PrintNodeInfo();
            for( ;; ) {
                node = astarsearch.GetSolutionNext();
                if(!node) {
                    break;
                }
                node->PrintNodeInfo();
                steps ++;
            };
            cout << "Solution steps " << steps << endl;

            // Once you're done with the solution you can free the nodes up
            astarsearch.FreeSolutionNodes();
        }
        else if( SearchState == AStarSearch<AStarAlgorithm::MapSearchNode>::SEARCH_STATE_FAILED ) {
            ROS_INFO("Search terminated. Did not find goal state");
        }
        // Display the number of loops the search went through
        cout << "SearchSteps : " << SearchSteps << "\n";
        SearchCount ++;
        astarsearch.EnsureMemoryFreed();
    }
}
