#include <cmath>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <chrono>

#include "/home/naveed/Documents/matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;


class Node{

    public:

        float x,y,theta,cost;
        Node* parent;
        std::vector<Node*> children;

        //Constructor
        Node(float x, float y, float theta, Node* parent = nullptr, float cost = numeric_limits<float>::max())\
                :x(x), y(y), theta(theta), parent(parent), cost(cost) {}

        //Destructor
        ~Node(){}

};

//calculate euclidian distance. 
template <typename T, typename Y>
float calculateDistanceNode(const T& a, const Y& b){

    float dist;
    dist = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + 
                                   0.001* (a.theta - b.theta) * (a.theta - b.theta));

    return dist; 
}

template <typename T, typename Y>
float calculateDistanceXY(const T& a, const Y& b){

    float dist;
    dist = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) );

    return dist; 
}

// generate random node
Node* getRandomNode(float Xmax, float Ymax, float thetamax){

    float x = static_cast<float> (rand()) * Xmax/RAND_MAX;
    float y = static_cast<float> (rand()) * Ymax/RAND_MAX;
    float theta = static_cast<float> (rand()) * thetamax/RAND_MAX;

    return new Node(x, y, theta);

}

//find nearest Node
Node* findNearestNode(vector<Node*>& tree, const Node& randomNode){

    float minDist = numeric_limits<float>::max();
    Node* nearestNode = nullptr;

    for (auto node : tree) {
        float dist = calculateDistanceNode(*node, randomNode);

        if (dist < minDist && node->x <= randomNode.x) { //check if it is behind the current node. 
            minDist = dist;
            nearestNode = node;
        }
    }

    return nearestNode;


}

struct RectangularObstacle{
    float x_min, x_max, y_min, y_max;

    RectangularObstacle(float x_min, float x_max, float y_min, float y_max):
                        x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max) {}
};

struct CircularObstacle{
    float x, y;
    float radius;

    CircularObstacle(float x, float y, float radius) : x(x), y(y), radius(radius) {}
};

// check if the node collides with obstacle
template <typename T>
bool isFree(const Node& node, vector<T>& obstacles){
    
    
    return true;
    }

template <>
bool isFree<CircularObstacle>(const Node& node, vector<CircularObstacle>& obstacles){

    float dist;

    // check if node is inside obstacle
    for(auto obs : obstacles){
        
        dist = calculateDistanceXY(obs, node);

        if (dist <= obs.radius){
            return false;
        }

    }

    return true;
}
template <>
bool isFree<RectangularObstacle>(const Node& node, vector<RectangularObstacle>& obstacles){

    //check if node is inside
    
    for(auto obs: obstacles){
       
        if((node.x >= obs.x_min) && (node.x <= obs.x_max) && (node.y >= obs.y_min) && (node.y <= obs.y_max)){
            //cout << "node" << node.x << "," << node.y << endl;
            //cout << "obs" << obs.x_min << "," << obs.x_max << ","
            //    << obs.y_min << "," << obs.y_max << endl;
            return false;
        }

    }

    return true;
}

struct vec
    {
        float x,y;

        vec(float x, float y): x(x), y(y) {}
    };


template <typename T>
bool isPathFree(const Node& node1, const Node& node2, vector<T>& obstacles){ return true;}

template <>
bool isPathFree<CircularObstacle>(const Node& node1, const Node& node2, vector<CircularObstacle>& obstacles){

    // check if the path connection passes through circular obstacle

    float dist, t; 

    for(auto obs : obstacles){
        
        //find angle between the vector connecting node 1 to center of obs and the vector from node 1 and node 2.
        vec v1(node2.x - node1.x, node2.y - node1.y);
        vec v2(obs.x - node1.x, obs.y - node1.y);

        //t is the projection of v2 on v1
        t = abs(v1.x * v2.x + v1.y * v2.y)/sqrt(v1.x * v1.x + v1.y * v1.y);

        vec pt(node1.x + (t * v1.x/sqrt(v1.x * v1.x + v1.y * v1.y)), 
                    node1.y + (t * v1.y/sqrt(v1.x * v1.x + v1.y * v1.y)));

        dist = calculateDistanceXY(obs, pt);

        if (dist <= obs.radius){
            return false;
        }

    }

    return true;
}

template <>
bool isPathFree<RectangularObstacle>(const Node& node1, const Node& node2, 
                                                vector<RectangularObstacle>& obstacles){
    
    for (auto obs : obstacles){

		// check if both the nodes are in one side of the rectangle

		if (node1.x < obs.x_min && node2.x < obs.x_min) return true;
		else if (node1.x > obs.x_max && node2.x > obs.x_max) return true;
		else if (node1.y < obs.y_min && node2.y < obs.y_min) return true;
		else if (node1.y > obs.y_max && node2.y > obs.y_max) return true;

		// if it not on one side, check if it intersects.
		if ( (node2.x - node1.x) != 0) {
			double m = (node2.y - node1.y)/(node2.x - node1.x);//slope of the line.

			double y_intercept1 = (m * (obs.x_min - node1.x) + node1.y);
			double y_intercept2 = (m * (obs.x_max - node1.x) + node1.y);
			double x_intercept1 = ((obs.y_min - node1.y)/m + node1.x);
			double x_intercept2 = ((obs.y_max - node1.y)/m + node1.x);

			if ( (y_intercept1 <= obs.y_max) && (y_intercept1 >= obs.y_min) ) return false;
			else if ( (y_intercept2 <= obs.y_max) && (y_intercept2 >= obs.y_min) ) return false;
			else if ( (x_intercept1 <= obs.x_max) && (x_intercept1 >= obs.x_min) ) return false;
			else if ( (x_intercept2 <= obs.x_max) && (x_intercept2 >= obs.x_min) ) return false;
		}
		else return false; //if the slope in 0 and both points are not in one side of the rectangle, 
							// then it crosses it.
		

    }	
    
    return true;
}

Node* KBM (Node* state, float* control) {

    double dt = 0.1; //seconds
    double L = 3.6;//meters
    float new_x = state->x + control[0]*cos(state->theta)*dt;
    float new_y = state->y + control[0]*sin(state->theta)*dt;
    float new_theta = state->theta + control[0]*tan(control[1])*dt/L; 

    Node* new_state = new Node(new_x, new_y, new_theta, state);

    return new_state;
}

Node* steer(Node* nearest, Node* randomNode, float stepSize){

    float d = calculateDistanceNode(*nearest, *randomNode);

    if ( d <= stepSize){
        float newCost = nearest->cost + d;
        
        return new Node(randomNode->x, randomNode->y, randomNode->theta, nearest, newCost);
    }
    else {
        float theta = atan2(randomNode->y - nearest->y, randomNode->x - nearest->x);
        float newX = nearest->x + stepSize * sin(theta);
        float newY = nearest->y + stepSize * cos(theta);
        float newtheta = randomNode->theta;
        Node* newNode = new Node(newX, newY, newtheta, nearest);

        d = calculateDistanceNode(*nearest, *newNode);
        float newCost = nearest->cost + d;
        newNode->cost = newCost;

        return newNode;

    }   
}

Node* steer_KBM(Node* nearest, Node* randomNode, float stepSize){

    float vel_max = 5; // meters/second
    float steer_max = static_cast <float> (M_PI)/3.0;

    // calculate velocity using Proportional controller
    float control[2];
    control[0] = 5.0*(randomNode->x - nearest->x);

    if (control[0] > vel_max) control[0] = vel_max;
    else if (control[0] < 0) control[0] = 0;
    
    //calculate steering angly using proportional controller. 

    double beta = atan2(randomNode->y - nearest->y, randomNode->x - nearest->x);
    double heading_error = beta - nearest->theta;
    control[1] = 3.0*heading_error;

    if (control[1] > steer_max) control[1] = steer_max;
    else if (control[1] < -steer_max) control[1] = -steer_max;

    Node* newNode = KBM(nearest, control);
    float d = calculateDistanceNode(*nearest, *newNode);

    newNode->cost = nearest->cost + d;

    return newNode;

}

void updateCosts(Node* node){

    for (Node* child : node->children) {
        
        float newCost = node->cost + calculateDistanceNode(*node, *child);

        if ( newCost < child->cost){
            child->cost = newCost;
            updateCosts(child);
        }

    }
}

template <typename T>
void rewire(vector<Node*>& tree, Node* newNode, float searchSize, vector<T>& obstacles){
    
    float d, newCost;

    for(Node* node : tree){

        if (node == newNode) continue;

        d = calculateDistanceNode(*node, *newNode);
        
        // check if node is within the search radius
        if (d <= searchSize && isPathFree(*newNode, *node, obstacles)){
            
            newCost = newNode->cost + d; //potential cost with newNode as parent

            if (node->cost > newCost && node->x >= newNode-> x){
                
                node->parent->children.erase(std::remove(node->parent->children.begin(), 
                            node->parent->children.end(), node), node->parent->children.end());
                node->parent = newNode; //update parent and cost
                node->cost = newCost; 
                newNode->children.push_back(node);

                //update cost of children recursively. 
                updateCosts(node); 

            }
        }
    }

}

// build tree
template <typename T>
std::pair<vector<Node*>, Node*> RRTstar(float startx, float starty, float starttheta, float goalx, 
    float goaly, float goaltheta, float Xmax, float Ymax, float thetamax, 
    int maxIterations, float stepSize, vector<T>& obstacles, float searchSize){
    
    Node* startNode = new Node(startx, starty, starttheta, nullptr, 0);
    Node* goalNode = new Node(goalx, goaly, goaltheta);

    bool goalFound = false;

    vector<Node*> tree;
    tree.push_back(startNode);

    for (int i=0; i < maxIterations; ++i) {

        
        //generate random node
        Node* randomNode = getRandomNode(Xmax, Ymax, thetamax);
        
        //find the node closest to the random node
        Node* nearest = findNearestNode(tree, *randomNode);

        //generate a new node in the same direction as the random node within the stepsize  
        Node* newNode = steer_KBM(nearest, randomNode, stepSize);
        
        // cout << "i=" << i <<endl;
        // cout << "Random Node x: " << randomNode->x << " y: " << randomNode->y <<
        //            " theta: " << randomNode->theta << endl;
        // cout << "nearest Node x: " << nearest->x << " y: " << nearest->y <<
        //            " theta: " << nearest->theta << endl;
        // cout << "newNode Node x: " << newNode->x << " y: " << newNode->y <<
        //            " theta: " << newNode->theta << endl;


        //check collision
        if (isFree(*newNode, obstacles) && isPathFree(*newNode, *nearest, obstacles)){

            tree.push_back(newNode);
            nearest->children.push_back(newNode);

            //rewire the tree considering the new node.
            rewire(tree, newNode, searchSize, obstacles);
        }
        delete randomNode;

        //check if newNode is near goal.
        if (calculateDistanceNode(*newNode, *goalNode) < stepSize && !goalFound){
            
            goalNode->parent = newNode;
            tree.push_back(goalNode);
            goalFound = true;
            cout << "goal reached" << "(" << goalNode->x << "," <<
                     goalNode->y << "," << goalNode->theta << ")" << endl;
            cout << "iterations taken = " << i << endl;
            //break; //uncomment this to stop after goal is found.
        }

        
    }

    return {tree, goalNode}; 
    
}
    
vector<Node*> getPath(Node* goal){

    vector<Node*> path;
    Node* current = goal;
    while (current->parent != nullptr){
        
        path.push_back(current);
        current = current->parent;
    }
    path.push_back(current);

    return path;
}

std::vector<std::pair<std::vector<double>, std::vector<double>>> generateCirclePoints(
    const CircularObstacle& obstacle, int num_points = 100) {

    std::vector<double> x_points(num_points);
    std::vector<double> y_points(num_points);

    for (int i = 0; i < num_points; ++i) {
        double theta = 2.0 * M_PI * i / num_points;
        x_points[i] = obstacle.x + obstacle.radius * std::cos(theta);
        y_points[i] = obstacle.y + obstacle.radius * std::sin(theta);
    }

    return {std::make_pair(x_points, y_points)};
}


void plotRRT(const std::vector<Node*>& tree, const std::vector<Node*>& path, const Node& goal,
            vector<RectangularObstacle>& obstacles) {
    
    
    //plot obstacles
    for (auto obs : obstacles){

        std::vector<double> x = {obs.x_min, obs.x_max, obs.x_max, obs.x_min, obs.x_min};
        std::vector<double> y = {obs.y_min, obs.y_min, obs.y_max, obs.y_max, obs.y_min};

        // Plot the rectangle
        plt::plot(x, y, "r-"); // "r-" specifies a red solid line

    }
    

    // plot RRT
    
    for (auto node : tree) {
        if (node->parent) {
           //plt::plot({node->parent->x, node->x}, {node->parent->y, node->y}, "b-");
           continue;
        }
        else{
            // plotting the start node
            plt::plot({node->x}, {node->y}, {{"color","red"},{"marker", "o"}, {"markersize", "10"}, 
            {"label","Start"}});
        }
    }
    
    //plot Path
    for (auto node : path) {
        if (node->parent) {
            plt::plot({node->parent->x, node->x}, {node->parent->y, node->y}, 
                {{"color", "orange"}, {"linestyle", "--"}, {"linewidth", "3"}});
            
        }

    }

    for (auto node : path) {
        if (node->theta) {
    
            std::vector<double> X = {node->x};
            std::vector<double> Y = {node->y};
            std::vector<double> U = {cos(node->theta)};
            std::vector<double> V = {sin(node->theta)};
            plt::quiver(X, Y, U, V);
            
        }

    }

    plt::plot({goal.x}, {goal.y}, {{"color","green"},{"marker", "o"}, {"markersize", "10"}, {"label","Goal"}});
    plt::plot({-1, 20}, {-1.5, -1.5}, {{"color", "black"}, {"linestyle", "-"}});
    plt::plot({-1, 20}, {4.5, 4.5}, {{"color", "black"}, {"linestyle", "-"}});
    plt::plot({-1, 20}, {1.5, 1.5}, {{"color", "black"}, {"linestyle", "--"}});
    plt::xlabel("X [m]");
    plt::ylabel("Y [m]");
    plt::xlim(-1.0, 20.0);
    plt::ylim(-2.0, 5.0);
    //plt::axis("equal");
    plt::legend({{"loc", "lower left"}});
    plt::show();
}


int main(){


    cout << "Using RRTstar" << endl;
    // Start the timer
    auto start = std::chrono::steady_clock::now();
    std::srand(10); // random seed. 
    float goalx = 8.5, goaly = 3.0, goaltheta = 0.0; // goal x and y and theta
    float startx = 0.0, starty = 0.0, starttheta = 0.0; //start x and y
    float Xmax = 13.0, Ymax = 4.0, thetamax = static_cast <float> (M_PI_2); // max value of X and Y and theta
    //float Xmin = 0.0, Ymin = -1, thetamax = -M_PI_2; // max value of X and Y and theta
    int maxIterations = 20000; // 
    float stepSize = 0.5; //max distance a new node is created
    float searchSize = 0.75; // search readius for rewiring.

    //vector<CircularObstacle> obstacles;
    //obstacles.push_back(CircularObstacle(0,2,1));
    //obstacles.push_back(CircularObstacle(8,2,1));

    vector<RectangularObstacle> obstacles;
    obstacles.push_back(RectangularObstacle(-0.5, 4.0, 2, 4));
    obstacles.push_back(RectangularObstacle(12.5, 17, 2, 4));

    //unit_test();
    
    std::pair<vector<Node*>, Node*> RRT_tree_goal = RRTstar(startx, starty, starttheta, goalx, goaly, goaltheta, Xmax, Ymax, thetamax, \
                     maxIterations, stepSize, obstacles, searchSize);

    vector<Node*> tree = RRT_tree_goal.first;
    Node* goalNode = RRT_tree_goal.second;

    // End the timer
    auto end = std::chrono::steady_clock::now();

    // Calculate the duration
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // Output the running time
    std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;

    // Calculating the path.
    vector<Node*> path = getPath(goalNode);
    cout << "Path found:" << endl;
    for (auto node : path){

        cout << "(" << node->x << "," <<  node->y << "," << node->theta << ")" << "->";
    }
    cout << endl;   

    plotRRT(tree, path, *goalNode, obstacles);

    // Clean up dynamically allocated nodes
    for (auto node : tree) {
        delete node;
    }
    
    return 0;
}