#include <cmath>
#include <iostream>
#include <vector>
#include <cstdlib>
#include "/home/naveed/Documents/matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;


class Node{

    public:

        float x,y;
        Node* parent;
        
        //Constructor
        Node(float x, float y, Node* parent = nullptr)\
                :x(x), y(y), parent(parent){}


};

//calculate euclidian distance. 
template <typename T, typename Y>
float calculateDistance(const T& a, const Y& b){

    float dist;
    dist = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));

    return dist; 
}

// generate random node
Node* getRandomNode(float Xmax, float Ymax){

    float x = static_cast<float> (rand()) * Xmax/RAND_MAX;
    float y = static_cast<float> (rand()) * Ymax/RAND_MAX;

    return new Node(x, y);

}

//find nearest Node
Node* findNearestNode(vector<Node*>& tree, const Node& randomNode){

    float minDist = numeric_limits<float>::max();
    Node* nearestNode = nullptr;

    for (auto node : tree) {
        float dist = calculateDistance(*node, randomNode);

        if (dist < minDist) {
            minDist = dist;
            nearestNode = node;
        }
    }

    return nearestNode;


}

struct CircularObstacle{
    float x, y;
    float radius;

    CircularObstacle(float x, float y, float radius) : x(x), y(y), radius(radius) {}
};

// check if the node collides with obstacle
bool isFree(const Node& node, vector<CircularObstacle>& obstacles){

    float dist;

    // check if node is inside obstacle
    for(auto obs : obstacles){
        
        dist = calculateDistance(obs, node);

        if (dist <= obs.radius){
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

// check if the path connection passes through obstacle
bool isPathFree(const Node& node1, const Node& node2, vector<CircularObstacle>& obstacles){

    float dist, t; 

    for(auto obs : obstacles){
        
        //find angle between the vector connecting node 1 to center of obs and the vector from node 1 and node 2.
        vec v1(node2.x - node1.x, node2.y - node1.y);
        vec v2(obs.x - node1.x, obs.y - node1.y);

        //t is the projection of v2 on v1
        t = abs(v1.x * v2.x + v1.y * v2.y)/sqrt(v1.x * v1.x + v1.x * v1.x);

        vec pt(node1.x + (t * v2.x/sqrt(v1.x * v1.x + v1.x * v1.x)), 
                    node1.y + (t * v2.y/sqrt(v1.x * v1.x + v1.x * v1.x)));

        dist = calculateDistance(obs, pt);

        if (dist <= obs.radius){
            return false;
        }

    }

    return true;
}
// build tree
vector<Node*> RRT(float startx, float starty, float goalx, float goaly, float Xmax, float Ymax, 
    int maxIterations, float stepSize, vector<CircularObstacle>& obstacles){
    
    Node* startNode = new Node(startx, starty);
    Node* goalNode = new Node(goalx, goaly);

    vector<Node*> tree;
    tree.push_back(startNode);

    for(int i=0; i < maxIterations; ++i){

        Node* randomNode = getRandomNode(Xmax, Ymax);

        Node* nearest = findNearestNode(tree, *randomNode);

        float theta = atan2(randomNode->y - nearest->y, randomNode->x - nearest->x);
        float newX = nearest->x + stepSize * sin(theta);
        float newY = nearest->y + stepSize * cos(theta);

        Node* newNode = new Node(newX, newY, nearest);

        if (isFree(*newNode, obstacles) && isPathFree(*newNode, *nearest, obstacles)){
            tree.push_back(newNode);
        }
        delete randomNode;

        //check if newNode is near goal.
        if (calculateDistance(*newNode, *goalNode) < stepSize){
            
            goalNode->parent = newNode;
            tree.push_back(goalNode);
            cout << "goal reached" << endl;
            break;
        }

        
    }

    return tree; 
    
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
            vector<CircularObstacle>& obstacles) {
    
    //plot obstacles
    for (const auto& obstacle : obstacles) {
        auto circle_points = generateCirclePoints(obstacle);

        plt::plot(circle_points[0].first, circle_points[0].second, "k-");
    }

    // plot RRT
    for (auto node : tree) {
        if (node->parent) {
            plt::plot({node->parent->x, node->x}, {node->parent->y, node->y}, "b-");
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

    plt::plot({goal.x}, {goal.y}, {{"color","green"},{"marker", "o"}, {"markersize", "10"}, {"label","Goal"}});
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::axis("equal");
    plt::legend();
    plt::show();
}

void unit_test(){

    vector<CircularObstacle> obstacles;
    obstacles.push_back(CircularObstacle(0,0,1));

    Node* Node1 = new Node(-1.5, 0.25);
    Node* Node2 = new Node(-1.75, -1.25);

    if(!isPathFree(*Node1, *Node2, obstacles)){
        cout << "success" << endl;
    }
    else{
        cout << "failed" << endl;
    }

}

int main(){

    cout << "Using RRT" << endl;

    float goalx = 6.0, goaly = 8.0; // goal x and y
    float startx = 0.0, starty =0.0; //start x and y
    float Xmax = 10.0, Ymax = 10.0; // max value of X and Y
    int maxIterations = 100000; // 
    float stepSize = 0.5; //

    vector<CircularObstacle> obstacles;
    obstacles.push_back(CircularObstacle(2,2,1.5));
    obstacles.push_back(CircularObstacle(2,8,1.5));

    //unit_test();
    
    vector<Node*> tree = RRT(startx, starty, goalx, goaly, Xmax, Ymax, maxIterations, \
                         stepSize, obstacles);
    
    Node* goalNode = tree.back();

    vector<Node*> path = getPath(goalNode);

    for (auto node : path){

        cout << "(" << node->x << "," <<  node->y << ")" << "->";
    }
    cout << endl;   

    plotRRT(tree, path, *goalNode, obstacles);

    // Clean up dynamically allocated nodes
    for (auto node : tree) {
        delete node;
    }
    
    return 0;
}