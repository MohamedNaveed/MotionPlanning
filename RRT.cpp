#include <cmath>
#include <iostream>
#include <vector>
#include <cstdlib>

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
double calculateDistance(const Node& a, const Node& b){

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
// build tree
vector<Node*> RRT(float startx, float starty, float goalx, float goaly, float Xmax, float Ymax, 
    int maxIterations, float stepSize){
    
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
        tree.push_back(newNode);
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

    
int main(){

    cout << "Using RRT" << endl;

    float goalx = 6.0, goaly = 8.0; // goal x and y
    float startx = 0.0, starty =0.0; //start x and y
    float Xmax = 10.0, Ymax = 10.0; // max value of X and Y
    int maxIterations = 100000; // 
    float stepSize = 0.5; //

    vector<Node*> tree = RRT(startx, starty, goalx, goaly, Xmax, Ymax, maxIterations, stepSize);
    
    Node* goalNode = tree.back();

    vector<Node*> path = getPath(goalNode);

    for (auto node : path){

        cout << "(" << node->x << "," <<  node->y << ")" << "->";
    }
    cout << endl;   

    // Clean up dynamically allocated nodes
    for (auto node : tree) {
        delete node;
    }

    return 0;
}