#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <cstdlib>

#include "/home/naveed/Documents/matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace std;

float polynomial(Eigen::VectorXd& coeff, double t){

    float value;
    value = coeff(0) + coeff(1) * t + coeff(2) * pow(t, 2) + coeff(3) * pow(t, 3) + \
                    coeff(4) * pow(t, 4) + coeff(5) * pow(t, 5);

    return value;
}

class Node{

    public:

        float x,y,theta,cost;
        Node* parent;
        std::vector<Node*> children;

        //Constructor
        Node(float x, float y, float theta, Node* parent = nullptr, float cost = numeric_limits<float>::max())\
                :x(x), y(y), theta(theta), parent(parent), cost(cost) {}


};

void plottrajectory(const Node& start, const Node& goal, vector<float>& x, vector<float>& y,
                    vector<float>& theta) {
    

    // plotting the start node
    plt::plot({start.x}, {start.y}, {{"color","red"},{"marker", "o"}, {"markersize", "10"}, 
    {"label","Start"}});

    
    //plot Path
    plt::plot(x, y, {{"color", "orange"}, {"linestyle", "--"}, {"linewidth", "3"}});
            
        

    // for (auto node : path) {
    //     if (node->theta) {
    
    //         std::vector<double> X = {node->x};
    //         std::vector<double> Y = {node->y};
    //         std::vector<double> U = {cos(node->theta)};
    //         std::vector<double> V = {sin(node->theta)};
    //         plt::quiver(X, Y, U, V);
            
    //     }

    // }

    plt::plot({goal.x}, {goal.y}, {{"color","green"},{"marker", "o"}, {"markersize", "10"}, {"label","Goal"}});
    plt::plot({-10, 10}, {-1, -1}, {{"color", "black"}, {"linestyle", "-"}});
    plt::plot({-10, 10}, {3, 3}, {{"color", "black"}, {"linestyle", "-"}});
    plt::plot({-10, 10}, {1, 1}, {{"color", "black"}, {"linestyle", "--"}});
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::xlim(-1.0, 11.0);
    plt::ylim(-2.0, 4.0);
    //plt::axis("equal");
    plt::legend();
    plt::show();
}


int main(){

    //positions
    float goalx = 5.0, goaly = 2.0, goaltheta = 0.0; // goal x and y and theta
    float startx = 0.0, starty = 0.0, starttheta = 0.0; //start x and y
    float Xmax = 10.0, Ymax = 3.0, thetamax = static_cast <float> (M_PI_2); // max value of X and Y and theta
    
    Node* startNode = new Node(startx, starty, starttheta, nullptr, 0);
    Node* goalNode = new Node(goalx, goaly, goaltheta);


    //velocities
    float goalxdot = 1.0, goalydot = 0.0, goalthetadot = 0.0; 
    float startxdot = 1.0, startydot = 0.0, startthetadot = 0.0;

    //acceleration
    float goalxddot = 0.0, goalyddot = 0.0, goalthetaddot = 0.0;
    float startxddot = 0.0, startyddot = 0.0, startthetaddot = 0.0; 

    //initial and final time.
    float t0 = 0.0, tf = 6.0;  // initial and final time.
    int steps = 101; // number of time-steps

    Eigen::MatrixXd data_mat(6,6);
    Eigen::MatrixXd data_matInv(6,6);

    data_mat << 1.0, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5),
                0.0, 1.0, 2.0 * t0, 3 * pow(t0,2), 4 * pow(t0,3), 5 * pow(t0,4),
                0.0, 0.0, 2.0, 6 * t0, 12 * pow(t0,2), 20 * pow(t0,3),
                1.0, tf, pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5),
                0.0, 1.0, 2.0 * tf, 3 * pow(tf,2), 4 * pow(tf,3), 5 * pow(tf,4),
                0.0, 0.0, 2.0, 6 * tf, 12 * pow(tf,2), 20 * pow(tf,3);

    data_matInv = data_mat.inverse();
    
    Eigen::VectorXd x_vec(6), y_vec(6), theta_vec(6);
    Eigen::VectorXd coeff_x(6), coeff_y(6), coeff_theta(6);

    x_vec << startx, startxdot, startxddot, goalx, goalxdot, goalxddot;
    y_vec << starty, startydot, startyddot, goaly, goalydot, goalyddot;
    theta_vec << starttheta, startthetadot, startthetaddot, goaltheta, goalthetadot, goalthetaddot;

    coeff_x = data_matInv * x_vec;
    coeff_y = data_matInv * y_vec;
    coeff_theta = data_matInv * theta_vec;

    Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(steps,t0,tf); 

    std::vector <float> x_traj, y_traj, theta_traj;
    x_traj.reserve(steps);
    y_traj.reserve(steps);
    theta_traj.reserve(steps);

    for (int i = 0; i < steps; i++){

        x_traj.push_back(polynomial(coeff_x, t(i)));
        y_traj.push_back(polynomial(coeff_y, t(i)));
        theta_traj.push_back(polynomial(coeff_theta, t(i)));
    }

    // Output the trajectories for verification
    for (int i = 0; i < steps; ++i) {
        cout << "t: " << t(i) << " x: " << x_traj[i] << " y: " << y_traj[i] << " theta: " << theta_traj[i] << endl;
    }

    plottrajectory(*startNode, *goalNode, x_traj, y_traj, theta_traj);
    return 0;
}