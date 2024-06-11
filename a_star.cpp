#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <queue>
#include <algorithm>

int ROW = 9, COL = 10; //size of the grid

class node{

    public:

        int i, j; // row index, column index
        node* parent;
        double f, g, h;
        // f- total cost
        // g- cost from start to this cell.
        // h- heuristic cost from the current cell to destination
        node(int row, int col, node* parent=nullptr): i{row}, j{col}, \
            parent{parent}, f{std::numeric_limits<double>::max()}, \
            g{std::numeric_limits<double>::max()}, h{0.0} {};

        ~node(){};
};

struct compare_node{
    
    bool operator()(const node* a, const node* b){
        return a->f > b->f;
    }
};

class a_star{

    public:
        std::vector <std::vector<int>> grid;
        node *src, *dest;
        //Constructor
        a_star(std::vector <std::vector<int>>& grid, node* src, node* dest):\
         grid{grid}, src{src}, dest{dest} {}; 

        //search
        void a_star_search () {

            //check if src and dest are valid

            if (!(is_valid(src->i, src->j) && is_valid(dest->i, dest->j)))

                std::cout << "Either source or destination is invalid." << std::endl;

            else if  (!(is_unblocked(grid, src->i, src->j) && is_unblocked(grid, dest->i, dest->j)))

                std::cout << "Source or Destination node is blocked." << std::endl;

            //initialize closed list
            std::vector <std::vector <bool>> closed_list(ROW, std::vector <bool>(COL, false));
            
            //node list
            std::vector <std::vector <node*>> node_list(ROW, std::vector <node*> (COL));
            
            // initializing node list
            for (int i = 0; i < ROW; i++){
                for (int j = 0; j < COL; j++){

                    if (i == src->i && j == src->j) 
                        node_list[i][j] = src;
                    else if (i == dest->i && j == dest->j) 
                        node_list[i][j] = dest;
                    else
                        node_list[i][j] = new node(i, j);
                }
            }

            // initialize src node
            node_list[src->i][src->j]->f = 0.0;
            node_list[src->i][src->j]->g = 0.0;
            node_list[src->i][src->j]->h = 0.0;
            node_list[src->i][src->j]->parent = nullptr;

            //create an open list as a priority queue
            std::priority_queue <node*, std::vector <node*>, compare_node> open_list;
            open_list.push(src);

            // destination found flag
            bool DEST_FOUND = false;

            //A* search loop

            while (!open_list.empty() && !DEST_FOUND) {

                node* cur_node = open_list.top(); //access the top element
                open_list.pop(); //removes the top element

                int i = cur_node->i; // cur row
                int j = cur_node->j;
                
                closed_list[i][j] = true;
                // directions to check for successors.
                std::vector <std::pair <int, int>> directions = {{0,1}, {0,-1}, 
                    {1,0}, {-1,0}, {1,1}, {1,-1}, {-1,-1}, {-1,1}};

                for (auto dir : directions){

                    int new_i = i + dir.first;
                    int new_j = j + dir.second;

                    //check if successor is valid
                    if (is_valid(new_i, new_j) && is_unblocked(grid, new_i, new_j)){

                        if (node_list[new_i][new_j] == dest) {

                            std::cout << "Destination reached. " << std::endl; 
                    
                            node_list[new_i][new_j]->parent = node_list[i][j];
                            
                            std::vector <node*> path = trace_path(dest);
                            DEST_FOUND = true;

                        }

                        else {
                            
                            //adding incremental cost. 
                            double g_new = node_list[i][j]->g + 1.0;
                            node_list[new_i][new_j]->h = cal_h(*node_list[new_i][new_j], *dest);
                            double f_new = g_new + node_list[new_i][new_j]->h;

                            //checking if new cost is lower.
                            if (node_list[new_i][new_j]->f > f_new){

                                node_list[new_i][new_j]->g = g_new;
                                node_list[new_i][new_j]->f = f_new;
                            
                                node_list[new_i][new_j]->parent = node_list[i][j];
                                
                                open_list.push(node_list[new_i][new_j]);
                            }
                        }
                    }
                }
            }

            for (int i = 0; i < ROW; i++){
                for (int j = 0; j < COL; j++){
                    delete node_list[i][j];
                }
            }

            if (!DEST_FOUND)
                std::cout << "Destination not found" << std::endl;
        }

        std::vector <node*> trace_path (node* dest) {

            node* temp = dest;
            std::vector <node*> path;

            while (temp->parent != nullptr){
                path.push_back(temp);
                temp = temp->parent;
            }
            path.push_back(temp);

            std::reverse(path.begin(), path.end());

            for (auto path_node : path){
                std::cout << "(" << path_node->i << "," << path_node->j << ")" << " - > " ;
            }
            std::cout << std::endl;

            return path;
        }
        //Destructor
        ~a_star(){};
    
    private:
        

        //check if cell is valid
        bool is_valid(int row, int col){

            return ((row >= 0) && (row < ROW) && (col >= 0) && (col < COL));
        }

        //check if cell in unblocked
        bool is_unblocked(std::vector <std::vector<int>>& grid, int row, int col){

            return (grid[row][col]==1); 
        }

        double cal_h(node& cur, node& dest){

            return sqrt((cur.i - dest.i) * (cur.i - dest.i) + (cur.j - dest.j) * (cur.j - dest.j)) ;
            //return 0 #for dijsktra
        }

};

int main() {

    // Define the grid (1 for unblocked, 0 for blocked)
    std::vector <std::vector<int>> grid = {
        {1, 0, 1, 1, 1, 1, 0, 1, 1, 1},
        {1, 1, 1, 0, 1, 1, 1, 0, 1, 1},
        {1, 1, 1, 0, 1, 1, 0, 1, 0, 1},
        {0, 0, 1, 0, 1, 0, 0, 0, 0, 1},
        {1, 1, 1, 0, 1, 1, 1, 0, 1, 0},
        {1, 0, 1, 1, 1, 1, 0, 1, 0, 0},
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 0, 1, 1, 1, 1, 0, 1, 1, 1},
        {1, 1, 1, 0, 0, 0, 1, 0, 0, 1}
    };
    
    
    // Define the source and destination
    node* src = new node(8, 0);
    node* dest = new node(0, 9);

    a_star a_star_obj(grid, src, dest);
    a_star_obj.a_star_search();
    
    return 0;
}