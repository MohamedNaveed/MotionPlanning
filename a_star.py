# A-star algorithm implementation. 

import heapq
import visulaize_grid


#size of the grid

ROW = 9
COL = 10


#define class for each node/cell

class node:

    def __init__(self):

        self.parent_i = 0 # row index of parent
        self.parent_j = 0 # col index of parent
        self.f = float('inf') # cost to go from the cell
        self.g = float('inf') # cost from start to this cell.
        self.h = 0 # heuristic cost from the current cell to destination

#check if cell is valid
def is_valid(row, col):

    return ((row >=0) and (row<ROW) and (col>=0) and (col<COL))

#check if cell in unblocked
def is_unblocked(grid, row, col):

    return (grid[row][col]==1) 

def cal_h(node, dest):

    return ((node[0] - dest[0]) ** 2 + (node[1] - dest[1]) ** 2) ** 0.5
    #return 0 #for dijsktra

def is_destination(node, dest):

    return (node[0] == dest[0] and node[1] == dest[1])

def trace_path(src, dest, node_details):

    i = dest[0]
    j = dest[1]
    path = []
    path.append(dest)

    while not (i == src[0] and j == src[1]):

        new_i = node_details[i][j].parent_i
        new_j = node_details[i][j].parent_j
        
        path.append([new_i, new_j])
        i = new_i
        j = new_j

    path.reverse()
    for i in path:
        print("->", i, end=" ")

    return path


def a_star_search(grid, src, dest):

    #check if src and dest are valid

    if not (is_valid(src[0], src[1]) and is_valid(dest[0], dest[1])):

        print('Either source or destination is invalid.')

    elif not (is_unblocked(grid, src[0], src[1]) and is_unblocked(grid, dest[0], dest[1])):

        print('Source or Destination node is blocked.')

    #initialize closed list
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]

    #initialize every node.
    node_details = [[node() for _ in range(COL)] for _ in range(ROW)]
    
    #initialize the start node details
    node_details[src[0]][src[1]].f = 0
    node_details[src[0]][src[1]].g = 0 
    node_details[src[0]][src[1]].h = 0
    node_details[src[0]][src[1]].parent_i = src[0]
    node_details[src[0]][src[1]].parent_j = src[1]

    #create open list
    open_list = []
    heapq.heappush(open_list, (0.0, src[0], src[1]))

    # destination found flag
    DEST_FOUND = False

    #A* search loop

    while len(open_list) > 0:

        cur_node = heapq.heappop(open_list)

        i = cur_node[1] # cur row
        j = cur_node[2]
        #print("node = ", i, j, "parent = ", node_details[i][j].parent_i, node_details[i][j].parent_j)
        closed_list[i][j] = True

        # directions to check for successors.
        directions = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,-1), (-1,1)]

        for dir in directions:
            new_i = i + dir[0]
            new_j = j + dir[1]
            
            
            #check if successor is valid
            if (is_valid(new_i, new_j) and is_unblocked(grid, new_i, new_j)):

                if is_destination([new_i, new_j], dest):
                    print('Destination reached. ')
                    
                    node_details[new_i][new_j].parent_i = i
                    node_details[new_i][new_j].parent_j = j
                    
                    path = trace_path(src, dest, node_details)
                    DEST_FOUND = True

                    return path
                
                else:
                    
                    #adding incremental cost. 
                    g_new = node_details[i][j].g + 1
                    node_details[new_i][new_j].h = cal_h([new_i, new_j], dest)
                    f_new = g_new + node_details[new_i][new_j].h
                    
                    #checking if new cost is lower.
                    if node_details[new_i][new_j].f > f_new:

                        node_details[new_i][new_j].g = g_new
                        node_details[new_i][new_j].f = f_new
                    
                        node_details[new_i][new_j].parent_i = i
                        node_details[new_i][new_j].parent_j = j
                        
                        heapq.heappush(open_list, (f_new, new_i, new_j))

    if not DEST_FOUND:
        print('Destination not found.')


if __name__ == "__main__":

    # Define the grid (1 for unblocked, 0 for blocked)
    grid = [
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1, 0, 1, 0, 1],
        [0, 0, 1, 0, 1, 0, 0, 0, 0, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
        [1, 0, 1, 1, 1, 1, 0, 1, 0, 0],
        [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 0, 0, 1, 0, 0, 1]
    ]
 
    # Define the source and destination
    src = [8, 0]
    dest = [0, 9]
 
    # Run the A* search algorithm
    path = a_star_search(grid, src, dest)


    #visualize the grid
    title = 'A* Algorithm Visualization'
    visulaize_grid.plot_grid(grid, src, dest, path,title)
    

