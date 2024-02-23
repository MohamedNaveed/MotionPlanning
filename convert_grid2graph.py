#convert a 2D grid to a graph

from collections import defaultdict

ROW = 0
COL = 0

class Graph():

    def __init__(self, V):

        self.V = V
        self.graph = defaultdict(list)

    def add_edge(self, vertex, edge):

        self.graph[vertex].append(edge)
        

def is_unblocked(grid, row, col):

    return (grid[row][col]==1) 

def is_valid(row, col):

    return ((row >=0) and (row<ROW) and (col>=0) and (col<COL))

def create_graph_from_grid(grid):

    global ROW, COL
    ROW = len(grid)
    COL = len(grid[0])

    V = ROW*COL

    graph = Graph(V)

    # directions to check for successors.
    directions = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,-1), (-1,1)]

    #grid points are indexed using a single number. 
    #If a point is at the (i,j) col the index is i*COL + j

    for i in range(ROW):
        for j in range(COL):
            
            if is_unblocked(grid, i, j):
                vertex_idx = i*COL + j

                #searching the neightbours
                for dir in directions:

                    new_i = i + dir[0]
                    new_j = j + dir[1]

                    if (is_valid(new_i, new_j) and is_unblocked(grid, new_i, new_j)):

                        neighbour_index = new_i*COL + new_j    
                        graph.add_edge(vertex_idx, neighbour_index)


    return graph


if __name__ == '__main__':

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
 
    graph = create_graph_from_grid(grid)

    for v in range(graph.V):
        print(v, ":", graph.graph[v])

    print(graph.graph)