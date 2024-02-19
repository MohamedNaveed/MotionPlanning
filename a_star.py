# A-star algorithm implementation. 

import heapq
import math
import numpy as np

#size of the grid

ROW = 9
COL = 10


#define class for each node/cell

class node:

    def init(self):

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

def a_star_search(grid, src, dest):

    pass

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
    dest = [0, 0]
 
    # Run the A* search algorithm
    a_star_search(grid, src, dest)

