#Breadth first search
import convert_grid2graph
from collections import deque
import visulaize_grid

class node:

    def __init__(self):

        self.status = 'unvisited'; # 'unvisited' | 'visited' | 'dead'
        self.d = float('inf')
        self.parent = None

def is_valid(graph, vertex):

    return (len(graph[vertex]) > 0)

def trace_path(src, dest, vertex_details, num_cols):

    i = int(dest/num_cols)
    j = dest - i*num_cols
    path = []
    path.append([i,j])

    cur_ver = dest

    while not (cur_ver == src):

        cur_ver = vertex_details[cur_ver].parent

        i = int(cur_ver/num_cols)
        j = cur_ver - i*num_cols

        path.append([i, j])


    path.reverse()
    for i in path:
        print("->", i, end=" ")

    print()
    return path

def bfs(graph, src_vertex, V):

    if is_valid(graph, src_vertex):

        vertex_details = [node() for _ in range(V)]

        queue = deque()
        queue.append(src_vertex)
        vertex_details[src_vertex].status = 'visited'
        vertex_details[src_vertex].d = 0
        vertex_details[src_vertex].parent = src_vertex

        while(len(queue) > 0):

            ver = queue.popleft()

            for next_ver in graph[ver]:

                if vertex_details[next_ver].status == 'unvisited':

                    vertex_details[next_ver].status = 'visited'
                    vertex_details[next_ver].d  = vertex_details[ver].d + 1
                    vertex_details[next_ver].parent = ver

                    queue.append(next_ver)

            vertex_details[ver].status = 'dead'

        return vertex_details


    else:
        print('Source vertex is not valid.')




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
    
    # Define the source and destination
    src = [8, 0]
    dest = [0, 9]   

    graph = convert_grid2graph.create_graph_from_grid(grid)
    
    rows = len(grid)
    cols = len(grid[0])

    
    src_vertex = src[0]*cols + src[1]
    dest_vertex = dest[0]*cols + dest[1]

    vertex_details = bfs(graph.graph, src_vertex, graph.V)

    path = trace_path(src_vertex, dest_vertex, vertex_details, cols)
    
    #visualize grid.
    title = 'Breadth First Search'
    visulaize_grid.plot_grid(grid, src, dest, path, title)
