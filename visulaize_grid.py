import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

# Define colors for visualization
colors = ListedColormap(['black', 'white', 'red', 'green'])

def plot_grid(grid, src, dest, path,title):

    grid[src[0]][src[1]] = 2
    grid[dest[0]][dest[1]] = 3

    path = np.array(path)
    plt.matshow(grid, cmap = colors)
    plt.plot(path[:, 1], path[:, 0], color='blue', marker='o')
    plt.title(title)
    plt.show()


if __name__ == "__main__":
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


    path = np.array([[8,0],[7,0],[6,0]])

    plot_grid(grid, src, dest, path, title = 'Sample grid')