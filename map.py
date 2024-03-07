import heapq

class Node:
    def __init__(self, row, col, cost=0, heuristic=0):
        self.row = row
        self.col = col
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic
        self.parent = None

    # Custom comparison for heap queue
    def __lt__(self, other):
        return self.total_cost < other.total_cost

def heuristic(node, goal):
    # Euclidean distance heuristic
    return ((node.row - goal.row)**2 + (node.col - goal.col)**2)**0.5

def generate_neighbors(node, grid):
    neighbors = []
    rows, cols = len(grid), len(grid[0])

    # Define possible movements (up, down, left, right)
    movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    for dr, dc in movements:
        new_row, new_col = node.row + dr, node.col + dc

        # Check if the new position is within the grid
        if 0 <= new_row < rows and 0 <= new_col < cols and grid[new_row][new_col] != 1:
            neighbors.append(Node(new_row, new_col))

    return neighbors

def reconstruct_path(node):
    path = []
    while node:
        path.append((node.row, node.col))
        node = node.parent
    return path[::-1]

def best_first_search(start, goal, grid):
    open_list = [start]
    closed_set = set()

    while open_list:
        current_node = open_list.pop(0)

        if (current_node.row, current_node.col) == (goal.row, goal.col):
            return reconstruct_path(current_node)

        closed_set.add((current_node.row, current_node.col))

        neighbors = generate_neighbors(current_node, grid)
        for neighbor in neighbors:
            if (neighbor.row, neighbor.col) not in closed_set and neighbor not in open_list:
                neighbor.parent = current_node
                open_list.append(neighbor)
                open_list.sort(key=lambda x: heuristic(x, goal))

    return None  # No path found

def a_star(start, goal, grid):
    open_list = [start]
    closed_set = set()

    while open_list:
        current_node = heapq.heappop(open_list)

        if (current_node.row, current_node.col) == (goal.row, goal.col):
            return reconstruct_path(current_node)

        closed_set.add((current_node.row, current_node.col))

        neighbors = generate_neighbors(current_node, grid)
        for neighbor in neighbors:
            if (neighbor.row, neighbor.col) not in closed_set:
                neighbor_cost = current_node.cost + 1
                if neighbor not in open_list or neighbor_cost < neighbor.cost:
                    neighbor.cost = neighbor_cost
                    neighbor.heuristic = heuristic(neighbor, goal)
                    neighbor.total_cost = neighbor_cost + neighbor.heuristic
                    neighbor.parent = current_node

                    if neighbor not in open_list:
                        heapq.heappush(open_list, neighbor)

    return None  # No path found

# Example usage:
start_node = Node(0, 0)
goal_node = Node(4, 4)

grid_map = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0]
]

print("Best First Search Path:", best_first_search(start_node, goal_node, grid_map))
print("A* Path:", a_star(start_node, goal_node, grid_map))