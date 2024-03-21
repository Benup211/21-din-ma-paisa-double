import heapq

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)
def calculate_score_and_cost(path, silver_points, tiles):
    score = 0
    cost = 0
    for position in path:
        if position in silver_points:
            score += 1 # Assuming each Silver Point adds 1 to the score
        if position in tiles:
            cost += 1 # Assuming each Tile used adds 1 to the cost
    return score, cost

def a_star_algorithm(start, goal, grid, tiles):
    start_node = Node(None, start)
    goal_node = Node(None, goal)
    open_list = []
    closed_list = []
    heapq.heappush(open_list, start_node)
    while len(open_list) > 0:
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent
            path.append(start_node.position)
            return path[::-1] # Return reversed path
        children = [] # Generate children
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent moves
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            if node_position[0] > (len(grid) - 1) or node_position[0] < 0 or node_position[1] > (len(grid[0]) -1) or node_position[1] < 0:
                continue
            new_node = Node(current_node, node_position)
            children.append(new_node)
        for child in children:
            for closed_child in closed_list:
                if child == closed_child:
                    continue
            child.g = current_node.g + 1
            child.h = heuristic(child.position, goal_node.position)
            child.f = child.g + child.h
            heapq.heappush(open_list, child)
    return []
def select_best_path(paths, scores, costs):
    best_path = None
    best_score = -1
    best_cost = float('inf')
    for i, path in enumerate(paths):
        if scores[i] > best_score or (scores[i] == best_score and costs[i] < best_cost):
            best_path = path
            best_score = scores[i]
            best_cost = costs[i]
    return best_path

def output_solution(selected_path):
    if selected_path is not None:
        for position in selected_path:
            print(f"Tile at position: {position}")
    else:
        print("No path found.")


def parse_input(input_file):
    with open('01-comedy.txt', 'r', encoding='utf-8-sig') as file:
        # Assuming the first line contains grid size
        grid_size = tuple(map(int, file.readline().split()))
        # Assuming the next lines contain Golden Points, Silver Points, and Tiles
        golden_points = []
        silver_points = []
        tiles = []
        for line in file:
            if line.startswith("Golden"):
                golden_points.append(tuple(map(int, line.split()[1:])))
            elif line.startswith("Silver"):
                silver_points.append(tuple(map(int, line.split()[1:])))
            elif line.startswith("Tile"):
                tiles.append(tuple(map(int, line.split()[1:])))
    return grid_size, golden_points, silver_points, tiles

def solve_problem(input_file):
    grid, golden_points, silver_points, tiles = parse_input(input_file)
    paths = [] # Assuming we have a way to generate all possible paths between Golden Points
    scores = []
    costs = []
    for start, goal in golden_points:
        path = a_star_algorithm(start, goal, grid, tiles)
        paths.append(path)
        score, cost = calculate_score_and_cost(path, silver_points, tiles)
        scores.append(score)
        costs.append(cost)
    selected_path = select_best_path(paths, scores, costs)
    output_solution(selected_path)

if __name__ == "__main__":
    input_file = "01-comedy.txt"
    solve_problem(input_file)
