# Read input from file
with open('00-trailer.txt', 'r', encoding='utf-8-sig') as file:
    lines = file.readlines()

# Extract W and H from the first line
W, H, GN, SM, TL = map(int, lines[0].split())

# Extract Golden Points (G)
G = []
for i in range(1, GN + 1):
    x, y = map(int, lines[i].split())
    G.append([x, y])

# Extract Silver Points (S)
S = []
for j in range(GN + 1, GN + SM + 1):
    x, y, score = map(int, lines[j].split())
    S.append([x, y, score])

# Extract Tiles (tile)
tile = {}
for k in range(GN + SM + 1, GN + SM + TL + 1):
    tk_id, tk_cost, tk_available = lines[k].split()
    tile[tk_id] = {"cost": int(tk_cost), "available": int(tk_available)}
print(W, H)
print(G)
print(S)
print(tile)
tile_directions = {}

# Tile 3
tile_directions.setdefault('3', []).append('left to right')

# Tile 5
tile_directions.setdefault('5', []).append('down to right')

# Tile 6
tile_directions.setdefault('6', []).append('left to down')

# Tile 7
tile_directions.setdefault('7', []).append('left to right')
tile_directions.setdefault('7', []).append('left to down')
tile_directions.setdefault('7', []).append('down to right')

# Tile 9
tile_directions.setdefault('9', []).append('up to right')

# Tile 96
tile_directions.setdefault('96', []).append('left to down')
tile_directions.setdefault('96', []).append('up to right')

# Tile A
tile_directions.setdefault('A', []).append('left to up')

# Tile A5
tile_directions.setdefault('A5', []).append('left to up')
tile_directions.setdefault('A5', []).append('down to right')

# Tile B
tile_directions.setdefault('B', []).append('left to right')
tile_directions.setdefault('B', []).append('left to up')
tile_directions.setdefault('B', []).append('up to right')

# Tile C
tile_directions.setdefault('C', []).append('up to down')

# Tile C3
tile_directions.setdefault('C3', []).append('left to right')
tile_directions.setdefault('C3', []).append('up to down')

# Tile D
tile_directions.setdefault('D', []).append('up to down')
tile_directions.setdefault('D', []).append('up to right')
tile_directions.setdefault('D', []).append('down to right')

# Tile E
tile_directions.setdefault('E', []).append('left to up')
tile_directions.setdefault('E', []).append('left to down')
tile_directions.setdefault('E', []).append('up to down')

# Tile F
tile_directions.setdefault('F', []).append('left to right')
tile_directions.setdefault('F', []).append('left to down')
tile_directions.setdefault('F', []).append('left to up')
tile_directions.setdefault('F', []).append('up to down')
tile_directions.setdefault('F', []).append('down to right')
tile_directions.setdefault('F', []).append('up to right')

print(tile_directions)

# Pseudocode for finding the best path between each Golden Point
def create_graph(W, H, G, S, tile_directions):
    # Initialize an empty graph
    graph = {}
    
    # Add all Golden Points and Silver Points as nodes
    for point in G + S:
        x, y = point[:2]
        graph[(x, y)] = {"neighbors": [], "cost": 0}
    
    # Add edges based on tile directions
    for tile_id, directions in tile_directions.items():
        for direction in directions:
            # Determine the movement based on the direction
            if direction == 'left to right':
                movement = (1, 0)
            elif direction == 'down to right':
                movement = (1, 1)
            # Add more conditions for other directions
            
            # Apply the movement to each point
            for point in G + S:
                x, y = point[:2]
                new_x, new_y = x + movement[0], y + movement[1]
                if (new_x, new_y) in graph:
                    graph[(x, y)]["neighbors"].append((new_x, new_y))
                    graph[(new_x, new_y)]["cost"] += tile[tile_id]["cost"]
    
    return graph
import heapq
def heuristic(a, b):
    """
    Calculate the Manhattan distance between two points.
    This is a simple heuristic that can be used in grid-based pathfinding.
    """
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def a_star(graph, start, goal):
    # Convert start and goal to tuples if they are lists
    if isinstance(start, list):
        start = tuple(start)
    if isinstance(goal, list):
        goal = tuple(goal)
    
    queue = [(0, start)]
    seen = set()
    mins = {start: 0}
    came_from = {start: None}

    while queue:
        (cost, current) = heapq.heappop(queue)
        if current not in seen:
            seen.add(current)
            if current == goal:
                break
            for next in graph[current]["neighbors"]:
                new_cost = cost + graph[next]["cost"]
                if next not in seen or new_cost < mins.get(next, float("inf")):
                    priority = new_cost + heuristic(goal, next)
                    heapq.heappush(queue, (priority, next))
                    mins[next] = new_cost
                    came_from[next] = current

    return came_from, mins


    return came_from, mins
def solve_tsp(graph, G):
    # Convert each list in G to a tuple
    G_tuples = [tuple(point) for point in G]
    
    unvisited = set(G_tuples)
    current = G_tuples[0]
    unvisited.remove(current)
    tour = [current]
    while unvisited:
        # Find the next city by iterating over the neighbors and selecting the one with the minimum cost
        next_city = min(unvisited, key=lambda city: sum(graph[current]["neighbors"][neighbor] for neighbor in graph[current]["neighbors"] if neighbor == city))
        tour.append(next_city)
        unvisited.remove(next_city)
        current = next_city
    return tour

def calculate_score(paths, S, tile_directions):
    total_score = 0
    for path in paths:
        for point in path:
            if point in S:
                total_score += S[point][2] # Assuming S is a dictionary with points as keys
    return total_score


# Preprocessing
graph = create_graph(W, H, G, S, tile_directions)

# Initialize variables for storing paths and costs
paths = {}
total_cost = 0

# Run A* for each pair of Golden Points
# Run A* for each pair of Golden Points
for i in range(len(G)):
    for j in range(i+1, len(G)):
        path, cost = a_star(graph, G[i], G[j])
        paths[(i, j)] = path
        # Extract the cost to the goal from the cost dictionary
        if isinstance(G[j], list):
            G[j] = tuple(G[j])
        goal_cost = cost.get(G[j], 0) # Assuming G[j] is the goal point
        total_cost += goal_cost


# Solve the Traveling Salesman Problem to find the optimal order of Golden Points
# Solve the Traveling Salesman Problem to find the optimal order of Golden Points
optimal_order = solve_tsp(graph, G)

# Calculate the final score
final_score = calculate_score(optimal_order, S, tile_directions)

# Output the results
print("Optimal Order:", optimal_order)
print("Total Cost:", total_cost)
print("Final Score:", final_score)
