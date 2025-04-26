import sys
import heapq

directions = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)}

def parse_file(filepath):
    with open(filepath, 'r') as f:
        cols = int(f.readline())
        rows = int(f.readline())
        grid = []
        for i in range(rows):
            line = f.readline().strip()
            grid.append(list(line))

    start_loc = None
    dirty_cells = set()

    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == '@': # robot starting loc
                start_loc = (r, c)
            elif grid[r][c] == '*': # dirty cell
                dirty_cells.add((r, c))
    
    # print("Grid:")
    # for row in grid:
    #     print(" ".join(row))
    # print("\nRobot Starting Location: ", start_loc)
    # print("Dirty Cells: ", dirty_cells, "\n")
    
    return grid, start_loc, dirty_cells

def dfs(grid, start_loc, dirty_cells):
    initial_state = {"loc": start_loc, "dirt": set(dirty_cells), "path": []} #had the state as a dict
    stack = [initial_state]
    visited = set()
    
    rows, cols = len(grid), len(grid[0])
    nodes_generated = 1
    nodes_expanded = 0

    while stack:
        node = stack.pop()
        loc, dirt_left, path = node["loc"], node["dirt"], node["path"]
        nodes_expanded += 1

        if not dirt_left: #no more dirty cells to clean
            return path, nodes_generated, nodes_expanded

        #vacuuming if dirty
        if loc in dirt_left:
            new_dirt = list(dirt_left)
            new_dirt.remove(loc)
            stack.append({"loc": loc, "dirt": new_dirt, "path": path + ["V"]})
            nodes_generated += 1
            continue

        r, c = loc
        
        # realized that the order of my directions matters but right now w like LIFO setup
        # it searches in order W, E, S, N?
        for action, (dr, dc) in directions.items():
            nr, nc = r + dr, c + dc
            if 0 <= nc < cols and 0 <= nr < rows and grid[nr][nc] != '#':
                new_state = ((nr, nc), tuple(dirt_left))
                if new_state not in visited:
                    visited.add(new_state)
                    stack.append({"loc": (nr, nc), "dirt": dirt_left, "path": path + [action]})
                    nodes_generated += 1

    # print("No path. The grid could not be cleaned")
    return [], nodes_generated, nodes_expanded #if grid can't be cleaned (still dirt left)

def ucs(grid, start_loc, dirty_cells):
    initial_state = (0, [], start_loc, tuple(dirty_cells)) # (cost, path, loc, dirt_left)
    priority_queue = [initial_state]
    visited = set()

    rows, cols = len(grid), len(grid[0])
    nodes_generated = 1
    nodes_expanded = 0

    while priority_queue:
        curr_cost, path, loc, dirt_left = heapq.heappop(priority_queue)
        new_state = (loc, dirt_left)
        if new_state in visited:
            continue
        visited.add(new_state)
        nodes_expanded += 1
        
        if not dirt_left:
            return path, nodes_generated, nodes_expanded
        
        if loc in dirt_left:
            new_dirt = list(dirt_left)
            new_dirt.remove(loc)
            heapq.heappush(priority_queue, (curr_cost + 1, path + ["V"], loc, tuple(new_dirt)))
            nodes_generated += 1
            continue
        
        r, c = loc
        for action, (dr, dc) in directions.items():
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != '#':
                heapq.heappush(priority_queue, (curr_cost + 1, path + [action], (nr, nc), dirt_left))
                nodes_generated += 1

    # print("No path. The grid could not be cleaned")
    return [], nodes_generated, nodes_expanded #if grid can't be cleaned (still dirt left)

def main():
    if len(sys.argv) != 3:
        print("Please use the following format: python3 planner.py [algorithm] [world-file]")
        sys.exit(1)

    algo = sys.argv[1]
    filepath = sys.argv[2]

    grid, start_loc, dirty_cells = parse_file(filepath)
    
    if algo == "depth-first":
        actions, nodes_generated, nodes_expanded = dfs(grid, start_loc, dirty_cells)
    elif algo == "uniform-cost":
        actions, nodes_generated, nodes_expanded = ucs(grid, start_loc, dirty_cells)
    else:
        print("Unknown algorithm, use 'depth-first' or 'uniform-cost'")
        sys.exit(1)

    for action in actions:
        print(action)
    print(f"{nodes_generated} nodes generated")
    print(f"{nodes_expanded} nodes expanded")

if __name__ == "__main__":
    main()