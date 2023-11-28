import heapq

# State class
# - Represents a state in the algorithm
# - Each state has a coordinate and an associated cost (default is infinity)
class State:
    def __init__(self, index, cost = float('inf')):
        self.index = index
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

# DStarLite Class
# - Initialize the D* Lite Algoritm
# - W/ start state, goal state, & binary array (environment)
class DStarLite:
    def __init__(self, start, goal, binary_array):
        self.start = start
        self.goal = goal
        self.binary_array = binary_array
        self.rows, self.cols = len(binary_array), len(binary_array[0])
        # Priority queue that manages states based on their costs
        self.open_list = [start]

    # Helper Method
    # checks to see if position @ (x, y) is unoccupied
    def is_valid(self, x, y):
        return 0 <= x < self.rows and 0 <= y < self.cols and self.binary_array[x][y] == 0

    # Method to update the binary array
    def update_binary_array(self, x, y, value):
        if 0 <= x < self.rows and 0 <= y < self.cols: # Check that the coordinates are within range
            self.binary_array[x][y] = value
        else:
            print("Error: Attempted to update binary array outside of bounds.")

    # Update the cost of a given state
    def update_cost(self, state, new_cost):
        # Comparison w/ new cost
        if new_cost < state.cost:
            state.cost = new_cost

            # If the state is in the open list, delete the old data
            # Push the new data
            if state in self.open_list:
                self.open_list.remove(state)
                heapq.heappush(self.open_list, state)

    # Cost_to_move
    def cost_to_move (self, current_state, neighbor_state):
        # Extract indices (x, y) for the current and neighbor state
        current_x, current_y = current_state.index
        neighbor_x, neighbor_y = neighbor_state.index

        # Calculate Euclidean distance as a simple cost measure
        distance = ((current_x - neighbor_x) **2 + (current_y - neighbor_y) **2) **0.5

        # Additional penalty for proximity to obstacle
        proximity_penalty = 0.1 

        # Check if neighbor is close to an obstacle
        if self.binary_array[neighbor_x][neighbor_y] == 1:
            distance += proximity_penalty

        # Assume a cost of 1 per unit distance
        cost = distance

        return cost
        
    # Propagate cost changes to neighbors 
    def propagate(self, state):
        # Propagate cost changes to neighbors
        x, y = state.index

        # Determine if LiDAR data is valid neighbors
        valid_neighbors = []
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if dx == 0 and dy == 0:
                    continue # Skip the current state
                
                neighbor_x, neighbor_y = x + dx, y + dy

                # Check if the neighbor is within the bounds and is valid based on LiDAR data
                if 0 <= neighbor_x <self.rows and 0 <= neighbor_y < self.cols and self.is_valid(neighbor_x, neighbor_y):
                    neighbor_state = State(index=(neighbor_x, neighbor_y))
                    new_cost = state.cost + self.cost_to_move(state, neighbor_state)

                    self.update_cost(neighbor_state, new_cost)

    # Main loop of algorithm
    # - Extracts the state w/ the lowest cost from priority queue
    # - Continues until the goal state is reached or no more paths can be improved
    def replan(self):
        # Main loop
        while self.open_list:
            # Extract lowest cost state
            current_state = heapq.heappop(self.open_list)

            # Break if at goal
            if current_state.index == self.goal.index:
                break

            # Update cost and propagate information B4 next loop
            new_cost = current_state.cost + 1
            self.update_cost(current_state, new_cost)
            self.propagate(current_state)

    # For backtracking
    # Determine if next state to move is has the minimum cost
    def find_minimum_cost_neighbor(self, state):
        neighbors = [(state.index[0] + 1, state.index[1]),
                     (state.index[0] - 1, state.index[1]),
                     (state.index[0], state.index[1] + 1),
                     (state.index[0], state.index[1] -1)]

        valid_neighbors = [neighbor for neighbor in neighbors if self.is_valid(*neighbor)]

        # Find the neighbor w/ the minimum cost
        min_cost_neighbor = min(valid_neighbors, key=lambda neighbor: neighbor.cost)

        return State(index = min_cost_neighbor, cost = min_cost_neighbor.cost)

    # Backtracking from the goal state to start state
    # - To construct final path
    def extract_path(self):
        path = [self.start]
        current_state = self.start

        # Backtrack from the goal to the start to construct path
        while current_state.index != self.goal.index:
            next_state = self.find_minimum_cost_neighbor(current_state)
            path.append(next_state)
            current_state = next_state


        return path
    
