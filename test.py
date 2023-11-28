# My Unit Test

import unittest
from DStarLite import DStarLite, State
from math import sqrt

class TestDStarLite(unittest.TestCase):
    def test_update_cost(self):
        # Create a DStarLite Instance
        start_state = State(index=(0,0))
        goal_state = State(index=(3,3))
        binary_array = [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ]

        dstar = DStarLite(start_state, goal_state, binary_array)

        # Create a State instance to test
        state = State(index=(1, 1), cost=3)

        # Perform an update with a lower cost
        dstar.update_cost(state, 2)

        # Assert that the cost has been upddated
        self.assertEqual(state.cost, 2)

        # Add more test cases for other functions

    # Test for Update_binary_array func
    def test_update_binary_array(self):
        #Create a DStar Instance
                # Create a DStarLite Instance
        start_state = State(index=(0,0))
        goal_state = State(index=(3,3))
        binary_array = [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ]
        
        dstar = DStarLite(start_state, goal_state, binary_array)

        dstar.update_binary_array(2, 2, 1)
        self.assertEqual(binary_array[2][2], 1)
        
        # Check for when the value is out of range
        dstar.update_binary_array(5, 5, 1)

    def test_cost_to_move(self):
        # Create a DStarLite instance
        start_state = State(index=(0, 0))
        goal_state = State(index=(3, 3))
        binary_array = [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ]
        dstar = DStarLite(start_state, goal_state, binary_array)

        # Create two states for testing (current_state and neighbor_state)
        current_state = State(index=(1, 1), cost=0)
        neighbor_state_clear = State(index=(2, 2), cost=0)
        neighbor_state_obstacle = State(index=(2, 3), cost=0)

        # Calculate cost for a clear path
        cost_clear = dstar.cost_to_move(current_state, neighbor_state_clear)

        # Calculate cost for a path with an obstacle
        cost_obstacle = dstar.cost_to_move(current_state, neighbor_state_obstacle)

        # Define expected values based on your cost calculation logic
        expected_cost_clear = 1.4142135623730951 # Replace w/ expected value
        expected_cost_obstacle = 2.23606797749979 # Replace w/ expected value

        # Assert that the cost calculations are approximately as expected
        self.assertEqual(cost_clear, expected_cost_clear)
        self.assertEqual(cost_obstacle, expected_cost_obstacle)

    def test_propagate(self):
        # Create a DStarLite instance
        start_state = State(index=(0, 0))
        goal_state = State(index=(3, 3))
        binary_array = [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ]
        dstar = DStarLite(start_state, goal_state, binary_array)

        # Set up a state to propagate from
        state_to_propagate = State(index=(1, 1), cost=3)

        # Call the propagate method
        dstar.propagate(state_to_propagate)

        # Verify that the costs of neighbors are updated as expected
        # Define the expected costs for neighbors based on your logic
        expected_cost_neighbor_1 = 3 + dstar.cost_to_move(state_to_propagate, State(index=(0, 1)))
        expected_cost_neighbor_2 = 3 + dstar.cost_to_move(state_to_propagate, State(index=(1, 0)))
        expected_cost_neighbor_3 = 3 + dstar.cost_to_move(state_to_propagate, State(index=(1, 2)))
        expected_cost_neighbor_4 = 3 + dstar.cost_to_move(state_to_propagate, State(index=(2, 1)))

        # Extract the neighbors of the propagated state
        neighbors = [
            State(index=(0, 1), cost=float('inf')),
            State(index=(1, 0), cost=float('inf')),
            State(index=(1, 2), cost=float('inf')),
            State(index=(2, 1), cost=float('inf')),
        ]

        # Assert that the costs of neighbors are updated as expected
        # If the initial cost is infinite, skip the comparison
        if neighbors[0].cost != float('inf'):
            self.assertEqual(neighbors[0].cost, expected_cost_neighbor_1)
        if neighbors[1].cost != float('inf'):
            self.assertEqual(neighbors[1].cost, expected_cost_neighbor_2)
        if neighbors[2].cost != float('inf'):
            self.assertEqual(neighbors[2].cost, expected_cost_neighbor_3)
        if neighbors[3].cost != float('inf'):
            self.assertEqual(neighbors[3].cost, expected_cost_neighbor_4)

    def test_replan(self):
        # Create a DStarLite instance
        start_state = State(index=(0, 0))
        goal_state = State(index=(3, 3))
        binary_array = [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ]
        dstar = DStarLite(start_state, goal_state, binary_array)

        # Perform initial planning
        dstar.replan()

        # Print the entire open list for debugging
        print("Open List:", [state.index for state in dstar.open_list])

        # Check that the algorithm stops when the goal state is reached
        self.assertEqual(dstar.open_list, [goal_state])

        # Check that the costs are updated appropriately
        expected_cost_to_goal = 3 * sqrt(2)  # Adjust based on your cost calculation logic
        self.assertNotIn(goal_state, dstar.open_list)
        self.assertAlmostEqual(goal_state.cost, expected_cost_to_goal, places=5)

        # Add these lines to print actual and expected costs
        print("Actual Cost:", goal_state.cost)
        print("Expected Cost:", expected_cost_to_goal)
    
if __name__ == '__main__':
    unittest.main()
