This BFS implementation, simulating a ship navigating through obstacle filled water to find a victim at sea,  allows revisiting previous positions but not in a step-by-step backtracking manner.
In reality, a ship would retrace its steps one at a time to a previous position. However, in this implementation, the algorithm may jump back to previous positions that are more than one step away, which is impossible in real life.
This is a limitation of the algorithm that I will address later on.
