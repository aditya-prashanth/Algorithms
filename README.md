This BFS implementation, simulating a USV navigating through obstacle filled water to find a victim at sea,  allows revisiting previous positions but not in a step-by-step backtracking manner.
In reality, a ship would retrace its steps one at a time to a previous position. However, in this implementation, the algorithm may jump back to previous positions that are more than one step away, which is impossible in real life.
This is a limitation of the algorithm that I will address later on.

End goal: The vision detection system will pinpoint if an area is non-navigable or has the target (the victim). Given each of the USVS are able to communicate with one another, hopefully we can create a grid pattern like the one in this implementation such that area's that are non-navigable or have already been visited are marked accordingly and the remaining USVs can use a swarm theory or search pattern approach to discover the remaining squares and find the target.

Considerations: 

- Turning a USV 90 degrees suddenly as done in my approach isn't feasible or efficient so a different search algorithm must be created accordingly.

- Depending on weather conditions we can't guarantee communication between all USVs

- This approach may work for multiple USVs if there is some amount of time between each launch, however in the case of this implementation there would be an error at launch as they all immediately try to follow the same path

- It is possible that our USV misses the victim in a given square of the search area, as such the algorithm needs to distinguish between checked and non-navigable areas so that USVs can recheck navigated squares

- This finally is only considering the search patten algorithm, it does not account for the integration between systems

Notes:

- Look into Hamiltonian and Eulerian Paths
