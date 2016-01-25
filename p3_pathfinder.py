from math import inf, sqrt
from heapq import heappop, heappush



def find_path(source_point,destination_point,mesh): #source_point is (x,y) pair, destination(x,y) pair , a data structure (fancy graph)
    path = []
    visited_nodes = []
    
    
    return (path , visited_nodes) # path is a list of points like ((x1,y1),(x2,y2)). 
#visited nodes = a list of boxes explored by your algorithm identified by their bounds (x1,x2,y1,y2) 


def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """

    distances = {initial_position: 0}           # Table of distances to cells 
    previous_cell = {initial_position: None}    # Back links from cells to predecessors
    queue = [(0, initial_position)]             # The heap/priority queue used

    # Initial distance for starting position
    distances[initial_position] = 0

    while queue:
        # Continue with next min unvisited node
        current_distance, current_node = heappop(queue)
        
        # Early termination check: if the destination is found, return the path
        if current_node == destination:
            node = destination
            path = [node]
            while node is not None:
                path.append(previous_cell[node])
                node = previous_cell[node]
            return path[::-1]

        # Calculate tentative distances to adjacent cells
        for adjacent_node, edge_cost in adj(graph, current_node):
            new_distance = current_distance + edge_cost

            if adjacent_node not in distances or new_distance < distances[adjacent_node]:
                # Assign new distance and update link to previous cell
                distances[adjacent_node] = new_distance
                previous_cell[adjacent_node] = current_node
                heappush(queue, (new_distance, adjacent_node))
                    
    # Failed to find a path
    print("Failed to find a path from", initial_position, "to", destination)
    return None
