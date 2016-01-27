from math import inf, sqrt
from heapq import heappop, heappush
from turtledemo.chaos import line
from _ast import Str



def find_path(source_point,destination_point,mesh): #source_point is (x,y) pair, destination(x,y) pair , a data structure (fancy graph)
    path = []
    visited_nodes = []
    #print (mesh)    
    for box in mesh['boxes']:
        

        b = find_box(source_point, mesh)
        if b != None :
            visited_nodes.append(b)
            
        b = find_box(destination_point, mesh)
        if b != None:
            visited_nodes.append(b)

            
    
    path, visited_nodes = dijkstras_shortest_path(source_point, destination_point, mesh, navigation_edges, visited_nodes)    
    #print ("path: " + str(path))
    #print(visited_nodes)
    return (path , visited_nodes)  # path is a list of points like ((x1,y1),(x2,y2)). 
# visited nodes = a list of boxes explored by your algorithm identified by their bounds (x1,x2,y1,y2) 

def find_box(point, mesh):
    for box in mesh['boxes']:
        if point[0] >= box[0] and point[0] <= box[1] \
            and point[1] >= box[2] and point[1] <= box[3]:
            return box
    return None

def dijkstras_shortest_path(initial_position, destination, graph, adj, visited_nodes):
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
    initial_box = find_box(initial_position, graph)
    destination_box = find_box(destination, graph)
    distances = {initial_box: 0}           # Table of distances to cells 
    previous_cell = {initial_box: None}    # Back links from cells to predecessors
    queue = [(0, initial_box)]             # The heap/priority queue used

    # Initial distance for starting position
    distances[initial_position] = 0

    while queue:
        # Continue with next min unvisited node
        current_distance, current_box = heappop(queue)
        print ("cur_node: " +str(current_box))
        
        # Early termination check: if the destination is found, return the path
        if current_box == destination_box:
            node = destination_box
            path = []
            detail_point = []
            prev_point = initial_position
            while node is not None:
                if previous_cell[node] != None:
                    line_end = (0,0)
                    line_start = (0,0)
                    if node == destination_box:
                        line_end = destination
                        #prev_point = 
                    else:
                        #line_end = ((node[0] + node[1])/2,(node[2]+node[3])/2)
                        line_end = next_point(prev_point, node, previous_cell[node])
                    if previous_cell[node] == initial_box:
                        line_start = initial_position
                    else:
                        #line_start = ((previous_cell[node][0] + previous_cell[node][1])/2, (previous_cell[node][2] + previous_cell[node][3])/2)
                        if node == destination_box:
                            line_start = next_point(prev_point, node, previous_cell[node])
                        else:
                            line_start = next_point(prev_point, previous_cell[node], previous_cell[previous_cell[node]])
                        
                        
                        
                    #print("Line start: " + str(line_start))
                    #print("line end:" + str(line_end))
                    #print("path: " + str(path))
                    visited_nodes.append(node)
                    path.append((line_start,line_end))
                    prev_point = line_end
                node = previous_cell[node]
            print ("djtra: " + str(path))

            return (path[::-1], visited_nodes) 

        # Calculate tentative distances to adjacent cells
        for adjacent_node, edge_cost in adj(graph, current_box):
            new_distance = current_distance + edge_cost

            if adjacent_node not in distances or new_distance < distances[adjacent_node]:
                # Assign new distance and update link to previous cell
                distances[adjacent_node] = new_distance
                previous_cell[adjacent_node] = current_box
                heappush(queue, (new_distance, adjacent_node))
                    
    # Failed to find a path
    print("Failed to find a path from", initial_position, "to", destination)
    return None

def next_point(prev_point, current_box, destination_box):
    print("prev: " +str(prev_point))
    print("current_box: " + str(current_box))
    print("destination_box: " + str(destination_box))
    border_p1 =(max(current_box[0], destination_box[0]), max(current_box[2],destination_box[2]))
    border_p2 =(min(current_box[1], destination_box[1]), min(current_box[3], destination_box[3]))
    
    mid_point = ((border_p2[0] + border_p1[0])/2, (border_p2[1] + border_p1[1])/2)
    print ("midpoint: " + str(mid_point))
    
    return mid_point
    

def navigation_edges(mesh, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    #print ("n_e: " + str(level["adj"][cell]))
    cells_and_costs = []
    for box in mesh['adj'][cell]:
        area = 1/(box[1] - box[0]) * (box[3] - box[2])
        cells_and_costs.append((box, area))
    
    return cells_and_costs
    
'''
    boxes = level['boxes']
    adj = level['adj']
    adjacent_nodes = {}

    for neighbors in adj[cell]:
        next_cell = (x + delta_x, y + delta_y)
        if next_cell != cell and next_cell in spaces:
            distance = sqrt(delta_x ** 2 + delta_y ** 2)
            adjacent_nodes[next_cell] = distance * (spaces[cell] + spaces[next_cell])/2
            adjacent_nodes.it
    return adjacent_nodes.items()'''
