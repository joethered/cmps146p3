from math import inf, sqrt
from heapq import heappop, heappush
from turtledemo.chaos import line
from _ast import Str
import collections



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
    fwd_distances = {initial_box: 0}           # Table of fwd_distances to cells 
    back_distances = {destination_box: 0}
    fwd_previous_cell = {initial_box: None}    # Back links from cells to predecessors
    back_previous_cell = {destination_box: None}
    queue = [(0, initial_box, destination_box)]             # The heap/priority queue used
    heappush(queue, (0, destination_box, initial_box))

    # Initial distance for starting position
    fwd_distances[initial_position] = 0
    back_distances[destination] = 0
    last_checked_box = None
    
    while queue:
        # Continue with next min unvisited node
        current_distance, current_box, current_direction = heappop(queue)
        #print (current_distance == fwd_distances[current_box])
        #print ("cur_node: " +str(current_box))
        
        # Early termination check: if the destination is found, return the path
        
        if current_direction == destination_box and current_box == destination_box:

            node1 = destination_box
            node2 = initial_box
            path = []
            
            prev_point1 = initial_position
            prev_point2 = destination
            
            while node1 is not None:
                
                line1_start, line1_end = find_lines(initial_box, destination_box, initial_position, destination, node1, fwd_previous_cell, prev_point1)
                  
                visited_nodes.append(node1)
                path.append((line1_start,line1_end))
                prev_point1 = line1_end
                node1 = fwd_previous_cell[node1]
                
            while node2 is not None:
                line2_start, line2_end = find_lines(destination_box, initial_box, destination, initial_position, node2, back_previous_cell, prev_point2)
                visited_nodes.append(node2)
                path.append((line2_start,line2_end))
                prev_point2 = line2_end 
                node2 = back_previous_cell[node2]
            #print ("djtra: " + str(path))
            #print("path: " + str(path))
            return (path[::-1], visited_nodes) 
        # Calculate tentative fwd_distances to adjacent cells
        if (current_direction == destination_box):
            for adjacent_node, edge_cost in adj(graph, current_box):
                new_distance = current_distance + heuristic(destination_box, adjacent_node)
        
                if adjacent_node not in fwd_distances or new_distance < fwd_distances[adjacent_node]:
                    # Assign new distance and update link to previous cell
                    fwd_distances[adjacent_node] = new_distance
                    fwd_previous_cell[adjacent_node] = current_box
                    heappush(queue, (new_distance, adjacent_node, destination_box))
                    #visited_nodes.append(adjacent_node)
    
               
        else:
            for adjacent_node, edge_cost in adj(graph, current_box):
                new_distance = current_distance + heuristic(initial_box, adjacent_node)
        
                if adjacent_node not in back_distances or new_distance < back_distances[adjacent_node]:
                    # Assign new distance and update link to previous cell
                    back_distances[adjacent_node] = new_distance
                    back_previous_cell[adjacent_node] = current_box
                    heappush(queue, (new_distance, adjacent_node, initial_box))
                    #visited_nodes.append(adjacent_node)
                    
    # Failed to find a path
    print("Failed to find a path from", initial_position, "to", destination)
    return None

def find_lines(initial_box, destination_box, initial_position, destination, node, previous_cell, prev_point):
    line_end = (0,0)
    line_start = (0,0)
    #print (str(initial_box) + " " + str(destination_box) + " " + str(fwd_previous_cell[node]))
    #print(fwd_previous_cell)
    #print(node)
    if destination_box == initial_box:
        #print("single box")
        line_start = initial_position
        line_end = destination
    elif previous_cell[node] != None or previous_cell[node] == initial_box:
        
        if node == destination_box:
            #print("destination box")
            line_start = destination
            line_end = next_point(destination, node, previous_cell[node])
        else:
            #print("the rest")
            line_start = prev_point
            line_end = next_point(prev_point, node, previous_cell[node])
    else:
        #print("initial box")
        line_start = prev_point
        line_end = initial_position

    return (line_start, line_end)

def next_point(prev_point, current_box, destination_box):
    #print("prev: " +str(prev_point))
    #print("current_box: " + str(current_box))
    #print("destination_box: " + str(destination_box))
    border_p1 =(max(current_box[0], destination_box[0]), max(current_box[2],destination_box[2]))
    border_p2 =(min(current_box[1], destination_box[1]), min(current_box[3], destination_box[3]))
    
    point = (0,0)
    #Vertical
    if (border_p1[0] == border_p2[0]):
        #py >= max(by1, by2)
        if (prev_point[1] >= max(border_p1[1],border_p2[1])):
            #p = bx1, max(by1,by2)
            point = (border_p1[0], max(border_p1[1],border_p2[1]))
        #py <= min(by1, by2)
        elif (prev_point[1] <= min(border_p1[1],border_p2[1])):
            #p = bx1, min(by1,by2)
            point = (border_p1[0], min(border_p1[1],border_p2[1]))
        else:
            point = (border_p1[0], prev_point[1])
            
    #Horizontal
    else:
        #px >= max(bx1, bx2)
        if (prev_point[0] >= max(border_p1[0],border_p2[0])):
            #p = max(bx1,bx2), 
            point = (max(border_p1[0],border_p2[0]), border_p1[1])
        elif (prev_point[0] <= min(border_p1[0],border_p2[0])):
            point = (min(border_p1[0],border_p2[0]), border_p1[1])
        else:
            point = (prev_point[0], border_p1[1])
    
    #mid_point = ((border_p2[0] + border_p1[0])/2, (border_p2[1] + border_p1[1])/2)
 
    #print ("midpoint: " + str(mid_point))
    
    return point

def heuristic(goal_box, cur_box): 
    mid_point_goal = ((goal_box[0] + goal_box[1])/2, (goal_box[2] + goal_box[3])/2)
    mid_point_cur = ((cur_box[0] + cur_box[1])/2, (cur_box[2] + cur_box[3])/2)
    return sqrt(pow(mid_point_goal[0] - mid_point_cur[0], 2) + pow(mid_point_goal[1] - mid_point_cur[1],2))
    #return abs(mid_point_goal[0]-mid_point_cur[0]) + abs(mid_point_goal[1]-mid_point_cur[1])

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
