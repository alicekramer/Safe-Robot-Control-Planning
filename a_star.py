from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely import affinity
import numpy as np
from copy import deepcopy
from math import sqrt

import heapq

class PriorityQueue:
    ...
    def __init__(self):
        self.elements = []
        self.element_set = set()  # New set of elements

    def put(self, priority, item):
        heapq.heappush(self.elements, (priority, item))
        self.element_set.add(item)  # Add the item to the set

    def get(self):
        item = heapq.heappop(self.elements)[1]
        self.element_set.remove(item)  # Remove the item from the set
        return item

    def __contains__(self, item):
        return item in self.element_set  # Check the set instead of the list


class MotionPrimitive:
    """ 
    Defines cost & parent node of the motion primitive
    """
    def __init__(self, reachable_tube):
        self.reachable_tube = reachable_tube
        self.parent = None
        self.cost = 0

def polyhedron_centroid(polyhedron):
    """
    Calculate the centroid of a polyhedron
    Output: centroid (np.array)
    
    """
    centroid = polyhedron.centroid
    return centroid

def distance_heuristic(next,goal):
    """ 
    Calculate the heuristic cost from next to goal
    Input: next (Point()), goal (np.array)
    Output: heuristic cost (float)
    """
    # next and goal either Point() or centroid of Polygon
    
    distance = sqrt((next.x - goal.x)**2 + (next.y - goal.y)**2)
    return distance

def reach_goal_check(end_centroid_rt, goal):

    distance = distance_heuristic(end_centroid_rt, goal)
    if distance < 0.2:
        print("Goal reached!")
        return True
    else:
        return False
    

def translate_poly(rot_primitives_2D, pnext):
    """ 
    Take in end_list_poly and translate it to the next point chosen based off of heuristic
    Input: poly_list (list), current point (array), next point (array)
    Output: translated_poly_list (list)
    """
    #pnext either Point() or centroid of polygon
    x = pnext.x
    y = pnext.y

    trans_primitives_2D = []
    for reach_tube in rot_primitives_2D:
        reach_tube_trans = []
        for polygon in reach_tube:
            poly_translated = affinity.translate(polygon, xoff=x, yoff=y)
            reach_tube_trans.append(poly_translated)
        trans_primitives_2D.append(reach_tube_trans)

    return trans_primitives_2D

def collision_check(trans_primitives_2D, obstacles_shapely):
    collfree_primitives_2D = []
    for primitive in trans_primitives_2D:
        collision_detected = False
        for polygon in primitive:
            for obstacle in obstacles_shapely:
                if polygon.intersects(obstacle):
                    collision_detected = True
                    break
            if collision_detected:
                break
        if not collision_detected:
            collfree_primitives_2D.append(primitive)
    return collfree_primitives_2D

    
    
def subset_check(current_last_poly, rot_primitives_2D, current_end_centroid, obstacles_shapely):
    """ 
    Check if the last polyhedron in current is contained in final polyhedron of next
    If True then the entire reachable tube is feasible as neighbour

    Inputs:
    Current: list of polyhedrons (reachable tube) or the start array
    Current_last: last polyhedron in current reachable tube
    Rot_primitives_2D: list of all rotated motion primitives (reachable tubes)
    Current_end_centroid: centroid of the last polyhedron in current reachable tube

    Output: list of feasible motion primitives as MotionPrimitive class
    """
    ### TRANSLATE FIRST ###
    trans_primitives_2D = translate_poly(rot_primitives_2D, current_end_centroid)
    ### CHECK IF LAST OF CURRENT IS CONTAINED INSIDE FIRST OF NEXT ###
    collfree_primitives_2D = collision_check(trans_primitives_2D, obstacles_shapely)
    feasible_MP = []
    print("Total collision free tubes:",len(collfree_primitives_2D))
    if len(collfree_primitives_2D) == 0:
        return feasible_MP
    for collfree_primitive in collfree_primitives_2D:
        next_first = collfree_primitive[0]

        if next_first.contains(current_last_poly):
            next_tube_class = MotionPrimitive(collfree_primitive)
            feasible_MP.append(next_tube_class)
    print("Feasible tubes",len(feasible_MP))
    return feasible_MP
    

def astar_2d(start_array, goal_array, rot_primitives_2D, obstacles_shapely):
    goal = Point(goal_array)
    start = Point(start_array)
    max_iterations = 10000
    iterations = 0
    open_set = PriorityQueue()
    start = MotionPrimitive(start) # start element a numpy array
    start.cost = 0
    open_set.put(0, start) # cost, np.array

    while len(open_set.element_set)>0 and iterations < max_iterations:
        # assign current to the item in the open_set that has minimum cost
        current_tube_class = open_set.get() # Motionprimitive class
        current_tube = current_tube_class.reachable_tube # current reachable tube (list of polyhedrons if not start)
        if np.all(current_tube == Point(start_array)):
            # if start array
            current_end_centroid = current_tube
            current_last_poly = current_tube
        else:
            # if polyhedron

            current_end_centroid =(current_tube[-1].centroid)
            current_last_poly = current_tube[-1]
        # check if current is  close enough to distance, if yes then terminate
        if reach_goal_check(current_end_centroid, goal):
            break
        # check for nearest neighbout of current in the open_set (subset & obstacle check)
        feasible_rot_primitives = subset_check(current_last_poly, rot_primitives_2D, current_end_centroid, obstacles_shapely)
        # for each feasible_rot_primitive assign cost and parent
        for next_primitive in feasible_rot_primitives:
            new_cost= current_tube_class.cost + next_primitive.cost
            if next_primitive not in open_set or new_cost < next_primitive.cost:
                next_primitive.cost = new_cost
                # h_cost =  distance_heuristic(next_primitive.reachable_tube[-1].centroid, goal)
                priority = new_cost + distance_heuristic(next_primitive.reachable_tube[-1].centroid, goal)
                open_set.put(priority, next_primitive)
                next_primitive.parent = current_tube_class

        iterations += 1
        
    
    print("iterations", iterations)
    path = []
    while current_tube_class is not None:
        path.append(current_tube_class.reachable_tube)
        current_tube_class = current_tube_class.parent
        print(path)
    path.reverse()

    return path
