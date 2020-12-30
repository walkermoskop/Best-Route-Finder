#!/usr/local/bin/python3
# route.py : City oute finding solver
#
# Code by: Yating Jiang, Walker Moskop, David Ross
#

from queue import PriorityQueue
import sys
import time
import csv
from math import ceil, radians, cos, sin, asin, sqrt
import numpy as np


### Read all city lat/lon into a lookup dict
def read_city_gps():
    city_lookup = {}
    with open('city-gps.txt', 'r') as f:
        reader = csv.reader(f, delimiter=' ')
        for row in reader:
            city_lookup[row[0]] = [float(row[1]), float(row[2])]
    return city_lookup
   

### for a current city, return all the connecting cities
def succ(state):
    successors = [[(set(row[0:2]) - set([state])).pop(), float(row[2]),
                   float(row[3]), row[4]] for row in roads if state in row]
    return successors


### read road segments into a set of arrays
def read_road_segments():
    with open('road-segments.txt', 'r') as f:
        reader = csv.reader(f, delimiter = ' ')
        segments = [row for row in reader]
        return segments
    

### this distance helps convert lat/lon euclidean distance to miles, with
### assistance from code borrowed from this postf:
# https://stackoverflow.com/questions/15736995/how-can-i-quickly-estimate-the-distance-between-two-latitude-longitude-points
def calc_distance(curr_city_coords):
    c_lat, c_lon, e_lat, e_lon = map(radians,
                                     [curr_city_coords[0],
                                      curr_city_coords[1],
                                      end_lat,
                                      end_lon])
    
    ## haversine formula 
    dlat = e_lat - c_lat 
    dlon = e_lon - c_lon 
    a = sin(dlat/2)**2 + cos(c_lat) * cos(e_lat) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    # Radius of earth in miles is 3,958.8
    miles = 3958.8 * c
    ### shrink miles for cases where the segment distance is shorter than 
    ### gps distance
    return miles - 5


## cost function 
def h(proposed_city, segment_length, spd_lim, cost_func):
    # some locations might not have gps available. In such cases, we wil set
    # distance to zero to eliminate the possibility of overestimating
    try:
        city_coords = city_gps_lkup[proposed_city]
        distance = calc_distance(city_coords)
    except KeyError: distance = 0
    
    segment_time = (segment_length / (spd_lim + 5))
    est_time_to_goal = (distance / (max_spd_lim + 5))
    est_segment_accidents = 0.000001 * spd_lim * segment_length
    est_accidents_to_goal = 0.000001 * (min_spd_lim + 5) * distance
    est_segments_left = ceil(distance / max_segment_dist) - 1
        
    if cost_func == 'distance':
        ### important to return segment cost so that we can keep track of 
        ### aggregate cost along a path
        est_goal_cost, segment_cost = distance, segment_length 
        
    elif cost_func == 'time':
        est_goal_cost, segment_cost = est_time_to_goal, segment_time

    elif cost_func == 'cycling':
        est_goal_cost, segment_cost = est_accidents_to_goal, est_segment_accidents
        
    elif cost_func == 'segments':
        ### 1 is the cost of the segment
        est_goal_cost, segment_cost = est_segments_left, 1     
    ### to keep a running tally of segment time and segment estimated accidents,
    ### we need to pass them back to the solver function. segment miles and num.
    ### segments are already accessible in that function and don't need to be
    ### returned here
    return est_goal_cost, segment_cost, segment_time, est_segment_accidents, distance


def is_goal(state):
    return state == end_city


def solve(start_city, end_city, cost_func):
    
    fringe = PriorityQueue()
    
    ### the items needed in the fringe (depending on the cost function and
    ### whetherinclude:
    # the estimated cost for the move being evaluated h(s)
    # the aggregate cost (distance, time, etc.) incurred so far g(s)
    # number of segments taken so far
    # number of miles traveled so far
    # time expired so far
    # total expected accidents so far
    # the city being evaluated
    # the list of places visited
    fringe.put((0, 0, 0, 0, 0, 0 , start_city, []))
    
    ### initialize closed obj to help limit revisited states
    closed = []
    
    while not fringe.empty():
        (est_cost, cost_so_far, tot_segments, tot_miles, tot_time, tot_acc,
         state, route_so_far) = fringe.get()
        
        ## add items to closed after removing from fringe, per algo #3
        closed.append(state)
        
        if is_goal(state):
            return '{} {} {} {} {} {}'.format(
                tot_segments, int(tot_miles), round(tot_time,2), round(tot_acc,6),
                    start_city, ' '.join([x for x in route_so_far])
                    )
        
        for (s, dist, spd_lim, hwy) in succ(state):
            ### ignore items already in closed, per algo #3
            if s not in closed:
                
                ### calculate the estimated cost from the proposed successor to
                ### the final goal and the individual segment cost (regardless
                ### of cost function being used). Because we want to keep track
                ### of the aggregate time and aggregate expected accidents, we
                ### will retreive them from the h() function as well.
                cost_to_goal, segment_cost, s_time, s_acc, est_rem_dist = \
                            h(s, dist, spd_lim, cost_func)
                
                ### update the aggregate cost (g(s))
                s_agg_cost = cost_so_far + segment_cost
                ### estimated cost for a successor = cost_to_goal (h(s)) +
                ### cost incurred so far (s_agg_cost, or g(s))
                s_est_cost = cost_to_goal + s_agg_cost
                
                ### update summary stats for path before adding to fringe
                s_tot_segments = tot_segments + 1
                s_tot_miles = tot_miles + dist
                s_tot_time = tot_time + s_time
                s_tot_acc =  tot_acc + s_acc
                
                fringe.put((s_est_cost, s_agg_cost, s_tot_segments, s_tot_miles,
                            s_tot_time, s_tot_acc, s, route_so_far + [s,]))
        
    
    return False


if __name__ == "__main__":
        
    # start = time.time()
    if(len(sys.argv) != 4):
        raise(Exception("Error: 3 inputs required: [start] [end\ [cost-function]"))
    
    start_city, end_city, cost_func = sys.argv[1], sys.argv[2], sys.argv[3]
    
    ### retreive city lat/lons into a lookup dict
    city_gps_lkup = read_city_gps()
    
    start_city_coords = city_gps_lkup[start_city]

    # retreive goal city coords to use as global var    
    end_city_coords = city_gps_lkup[end_city]
    end_lat, end_lon = end_city_coords[0], end_city_coords[1]
    
    ### retreive road segements into a set of arrays
    roads = read_road_segments()
    
    ### retreive max_spd_lim in dataset
    max_spd_lim = int(max([x[3] for x in roads]))
    min_spd_lim = int(min([x[3] for x in roads]))
    
    max_segment_dist = np.mean([int(x[2]) for x in roads])
    
    # print("Solving...")
    route = solve(start_city, end_city, cost_func)
    print(route)
    
    # print(time.time() - start)