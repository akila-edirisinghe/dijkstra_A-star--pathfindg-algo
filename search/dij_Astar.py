import heapq
from search.algorithms import *
from search.map import *


def main_dik(start_state, goal_state, map_instance):
    open_list = []
    closed_list = {}
    
    heapq.heappush(open_list, start_state)
    start_state.set_cost(0)
    closed_list = {start_state.state_hash():start_state} 
    expanded_nodes = 0
    while len(open_list) != 0:           
        state = heapq.heappop(open_list)
        if state.__eq__(goal_state):
            return state.get_cost(), expanded_nodes
        children = map_instance.successors(state)
        expanded_nodes +=1
        for child in children:
            hash_child_value = child.state_hash()
            child_cost = child.get_g()
            child.set_cost(child_cost)
            if hash_child_value not in closed_list:  
                heapq.heappush(open_list,child)
                closed_list[hash_child_value]=child
            elif hash_child_value in closed_list and child.__lt__(closed_list[hash_child_value]):  
                closed_list[hash_child_value].set_cost(child_cost)
                heapq.heappush(open_list,child)
           
        heapq.heapify(open_list)
    return -1,expanded_nodes
            
def main_astar(start_state, goal_state, map_instance):
    open_list = []
    closed_list = {}
    heapq.heappush(open_list, start_state)
    distance = octile_distance(start_state,goal_state)
    f_value = 0 + distance
    start_state.set_cost(f_value)  
    closed_list = {start_state.state_hash():start_state} 
    expanded_nodes = 0
    while len(open_list) != 0:     
        
        state = heapq.heappop(open_list)  
        if state.__eq__(goal_state):
            return state.get_cost(), expanded_nodes
        children = map_instance.successors(state)
        expanded_nodes +=1
        
        for child in children:
            h_value = octile_distance(child,goal_state)
            g_value = child.get_g()
            f_value = g_value + h_value
            hash_child_value = child.state_hash()
            child.set_cost(f_value)
            
            if hash_child_value not in closed_list:   
                heapq.heappush(open_list,child)
                closed_list[hash_child_value]=child
                
            if hash_child_value in closed_list and child.__lt__(closed_list[hash_child_value]): 
                closed_list[hash_child_value].set_cost(f_value)
                closed_list[hash_child_value].set_g(g_value)
                heapq.heappush(open_list,child)
                
        heapq.heapify(open_list) 
                
        
    return -1,expanded_nodes
    
                
def octile_distance(child_state,goal_state):
    change_x = abs(child_state.get_x() - goal_state.get_x())
    change_y = abs(child_state.get_y() - goal_state.get_y())
    octile = (1.5*min(change_x,change_y) + abs(change_x - change_y))
    return int(octile)
             
                
                
                
                
        
        
        
            
    
    
    
    

    
    
    