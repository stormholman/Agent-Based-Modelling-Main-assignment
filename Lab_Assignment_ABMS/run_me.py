"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""
import random
import os
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
from single_agent_planner import calc_heuristics
from visualization import map_initialization, map_running
from Aircraft import Aircraft
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from cbs import run_CBS

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#Parameters that can be changed:
simulation_time = 20
planner = "CBS" #choose which planner to use (currently only Independent is implemented)

#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.5  #set at 0.1 as default

#%%Function definitions
def import_layout(nodes_file, edges_file):
    """
    Imports layout information from xlsx files and converts this into dictionaries.
    INPUT:
        - nodes_file = xlsx file with node input data
        - edges_file = xlsx file with edge input data
    RETURNS:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - start_and_goal_locations = dictionary with node ids for arrival runways, departure runways and gates 
    """
    gates_xy = []   #lst with (x,y) positions of gates
    rwy_dep_xy = [] #lst with (x,y) positions of entry points of departure runways
    rwy_arr_xy = [] #lst with (x,y) positions of exit points of arrival runways
    
    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)
    
    #Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"],row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set()
                           }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties
        
        #Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"],row["y_pos"]))

    #Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {"gates": gates_xy, 
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy}
    
    #Create edges_dict from df_edges
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"],row["to"])
        from_node =  edge_id[0]
        to_node = edge_id[1]
        start_end_pos = (nodes_dict[from_node]["xy_pos"], nodes_dict[to_node]["xy_pos"])
        edge_properties = {"id": edge_id,
                           "from": row["from"],
                           "to": row["to"],
                           "length": row["length"],
                           "weight": row["length"],
                           "start_end_pos": start_end_pos
                           }
        edges_dict[edge_id] = edge_properties
   
    #Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)  
    
    return nodes_dict, edges_dict, start_and_goal_locations

def create_graph(nodes_dict, edges_dict, plot_graph = True):
    """
    Creates networkX graph based on nodes and edges and plots 
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """
    
    graph = nx.DiGraph() #create directed graph in NetworkX
    
    #Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(node, 
                       node_id = nodes_dict[node]["id"],
                       xy_pos = nodes_dict[node]["xy_pos"],
                       node_type = nodes_dict[node]["type"])
        
    for edge in edges_dict.keys():
        graph.add_edge(edge[0], edge[1], 
                       edge_id = edge,
                       from_node =  edges_dict[edge]["from"],
                       to_node = edges_dict[edge]["to"],
                       weight = edges_dict[edge]["length"])
    
    #Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)
        
    return graph

#%% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)

aircraft_lst = []   #List which can contain aircraft agents

if visualization:
    map_properties = map_initialization(nodes_dict, edges_dict) #visualization properties

# =============================================================================
# 1. While loop and visualization
# =============================================================================
 
#Start of while loop    
running=True
escape_pressed = False
time_end = simulation_time
dt = 0.5 #should be factor of 0.5 (0.5/dt should be integer)
clock = pg.time.Clock()
t = 0
id = -1
evodd = 1
id_dummy = -1
acc = 1

print("Simulation Started")
t_list = [0,2,4,6,8]
# Spawn aircraft for this timestep (random process)
for i in range(t_list[-1]+1):
    if evodd % 2 == 0:
        id_dummy += 1
        ac_a = Aircraft(id_dummy, 'A', random.choice((37, 38)), random.choice((97, 34, 35, 36, 98)), i, nodes_dict)
        # print('id',id)
        aircraft_lst.append(ac_a)
        evodd += 1

    else:
        id_dummy += 1
        ac_d = Aircraft(id_dummy, 'D', random.choice((97, 34, 35, 36, 98)), random.choice((1, 2)), i, nodes_dict)
        # print('id', id)
        aircraft_lst.append(ac_d)
        evodd += 1

constraints = []
constraint_table_all = {}
root = {'cost': 0,
        'constraints': [],
        'paths': [],
        'collisions': []}
open_list = []
num_of_generated = 0
num_of_expanded = 0

while running:
    t = round(t,2)

    # Spawn aircraft for this timestep (random process)
    if t in t_list:

        if acc % 2 == 0:
            id += 1
            ac_a = Aircraft(id, 'A', random.choice((37, 38)), random.choice((97, 34, 35, 36, 98)), t, nodes_dict)
            # print('id',id)

            acc += 1

        else:
            id += 1
            ac_d = Aircraft(id, 'D', random.choice((97, 34, 35, 36, 98)), random.choice((1, 2)), t, nodes_dict)
            # print('id', id)

            acc += 1

    #Check conditions for termination
    if t >= time_end or escape_pressed: 
        running = False
        pg.quit()
        print("Simulation Stopped")
        break 
    
    #Visualization: Update map if visualization is true
    if visualization:
        current_states = {} #Collect current states of all aircraft
        for ac in aircraft_lst:
            if ac.status == "taxiing":
                current_states[ac.id] = {"ac_id": ac.id,
                                         "xy_pos": ac.position,
                                         "heading": ac.heading}
        escape_pressed = map_running(map_properties, current_states, t)
        timer.sleep(visualization_speed)
        
      


    # Spawn aircraft for this timestep
    # if t == 1:
    #     # As an example we will create one aicraft arriving at node 37 with the goal of reaching node 36
    #     id += 1
    #     ac_a = Aircraft(id, 'A', 37, 36, t, nodes_dict)
    #     aircraft_lst.append(ac_a)
    #
    #     id += 1
    #     ac_d = Aircraft(id, 'D', 36, 37, t, nodes_dict)
    #     aircraft_lst.append(ac_d)
    #
    #     id += 1
    #     ac_e = Aircraft(id, 'D', 97, 2, t, nodes_dict)
    #     aircraft_lst.append(ac_e)
    #
    #     id += 1
    #     ac_f = Aircraft(id, 'D', 34, 1, t, nodes_dict)
    #     aircraft_lst.append(ac_f)
    #
    #     id += 1
    #     ac_g = Aircraft(id, 'D', 38, 1, t, nodes_dict)
    #     aircraft_lst.append(ac_g)

        # id += 1
        # ac_h = Aircraft(id, 'D', 98, 1, t, nodes_dict)
        # aircraft_lst.append(ac_h)
        #
        # id += 1
        # ac_h = Aircraft(id, 'D', 35, 1, t, nodes_dict)
        # aircraft_lst.append(ac_h)


    
    #Do planning 
    if planner == "Independent":     
        #if t == 1: #(Hint: Think about the condition that triggers (re)planning) 
        run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, )
    elif planner == "Prioritized":
        run_prioritized_planner(aircraft_lst, nodes_dict, heuristics, t, constraints, constraint_table_all)
    elif planner == "CBS":
        run_CBS(aircraft_lst, nodes_dict, heuristics, t, root, open_list, num_of_generated, num_of_expanded)
    #elif planner == -> you may introduce other planners here
    else:
        raise Exception("Planner:", planner, "is not defined.")
                       
    #Move the aircraft that are taxiing
    for ac in aircraft_lst: 
        if ac.status == "taxiing":
            ac.move(dt, t)
                           
    t = t + dt
          
# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?
