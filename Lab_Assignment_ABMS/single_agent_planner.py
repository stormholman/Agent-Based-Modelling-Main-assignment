"""
This file contains single agent planner functions  that can be used by other planners.
Consider functions in this file as supporting functions.
"""

import heapq
import networkx as nx

def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the
    #               is_constrained function.
    constraint_table = {}
    # print('constraints',constraints)
    for i in constraints:
        # print('constraint', i)
        if agent == i['aircraft']:
            if i['timestep'] in constraint_table:
                constraint_table[i['timestep']].append(i['node'])
            else:
                constraint_table[i['timestep']] = [i['node']]
    # print('constraint_table', constraint_table)
    return constraint_table


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    #curr_loc: just node, next_loc: just node, next_time: just time
    if next_time not in constraint_table:
        # print('nxtt in constrtabkle', True)
        return False
    constraintsarr = constraint_table[next_time]
    for i in range(len(constraintsarr)):         #if the timestep is in the constraint_table, then we check the loc
        constraint = constraintsarr[i]
        if len(constraint) == 1:   #if the length of the loc is 1, then it is edge constraint
            if constraint[0] == next_loc:
                return True
        elif len(constraint) == 2:      #if the length of the loc is 2, then it is vertex constraint
            if constraint[0] == curr_loc and constraint[1] == next_loc:
                return True
    return False


def calc_heuristics(graph, nodes_dict):
    """
    Calculates the exact heuristic dict (shortest distance between two nodes) to be used in A* search.
    INPUT:
        - graph = networkX graph object
        - nodes_dict = dictionary with nodes and node properties
    RETURNS:
        - heuristics = dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
    """
    
    heuristics = {}
    for i in nodes_dict:
        heuristics[nodes_dict[i]["id"]] = {}
        for j in nodes_dict:
            path, path_length = heuristicFinder(graph, nodes_dict[i]["id"], nodes_dict[j]["id"])
            if path == False:
                pass
            else:
                heuristics[nodes_dict[i]["id"]][nodes_dict[j]["id"]] = path_length
    return heuristics

def heuristicFinder(graph, start_node, goal_node):
    """
    Finds exact distance between start_node and goal_node using the NetworkX graph.
    INPUT:
        - graph = networkX graph object
        - start_node, goal_node [int] = node_ids of start and goal node
    RETURNS:
        - path = list with node_ids that are part of the shortest path
        - path_length = length of the shortest path
    """
    try:
        path = nx.dijkstra_path(graph, start_node, goal_node, weight="weight")
        path_length = nx.dijkstra_path_length(graph, start_node, goal_node)
    except:
        path = False
        path_length = False
        raise Exception('Heuristic cannot be calculated: No connection between', start_node, "and", goal_node)
    return path, path_length


def simple_single_agent_astar(nodes_dict, from_node, goal_node, heuristics, time_start, constraint_table):
    # def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time. 
        - Hint: do you need more inputs?
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep) pairs -> example [(37, 1), (101, 2)]. Empty list if success == False.
    """
    
    from_node_id = from_node
    goal_node_id = goal_node
    time_start = time_start
    # constraint_table = build_constraint_table(constraints, agent)
    # constraint_table = {5: [[14]]}
    # print('constrainttable',constraint_table)
    # constaints_agents = {'agent':agent, 'constrain_table':constraint_table}
    # print('constraint_agents',constaints_agents)

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr)

        list_next_nodes = list(nodes_dict[curr['loc']]["neighbors"])
        # list_next_nodes.append(curr['loc'])
        # print('currloc',curr['loc'])
        # print('list next node', list_next_nodes)
        for neighbor in list_next_nodes:
            # print('neighbor', neighbor)
            # print('currloc', curr['loc'])
            # implement constraints here!!
            # print('currloc', curr['loc'])
            # print('nextloc', neighbor)
            # print('currtime',curr['timestep'])
            # print('timestep,curr['timestep'])
            # if constaints_agents['agent'] == agent:
            #     constraint_table = constaints_agents['constrain_table']
            # else:
            #     constraint_table = {}
            # print('constrainttable',constraint_table)

            if is_constrained(curr['loc'], neighbor, curr['timestep']+0.5, constraint_table):
                continue

            # if child_loc[1] >= len(my_map[0]) or child_loc[0] >= len(my_map):
            #     continue
            # if child_loc[1] < 0 or child_loc[0] < 0:
            #     continue

            child = {'loc': neighbor,
                    'g_val': curr['g_val'] + 0.5,
                    'h_val': heuristics[neighbor][goal_node_id],
                    'parent': curr,
                    'timestep': curr['timestep'] + 0.5}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
    print("No path found, "+str(len(closed_list))+" nodes visited")
    return False, [] # Failed to find solutions

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location

def get_path(goal_node):
    """Construct path if goal node is reached"""
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['timestep']))
        curr = curr['parent']
    path.reverse()
    #print(path)
    return path