"""
Implement CBS here!!
"""

import time as timer
from Aircraft import Aircraft
import heapq
from single_agent_planner import get_location, astar_CBS, get_sum_of_cost

def seq(start, stop, step=1):
    n = int(round((stop - start)/float(step)))
    if n > 1:
        return([start + step*i for i in range(n+1)])
    elif n == 1:
        return([start])
    else:
        return([])

def push_node(open_list, node, num_of_generated):
    print('psh node')
    print('open list', open_list)
    print('node',node)
    print('num of generated', num_of_generated)
    heapq.heappush(open_list, (node['cost'], len(node['collisions']), num_of_generated, node))
    print("Generate node {}".format(num_of_generated))
    num_of_generated += 1


def pop_node(open_list, num_of_expanded):
    _, _, id, node = heapq.heappop(open_list)
    print("Expand node {}".format(id))
    num_of_expanded += 1
    return node

def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    col = dict()
    total_time = max(len(path1), len(path2))

    for time in seq(0,total_time/2,0.5):
        # print('t',time)
        # print('getl1', get_location(path1, time)[0])
        if get_location(path2, time - 0.5) == get_location(path1, time) and get_location(path1, time -0.5) == get_location(path2, time) and time > 0:
        #     # print('col1',True)
            col = {'node' : [get_location(path2, time-0.5), get_location(path2, time)], 'timestep' : time} # edge col
        if get_location(path1, time) == get_location(path2, time):
        # if time == 5:
        #     print('getloc', [get_location(path1, time)])
            col = {'node': [get_location(path1, time)], 'timestep': time} # vertex col
            # print('col2', True)
    # print('listcol', col)
    return col


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    # print('paths', paths)
    num_agents = len(paths)
    list_col = []
    for first_agent in range(num_agents - 1):
        for sec_agent in range(first_agent + 1, num_agents):
            loc_col = detect_collision(paths[first_agent], paths[sec_agent])
            if loc_col:
                list_col.append({'first_agent': first_agent, 'second_agent': sec_agent, 'node': loc_col['node'], 'timestep': loc_col['timestep']})
    # print('listcol', list_col)
    return list_col


def standard_splitting(col):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    timestep = col['timestep']
    location = col['node']
    if len(location) == 1: #vertex col
        c1 = {'aircraft': col['first_agent'], 'node': [location[0]], 'timestep': timestep}
        c2 = {'aircraft': col['second_agent'], 'node': [location[0]], 'timestep': timestep}
        # print('c1,c2', c1, c2)
        return [c1, c2]

    if len(location) > 1: #edge col
        c1 = {'aircraft': col['first_agent'], 'node': [location[1], location[0]], 'timestep': timestep}
        c2 = {'aircraft': col['second_agent'], 'node': [location[0], location[1]], 'timestep': timestep}
        print('c1,c2',c1,c2)
        return [c1, c2]


def run_CBS(aircraft_lst, nodes_dict, heuristics, t, root, open_list, num_of_generated, num_of_expanded):
    # start_time = timer.time()
    # Generate the root node
    # constraints   - list of constraints
    # paths         - list of paths, one for each agent
    #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
    # collisions     - list of collisions in paths


    for ac in aircraft_lst:
        ID = ac.id
        if ac.spawntime == t: #maybe not necessary
            start_node = ac.start  # node from which planning should be done
            # print('start_node agent', ID, start_node, 'time', t)
            goal_node = ac.goal
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            # print('root constraints', root['constraints'])
            success, path = astar_CBS(nodes_dict, start_node, goal_node, heuristics, t, ID, root['constraints'])


            if path is None:
                raise BaseException('No solutions')
            # print('path', root['paths'])
            root['paths'].append(path)

            if success:
                ac.path_to_goal = path[1:]
                next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                ac.from_to = [path[0][0], next_node_id]
                print("Path AC", ac.id, ":", path)
            else:
                raise Exception("No solution found for", ac.id)
            # Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")

    root['cost'] = get_sum_of_cost(root['paths'])
    root['collisions'] = detect_collisions(root['paths'])
    push_node(open_list, root, num_of_generated)
    # print('openlist',open_list)

    # Task 3.1: Testing
    # print(root['collisions'])

    # Task 3.2: Testing
    # for collision in root['collisions']:
        # print(standard_splitting(collision))

    ##############################
    # Task 3.3: High-Level Search
    #           Repeat the following as long as the open list is not empty:
    #             1. Get the next node from the open list (you can use self.pop_node()
    #             2. If this node has no collision, return solution
    #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
    #                standard_splitting function). Add a new child node to your open list for each constraint
    #           Ensure to create a copy of any objects that your child nodes might inherit
    #

    # perform actions while there is still an open_list
    while len(open_list) != 0:
        P = pop_node(open_list, num_of_expanded)
        # print('new node constraints ', P['constraints'])
        if len(P['collisions']) == 0:
            return P['paths']

        list_of_constraints = standard_splitting(P['collisions'][0])
        # print('listconstr', list_of_constraints)

        # constructing new node Q
        for item in list_of_constraints:
            Q = {'cost': list(),
                 'constraints': list(),
                 'paths': list(),
                 'collisions': list()}

            for x in P['constraints']:
                Q['constraints'].append(x)
            Q['constraints'].append(item)

            for x in P['paths']:
                Q['paths'].append(x)

            agent = item['aircraft']
            success, path = astar_CBS(nodes_dict, start_node, goal_node, heuristics, t, ID, Q['constraints'])

            if path:
                Q["paths"][agent] = path
                Q['cost'] = get_sum_of_cost(Q['paths'])
                Q['collisions'] = detect_collisions(Q['paths'])

                push_node(open_list, Q, num_of_generated)

    start_time = timer.time()
    ac.CPU_time = timer.time() - start_time
    # print("CPU time (s):    {:.2f}".format(ac.CPU_time))







