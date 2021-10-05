"""
Implement CBS here!!
"""

import time as timer
from Aircraft import Aircraft
import heapq
from single_agent_planner import get_location, simple_single_agent_astar, get_sum_of_cost


def push_node(open_list, node, num_of_generated):
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
    for time in range(total_time):
        if get_location(path2, time - 1) == get_location(path1, time) and get_location(path1, time -1) == get_location(path2, time) and time > 0:
            col = {'loc' : [get_location(path2, time-1), get_location(path2, time)], 'timestep' : time} # edge col
        if get_location(path1, time) == get_location(path2, time):
            col = {'loc': [get_location(path1, time)], 'timestep': time} # vertex col
    return col


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    num_agents = len(paths)
    list_col = []
    for first_agent in range(num_agents - 1):
        for sec_agent in range(first_agent + 1, num_agents):
            loc_col = detect_collision(paths[first_agent], paths[sec_agent])
            if loc_col:
                list_col.append({'first_agent': first_agent, 'second_agent': sec_agent, 'loc': loc_col['loc'], 'timestep': loc_col['timestep']})
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
    location = col['loc']
    if len(location) == 1: #vertex col
        c1 = {'agent': col['first_agent'], 'loc': [location[0]], 'timestep': timestep}
        c2 = {'agent': col['second_agent'], 'loc': [location[0]], 'timestep': timestep}
        return [c1, c2]

    if len(location) > 1: #edge col
        c1 = {'agent': col['first_agent'], 'loc': [location[1], location[0]], 'timestep': timestep}
        c2 = {'agent': col['second_agent'], 'loc': [location[0], location[1]], 'timestep': timestep}
        return [c1, c2]


def run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints, constraint_table_all):
    start_time = timer.time()
    open_list = []
    num_of_generated = 0
    num_of_expanded = 0

    # Generate the root node
    # constraints   - list of constraints
    # paths         - list of paths, one for each agent
    #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
    # collisions     - list of collisions in paths
    root = {'cost': 0,
            'constraints': [],
            'paths': [],
            'collisions': []}
    for ac in aircraft_lst:
        ID = ac.id

        if ac.spawntime == t:
            start_node = ac.start  # node from which planning should be done
            print('start_node agent', ID, start_node, 'time', t)
            goal_node = ac.goal
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t, root['constraints'])

            if path is None:
                raise BaseException('No solutions')
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

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

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
            n = pop_node(open_list, num_of_expanded)
            # print('new node constraints ', n['constraints'])
            if len(n['collisions']) == 0:
                return n['paths']

            list_of_constraints = standard_splitting(n['collisions'][0])

            # constructing new node Q
            for item in list_of_constraints:
                Q = {'cost': list(),
                     'constraints': list(),
                     'paths': list(),
                     'collisions': list()}

                for x in n['constraints']:
                    Q['constraints'].append(x)
                Q['constraints'].append(item)

                for x in n['paths']:
                    Q['paths'].append(x)

                agent = item['agent']
                path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t, Q['constraints'])
                if path:
                    Q["paths"][agent] = path
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    Q['collisions'] = detect_collisions(Q['paths'])

                    push_node(open_list, Q, num_of_generated)

        ac.start_time = timer.time()
        ac.CPU_time = timer.time() - start_time
        # print("CPU time (s):    {:.2f}".format(ac.CPU_time))



