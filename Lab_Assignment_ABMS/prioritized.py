"""
Implement prioritized planner here
"""
from single_agent_planner import simple_single_agent_astar
import time as timer
from single_agent_planner import build_constraint_table

def run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints, constraint_table_all):
    # print('alist', aircraft_lst)
    start_time = timer.time()
    result = []

    # constraints = [{'aircraft': 1, 'node': [14], 'timestep': 5.0},{'aircraft': 0, 'node': [14], 'timestep': 5.0}, {'aircraft': 0, 'node': [20], 'timestep': 5.0},
    #                {'aircraft': 0, 'node': [25], 'timestep': 5.0}]

 # node to which planning should be done
    num_agents = len(aircraft_lst)
    # print('nvlieg',len(aircraft_lst))


    for ac in aircraft_lst:

        ID = ac.id
        #think about for what aircraft you want to replan if an aircraft is added
        #
        if ac.spawntime == t:
            start_node = ac.start  # node from which planning should be done
            goal_node = ac.goal
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            constraint_table = build_constraint_table(constraints, ID)
            for i in constraint_table: #5,6,7
                if i in constraint_table_all:
                    constraint_table_all[i].extend(constraint_table[i])
                else:
                    constraint_table_all[i] = constraint_table[i]

            # print('call agent', ID , constraint_table_all)
            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t, constraint_table_all)

            if path is None:
                raise BaseException('No solutions')
            result.append(path)

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

            for y in range(num_agents+1): #0,1,2,3,4,- 8
                for x in range(len(path)):  #0,1,2,3 - 6
                    if y != ID: # lower aircrafts get priority
                        constraints.append({'aircraft': y, 'node': [path[x][0]], 'timestep': path[x][1]}) #vertex constraint
                        if x > 0:
                            constraints.append({'aircraft': y, 'node': [path[x][0], path[x-1][0]], 'timestep': path[x][1]}) #edge constraint
            # print('constraints2', constraints)


        ac.CPU_time = timer.time() - start_time
        # print("CPU time (s):    {:.2f}".format(ac.CPU_time))
    return result

#flow of programming:
# run_me.py (run_prioritized_planner()) -> Prioritized.py (run_prioritized_planner()) -> Aircraft (plan_prioritized)
