"""
Implement CBS here!!
"""
from single_agent_planner import simple_single_agent_astar
import time as timer
from single_agent_planner import build_constraint_table

def run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints, constraint_table_all):
    # print('alist', aircraft_lst)
    start_time = timer.time()
    result = []

    for ac in aircraft_lst:
        ID = ac.id
        #think about for what aircraft you want to replan if an aircraft is added
        #
        if ac.spawntime == t:
            start_node = ac.start  # node from which planning should be done
            print('start_node agent', ID, start_node, 'time', t)
            goal_node = ac.goal
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
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


        ac.CPU_time = timer.time() - start_time
        # print("CPU time (s):    {:.2f}".format(ac.CPU_time))
    return result


