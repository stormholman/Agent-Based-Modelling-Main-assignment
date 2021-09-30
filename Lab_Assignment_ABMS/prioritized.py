"""
Implement prioritized planner here
"""
from single_agent_planner import simple_single_agent_astar
import time as timer

def run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    # print('alist', aircraft_lst)
    # start_time = timer.time()
    result = []
    constraints = []
    constraints = [{'aircraft': 1, 'node': [14], 'timestep': 5.0},{'aircraft': 0, 'node': [14], 'timestep': 5.0}, {'aircraft': 0, 'node': [20], 'timestep': 5.0},
                   {'aircraft': 0, 'node': [25], 'timestep': 5.0}]
    ID = -1
 # node to which planning should be done
    for ac in aircraft_lst:
        ID += 1
        print('ID', ID)
        if ac.spawntime == t:
            start_node = ac.start  # node from which planning should be done
            goal_node = ac.goal
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t, ID, constraints)
        # print('ac', ac)
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

    # ac.CPU_time = timer.time() - start_time
    # print("CPU time (s):    {:.2f}".format(ac.CPU_time))
    return result
#flow of programming:
# run_me.py (run_prioritized_planner()) -> Prioritized.py (run_prioritized_planner()) -> Aircraft (plan_prioritized)
