"""
Implement prioritized planner here
"""




def run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    # print('alist', aircraft_lst)
    for ac in aircraft_lst:
        # print('ac', ac)
        if ac.spawntime == t:
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            ac.plan_prioritized(nodes_dict, edges_dict, heuristics, t)

    # raise Exception("Prioritized planner not defined yet.")
    return

#flow of programming:
# run_me.py (run_prioritized_planner()) -> Prioritized.py (run_prioritized_planner()) -> Aircraft (plan_prioritized)