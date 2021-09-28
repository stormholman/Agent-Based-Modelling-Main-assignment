"""
Implement prioritized planner here
"""

def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the
    #               is_constrained function.
    constrainttable = {}
    for i in constraints:
        if agent == i['agent']:
            if i['timestep'] in constrainttable.keys():
                constrainttable[i['timestep']].append(i['loc'])
            else:
                constrainttable[i['timestep']] = [i['loc']]

    return constrainttable

def is_constrained(curr_loc, next_loc, next_time, constrainttable):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time not in constrainttable:
        return False
    for i in constrainttable[next_time]:
        if len(i) == 1:  #edge constraint
            # print('i', i, len(i))
            if i == [next_loc]:
                return True
        elif i == [curr_loc, next_loc]: #vertex constraint
            return True
    return False

def run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    for ac in aircraft_lst:
        if ac.spawntime == t:
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            ac.plan_prioritized(nodes_dict, edges_dict, heuristics, t)

    # raise Exception("Prioritized planner not defined yet.")
    return

#flow of programming:
# run_me.py (run_prioritized_planner()) -> Prioritized.py (run_prioritized_planner()) -> Aircraft (plan_prioritized)