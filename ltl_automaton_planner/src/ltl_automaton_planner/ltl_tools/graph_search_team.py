import rospy
from ltl_automaton_planner.ltl_tools.discrete_plan import compute_path_from_pre
from ltl_automaton_planner.ltl_tools.team import Team_Run
import networkx as nx
import time

def compute_team_plans(team):
    # Perform a single-objective search for the multi-robot task allocation
    start = time.time()
    plans = {}
    runs = {}
    team_init = team.graph['initial']
    team_finals = team.graph['accept']
    for t_init in team_init:
        plan_pre, plan_dist = nx.dijkstra_predecessor_and_distance(team, t_init)
        for target in team_finals:
            if target in plan_dist:
                plans[target] = plan_dist[target]

        if plans:
            opti_targ = min(plans, key=plans.get)
            plan = compute_path_from_pre(plan_pre, opti_targ)
            plan_cost = plan_dist[opti_targ]
            runs[(t_init, opti_targ)] = (plan, plan_cost)

    if runs:
        plan, plan_cost = min(runs.values(), key=lambda p: p[1])
        run = Team_Run(team, plan, plan_cost)
        rospy.logdebug('Dijkstra team search done within %.2fs: cumulated cost' %(time.time()-start))
        return run, time.time()-start

    rospy.logerr('No accepting run found in optimal planning!')
    return None, None