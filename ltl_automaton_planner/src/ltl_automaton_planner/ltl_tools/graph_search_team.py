import rospy
from ltl_automaton_planner.ltl_tools.discrete_plan import compute_path_from_pre
from ltl_automaton_planner.ltl_tools.team import Team_Run
from ltl_automaton_planner.ltl_tools.product import ProdAut_Run
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
        rospy.logdebug('Dijkstra team search done within %.2fs' %(time.time()-start))
        return run, time.time()-start

    rospy.logerr('No accepting run found in optimal planning!')
    return None, None

def compute_local_plan(team, rname):
    start = time.time()
    plans = {}
    runs = {}
    curr_prod = team.graph['pro_list'][rname]
    updated_init = curr_prod.graph['updated_initial']
    updated_accept = curr_prod.graph['updated_accept']
    for t_init in updated_init:
        plan_pre, plan_dist = nx.dijkstra_predecessor_and_distance(curr_prod, t_init)
        for target in updated_accept:
            if target in plan_dist:
                plans[target] = plan_dist[target]

            if plans:
                opti_targ = min(plans, key=plans.get)
                plan = compute_path_from_pre(plan_pre, opti_targ)
                plan_cost = plan_dist[opti_targ]
                runs[(t_init, opti_targ)] = (plan, plan_cost)

    if runs:
        plan, plan_cost = min(runs.values(), key=lambda p: p[1])
        run = ProdAut_Run(curr_prod, plan, plan_cost)
        rospy.logdebug('Dijkstra local PA search done within %.2fs' %(time.time()-start))
        return run, time.time()-start

    # rospy.logerr('No accepting run found in optimal planning!')
    return None, None

def find_reusable_plan(team, rname, old_run):
    start = time.time()
    local_pa_plan = old_run.state_sequence[rname]
    curr_prod = team.graph['pro_list'][rname]
    updated_init = curr_prod.graph['updated_initial']
    new_local_plan = list()
    for i in range(len(local_pa_plan)):
        if updated_init == local_pa_plan[i]:
            new_local_plan = local_pa_plan[i:]
            rospy.logdebug('Reusable path found within %.2fs' %(time.time()-start))
            break

    if len(new_local_plan) != 0:
        run = ProdAut_Run(curr_prod, new_local_plan, 0)  # cost is hardcoded for now since not used
        return run, time.time()-start

    return None, None