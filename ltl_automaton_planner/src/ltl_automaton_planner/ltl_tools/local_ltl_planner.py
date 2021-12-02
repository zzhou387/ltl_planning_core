import rospy
from ltl_automaton_planner.ltl_tools.product import ProdAut
from ltl_automaton_planner.ltl_tools.buchi import mission_to_buchi
from ltl_automaton_planner.ltl_tools.decomposition_set import get_decomposition_set
from ltl_automaton_planner.ltl_tools.graph_search_team import compute_team_plans, compute_local_plan, find_reusable_plan
from ltl_automaton_planner.ltl_tools.discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal, improve_plan_given_history,\
                                                          compute_local_plan_decentral, find_reusable_plan_decentral, \
                                                          dijkstra_plan_networkX_finite
from ltl_automation_a1.srv import LTLTrace


###########################################################
# Local planner that only takes charge of
# local replanning
###########################################################

class LocalLTLPlanner(object):
    def __init__(self, ts, hard_spec, soft_spec, beta=1000, gamma=10):
        self.hard_spec = hard_spec
        self.soft_spec = soft_spec
        if ts is not None:
            self.ts = ts
        else:
            rospy.logerr("TS input ERROR")

        self.buchi = None
        self.product = None
        self.local_plan = None
        self.trace = list() # record the TS states been visited
        self.traj = [] # record the full trajectory
        self.ts_info = None
        self.update_info = {}

        self.beta = beta                    # importance of taking soft task into account
        self.gamma = gamma                  # cost ratio between prefix and suffix
        self.build_product()

    def build_product(self):
        self.buchi = mission_to_buchi(self.hard_spec, self.soft_spec)
        self.product = ProdAut(self.ts, self.buchi)
        self.product.graph['ts'].build_full()
        self.product.build_full()

    def local_task_reallocate(self, style):
        rospy.loginfo("LTL Local Planner: --- Planning in progress ("+style+") ---")
        rospy.loginfo("LTL Local Planner: Hard task is: "+str(self.hard_spec))
        rospy.loginfo("LTL Local Planner: Soft task is: "+str(self.soft_spec))

        # Only called when no plan is initialized at the beginning (to be tested)
        if style == 'static':
            if self.product:
                self.local_plan, plantime = dijkstra_plan_networkX_finite(self.product, self.gamma)

        # Called when there is a local state change
        elif style == 'Local_state_change':
            if self.product and self.local_plan:
                self.product.update_local_pa(self.trace, self.local_plan)
                self.local_plan, self.local_plan_time = find_reusable_plan_decentral(self.product, self.local_plan)
                if self.local_plan is None:
                    rospy.logwarn("LTL Local Planner: Reusable path not found; Try local replanning")
                self.local_plan, self.local_plan_time = compute_local_plan_decentral(self.product)
                if self.local_plan is None:
                    rospy.logwarn("LTL Local Planner: No valid local plan has been found given state change! Try global option")
                    return False

            else:
                raise InitError("LTL Local Planner: \"replanning_local_state_change: \" planning was requested but product model or previous plan was never built, aborting...")

        # Called when there is a TS change
        elif style == 'Local_ts_update':
            if self.product and self.local_plan:
                self.product.revise_local_pa(self.trace, self.local_plan, self.update_info)
                self.local_plan, self.local_plan_time = compute_local_plan_decentral(self.product)
                if self.local_plan is None:
                    rospy.logwarn("LTL Local Planner: No valid local plan has been found given TS updates! Try global option")
                    return False
            else:
                raise InitError("LTL Local Planner: \"replanning_local_ts_change: \" planning was requested but product model or previous plan was never built, aborting...")

        else:
            raise InitError("LTL Local Planner: local reallocation style is wrong")

        return True

    # Update the local product automaton before sending global request
    def replan_level_1(self):
        # Directly do global reallocation because of malfunction
        # Remove the edges related to the malfunction agent
        self.update_info["added"] = set()
        self.update_info["relabel"] = set()
        self.update_info["deleted"] = self.product.find_deleted_malfunction(self.trace)

        self.product.revise_local_pa(self.trace, self.local_plan, self.update_info)
        # TODO: send global replanning request to global planner
        rospy.logwarn("LTL Local Planner: sending request to global planner for reallocation!")

    def replan_level_2(self):
        #Try local replanning first
        if self.local_task_reallocate(style="Local_state_change"):
            # self.plans.state_sequence[self.local_replan_rname] = [(self.local_replan_rname, tt[0], tt[1]) for tt in self.local_plan.prefix]
            return True

        else:
            # send global replanning request to global planner
            return False


    def replan_level_3(self):
        #Update the TS info
        self.update_info["added"] = set()
        self.update_info["relabel"] = set()
        self.update_info["deleted"] = self.product.find_deleted_ts_update(self.trace, self.local_plan)

        #Try local replanning first
        if self.local_task_reallocate(style="Local_ts_update"):
            # TODO: send the new local plan to the global planner
            # self.plans.state_sequence[self.local_replan_rname] = [(self.local_replan_rname, tt[0], tt[1]) for tt in self.local_plan.prefix]
            return True

        else:
            # send global replanning request to global planner
            return False
