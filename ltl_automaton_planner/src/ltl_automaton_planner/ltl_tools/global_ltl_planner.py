import rospy
from ltl_automaton_planner.ltl_tools.team import TeamModel
from ltl_automaton_planner.ltl_tools.product import ProdAut
from ltl_automaton_planner.ltl_tools.buchi import mission_to_buchi
from ltl_automaton_planner.ltl_tools.decomposition_set import get_decomposition_set
from ltl_automaton_planner.ltl_tools.graph_search_team import compute_team_plans, compute_local_plan, find_reusable_plan
from ltl_automaton_bt.srv import LTLTrace


###########################################################
# Global/central planner that only takes charge of
# initial allocation and global replanning
###########################################################

class GlobalLTLPlanner(object):
    def __init__(self, ts, hard_spec, soft_spec, beta=1000, gamma=10):
        self.hard_spec = hard_spec
        self.soft_spec = soft_spec
        if ts:
            self.ts = ts
        else:
            rospy.logerr("TS input ERROR")

        self.pro_list_initial = []
        self.team = None
        self.buchi = None
        self.decomposition_set = set()
        self.trace_dic = {} # record the regions been visited
        self.trace_dic[0] = list()
        self.trace_dic[1] = list()
        self.trace_dic[2] = list()
        self.traj = [] # record the full trajectory
        self.ts_info = None
        self.local_replan_rname = None
        self.update_info = {}

        self.beta = beta                    # importance of taking soft task into account
        self.gamma = gamma                  # cost ratio between prefix and suffix
        self.build_product_list()

    def build_product_list(self):
        self.buchi = mission_to_buchi(self.hard_spec, self.soft_spec)
        self.decomposition_set = get_decomposition_set(self.buchi)
        for ts_0 in self.ts:
            product = ProdAut(ts_0, self.buchi)
            product.graph['ts'].build_full()
            product.build_full()
            self.pro_list_initial.append(product)

    def task_allocate(self, style='static'):
        rospy.loginfo("LTL Global Planner: --- Planning in progress ("+style+") ---")
        rospy.loginfo("LTL Global Planner: Hard task is: "+str(self.hard_spec))
        rospy.loginfo("LTL Global Planner: Soft task is: "+str(self.soft_spec))

        if style == 'static':
            self.team = TeamModel(self.pro_list_initial, self.decomposition_set)
            self.team.build_team()
            self.plans, plan_time = compute_team_plans(self.team)
            if self.plans is None:
                rospy.logerr("LTL Global Planner: No valid plan has been found!")
                return False

        if style == 'Global_state_change':
            if self.team and self.plans:
                self.team.update_local_pa(self.trace_dic, self.local_replan_rname, self.plans)
                self.team.revise_team(self.trace_dic, self.local_replan_rname, self.plans)
                self.plans, plan_time = compute_team_plans(self.team)
                if self.plans is None:
                    rospy.logerr("LTL Global Planner: No valid global reallocation plan has been found!")
                    return False

            else:
                rospy.logerr("LTL Global Planner: \"replanning_local_state_change: \" planning was requested but team model or previous plan was never built, aborting...")
                return False

        if style == 'Global_ts_update':
            if self.team and self.plans:
                self.team.revise_local_pa(self.trace_dic, self.local_replan_rname, self.plans, self.update_info)
                self.team.revise_team(self.trace_dic, self.local_replan_rname, self.plans)
                self.plans, plan_time = compute_team_plans(self.team)
                if self.plans is None:
                    rospy.logwarn("LTL Global Planner: No valid global reallocation plan has been found!")
                    return False
            else:
                rospy.logerr("LTL Global Planner: \"replanning_local_ts_change: \" planning was requested but team model or previous plan was never built, aborting...")
                return False

        return True

    # used for only updating product automaton
    def update_product(self, has_ts_update):
        if self.team and self.plans:
            if has_ts_update:
                self.team.revise_local_pa(self.trace_dic, self.local_replan_rname, self.plans, self.update_info)
            else:
                self.team.update_local_pa(self.trace_dic, self.local_replan_rname, self.plans)
        else:
            rospy.logerr("LTL Global Planner: \"replanning_local_ts_change: \" planning was requested but team model or previous plan was never built, aborting...")

    def replan_level_1(self):
        # Directly do global reallocation because of malfunction
        # Remove the edges related to the malfunction agent
        self.update_info["added"] = set()
        self.update_info["relabel"] = set()
        self.update_info["deleted"] = self.team.find_deleted_malfunction(self.trace_dic, self.local_replan_rname)

        # Call ros service for requesting the synchronization
        self.trace_dic = {}
        self.trace_dic[0] = list()
        self.trace_dic[1] = list()
        self.trace_dic[2] = list()
        service_1 = rospy.ServiceProxy('/turtlebot_08/synchronization_service', LTLTrace)
        service_1(request=1)
        service_2 = rospy.ServiceProxy('/turtlebot_14/synchronization_service', LTLTrace)
        service_2(request=1)
        service_3 = rospy.ServiceProxy('/mini_cheetah/synchronization_service', LTLTrace)
        service_3(request=1)

        while (len(self.trace_dic[0]) == 0) or \
                (len(self.trace_dic[1]) == 0) or \
                (len(self.trace_dic[2]) == 0):
            rospy.logwarn('Waiting for the trace callback from all agents')

        # Critical failure is treated as a special case of TS change
        if self.task_allocate(style="Global_ts_update"):
            return True

        rospy.logerr("LTL Global Planner: No valid plan has been found for level 1 of critical failure!")
        return False

    def replan_level_2(self):
        #Call ros service for requesting the synchronization
        self.trace_dic = {}
        self.trace_dic[0] = list()
        self.trace_dic[1] = list()
        self.trace_dic[2] = list()
        service_1 = rospy.ServiceProxy('/turtlebot_08/synchronization_service', LTLTrace)
        service_1(request=1)
        service_2 = rospy.ServiceProxy('/turtlebot_14/synchronization_service', LTLTrace)
        service_2(request=1)
        service_3 = rospy.ServiceProxy('/mini_cheetah/synchronization_service', LTLTrace)
        service_3(request=1)

        while (len(self.trace_dic[0]) == 0) or \
                (len(self.trace_dic[1]) == 0) or \
                (len(self.trace_dic[2]) == 0):
            rospy.logwarn('Waiting for the trace callback from all agents')

        if self.task_allocate(style="Global_state_change"):
            return True

        rospy.logerr("LTL Global Planner: No valid plan has been found for level 2 of state change!")
        return False


    def replan_level_3(self):
        # Receiving messages from local planner
        # self.update_info["added"] = set()
        # self.update_info["relabel"] = set()
        # self.update_info["deleted"] = self.team.find_deleted_ts_update(self.trace_dic, self.local_replan_rname, self.plans)

        #Try local replanning first
        # if self.task_allocate(style="Global_ts_update"):
        #     self.plans.state_sequence[self.local_replan_rname] = [(self.local_replan_rname, tt[0], tt[1]) for tt in self.local_plan.prefix]
        #     return "Local", True

        # Call ros service for requesting the synchronization
        self.trace_dic = {}
        self.trace_dic[0] = list()
        self.trace_dic[1] = list()
        self.trace_dic[2] = list()
        service_1 = rospy.ServiceProxy('/turtlebot_08/synchronization_service', LTLTrace)
        service_1(request=1)
        service_2 = rospy.ServiceProxy('/turtlebot_14/synchronization_service', LTLTrace)
        service_2(request=1)
        service_3 = rospy.ServiceProxy('/mini_cheetah/synchronization_service', LTLTrace)
        service_3(request=1)

        while (len(self.trace_dic[0]) == 0) or \
                (len(self.trace_dic[1]) == 0) or \
                (len(self.trace_dic[2]) == 0):
            rospy.logwarn('Waiting for the trace callback from all agents')

        if self.task_allocate(style="Global_ts_update"):
            return True

        rospy.logerr("LTL Global Planner: No valid plan has been found for level 3 of TS change!")
        return False

