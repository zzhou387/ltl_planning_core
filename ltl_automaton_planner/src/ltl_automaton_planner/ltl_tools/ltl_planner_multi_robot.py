import rospy
from ltl_automaton_planner.ltl_tools.team import TeamModel
from ltl_automaton_planner.ltl_tools.product import ProdAut
from ltl_automaton_planner.ltl_tools.buchi import mission_to_buchi
from ltl_automaton_planner.ltl_tools.decomposition_set import get_decomposition_set
from ltl_automaton_planner.ltl_tools.graph_search_team import compute_team_plans

class LTLPlanner_MultiRobot(object):
    def __init__(self, ts, hard_spec, soft_spec, beta=1000, gamma=10):
        self.hard_spec = hard_spec
        self.soft_spec = soft_spec
        if ts:
            self.ts = ts
        else:
            rospy.logerr("TS input ERROR")

        self.pro_list = []
        self.team = None
        self.curr_ts_state = None
        self.buchi = None
        self.decomposition_set = []
        self.trace = [] # record the regions been visited
        self.traj = [] # record the full trajectory

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
            self.pro_list.append(product)

    def task_allocate(self, style='static'):
        rospy.loginfo("LTL Planner: --- Planning in progress ("+style+") ---")
        rospy.loginfo("LTL Planner: Hard task is: "+str(self.hard_spec))
        rospy.loginfo("LTL Planner: Soft task is: "+str(self.soft_spec))

        if style == 'static':
            self.team = TeamModel(self.pro_list, self.decomposition_set)
            self.team.build_team()
            self.plans, plan_time = compute_team_plans(self.team)

        if style == 'replan_level_2':
            if self.team:

        if style == 'replan_level_3':
            if self.team:

        if self.plans == None:
            rospy.logerr("LTL Planner: No valid plan has been found! Check you FTS or task")
            return False
        return True

    def replan_level_2(self):


    def replan_level_3(self):
