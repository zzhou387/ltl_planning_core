#!/usr/bin/env python
import roslib
import numpy
import rospy
import sys
import importlib
import yaml
import time

from copy import deepcopy

import std_msgs

from ltl_automaton_planner.ltl_tools.ts import TSModel
from ltl_automaton_planner.ltl_tools.team import TeamModel
from ltl_automaton_planner.ltl_tools.global_ltl_planner import GlobalLTLPlanner

import matplotlib.pyplot as plt
import networkx as nx
from ltl_automaton_planner.ltl_automaton_utilities import state_models_from_ts, import_ts_from_file, \
                                                          handle_ts_state_msg, handle_update_info_msg

# Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLState, \
    LTLStateArray, \
    GlobalReplanInfo, UpdateInfo, TransitionSystemInfo
from ltl_automaton_msgs.srv import TaskPlanning, TaskPlanningResponse
from networkx.drawing.nx_agraph import to_agraph

# Import dynamic reconfigure components for dynamic parameters (see dynamic_reconfigure and dynamic_params package)
from dynamic_reconfigure.server import Server as DRServer
from ltl_automaton_planner.cfg import LTLAutomatonDPConfig
from ltl_automaton_bt.srv import LTLTrace


###########################################################
# Global/central planner node
###########################################################

def show_automaton(automaton_graph):
    # pos=nx.circular_layout(automaton_graph)
    # nx.draw(automaton_graph, pos)
    # nx.draw_networkx_labels(automaton_graph, pos)
    # edge_labels = nx.get_edge_attributes(automaton_graph, 'action')
    # nx.draw_networkx_edge_labels(automaton_graph, pos, edge_labels = edge_labels)
    # plt.show()
    automaton_graph.graph['edge'] = {'arrowsize': '0.6', 'splines': 'curved'}
    A = to_agraph(automaton_graph)
    A.layout('dot')
    A.draw('team.png')
    plt.show()
    return


class Central_Planner(object):
    def __init__(self):
        # init parameters, automaton, etc...
        self.init_params()

        self.build_automaton()

        self.setup_pub_sub()

        # Output plan and first command of plan
        self.publish_plan_initial()

    def init_params(self):
        # Get parameters from parameter server
        self.initial_beta = rospy.get_param('initial_beta', 1000)
        self.gamma = rospy.get_param('gamma', 10)

        # LTL task
        # ----------
        # Get LTL hard task and raise error if don't exist
        if (rospy.has_param('~hard_task')):
            self.hard_task = rospy.get_param('~hard_task')
        else:
            raise InitError("Cannot initialize LTL Global Planner, no hard_task defined")
        # Get LTL soft task
        self.soft_task = rospy.get_param('~soft_task', "")

        # Transition system
        # -------------------
        # Get TS from param
        transition_system_mobile_1_textfile = rospy.get_param('~transition_system_turtlebot_08_textfile')
        self.transition_system_mobile_1 = import_ts_from_file(transition_system_mobile_1_textfile)

        transition_system_mobile_2_textfile = rospy.get_param('~transition_system_turtlebot_14_textfile')
        self.transition_system_mobile_2 = import_ts_from_file(transition_system_mobile_2_textfile)

        transition_system_quadruped_textfile = rospy.get_param('~transition_system_mini_cheetah_textfile')
        self.transition_system_quadruped = import_ts_from_file(transition_system_quadruped_textfile)
        # print(self.transition_system)

        # Parameter if initial TS is set from agent callback or from TS config file
        self.initial_ts_state_from_agent = rospy.get_param('~initial_ts_state_from_agent', False)

        # If initial TS states is from agent, wait from agent state callback
        # if self.initial_ts_state_from_agent:
        #     self.initial_state_ts_dict = None
        #     rospy.loginfo("LTL Global Planner: waiting for initial TS state from agent to initialize")
        #     while not self.initial_state_ts_dict:
        #         self.initial_state_ts_dict = self.init_ts_state_from_agent(rospy.wait_for_message("ts_state", TransitionSystemStateStamped))
        # else:
        self.initial_state_ts_dict = None

        # Setup dynamic parameters (defined in dynamic_params/cfg/LTL_automaton_dynparam.cfg)
        self.replan_on_unplanned_move = True
        self.check_timestamp = True
        self.prev_received_timestamp_1 = rospy.Time()
        self.prev_received_timestamp_2 = rospy.Time()
        self.prev_received_timestamp_3 = rospy.Time()

    def init_ts_state_from_agent(self, msg=TransitionSystemStateStamped):
        initial_state_ts_dict_ = None

        # If message is conform (same number of state as number of state dimensions)
        if (len(msg.ts_state.states) == len(msg.ts_state.state_dimension_names)):
            # Create dictionnary with paired dimension_name/state_value
            initial_state_ts_dict_ = dict()
            for i in range(len(msg.ts_state.states)):
                initial_state_ts_dict_.update({msg.ts_state.state_dimension_names[i]: msg.ts_state.states[i]})

                # Else message is malformed, raise error
        else:
            rospy.logerr("LTL Global Planner: received initial states don't match TS state models: " + str(
                len(msg.ts_state.states)) + " initial states and " + str(
                len(msg.ts_state.state_dimension_names)) + " state models")

        # Return initial state dictionnary
        return initial_state_ts_dict_

    def build_automaton(self):
        # Import state models from TS
        state_models_mobile_1 = state_models_from_ts(self.transition_system_mobile_1, self.initial_state_ts_dict)
        state_models_mobile_2 = state_models_from_ts(self.transition_system_mobile_2, self.initial_state_ts_dict)
        state_models_quadruped = state_models_from_ts(self.transition_system_quadruped, self.initial_state_ts_dict)

        # Here we take the product of each element of state_models to define the full TS
        self.robot_model_mobile_1 = TSModel(state_models_mobile_1)
        self.robot_model_mobile_2 = TSModel(state_models_mobile_2)
        self.robot_model_quadruped = TSModel(state_models_quadruped)
        self.ts_list = [self.robot_model_mobile_1, self.robot_model_mobile_2, self.robot_model_quadruped]

        # Initial allocation
        self.ltl_planner_multi_robot = GlobalLTLPlanner(self.ts_list, self.hard_task, self.soft_task, self.initial_beta,
                                                        self.gamma)
        self.ltl_planner_multi_robot.task_allocate()

        # Get first value from set
        # self.ltl_planner.curr_ts_state = list(self.ltl_planner.product.graph['ts'].graph['initial'])[0]

        # initialize storage of set of possible runs in product
        # self.ltl_planner.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])

        # show_automaton(self.robot_model)
        # show_automaton(self.ltl_planner_multi_robot.buchi)
        # show_automaton(self.ltl_planner.product)
        # show_automaton(self.ltl_planner_multi_robot.team)

    def setup_pub_sub(self):
        # Prefix plan publisher
        self.plan_pub_1 = rospy.Publisher('/turtlebot_08/global_planner/action_plan', LTLStateArray, latch=True, queue_size=1)
        self.plan_pub_2 = rospy.Publisher('/turtlebot_14/global_planner/action_plan', LTLStateArray, latch=True,
                                          queue_size=1)
        self.plan_pub_3 = rospy.Publisher('/mini_cheetah/global_planner/action_plan', LTLStateArray, latch=True,
                                          queue_size=1)

        # Possible states publisher
        # self.possible_states_pub = rospy.Publisher('possible_ltl_states', LTLStateArray, latch=True, queue_size=1)

        # Initialize subscriber to provide current state of robot
        self.trace_sub_1 = rospy.Subscriber('/turtlebot_08/ltl_trace', LTLPlan, self.ts_trace_callback_1, queue_size=1)
        self.trace_sub_2 = rospy.Subscriber('/turtlebot_14/ltl_trace', LTLPlan, self.ts_trace_callback_2, queue_size=1)
        self.trace_sub_3 = rospy.Subscriber('/mini_cheetah/ltl_trace', LTLPlan, self.ts_trace_callback_3, queue_size=1)

        # Subscribe to the replanning status
        self.replan_sub_1 = rospy.Subscriber('/turtlebot_08/global_replanning_request', GlobalReplanInfo,
                                             self.ltl_replan_callback_1, queue_size=1)
        self.replan_sub_2 = rospy.Subscriber('/turtlebot_14/global_replanning_request', GlobalReplanInfo,
                                             self.ltl_replan_callback_2, queue_size=1)
        self.replan_sub_3 = rospy.Subscriber('/mini_cheetah/global_replanning_request', GlobalReplanInfo,
                                             self.ltl_replan_callback_3, queue_size=1)

        # # Subscribe to the new local plans
        # self.new_local_plan_sub_1 = rospy.Subscriber('/dr_0/local_planner/new_local_plan', std_msgs.msg.Int8, self.ltl_new_local_callback_1, queue_size=1)
        # self.new_local_plan_sub_2 = rospy.Subscriber('/a1_gazebo/local_planner/new_local_plan', std_msgs.msg.Int8, self.ltl_new_local_callback_2, queue_size=1)
        # self.new_local_plan_sub_3 = rospy.Subscriber('/wassi_0/local_planner/new_local_plan', std_msgs.msg.Int8, self.ltl_new_local_callback_3, queue_size=1)

    def ltl_replan_callback_1(self, msg=GlobalReplanInfo()):
        replan_status = msg.level
        self.ltl_planner_multi_robot.local_replan_rname = 0
        if (replan_status == 0):
            rospy.logwarn('LTL Global Planner: local replanning for robot %d was successful; '
                          'update team model according to new local plan' % self.ltl_planner_multi_robot.local_replan_rname)

            # Update the product automaton
            if len(msg.update_info.added_ts) != 0 or len(msg.update_info.relabel_ts) != 0 or len(
                    msg.update_info.deleted_ts) != 0:
                self.ltl_planner_multi_robot.update_info = handle_update_info_msg(msg.update_info)
                self.ltl_planner_multi_robot.update_product(has_ts_update=True)
            else:
                self.ltl_planner_multi_robot.update_product(has_ts_update=False)

            # Update the new local plan
            if self.ltl_planner_multi_robot.plans is not None and len(msg.ltl_states) is not 0:
                new_local_plan = list()
                for ltl_state_msg in msg.ltl_states:
                    ts_state = handle_ts_state_msg(ltl_state_msg.ts_state)
                    buchi_state = ltl_state_msg.buchi_state
                    new_local_plan.append((self.ltl_planner_multi_robot.local_replan_rname, ts_state, buchi_state))

                self.ltl_planner_multi_robot.plans.state_sequence[
                    self.ltl_planner_multi_robot.local_replan_rname] = new_local_plan

            else:
                raise ValueError(
                    "the original team plan does not exist or global replan msg does not contain latest local plan")

            self.ltl_planner_multi_robot.local_replan_rname = None
            self.ltl_planner_multi_robot.update_info = {}

        if (replan_status == 1):
            rospy.logwarn('LTL Global Planner: received replanning Level 1; handling malfunction ')
            start = time.time()
            # Replan
            rospy.loginfo(
                "LTL Global Planner: problematic robot is: %d" % self.ltl_planner_multi_robot.local_replan_rname)

            if self.ltl_planner_multi_robot.replan_level_1():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 1 done within %.2fs' % (time.time() - start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None
                self.ltl_planner_multi_robot.update_info = {}

            # else:
                # Global reallocation fails
                # TODO: add safety mechanism

        if (replan_status == 2):
            rospy.logwarn('LTL Global Planner: received replanning Level 2: handling abrupt state change ')
            # Replan
            start = time.time()
            rospy.loginfo(
                "LTL Global Planner: problematic robot is: %d" % self.ltl_planner_multi_robot.local_replan_rname)

            if self.ltl_planner_multi_robot.replan_level_2():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 2 done within %.2fs' % (time.time() - start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None

        if (replan_status == 3):
            rospy.logwarn(
                'LTL Global Planner: received replanning Level 3: handling transition system change ')
            # Replan
            start = time.time()
            rospy.loginfo(
                "LTL Global Planner: problematic robot is: %d" % self.ltl_planner_multi_robot.local_replan_rname)

            # Update the product automaton
            if len(msg.update_info.added_ts) != 0 or len(msg.update_info.relabel_ts) != 0 or len(
                    msg.update_info.deleted_ts) != 0:
                self.ltl_planner_multi_robot.update_info = handle_update_info_msg(msg.update_info)

            else:
                raise ValueError(
                    "The TS must have updates in this case")

            if self.ltl_planner_multi_robot.replan_level_3():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 3 done within %.2fs' % (time.time() - start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None
                self.ltl_planner_multi_robot.update_info = {}

    def ltl_replan_callback_2(self, msg=GlobalReplanInfo()):
        replan_status = msg.level
        self.ltl_planner_multi_robot.local_replan_rname = 1
        if (replan_status == 0):
            rospy.logwarn('LTL Global Planner: local replanning for robot %d was successful; '
                          'update team model according to new local plan' % self.ltl_planner_multi_robot.local_replan_rname)

            # Update the product automaton
            if len(msg.update_info.added_ts) != 0 or len(msg.update_info.relabel_ts) != 0 or len(
                    msg.update_info.deleted_ts) != 0:
                self.ltl_planner_multi_robot.update_info = handle_update_info_msg(msg.update_info)
                self.ltl_planner_multi_robot.update_product(has_ts_update=True)
            else:
                self.ltl_planner_multi_robot.update_product(has_ts_update=False)

            # Update the new local plan
            if self.ltl_planner_multi_robot.plans is not None and len(msg.ltl_states) is not 0:
                new_local_plan = list()
                for ltl_state_msg in msg.ltl_states:
                    ts_state = handle_ts_state_msg(ltl_state_msg.ts_state)
                    buchi_state = ltl_state_msg.buchi_state
                    new_local_plan.append((self.ltl_planner_multi_robot.local_replan_rname, ts_state, buchi_state))

                self.ltl_planner_multi_robot.plans.state_sequence[
                    self.ltl_planner_multi_robot.local_replan_rname] = new_local_plan

            else:
                raise ValueError(
                    "the original team plan does not exist or global replan msg does not contain latest local plan")

            self.ltl_planner_multi_robot.local_replan_rname = None
            self.ltl_planner_multi_robot.update_info = {}

        if (replan_status == 1):
            rospy.logwarn('LTL Global Planner: received replanning Level 1; handling malfunction ')
            start = time.time()
            # Replan
            rospy.loginfo(
                "LTL Global Planner: problematic robot is: %d" % self.ltl_planner_multi_robot.local_replan_rname)

            if self.ltl_planner_multi_robot.replan_level_1():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 1 done within %.2fs' % (time.time() - start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None
                self.ltl_planner_multi_robot.update_info = {}

            # else:
            # Global reallocation fails
            # TODO: add safety mechanism

        if (replan_status == 2):
            rospy.logwarn('LTL Global Planner: received replanning Level 2: handling abrupt state change ')
            # Replan
            start = time.time()
            rospy.loginfo(
                "LTL Global Planner: problematic robot is: %d" % self.ltl_planner_multi_robot.local_replan_rname)

            if self.ltl_planner_multi_robot.replan_level_2():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 2 done within %.2fs' % (time.time() - start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None

        if (replan_status == 3):
            rospy.logwarn(
                'LTL Global Planner: received replanning Level 3: handling transition system change ')
            # Replan
            start = time.time()
            rospy.loginfo(
                "LTL Global Planner: problematic robot is: %d" % self.ltl_planner_multi_robot.local_replan_rname)

            # Update the product automaton
            if len(msg.update_info.added_ts) != 0 or len(msg.update_info.relabel_ts) != 0 or len(
                    msg.update_info.deleted_ts) != 0:
                self.ltl_planner_multi_robot.update_info = handle_update_info_msg(msg.update_info)

            else:
                raise ValueError(
                    "The TS must have updates in this case")

            if self.ltl_planner_multi_robot.replan_level_3():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 3 done within %.2fs' % (time.time() - start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None
                self.ltl_planner_multi_robot.update_info = {}

    def ltl_replan_callback_3(self, msg=GlobalReplanInfo()):
        replan_status = msg.level
        self.ltl_planner_multi_robot.local_replan_rname = 2
        if (replan_status == 0):
            rospy.logwarn('LTL Global Planner: local replanning for robot %d was successful; '
                          'update team model according to new local plan' % self.ltl_planner_multi_robot.local_replan_rname)

            # Update the product automaton
            if len(msg.update_info.added_ts) != 0 or len(msg.update_info.relabel_ts) != 0 or len(
                    msg.update_info.deleted_ts) != 0:
                self.ltl_planner_multi_robot.update_info = handle_update_info_msg(msg.update_info)
                self.ltl_planner_multi_robot.update_product(has_ts_update=True)
            else:
                self.ltl_planner_multi_robot.update_product(has_ts_update=False)

            # Update the new local plan
            if self.ltl_planner_multi_robot.plans is not None and len(msg.ltl_states) is not 0:
                new_local_plan = list()
                for ltl_state_msg in msg.ltl_states:
                    ts_state = handle_ts_state_msg(ltl_state_msg.ts_state)
                    buchi_state = ltl_state_msg.buchi_state
                    new_local_plan.append((self.ltl_planner_multi_robot.local_replan_rname, ts_state, buchi_state))

                self.ltl_planner_multi_robot.plans.state_sequence[
                    self.ltl_planner_multi_robot.local_replan_rname] = new_local_plan

            else:
                raise ValueError(
                    "the original team plan does not exist or global replan msg does not contain latest local plan")

            self.ltl_planner_multi_robot.local_replan_rname = None
            self.ltl_planner_multi_robot.update_info = {}

        if (replan_status == 1):
            rospy.logwarn('LTL Global Planner: received replanning Level 1; handling malfunction ')
            start = time.time()
            # Replan
            rospy.loginfo(
                "LTL Global Planner: problematic robot is: %d" % self.ltl_planner_multi_robot.local_replan_rname)

            if self.ltl_planner_multi_robot.replan_level_1():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 1 done within %.2fs' % (time.time() - start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None
                self.ltl_planner_multi_robot.update_info = {}

            # else:
            # Global reallocation fails
            # TODO: add safety mechanism

        if (replan_status == 2):
            rospy.logwarn('LTL Global Planner: received replanning Level 2: handling abrupt state change ')
            # Replan
            start = time.time()
            rospy.loginfo(
                "LTL Global Planner: problematic robot is: %d" % self.ltl_planner_multi_robot.local_replan_rname)

            if self.ltl_planner_multi_robot.replan_level_2():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 2 done within %.2fs' % (time.time() - start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None

        if (replan_status == 3):
            rospy.logwarn(
                'LTL Global Planner: received replanning Level 3: handling transition system change ')
            # Replan
            start = time.time()
            rospy.loginfo(
                "LTL Global Planner: problematic robot is: %d" % self.ltl_planner_multi_robot.local_replan_rname)

            # Update the product automaton
            if len(msg.update_info.added_ts) != 0 or len(msg.update_info.relabel_ts) != 0 or len(
                    msg.update_info.deleted_ts) != 0:
                self.ltl_planner_multi_robot.update_info = handle_update_info_msg(msg.update_info)

            else:
                raise ValueError(
                    "The TS must have updates in this case")

            if self.ltl_planner_multi_robot.replan_level_3():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 3 done within %.2fs' % (time.time() - start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None
                self.ltl_planner_multi_robot.update_info = {}

    def ts_trace_callback_1(self, msg=LTLPlan()):
        # Extract TS state from message
        if not (self.check_timestamp and (msg.header.stamp.to_sec() == self.prev_received_timestamp_1.to_sec())):
            # Update previously received timestamp
            self.prev_received_timestamp_1 = deepcopy(msg.header.stamp)
            self.ltl_planner_multi_robot.trace_dic[0] = list()
            # self.ltl_planner_multi_robot.local_replan_rname = 0

            for state_msg in msg.ts_state_sequence:
                state = handle_ts_state_msg(state_msg)

                # -------------------------
                # Check if state is in TS
                # -------------------------
                if (state in self.robot_model_mobile_1.nodes()):

                    # Update trace for robot 1
                    self.ltl_planner_multi_robot.trace_dic[0].append(state)

                # --------------------------------------------
                # If state not part of the transition system
                # --------------------------------------------
                else:
                    # ERROR: unknown state (not part of TS)
                    rospy.logwarn('State is not in TS plan!')
        else:
            rospy.logwarn(
                "LTL Global Planner: not updating with received trace, timestamp identical to previously received "
                "message timestamp at time %f" % self.prev_received_timestamp_1.to_sec())

    def ts_trace_callback_2(self, msg=LTLPlan()):
        # Extract TS state from message
        if not (self.check_timestamp and (msg.header.stamp.to_sec() == self.prev_received_timestamp_2.to_sec())):
            # Update previously received timestamp
            self.prev_received_timestamp_2 = deepcopy(msg.header.stamp)
            self.ltl_planner_multi_robot.trace_dic[1] = list()
            # self.ltl_planner_multi_robot.local_replan_rname = 1

            for state_msg in msg.ts_state_sequence:
                state = handle_ts_state_msg(state_msg)

                # -------------------------
                # Check if state is in TS
                # -------------------------
                if (state in self.robot_model_quadruped.nodes()):

                    # Update trace for robot 2
                    self.ltl_planner_multi_robot.trace_dic[1].append(state)

                # --------------------------------------------
                # If state not part of the transition system
                # --------------------------------------------
                else:
                    # ERROR: unknown state (not part of TS)
                    rospy.logwarn('State is not in TS plan!')
        else:
            rospy.logwarn(
                "LTL Global Planner: not updating with received trace, timestamp identical to previously received "
                "message timestamp at time %f" % self.prev_received_timestamp_2.to_sec())

    def ts_trace_callback_3(self, msg=LTLPlan()):
        # Extract TS state from message
        if not (self.check_timestamp and (msg.header.stamp.to_sec() == self.prev_received_timestamp_3.to_sec())):
            # Update previously received timestamp
            self.prev_received_timestamp_3 = deepcopy(msg.header.stamp)
            self.ltl_planner_multi_robot.trace_dic[2] = list()
            # self.ltl_planner_multi_robot.local_replan_rname = 2

            for state_msg in msg.ts_state_sequence:
                state = handle_ts_state_msg(state_msg)

                # -------------------------
                # Check if state is in TS
                # -------------------------
                if (state in self.robot_model_mobile_2.nodes()):

                    # Update trace for robot 2
                    self.ltl_planner_multi_robot.trace_dic[2].append(state)

                # --------------------------------------------
                # If state not part of the transition system
                # --------------------------------------------
                else:
                    # ERROR: unknown state (not part of TS)
                    rospy.logwarn('State is not in TS plan!')
        else:
            rospy.logwarn(
                "LTL Global Planner: not updating with received trace, timestamp identical to previously received "
                "message timestamp at time %f" % self.prev_received_timestamp_3.to_sec())

    # ----------------------------------------------
    # Publish prefix and suffix plans from planner
    # ----------------------------------------------
    def publish_plan_initial(self):
        # If plan exists
        if not (self.ltl_planner_multi_robot.plans == None):
            # Prefix plan
            # -------------
            plan_1_msg = LTLStateArray()
            plan_1_msg.header.stamp = rospy.Time.now()
            plan_1_status = False
            plan_2_msg = LTLStateArray()
            plan_2_msg.header.stamp = rospy.Time.now()
            plan_2_status = False
            plan_3_msg = LTLStateArray()
            plan_3_msg.header.stamp = rospy.Time.now()
            plan_3_status = False

            for (r_idx, ts_seq), (r_idx_2, buchi_seq) in zip(
                    self.ltl_planner_multi_robot.plans.ts_state_sequence.items(),
                    self.ltl_planner_multi_robot.plans.buchi_sequence.items()):
                assert r_idx == r_idx_2
                if r_idx == 0:
                    for ts_state, buchi_state in zip(ts_seq, buchi_seq):
                        ltl_state_msg = LTLState()
                        ts_state_msg = TransitionSystemState()
                        ts_state_msg.state_dimension_names = [item for sublist in
                                                              self.ltl_planner_multi_robot.pro_list_initial[
                                                                  r_idx].graph['ts'].graph['ts_state_format'] for item
                                                              in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(ts_state) is tuple:
                            ts_state_msg.states = list(ts_state)
                        # Else state is a single string
                        else:
                            ts_state_msg.states = [ts_state]
                        # Add to plan TS state sequence
                        ltl_state_msg.ts_state = ts_state_msg
                        ltl_state_msg.buchi_state = buchi_state

                        plan_1_msg.ltl_states.append(ltl_state_msg)

                    # Publish
                    self.plan_pub_1.publish(plan_1_msg)

                if r_idx == 1:
                    for ts_state, buchi_state in zip(ts_seq, buchi_seq):
                        ltl_state_msg = LTLState()
                        ts_state_msg = TransitionSystemState()
                        ts_state_msg.state_dimension_names = [item for sublist in
                                                              self.ltl_planner_multi_robot.pro_list_initial[
                                                                  r_idx].graph['ts'].graph['ts_state_format'] for item
                                                              in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(ts_state) is tuple:
                            ts_state_msg.states = list(ts_state)
                        # Else state is a single string
                        else:
                            ts_state_msg.states = [ts_state]
                        # Add to plan TS state sequence
                        ltl_state_msg.ts_state = ts_state_msg
                        ltl_state_msg.buchi_state = buchi_state

                        plan_2_msg.ltl_states.append(ltl_state_msg)

                    # Publish
                    self.plan_pub_2.publish(plan_2_msg)

                if r_idx == 2:
                    for ts_state, buchi_state in zip(ts_seq, buchi_seq):
                        ltl_state_msg = LTLState()
                        ts_state_msg = TransitionSystemState()
                        ts_state_msg.state_dimension_names = [item for sublist in
                                                              self.ltl_planner_multi_robot.pro_list_initial[
                                                                  r_idx].graph['ts'].graph['ts_state_format'] for item
                                                              in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(ts_state) is tuple:
                            ts_state_msg.states = list(ts_state)
                        # Else state is a single string
                        else:
                            ts_state_msg.states = [ts_state]
                        # Add to plan TS state sequence
                        ltl_state_msg.ts_state = ts_state_msg
                        ltl_state_msg.buchi_state = buchi_state

                        plan_3_msg.ltl_states.append(ltl_state_msg)

                    # Publish
                    self.plan_pub_3.publish(plan_3_msg)

    def publish_local(self, rname):
        # If plan exists
        if self.ltl_planner_multi_robot.local_plan is not None:
            # Prefix plan
            # -------------
            plan_local_msg = LTLPlan()
            plan_local_msg.header.stamp = rospy.Time.now()
            plan_local_status = False

            plan_local_msg.action_sequence = self.ltl_planner_multi_robot.local_plan.action_sequence

            for ts_state in self.ltl_planner_multi_robot.local_plan.ts_state_sequence:
                ts_state_msg = TransitionSystemState()
                ts_state_msg.state_dimension_names = [item for sublist in
                                                      self.ltl_planner_multi_robot.pro_list_initial[rname].graph[
                                                          'ts'].graph['ts_state_format'] for item in sublist]
                # If TS state is more than 1 dimension (is a tuple)
                if type(ts_state) is tuple:
                    ts_state_msg.states = list(ts_state)
                # Else state is a single string
                else:
                    ts_state_msg.states = [ts_state]
                # Add to plan TS state sequence
                plan_local_msg.ts_state_sequence.append(ts_state_msg)

            # Publish
            if rname == 0:
                self.plan_pub_1.publish(plan_local_msg)
            elif rname == 1:
                self.plan_pub_2.publish(plan_local_msg)
            elif rname == 2:
                self.plan_pub_3.publish(plan_local_msg)
            else:
                rospy.logerr("LTL Global Planner: rname in local publisher doesn't match!")
                return False

        return True


# ==============================
#             Main
# ==============================
if __name__ == '__main__':
    rospy.init_node('ltl_central_planner', anonymous=False)
    try:
        multi_robot_ltl_planner_node = Central_Planner()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("LTL Global Planner: " + str(e))
        rospy.logerr("LTL Global Planner: shutting down...")
