#!/usr/bin/env python
import roslib
import numpy
import rospy
import sys
import importlib
import yaml

from copy import deepcopy

import std_msgs

from ltl_automaton_planner.ltl_tools.ts import TSModel
from ltl_automaton_planner.ltl_tools.team import TeamModel
from ltl_automaton_planner.ltl_tools.ltl_planner_multi_robot import LTLPlanner_MultiRobot

import matplotlib.pyplot as plt
import networkx as nx
from ltl_automaton_planner.ltl_automaton_utilities import state_models_from_ts, import_ts_from_file, handle_ts_state_msg

# Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLState, LTLStateArray
from ltl_automaton_msgs.srv import TaskPlanning, TaskPlanningResponse
from networkx.drawing.nx_agraph import to_agraph

# Import dynamic reconfigure components for dynamic parameters (see dynamic_reconfigure and dynamic_params package)
from dynamic_reconfigure.server import Server as DRServer
from ltl_automaton_planner.cfg import LTLAutomatonDPConfig


def show_automaton(automaton_graph):
    pos=nx.circular_layout(automaton_graph)
    nx.draw(automaton_graph, pos)
    nx.draw_networkx_labels(automaton_graph, pos)
    edge_labels = nx.get_edge_attributes(automaton_graph, 'action')
    nx.draw_networkx_edge_labels(automaton_graph, pos, edge_labels = edge_labels)
    plt.show()
    # automaton_graph.graph['edge'] = {'arrowsize': '0.6', 'splines': 'curved'}
    # A = to_agraph(automaton_graph)
    # A.layout('dot')
    # A.draw('team.png')
    # plt.show()
    return

class MultiRobot_Planner(object):
    def __init__(self):
        # init parameters, automaton, etc...
        self.init_params()

        self.build_automaton()

        self.setup_pub_sub()

        # Output plan and first command of plan
        # self.publish_possible_states()
        self.publish_plan_initial()
        # self.plan_pub.publish(self.ltl_planner.next_move)


    def init_params(self):
        #Get parameters from parameter server
        self.agent_name_mobile_1 = rospy.get_param('agent_name_mobile_1', "agent_1")
        # self.agent_name_mobile_2 = rospy.get_param('agent_name_mobile_2', "agent_2")
        self.agent_name_dog_1 = rospy.get_param('agent_name_dog_1', "dog_1")
        self.initial_beta = rospy.get_param('initial_beta', 1000)
        self.gamma = rospy.get_param('gamma', 10)


        # LTL task
        #----------
        # Get LTL hard task and raise error if don't exist
        if (rospy.has_param('hard_task')):
            self.hard_task = rospy.get_param('hard_task')
        else:
            raise InitError("Cannot initialize LTL planner, no hard_task defined")
        # Get LTL soft task
        self.soft_task = rospy.get_param('soft_task', "")


        # Transition system
        #-------------------
        # Get TS from param
        transition_system_mobile_textfile = rospy.get_param('transition_system_mobile_textfile')
        self.transition_system_mobile = import_ts_from_file(transition_system_mobile_textfile)

        transition_system_quadruped_textfile = rospy.get_param('transition_system_quadruped_textfile')
        self.transition_system_quadruped = import_ts_from_file(transition_system_quadruped_textfile)
        #print(self.transition_system)

        # Parameter if initial TS is set from agent callback or from TS config file
        self.initial_ts_state_from_agent = rospy.get_param('~initial_ts_state_from_agent', False)

        #If initial TS states is from agent, wait from agent state callback
        # if self.initial_ts_state_from_agent:
        #     self.initial_state_ts_dict = None
        #     rospy.loginfo("LTL planner: waiting for initial TS state from agent to initialize")
        #     while not self.initial_state_ts_dict:
        #         self.initial_state_ts_dict = self.init_ts_state_from_agent(rospy.wait_for_message("ts_state", TransitionSystemStateStamped))
        # else:
        self.initial_state_ts_dict = None


        # Setup dynamic parameters (defined in dynamic_params/cfg/LTL_automaton_dynparam.cfg)
        self.replan_on_unplanned_move = True
        self.check_timestamp = True
        self.prev_received_timestamp_1 = rospy.Time()
        self.prev_received_timestamp_2 = rospy.Time()


    def init_ts_state_from_agent(self, msg=TransitionSystemStateStamped):
        initial_state_ts_dict_ = None

        # If message is conform (same number of state as number of state dimensions)
        if (len(msg.ts_state.states) == len(msg.ts_state.state_dimension_names)):
            # Create dictionnary with paired dimension_name/state_value
            initial_state_ts_dict_ = dict()
            for i in range(len(msg.ts_state.states)):
                initial_state_ts_dict_.update({msg.ts_state.state_dimension_names[i] : msg.ts_state.states[i]})

                # Else message is malformed, raise error
        else:
            rospy.logerr("LTL planner: received initial states don't match TS state models: "+str(len(msg.ts_state.states))+" initial states and "+str(len(msg.ts_state.state_dimension_names))+" state models")

        # Return initial state dictionnary
        return initial_state_ts_dict_


    def build_automaton(self):
        # Import state models from TS
        state_models_mobile = state_models_from_ts(self.transition_system_mobile, self.initial_state_ts_dict)
        state_models_quadruped = state_models_from_ts(self.transition_system_quadruped, self.initial_state_ts_dict)

        # Here we take the product of each element of state_models to define the full TS
        self.robot_model_mobile = TSModel(state_models_mobile)
        self.robot_model_quadruped = TSModel(state_models_quadruped)
        self.ts_list = [self.robot_model_mobile, self.robot_model_quadruped]

        self.ltl_planner_multi_robot = LTLPlanner_MultiRobot(self.ts_list, self.hard_task, self.soft_task, self.initial_beta, self.gamma)
        self.ltl_planner_multi_robot.task_allocate()
        # Get first value from set
        # self.ltl_planner.curr_ts_state = list(self.ltl_planner.product.graph['ts'].graph['initial'])[0]

        # initialize storage of set of possible runs in product
        # self.ltl_planner.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])

        # show_automaton(self.robot_model)
        # show_automaton(self.ltl_planner.product.graph['buchi'])
        # show_automaton(self.ltl_planner.product)
        # show_automaton(self.ltl_planner_multi_robot.team)


    def setup_pub_sub(self):
        # Prefix plan publisher
        self.plan_pub_1 = rospy.Publisher('/action_plan_1', LTLPlan, latch=True, queue_size = 1)
        self.plan_pub_2 = rospy.Publisher('/action_plan_2', LTLPlan, latch=True, queue_size = 1)

        # Possible states publisher
        # self.possible_states_pub = rospy.Publisher('possible_ltl_states', LTLStateArray, latch=True, queue_size=1)

        # Initialize subscriber to provide current state of robot
        self.trace_sub_1 = rospy.Subscriber('/openshelf_0/ts_trace', LTLPlan, self.ts_trace_callback_1, queue_size=1)
        self.trace_sub_2 = rospy.Subscriber('/a1_gazebo/ts_trace', LTLPlan, self.ts_trace_callback_2, queue_size=1)

        # Initialize publisher to send plan commands
        # self.plan_pub = rospy.Publisher('next_move_cmd', std_msgs.msg.String, queue_size=1, latch=True)

        # Initialize task replanning service
        # self.trap_srv = rospy.Service('replanning', TaskPlanning, self.task_replanning_callback)

        # Subscribe to the replanning status
        self.replan_sub_1 = rospy.Subscriber('/openshelf_0/replanning_request', std_msgs.msg.Int8, self.ltl_replan_callback_1, queue_size=1)
        self.replan_sub_2 = rospy.Subscriber('/a1_gazebo/replanning_request', std_msgs.msg.Int8, self.ltl_replan_callback_2, queue_size=1)


    def ltl_replan_callback_1(self, msg):
        replan_status = msg.data
        if(replan_status == 0):
            rospy.logerr('LTL planner: replanning ERROR')

        if(replan_status == 1):
            rospy.logwarn('LTL planner: received replanning Level 1; handling malfunction from agent 1')
            #Replan
            #TODO: Add ros service for requesting the synchronization
            while not self.ltl_planner_multi_robot.trace:
                rospy.logwarn('Waiting for the trace callback from agent 1')

            if self.ltl_planner_multi_robot.replan_level_1():
                self.publish_plan_initial()

        if(replan_status == 2):
            rospy.logwarn('LTL planner: received replanning Level 2: handling abrupt state change')
            # Replan
            while not self.ltl_planner_multi_robot.trace:
                rospy.logwarn('Waiting for the trace callback from agent 1')

            level_flag, success = self.ltl_planner_multi_robot.replan_level_2()
            if success:
                if level_flag=="Local":
                    self.publish_local()

                if level_flag=="Global":
                    self.publish_plan_initial()

        if(replan_status == 3):
            rospy.logwarn('LTL planner: received replanning Level 3: handling transition system change')
            # Replan
            while not (self.ltl_planner_multi_robot.trace and self.ltl_planner_multi_robot.ts_info):
                rospy.logwarn('Waiting for the trace and Updated TS callbacks from agent 1')

            level_flag, success = self.ltl_planner_multi_robot.replan_level_3()
            if success:
                if level_flag=="Local":
                    self.publish_local()

                if level_flag=="Global":
                    self.publish_plan_initial()


    def ltl_replan_callback_2(self, msg):
        replan_status = msg.data
        if(replan_status == 0):
            rospy.logerr('LTL planner: replanning ERROR')

        if(replan_status == 1):
            rospy.logwarn('LTL planner: received replanning Level 1; handling malfunction from agent 1')
            #Replan
            #TODO: Add ros service for requesting the synchronization
            while not self.ltl_planner_multi_robot.trace:
                rospy.logwarn('Waiting for the trace callback from agent 1')

            if self.ltl_planner_multi_robot.replan_level_1():
                self.publish_plan_initial()

        if(replan_status == 2):
            rospy.logwarn('LTL planner: received replanning Level 2: handling abrupt state change')
            # Replan
            while not self.ltl_planner_multi_robot.trace:
                rospy.logwarn('Waiting for the trace callback from agent 1')

            level_flag, success = self.ltl_planner_multi_robot.replan_level_2()
            if success:
                if level_flag=="Local":
                    self.publish_local()

                if level_flag=="Global":
                    self.publish_plan_initial()

        if(replan_status == 3):
            rospy.logwarn('LTL planner: received replanning Level 3: handling transition system change')
            # Replan
            while not (self.ltl_planner_multi_robot.trace and self.ltl_planner_multi_robot.ts_info):
                rospy.logwarn('Waiting for the trace and Updated TS callbacks from agent 1')

            level_flag, success = self.ltl_planner_multi_robot.replan_level_3()
            if success:
                if level_flag=="Local":
                    self.publish_local()

                if level_flag=="Global":
                    self.publish_plan_initial()


    def ts_trace_callback_1(self, msg=LTLPlan()):
        # Extract TS state from message
        state = handle_ts_state_msg(msg.ts_state)

        #-------------------------
        # Check if state is in TS
        #-------------------------
        if (state in self.robot_model_mobile.nodes()):

            # If timestamp check is enabled, check the timestamp
            if not (self.check_timestamp and (msg.header.stamp.to_sec() == self.prev_received_timestamp_1.to_sec())):
                # Update previously received timestamp
                self.prev_received_timestamp_1 = deepcopy(msg.header.stamp)

                # Update current state
                self.ltl_planner_multi_robot.current_ts_state_dic[0] = state
                self.ltl_planner_multi_robot.trace_dic[0].append(state)

            # If timestamp is indentical to previoulsy received message and parameters "check_timestamp" is true
            else:
                rospy.logwarn("LTL planner: not updating with received TS state %s, timestamp identical to previously received message timestamp at time %f" % (str(state), self.prev_received_timestamp.to_sec()))

        #--------------------------------------------
        # If state not part of the transition system
        #--------------------------------------------
        else:
            #ERROR: unknown state (not part of TS)
            rospy.logwarn('State is not in TS plan!')


    def ts_trace_callback_2(self, msg=LTLPlan()):
        # Extract TS state from message
        state = handle_ts_state_msg(msg.ts_state)

        #-------------------------
        # Check if state is in TS
        #-------------------------
        if (state in self.robot_model_mobile.nodes()):

            # If timestamp check is enabled, check the timestamp
            if not (self.check_timestamp and (msg.header.stamp.to_sec() == self.prev_received_timestamp_1.to_sec())):
                # Update previously received timestamp
                self.prev_received_timestamp_1 = deepcopy(msg.header.stamp)

                # Update current state
                self.ltl_planner_multi_robot.current_ts_state_dic[1] = state
                self.ltl_planner_multi_robot.trace_dic[1].append(state)

            # If timestamp is indentical to previoulsy received message and parameters "check_timestamp" is true
            else:
                rospy.logwarn("LTL planner: not updating with received TS state %s, timestamp identical to previously received message timestamp at time %f" % (str(state), self.prev_received_timestamp.to_sec()))

        #--------------------------------------------
        # If state not part of the transition system
        #--------------------------------------------
        else:
            #ERROR: unknown state (not part of TS)
            rospy.logwarn('State is not in TS plan!')


    #----------------------------------------------
    # Publish prefix and suffix plans from planner
    #----------------------------------------------
    def publish_plan_initial(self):
        # If plan exists
        if not (self.ltl_planner_multi_robot.plans == None):
            # Prefix plan
            #-------------
            plan_1_msg = LTLPlan()
            plan_1_msg.header.stamp = rospy.Time.now()
            plan_1_status = False
            plan_2_msg = LTLPlan()
            plan_2_msg.header.stamp = rospy.Time.now()
            plan_2_status = False

            # plan_1_msg.action_sequence = self.ltl_planner.run.pre_plan
            # Go through all TS state in plan and add it as TransitionSystemState message
            for r_idx, act_seq in self.ltl_planner_multi_robot.plans.action_sequence.items():
                if r_idx==0 and len(act_seq) != 0:
                    plan_1_status = True
                    plan_1_msg.action_sequence = act_seq
                if r_idx==1 and len(act_seq) != 0:
                    plan_2_status = True
                    plan_2_msg.action_sequence = act_seq

            for r_idx, stat_seq in self.ltl_planner_multi_robot.plans.ts_state_sequence.items():
                if r_idx==0 and plan_1_status:
                    for ts_state in stat_seq:
                        ts_state_msg = TransitionSystemState()
                        ts_state_msg.state_dimension_names = [item for sublist in self.ltl_planner_multi_robot.pro_list[r_idx].graph['ts'].graph['ts_state_format'] for item in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(ts_state) is tuple:
                            ts_state_msg.states = list(ts_state)
                        # Else state is a single string
                        else:
                            ts_state_msg.states = [ts_state]
                        # Add to plan TS state sequence
                        plan_1_msg.ts_state_sequence.append(ts_state_msg)

                    # Publish
                    self.plan_pub_1.publish(plan_1_msg)

                if r_idx==1 and plan_2_status:
                    for ts_state in stat_seq:
                        ts_state_msg = TransitionSystemState()
                        ts_state_msg.state_dimension_names = [item for sublist in self.ltl_planner_multi_robot.pro_list[r_idx].graph['ts'].graph['ts_state_format'] for item in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(ts_state) is tuple:
                            ts_state_msg.states = list(ts_state)
                        # Else state is a single string
                        else:
                            ts_state_msg.states = [ts_state]
                        # Add to plan TS state sequence
                        plan_2_msg.ts_state_sequence.append(ts_state_msg)

                    # Publish
                    self.plan_pub_2.publish(plan_2_msg)


    def publish_local(self):
        #TODO
        return True


#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('ltl_planner_multi_robot', anonymous=False)
    try:
        multi_robot_ltl_planner_node = MultiRobot_Planner()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("LTL Planner: "+str(e))
        rospy.logerr("LTL Planner: shutting down...")