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
from ltl_automaton_planner.ltl_tools.product import ProdAut, ProdAut_Run
from ltl_automaton_planner.ltl_tools.ltl_planner_multi_robot_exp import LocalLTLPlanner

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
from ltl_automation_a1.srv import LTLTrace


###########################################################
# Local planner node
###########################################################

class Local_Planner(object):
    def __init__(self, robot_name):
        # init parameters, automaton, etc...
        self.robot_name_ = robot_name

        self.init_params()

        self.build_automaton()

        self.setup_pub_sub()

    def init_params(self):
        #Get parameters from parameter server
        self.initial_beta = rospy.get_param('initial_beta', 1000)
        self.gamma = rospy.get_param('gamma', 10)

        # LTL task
        #----------
        # Get LTL hard task and raise error if don't exist
        if (rospy.has_param('hard_task')):
            self.hard_task = rospy.get_param('hard_task')
        else:
            raise InitError("Cannot initialize LTL Local Planner, no hard_task defined")
        # Get LTL soft task
        self.soft_task = rospy.get_param('soft_task', "")


        # Transition system
        #-------------------
        if self.robot_name_ is "dr":
            transition_system = rospy.get_param('transition_system_mobile_1_textfile')
            self.transition_system_ = import_ts_from_file(transition_system)
            self.action_topic_name_ = '/dr_0/local_planner/action_plan'
            self.trace_topic_name_ = '/dr_0/ltl_trace'
            self.replanning_topic_name_ = '/dr_0/replanning_request'
            self.global_replanning_topic_name_ = '/dr_0/global_planner/replanning_request'
            self.global_action_plan_topic_name_ = '/dr_0/global_planner/action_plan'

        elif self.robot_name_ is "a1":
            transition_system = rospy.get_param('transition_system_quadruped_textfile')
            self.transition_system_ = import_ts_from_file(transition_system)
            self.action_topic_name_ = '/a1_gazebo/local_planner/action_plan'
            self.trace_topic_name_ = '/a1_gazebo/ltl_trace'
            self.replanning_topic_name_ = '/a1_gazebo/replanning_request'
            self.global_replanning_topic_name_ = '/a1_gazebo/global_replanning_request'
            self.global_action_plan_topic_name_ = '/a1_gazebo/global_planner/action_plan'

        elif self.robot_name_ is "wassi":
            transition_system = rospy.get_param('transition_system_mobile_2_textfile')
            self.transition_system_ = import_ts_from_file(transition_system)
            self.action_topic_name_ = '/wassi_0/local_planner/action_plan'
            self.trace_topic_name_ = '/wassi_0/ltl_trace'
            self.replanning_topic_name_ = '/wassi_0/replanning_request'
            self.global_replanning_topic_name_ = '/wassi_0/global_replanning_request'
            self.global_action_plan_topic_name_ = '/wassi_0/global_planner/action_plan'

        else:
            raise InitError("Robot type is unknown")

        self.initial_state_ts_dict = None

        self.check_timestamp = True
        self.prev_received_timestamp_trace = rospy.Time()
        self.prev_received_timestamp_plan = rospy.Time()


    def build_automaton(self):
        # Import state models from TS
        state_models_ = state_models_from_ts(self.transition_system_, self.initial_state_ts_dict)

        # Here we take the product of each element of state_models to define the full TS
        self.robot_model_ = TSModel(state_models_)

        self.ltl_planner_local = LocalLTLPlanner(self.robot_model_, self.hard_task, self.soft_task, self.initial_beta, self.gamma)


        # Get first value from set
        # self.ltl_planner.curr_ts_state = list(self.ltl_planner.product.graph['ts'].graph['initial'])[0]

        # initialize storage of set of possible runs in product
        # self.ltl_planner.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])

        # show_automaton(self.robot_model)
        # show_automaton(self.ltl_planner_multi_robot.buchi)
        # show_automaton(self.ltl_planner.product)
        # show_automaton(self.ltl_planner_multi_robot.team)


    def setup_pub_sub(self):
        # local plan publisher
        self.plan_pub_ = rospy.Publisher(self.action_topic_name_, LTLPlan, latch=True, queue_size=1)

        # Global replanning request publisher (contains: 1) replanning status, 2) TS info, 3) latest local plan)
        self.global_replan_pub_ = rospy.Publisher(self.global_replanning_topic_name_, LTLPlan, latch=True, queue_size=1)

        # Initialize subscriber to provide current state of robot
        self.trace_sub_ = rospy.Subscriber(self.trace_topic_name_, LTLPlan, self.ts_trace_callback, queue_size=1)

        # Subscribe to the replanning status
        self.replan_sub_ = rospy.Subscriber(self.replanning_topic_name_, std_msgs.msg.Int8, self.ltl_replan_callback, queue_size=1)

        # Subscribe to the action plan from global planner (including the initial allocation)
        self.new_local_plan_pub_ = rospy.Subscriber(self.global_action_plan_topic_name_, LTLPlan, self.global_action_callback, queue_size=1)


    def ts_trace_callback(self, msg=LTLPlan()):
        # Extract TS state from message
        if not (self.check_timestamp and (msg.header.stamp.to_sec() == self.prev_received_timestamp_trace.to_sec())):
            # Update previously received timestamp
            self.prev_received_timestamp_trace = deepcopy(msg.header.stamp)
            self.ltl_planner_local.trace = list()
            # self.ltl_planner_multi_robot.local_replan_rname = 0

            for state_msg in msg.ts_state_sequence:
                state = handle_ts_state_msg(state_msg)

                #-------------------------
                # Check if state is in TS
                #-------------------------
                if (state in self.robot_model_.nodes()):

                    # Update trace for current robot
                    self.ltl_planner_local.trace.append(state)

                #--------------------------------------------
                # If state not part of the transition system
                #--------------------------------------------
                else:
                    #ERROR: unknown state (not part of TS)
                    rospy.logerr('State is not in TS plan!')
        else:
            rospy.logwarn("LTL Local Planner: not updating with received trace, timestamp identical to previously received message timestamp at time %f" %self.prev_received_timestamp_trace.to_sec())


    def global_action_callback(self, msg=LTLStateArray()):
        product_state_sequence = list()
        if not (self.check_timestamp and (msg.header.stamp.to_sec() == self.prev_received_timestamp_plan.to_sec())):
            # Update previously received timestamp
            self.prev_received_timestamp_plan = deepcopy(msg.header.stamp)
            for state_msg in msg.ltl_states:
                ts_state = handle_ts_state_msg(state_msg.ts_state)
                buchi_state = state_msg.buchi_state
                product_state = (ts_state, buchi_state)

                #-------------------------
                # Check if state is in TS
                #-------------------------
                if (ts_state in self.robot_model_.nodes()):

                    # Update product state sequence
                    product_state_sequence.append(product_state)

                #--------------------------------------------
                # If state not part of the transition system
                #--------------------------------------------
                else:
                    #ERROR: unknown state (not part of TS)
                    rospy.logerr('State is not in TS plan!')

            # Update the local plan according to the global planner
            self.ltl_planner_local.local_plan = ProdAut_Run(self.ltl_planner_local.product, product_state_sequence, 0)

            # Publish to the robot every time a new plan is received from global planner
            self.publish_local()

        else:
            rospy.logwarn("LTL Local Planner: not updating with received trace, timestamp identical to previously received message timestamp at time %f" %self.prev_received_timestamp_plan.to_sec())



    def ltl_replan_callback(self, msg):
        replan_status = msg.data
        if(replan_status == 0):
            rospy.logerr('LTL Local Planner: replanning ERROR; wrong replanning status received')

        if(replan_status == 1):
            # Do nothing; global planner will handle replanning level 1
            rospy.logwarn('LTL Local Planner: received replanning Level 1; do nothing; global planner will handle malfunction')

        if(replan_status == 2):
            rospy.logwarn('LTL Local Planner: received replanning Level 2: handling abrupt state change from agent 1')
            # Replan
            start = time.time()
            # if self.ltl_planner_multi_robot.local_replan_rname is not None:
            self.ltl_planner_local.local_replan_rname = 0
            rospy.loginfo("LTL Local Planner: local replan rname is: %d" %self.ltl_planner_multi_robot.local_replan_rname)
            # else:
            #     rospy.logerr("LTL Local Planner: local replan rname is not empty")

            while len(self.ltl_planner_local.trace_dic[0]) == 0:
                rospy.logwarn('Waiting for the trace callback from agent 1')

            level_flag, success = self.ltl_planner_local.replan_level_2()
            if success:
                if level_flag=="Local":
                    self.publish_local(self.ltl_planner_multi_robot.local_replan_rname)
                    rospy.logwarn('Replanning level 2 local done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic[0] = list()

                if level_flag=="Global":
                    self.publish_plan_initial()
                    rospy.logwarn('Replanning level 2 global done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic = {}
                    self.ltl_planner_multi_robot.trace_dic[0] = list()
                    self.ltl_planner_multi_robot.trace_dic[1] = list()
                    self.ltl_planner_multi_robot.trace_dic[2] = list()

                self.ltl_planner_multi_robot.local_replan_rname = None

        if(replan_status == 3):
            rospy.logwarn('LTL Local Planner: received replanning Level 3: handling transition system change')
            # Replan
            start = time.time()
            # if self.ltl_planner_multi_robot.local_replan_rname is not None:
            self.ltl_planner_multi_robot.local_replan_rname = 0
            rospy.loginfo("LTL Local Planner: local replan rname is: %d" %self.ltl_planner_multi_robot.local_replan_rname)
            # else:
            #     rospy.logerr("LTL Local Planner: local replan rname is not empty")

            while len(self.ltl_planner_multi_robot.trace_dic[0]) == 0:
                rospy.logwarn('Waiting for the trace and Updated TS callbacks from agent 1')

            level_flag, success = self.ltl_planner_multi_robot.replan_level_3()
            if success:
                if level_flag=="Local":
                    self.publish_local(self.ltl_planner_multi_robot.local_replan_rname)
                    rospy.logwarn('Replanning level 3 local done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic[0] = list()

                if level_flag=="Global":
                    self.publish_plan_initial()
                    rospy.logwarn('Replanning level 3 global done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic = {}
                    self.ltl_planner_multi_robot.trace_dic[0] = list()
                    self.ltl_planner_multi_robot.trace_dic[1] = list()
                    self.ltl_planner_multi_robot.trace_dic[2] = list()

                self.ltl_planner_multi_robot.local_replan_rname = None
                self.ltl_planner_multi_robot.update_info = {}


    def publish_local(self):
        # If plan exists
        if self.ltl_planner_local.local_plan is not None:
            # Prefix plan
            #-------------
            plan_local_msg = LTLPlan()
            plan_local_msg.header.stamp = rospy.Time.now()
            plan_local_status = False

            plan_local_msg.action_sequence = self.ltl_planner_local.local_plan.action_sequence

            for ts_state in self.ltl_planner_local.local_plan.ts_state_sequence:
                ts_state_msg = TransitionSystemState()
                ts_state_msg.state_dimension_names = [item for sublist in self.ltl_planner_local.product.graph['ts'].graph['ts_state_format'] for item in sublist]
                # If TS state is more than 1 dimension (is a tuple)
                if type(ts_state) is tuple:
                    ts_state_msg.states = list(ts_state)
                # Else state is a single string
                else:
                    ts_state_msg.states = [ts_state]
                # Add to plan TS state sequence
                plan_local_msg.ts_state_sequence.append(ts_state_msg)

            # Publish
            self.plan_pub_.publish(plan_local_msg)

        return True


#==============================
#             Main
#==============================
if __name__ == '__main__':

    if len(sys.argv) < 2:
        rospy.logerr("LTL Local Planner: not enough inputs for robot name")
    else:
        rospy.init_node('ltl_planner' + sys.argv[1], anonymous=False)
        try:
            ltl_planner_node = Local_Planner(sys.argv[1])
            rospy.spin()
        except ValueError as e:
            rospy.logerr("LTL Local Planner: "+str(e))
            rospy.logerr("LTL Local Planner: shutting down...")