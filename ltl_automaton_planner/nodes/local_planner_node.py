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
from ltl_automaton_planner.ltl_tools.local_ltl_planner import LocalLTLPlanner

import matplotlib.pyplot as plt
import networkx as nx
from ltl_automaton_planner.ltl_automaton_utilities import state_models_from_ts, import_ts_from_file, handle_ts_state_msg

# Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLState, LTLStateArray,\
                                   GlobalReplanInfo, UpdateInfo, TransitionSystemInfo
from ltl_automaton_msgs.srv import TaskPlanning, TaskPlanningResponse
from networkx.drawing.nx_agraph import to_agraph

# Import dynamic reconfigure components for dynamic parameters (see dynamic_reconfigure and dynamic_params package)
from dynamic_reconfigure.server import Server as DRServer
from ltl_automaton_planner.cfg import LTLAutomatonDPConfig
from ltl_automaton_bt.srv import LTLTrace


###########################################################
# Local planner node
###########################################################

class Local_Planner(object):
    def __init__(self):
        # init parameters, automaton, etc...
        if (rospy.has_param("~robot_name")):
            self.robot_name_ = rospy.get_param("~robot_name")
            rospy.logwarn(self.robot_name_)
        else:
            # self.robot_name_ = "dr"
            # rospy.logwarn(self.robot_name_)
            raise InitError("Cannot initialize LTL Local Planner, no robot name provided")

        self.init_params()

        self.setup_pub_sub()

        self.build_automaton()


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

        if (rospy.has_param('homing_task')):
            self.homing_task = rospy.get_param('homing_task')
        else:
            self.homing_task = "<>(reception && standby)"


        # Transition system
        #-------------------
        if self.robot_name_ == "dr":
            transition_system = rospy.get_param('transition_system_mobile_1_textfile')
            self.transition_system_ = import_ts_from_file(transition_system)
            self.action_topic_name_ = '/dr_0/local_planner/action_plan'
            self.homing_action_topic_name_ = '/dr_0/local_planner/homing_action_plan'
            self.trace_topic_name_ = '/dr_0/ltl_trace'
            self.replanning_topic_name_ = '/dr_0/replanning_request'
            self.global_replanning_topic_name_ = '/dr_0/global_replanning_request'
            self.global_action_plan_topic_name_ = '/dr_0/global_planner/action_plan'

        elif self.robot_name_ == "a1":
            transition_system = rospy.get_param('transition_system_quadruped_textfile')
            self.transition_system_ = import_ts_from_file(transition_system)
            self.action_topic_name_ = '/a1_gazebo/local_planner/action_plan'
            self.homing_action_topic_name_ = '/a1_gazebo/local_planner/homing_action_plan'
            self.trace_topic_name_ = '/a1_gazebo/ltl_trace'
            self.replanning_topic_name_ = '/a1_gazebo/replanning_request'
            self.global_replanning_topic_name_ = '/a1_gazebo/global_replanning_request'
            self.global_action_plan_topic_name_ = '/a1_gazebo/global_planner/action_plan'

        elif self.robot_name_ == "wassi":
            transition_system = rospy.get_param('transition_system_mobile_2_textfile')
            self.transition_system_ = import_ts_from_file(transition_system)
            self.action_topic_name_ = '/wassi_0/local_planner/action_plan'
            self.homing_action_topic_name_ = '/wassi_0/local_planner/homing_action_plan'
            self.trace_topic_name_ = '/wassi_0/ltl_trace'
            self.replanning_topic_name_ = '/wassi_0/replanning_request'
            self.global_replanning_topic_name_ = '/wassi_0/global_replanning_request'
            self.global_action_plan_topic_name_ = '/wassi_0/global_planner/action_plan'

        elif self.robot_name_ == "turtlebot_08":
            transition_system = rospy.get_param('transition_system_turtlebot_08_textfile')
            self.transition_system_ = import_ts_from_file(transition_system)
            self.action_topic_name_ = '/turtlebot_08/local_planner/action_plan'
            self.homing_action_topic_name_ = '/turtlebot_08/local_planner/homing_action_plan'
            self.trace_topic_name_ = '/turtlebot_08/ltl_trace'
            self.replanning_topic_name_ = '/turtlebot_08/replanning_request'
            self.global_replanning_topic_name_ = '/turtlebot_08/global_replanning_request'
            self.global_action_plan_topic_name_ = '/turtlebot_08/global_planner/action_plan'

        elif self.robot_name_ == "turtlebot_14":
            transition_system = rospy.get_param('transition_system_turtlebot_14_textfile')
            self.transition_system_ = import_ts_from_file(transition_system)
            self.action_topic_name_ = '/turtlebot_14/local_planner/action_plan'
            self.homing_action_topic_name_ = '/turtlebot_14/local_planner/homing_action_plan'
            self.trace_topic_name_ = '/turtlebot_14/ltl_trace'
            self.replanning_topic_name_ = '/turtlebot_14/replanning_request'
            self.global_replanning_topic_name_ = '/turtlebot_14/global_replanning_request'
            self.global_action_plan_topic_name_ = '/turtlebot_14/global_planner/action_plan'

        elif self.robot_name_ == "mini_cheetah":
            transition_system = rospy.get_param('transition_system_mini_cheetah_textfile')
            self.transition_system_ = import_ts_from_file(transition_system)
            self.action_topic_name_ = '/mini_cheetah/local_planner/action_plan'
            self.homing_action_topic_name_ = '/mini_cheetah/local_planner/homing_action_plan'
            self.trace_topic_name_ = '/mini_cheetah/ltl_trace'
            self.replanning_topic_name_ = '/mini_cheetah/replanning_request'
            self.global_replanning_topic_name_ = '/mini_cheetah/global_replanning_request'
            self.global_action_plan_topic_name_ = '/mini_cheetah/global_planner/action_plan'

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

        self.ltl_planner_local = LocalLTLPlanner(self.robot_model_, self.hard_task, self.soft_task, self.homing_task, self.initial_beta, self.gamma)


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

        # local plan publisher
        self.homing_plan_pub_ = rospy.Publisher(self.homing_action_topic_name_, LTLPlan, latch=True, queue_size=1)

        # Global replanning request publisher (contains: 1) replanning status, 2) TS info, 3) latest local plan)
        self.global_replan_pub_ = rospy.Publisher(self.global_replanning_topic_name_, GlobalReplanInfo, latch=True, queue_size=1)

        # Initialize subscriber to provide current state of robot
        self.trace_sub_ = rospy.Subscriber(self.trace_topic_name_, LTLPlan, self.ts_trace_callback, queue_size=1)

        # Subscribe to the replanning status
        self.replan_sub_ = rospy.Subscriber(self.replanning_topic_name_, std_msgs.msg.Int8, self.ltl_replan_callback, queue_size=1)

        # Subscribe to the action plan from global planner (including the initial allocation)
        self.new_local_plan_pub_ = rospy.Subscriber(self.global_action_plan_topic_name_, LTLStateArray, self.global_action_callback, queue_size=1)


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
            # Send global replanning request to the global planner (UpdateInfo is handled in global planner for now)
            while len(self.ltl_planner_local.trace) == 0:
                rospy.logwarn('Waiting for the trace callback from current agent')

            self.ltl_planner_local.replan_level_1()
            self.publish_global_replanning_request(1)
            rospy.logwarn('LTL Local Planner: received replanning Level 1; do nothing; global planner will handle malfunction')
            self.ltl_planner_local.trace = list()

        if(replan_status == 2):
            rospy.logwarn('LTL Local Planner: received replanning Level 2: handling abrupt state change')
            # Replan
            start = time.time()

            while len(self.ltl_planner_local.trace) == 0:
                rospy.logwarn('Waiting for the trace callback from current agent')

            if self.ltl_planner_local.replan_level_2():
                self.publish_local()
                rospy.logwarn('Replanning level 2 local done within %.2fs' %(time.time()-start))
                # Send the new local plan to the global planner
                self.publish_global_replanning_request(0, self.ltl_planner_local.local_plan.ts_state_sequence,
                                                       self.ltl_planner_local.local_plan.buchi_state_sequence)
                self.ltl_planner_local.trace = list()

            else:
                # Send global replanning request to global planner
                rospy.logwarn("LTL Local Planner: sending request to global planner for reallocation!")
                self.publish_global_replanning_request(2)
                self.ltl_planner_local.trace = list()


        if(replan_status == 3):
            rospy.logwarn('LTL Local Planner: received replanning Level 3: handling transition system change')
            # Replan
            start = time.time()

            while len(self.ltl_planner_local.trace) == 0:
                rospy.logwarn('Waiting for the trace callbacks from current agent')

            if self.ltl_planner_local.replan_level_3():
                self.publish_local()
                rospy.logwarn('Replanning level 3 local done within %.2fs' %(time.time()-start))
                # Send the new local plan to the global planner
                self.publish_global_replanning_request(0, self.ltl_planner_local.local_plan.ts_state_sequence,
                                                       self.ltl_planner_local.local_plan.buchi_state_sequence,
                                                       self.ltl_planner_local.update_info)

            else:
                # Send global replanning request to global planner
                rospy.logwarn("LTL Local Planner: sending request to global planner for reallocation!")
                rospy.logwarn("LTL Local Planner: updating info is: "+str(self.ltl_planner_local.update_info))
                self.publish_global_replanning_request(3, update_info=self.ltl_planner_local.update_info)

            self.ltl_planner_local.trace = list()
            self.ltl_planner_local.update_info = {}

        if(replan_status == 4):
            rospy.logwarn('LTL Local Planner: received homing signal')
            while len(self.ltl_planner_local.trace) == 0:
                rospy.logwarn('Waiting for the trace callbacks from current agent')

            if self.ltl_planner_local.local_task_reallocate(style="homing"):
                self.publish_homing()

            else:
                rospy.logwarn("LTL Local Planner: homing path not found!")

            self.ltl_planner_local.trace = list()



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


    def publish_homing(self):
        # If plan exists
        if self.ltl_planner_local.homing_plan is not None:
            # Prefix plan
            #-------------
            plan_local_msg = LTLPlan()
            plan_local_msg.header.stamp = rospy.Time.now()
            plan_local_status = False

            plan_local_msg.action_sequence = self.ltl_planner_local.homing_plan.action_sequence

            for ts_state in self.ltl_planner_local.homing_plan.ts_state_sequence:
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
            self.homing_plan_pub_.publish(plan_local_msg)

        return True


    # this function handles both plan synchronization and global reallocation request
    def publish_global_replanning_request(self, replan_level, ts_seq=None, buchi_seq=None, update_info=None):
        global_replanning_msg = GlobalReplanInfo()
        global_replanning_msg.header.stamp = rospy.Time.now()
        global_replanning_msg.level = replan_level
        if replan_level == 0:
            # level 0 is for sending successful local plans
            if (ts_seq is not None) and (buchi_seq is not None):
                for ts_state, buchi_state in zip(ts_seq, buchi_seq):
                    ltl_state_msg = LTLState()
                    ts_state_msg = TransitionSystemState()
                    ts_state_msg.state_dimension_names = [item for sublist in self.ltl_planner_local.product.graph['ts'].graph['ts_state_format'] for item in sublist]
                    # If TS state is more than 1 dimension (is a tuple)
                    if type(ts_state) is tuple:
                        ts_state_msg.states = list(ts_state)
                    # Else state is a single string
                    else:
                        ts_state_msg.states = [ts_state]
                    # Add to plan TS state sequence
                    ltl_state_msg.ts_state = ts_state_msg
                    ltl_state_msg.buchi_state = buchi_state

                    global_replanning_msg.ltl_states.append(ltl_state_msg)

            else:
                raise InitError("buchi or TS sequence is empty")

            # TS could change
            if update_info is not None:
                update_info_msg = UpdateInfo()
                for added_pair in list(update_info["added"]):
                    trans_info_msg = TransitionSystemInfo()
                    for tt in added_pair:
                        ts_msg = TransitionSystemState()
                        ts_msg.state_dimension_names = [item for sublist in
                                                              self.ltl_planner_local.product.graph['ts'].graph['ts_state_format'] for item
                                                              in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(tt) is tuple:
                            ts_msg.states = list(tt)
                        # Else state is a single string
                        else:
                            ts_msg.states = [tt]

                        trans_info_msg.ts_pair.append(ts_msg)

                    update_info_msg.added_ts.append(trans_info_msg)

                for relable_pair in list(update_info["relabel"]):
                    trans_info_msg = TransitionSystemInfo()
                    for tt in relable_pair:
                        ts_msg = TransitionSystemState()
                        ts_msg.state_dimension_names = [item for sublist in
                                                        self.ltl_planner_local.product.graph['ts'].graph['ts_state_format'] for item
                                                        in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(tt) is tuple:
                            ts_msg.states = list(tt)
                        # Else state is a single string
                        else:
                            ts_msg.states = [tt]

                        trans_info_msg.ts_pair.append(ts_msg)

                    update_info_msg.relabel_ts.append(trans_info_msg)

                for deleted_pair in list(update_info["deleted"]):
                    trans_info_msg = TransitionSystemInfo()
                    for tt in deleted_pair:
                        ts_msg = TransitionSystemState()
                        ts_msg.state_dimension_names = [item for sublist in
                                                        self.ltl_planner_local.product.graph['ts'].graph['ts_state_format'] for item
                                                        in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(tt) is tuple:
                            ts_msg.states = list(tt)
                        # Else state is a single string
                        else:
                            ts_msg.states = [tt]

                        trans_info_msg.ts_pair.append(ts_msg)

                    update_info_msg.deleted_ts.append(trans_info_msg)

                global_replanning_msg.update_info = update_info_msg

        else:
            # the rest cases will require a global reallocation and only UpdateInfo is possibly needed
            if update_info is not None:
                update_info_msg = UpdateInfo()
                for added_pair in list(update_info["added"]):
                    trans_info_msg = TransitionSystemInfo()
                    for tt in added_pair:
                        ts_msg = TransitionSystemState()
                        ts_msg.state_dimension_names = [item for sublist in
                                                        self.ltl_planner_local.product.graph['ts'].graph['ts_state_format'] for item
                                                        in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(tt) is tuple:
                            ts_msg.states = list(tt)
                        # Else state is a single string
                        else:
                            ts_msg.states = [tt]

                        trans_info_msg.ts_pair.append(ts_msg)

                    update_info_msg.added_ts.append(trans_info_msg)

                for relable_pair in list(update_info["relabel"]):
                    trans_info_msg = TransitionSystemInfo()
                    for tt in relable_pair:
                        ts_msg = TransitionSystemState()
                        ts_msg.state_dimension_names = [item for sublist in
                                                        self.ltl_planner_local.product.graph['ts'].graph['ts_state_format'] for item
                                                        in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(tt) is tuple:
                            ts_msg.states = list(tt)
                        # Else state is a single string
                        else:
                            ts_msg.states = [tt]

                        trans_info_msg.ts_pair.append(ts_msg)

                    update_info_msg.relabel_ts.append(trans_info_msg)

                for deleted_pair in list(update_info["deleted"]):
                    trans_info_msg = TransitionSystemInfo()
                    for tt in deleted_pair:
                        ts_msg = TransitionSystemState()
                        ts_msg.state_dimension_names = [item for sublist in
                                                        self.ltl_planner_local.product.graph['ts'].graph['ts_state_format'] for item
                                                        in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(tt) is tuple:
                            ts_msg.states = list(tt)
                        # Else state is a single string
                        else:
                            ts_msg.states = [tt]

                        trans_info_msg.ts_pair.append(ts_msg)

                    update_info_msg.deleted_ts.append(trans_info_msg)

                global_replanning_msg.update_info = update_info_msg

        self.global_replan_pub_.publish(global_replanning_msg)


#==============================
#             Main
#==============================
if __name__ == '__main__':

    rospy.init_node('local_ltl_planner', anonymous=False)
    try:
        ltl_planner_node = Local_Planner()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("LTL Local Planner: "+str(e))
        rospy.logerr("LTL Local Planner: shutting down...")