import rospy

import networkx as nx
from networkx.classes.digraph import DiGraph

class TeamModel(DiGraph):
    def __init__(self, product_list, decomposition_set):
        DiGraph.__init__(self, pro_list=product_list, decomposition_set=decomposition_set, initials=set(), initial=set(), accept=set())

    def build_team(self):
        # append the robot number into the current nodes
        # for node in buchi_1.nodes:
        for idx, prod in enumerate(self.graph['pro_list']):
            for node in prod.nodes:
                ts_node = prod.nodes[node]['ts']
                buchi_node = prod.nodes[node]['buchi']
                team_node = (idx, ts_node, buchi_node)
                if not self.has_node(team_node):
                    self.add_node(team_node, rname=idx, ts=ts_node, buchi=buchi_node, label=ts_node)

                    if (prod.nodes[node]['ts'] in prod.graph['ts'].graph['initial']) and (prod.nodes[node]['buchi'] in prod.graph['buchi'].graph['initial']):
                        self.graph['initials'].add(team_node)
                        if idx == 0:
                            self.graph['initial'].add(team_node)

                    if prod.nodes[node]['buchi'] in prod.graph['buchi'].graph['accept']:
                        self.graph['accept'].add(team_node)

                for suc in prod.successors(node):
                    ts_node_suc = prod.nodes[suc]['ts']
                    buchi_node_suc = prod.nodes[suc]['buchi']
                    team_node_suc = (idx, ts_node_suc, buchi_node_suc)
                    if not self.has_node(team_node_suc):
                        self.add_node(team_node_suc, rname=idx, ts=ts_node_suc, buchi=buchi_node_suc, label=ts_node_suc)

                        if (prod.nodes[suc]['ts'] in prod.graph['ts'].graph['initial']) and (prod.nodes[suc]['buchi'] in prod.graph['buchi'].graph['initial']):
                            self.graph['initials'].add(team_node_suc)
                            if idx == 0:
                                self.graph['initial'].add(team_node_suc)

                        if prod.nodes[suc]['buchi'] in prod.graph['buchi'].graph['accept']:
                            self.graph['accept'].add(team_node_suc)

                    cost = prod.graph['ts'][ts_node][ts_node_suc]['weight']
                    action = prod.graph['ts'][ts_node][ts_node_suc]['action']
                    self.add_edge(team_node, team_node_suc, transition_cost=cost, action=action, weight=cost)

        # Add switch transition between different robots
        for node in self.nodes:
            if self.nodes[node]['rname'] == len(self.graph['pro_list'])-1:
                continue

            if self.nodes[node]['buchi'] in self.graph['decomposition_set']:
                next_rname = self.nodes[node]['rname'] + 1
                next_ts_state = None
                for next_team_init in self.graph['initials']:
                    if next_team_init[0] == next_rname:
                        next_ts_state = next_team_init[1]

                if next_ts_state == None:
                    raise AssertionError()

                next_buchi_state = self.nodes[node]['buchi']
                next_node = (next_rname, next_ts_state, next_buchi_state)

                # Check self-transition and the respective condition is fulfilled
                label = self.graph['pro_list'][self.nodes[node]['rname']].graph['ts'].nodes[self.nodes[node]['ts']]['label']
                guard = self.graph['pro_list'][self.nodes[node]['rname']].graph['buchi'].edges[next_buchi_state, next_buchi_state]['guard']

                if guard.check(label):
                    self.add_edge(node, next_node, transition_cost=0, action='switch_transition', weight=0)

        rospy.loginfo('Decomposition finished: buchi automation contains %d decomposable states' %(len(self.graph['decomposition_set'])))
        rospy.loginfo('LTL Planner Multi Robot: full team model constructed with %d states and %s transitions' %(len(self.nodes()), len(self.edges())))

    def projection(self, team_node):
        rname = self.nodes[team_node]['rname']
        ts_node = self.nodes[team_node]['ts']
        buchi_node = self.nodes[team_node]['buchi']
        return rname, ts_node, buchi_node


class Team_Run(object):
    def __init__(self, team, plan, plan_cost):
        self.team_plan = plan
        self.totalcost = plan_cost
        self.plan_output(team)

    def plan_output(self, team):
        self.action_sequence = {}
        self.state_sequence = {}
        rname_init = 0
        self.plan_local = list()
        for node in self.team_plan:
            # This is allowable since the order of robot is fixed according to switch transition
            rname, ts_node, buchi_node = team.projection(node)
            if rname > rname_init:
                self.state_sequence[rname-1] = self.plan_local
                self.plan_local = list()
                rname_init = rname
            self.plan_local.append(node)
        self.state_sequence[rname] = self.plan_local

        for r_idx, state_seq in self.state_sequence.items():
            team_edges = zip(state_seq[0:-1], state_seq[1:])
            action_sequence_local = list()
            for team_edge in team_edges:
                action_sequence_local.append(team[team_edge[0]][team_edge[1]]['action'])
            self.action_sequence[r_idx] = action_sequence_local

        for r_idx, act_seq in self.action_sequence.items():
            if len(act_seq) == 0:
                rospy.loginfo('LTL Planner: Robot-%d plan: Empty; no task assigned' %r_idx)
            else:
                rospy.loginfo('LTL Planner: Robot-%d plan: ' %r_idx + str(act_seq))

        # if self.ts_plan_sorted:
        #     self.ts_edges_sorted = list()
        #     for idx, ts_nodes in enumerate(self.ts_plan_sorted):
        #         self.ts_edges = zip(ts_nodes[0:-1], ts_nodes[1:])
        #         self.ts_edges_sorted.append(self.ts_edges)
