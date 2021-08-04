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
                    self.add_edge(team_node, team_node_suc, transition_cost=cost, action=action)

        # Add switch transition between different robots
        for node in self.nodes:
            if self.nodes[node]['rname'] == len(self.graph['pro_list'])-1:
                continue

            if self.nodes[node]['buchi'] in self.graph['decompose_set']:
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
                    self.add_edge(node, next_node, transition_cost=0, action='switch_transition')

    def projection(self, team_node):
        rname = self.nodes[team_node]['rname']
        ts_node = self.nodes[team_node]['ts']
        buchi_node = self.nodes[team_node]['buchi']


