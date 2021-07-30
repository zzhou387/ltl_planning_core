import rospy
import networkx as nx
from networkx.algorithms.simple_paths import is_simple_path, _all_simple_paths_graph, _all_simple_paths_multigraph
from networkx.classes.digraph import DiGraph


def get_decomposition_set(buchi):
    decomp_set = list()
    initial = buchi.graph['initial']
    final = buchi.graph['accept']
    if (not len(initial) == 1) or (not len(final) == 1):
        rospy.logerr("INITIAL AND FINAL DIMENSION ERROR")
    decomp_set += [initial[0]]
    decomp_set += [final[0]]
    debug1 = buchi.nodes
    for b_node in buchi.nodes:
        if b_node == initial[0] or b_node == final[0]:
            continue

        try:
            seq_1 = nx.all_simple_paths(buchi, initial[0], b_node)
            seq_2 = nx.all_simple_paths(buchi, b_node, final[0])
            tau_1 = list()
            tau_2 = list()
            # for path_1 in map(nx.utils.pairwise, seq_1):
            path_1 = map(nx.utils.pairwise, seq_1)[0]
            for edge in path_1:
                aa = buchi.edges[edge]
                tau_1 += [buchi.edges[edge]['guard_formula']]

            # for path_2 in map(nx.utils.pairwise, seq_2):
            path_2 = map(nx.utils.pairwise, seq_2)[0]
            for edge in path_2:
                tau_2 += [buchi.edges[edge]['guard_formula']]

            tau = tau_2 + tau_1
            current_node = initial[0]
            essential_run = list()
            essential_run += [current_node]


            for tau_00 in tau:
                selected = False
                for succ in buchi.successors(current_node):
                    debug0 = [buchi.edges[current_node, succ]['guard_formula']]
                    transition_can = buchi.edges[current_node, succ]['guard_formula']
                    if transition_can == tau_00:
                        essential_run += [succ]
                        current_node = unicode(succ)
                        selected = True
                        break

                if (not selected) and (current_node in buchi.successors(current_node)) and (buchi.edges[current_node, current_node]['guard_formula'] == unicode("(1)") or buchi.edges[current_node, current_node]['guard_formula'] == "1"):
                    essential_run += [current_node]
                    selected = True

                if not selected:
                    rospy.logerr("ERROR WHEN EXPANDING THE SEQUENCE")
                    print(b_node)
                    print('Current node:')
                    print(current_node)

            if final[0] in essential_run:
                decomp_set += [b_node]

        except:
            rospy.logerr("ERROR WHEN DETERMINING THE DECOMPOSITION SET:")
            print(b_node)
            continue
    return decomp_set
