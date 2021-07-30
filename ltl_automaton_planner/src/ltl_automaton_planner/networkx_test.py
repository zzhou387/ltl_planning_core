import networkx as nx
import matplotlib.pyplot as plt

G = nx.Graph()
G.add_node(1)
G.add_edges_from([(1, 2, {"cost": 0.4}),(1, 3)])
node_test = G.nodes[1]
node_cost = G[1][2]['cost']
# print(G.edges)
# print(list(G.adj.items()))
# print(G.degree([2,3]))
# print(G.number_of_nodes())
# print(G[1][2]['cost'])

DG = nx.DiGraph()
DG.add_weighted_edges_from([('1', 'jj', 0.5), ('3', '1', 0.75)])
DG2 = nx.DiGraph()
DG2.add_weighted_edges_from([('1', 'pp', 0.5), ('lo', '1', 0.75)])
team = nx.union(DG, DG2, rename=('G-','H-'))
# aa = DG.successors(1)
# for aaa in aa:
#     print(aaa)
# for state in DG:
#     print(state)
# print(DG.out_degree(1))
plt.subplot(131)
nx.draw(DG)
plt.subplot(132)
nx.draw(DG2)
plt.subplot(133)
nx.draw(team)
plt.show()