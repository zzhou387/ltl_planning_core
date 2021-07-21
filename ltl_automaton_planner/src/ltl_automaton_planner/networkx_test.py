import networkx as nx
import matplotlib.pyplot as plt

G = nx.Graph()
G.add_node(1)
G.add_edges_from([(1, 2, {"cost": 0.4}),(1, 3)])
print(G.edges)
print(list(G.adj.items()))
print(G.degree([2,3]))
print(G.number_of_nodes())
print(G[1][2]['cost'])

DG = nx.DiGraph()
DG.add_weighted_edges_from([(1, 2, 0.5), (3, 1, 0.75)])
print(DG.out_degree(1))
plt.subplot(121)
nx.draw(DG)
plt.show()