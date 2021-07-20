import networkx as nx

G = nx.Graph()
G.add_node(1)
G.add_edges_from([(1, 2),(1, 3)])
print(G.edges)
print(list(G.adj.items()))
print(G.degree([2,3]))
print(G.number_of_nodes())