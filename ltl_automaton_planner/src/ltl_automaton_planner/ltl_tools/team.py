import networkx as nx
from networkx.classes.digraph import DiGraph

class TeamModel(DiGraph):
    def __init__(self, product_list):
        self.product_list = product_list

    def build_team(self):
        if len(self.product_list) == 1:
            DiGraph.__init__(self.product_list)
        else:
            for product in self.product_list:



