import osmium
import networkx as nx
import matplotlib.pyplot as plt

class OSMHandler(osmium.SimpleHandler):
    def __init__(self, graph):
        super(OSMHandler, self).__init__()
        self.graph = graph
        self.added_nodes = set()
        self.multiple_edges_nodes = []

    def node(self, n):
        self.graph.add_node(n.id, lon=n.location.lon, lat=n.location.lat)
        self.added_nodes.add(n.id)

    def way(self, w):
        nodes = w.nodes
        if len(nodes) < 2:
            return

        if nodes[-1].ref in self.added_nodes and nodes[0].ref in self.added_nodes:
            self.multiple_edges_nodes.append(nodes[0].ref)

        for i in range(1, len(nodes)):
            if nodes[i - 1].ref in self.added_nodes and nodes[i].ref in self.added_nodes:
                weight = 1.0
                for tag in w.tags:
                    if tag.k == "weight":
                        weight = float(tag.v)
                        break
                if "oneway" in w.tags and w.tags["oneway"] == "yes":
                    self.graph.add_edge(nodes[i - 1].ref, nodes[i].ref, weight=weight)
                else:
                    self.graph.add_edge(nodes[i - 1].ref, nodes[i].ref, weight=weight)
                    self.graph.add_edge(nodes[i].ref, nodes[i - 1].ref, weight=weight)
                    self.multiple_edges_nodes.append(nodes[i].ref)

    def get_multiple_edges_nodes(self):
        return self.multiple_edges_nodes

# Crear el grafo
graph = nx.MultiDiGraph()

# Crear el manejador de OSM y aplicarlo al archivo OSM
osm_handler = OSMHandler(graph)
osm_handler.apply_file("map.osm")

# Obtener los nodos con múltiples aristas
multiple_edges_nodes = osm_handler.get_multiple_edges_nodes()
print("Nodos con múltiples aristas:", multiple_edges_nodes)

# print(graph.nodes)

# # Verificar si todos los nodos están conectados
# if not nx.is_connected(graph.to_undirected()):
#     print("No se puede alcanzar todos los nodos desde los nodos anteriores.")
