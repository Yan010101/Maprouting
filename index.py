from flask import Flask, render_template, request

# from utilidades import *

# from exportar_osm_copy import *

from nodo_aproximado_2 import *

from Dijsktra import *

import json
from general_mapa import *
from geopy.distance import geodesic
from importar_osm import *
import networkx as nx
from general_mapa import *

import math




app = Flask(__name__)


pagina_entrada = open('index.html', 'r').read()




@app.route('/', methods=['GET', 'POST'])

def index():


    if request.method == 'POST':

        coordinates_string = request.form.get('coordinates')  # Suponiendo que estás obteniendo el valor del formulario en Flask con request.form
        lista_cordenadas = json.loads(coordinates_string)

        # print (lista_cordenadas[0])
        


        aa = lista_cordenadas[0]["lat"]
        ab =  lista_cordenadas[0]["lng"]

        grafo = nx.DiGraph()
        # grafo2 = nx.DiGraph()
        

        osm_handler = OSMHandler(grafo)
        # osm_handler2 = OSMHandler2(grafo2)

        osm_handler.apply_file("map.osm")
        # osm_handler2.apply_file("map.osm")
        
        


        ruta_inicial = find_closest_node(grafo, lista_cordenadas[0]["lat"], lista_cordenadas[0]["lng"])
        
        destino = find_closest_node(grafo, lista_cordenadas[1]["lat"], lista_cordenadas[1]["lng"])
        
        print(ruta_inicial , destino)
        
        
        # print (ruta_inicial)



        ruta =  ruta_dijkstra(grafo , ruta_inicial , destino , weight='weight')
        print(ruta[0])
        
          # Reemplaza con tu resultado de Dijkstra

        ruta_cordenadas = []
        for node_id in ruta[0]:
            node = graph.nodes[node_id]
            ruta_cordenadas.append([node['lat'], node['lon']])

        print(ruta_cordenadas)
        
        exportar(ruta_cordenadas)

        
        
        # general(ee,grafo,ruta_dijkstra)
        




    return pagina_entrada






if __name__ == '__main__':
    app.run()

