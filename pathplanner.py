import matplotlib.pyplot as plt
import networkx as nx
import math, more_itertools as iter
def main():
# look at field from side of start point facing end pt
# using reference photo online for testing
    Field = nx.Graph()

    obstacles=[(2,3),
            (3,4),
            (2,6),
            (5,9),
            (7,11),
            (10,11),
            (10,14),
            (12,16),
            ]
    
    
    start_node = 14
    gates = [9,12,3]
    end_node = 2

    #path vars
    paths = []
    lens = []
    final_way = None
    final_path = [start_node,]

    #set up NetworkX
    def __init__():
        Field.add_nodes_from(range(1,16))
        for i in range(1,4):
            for j in range(0,4):
                Field.add_edge(j*4+i,j*4+i+1)
        for i in range(1,13):
            Field.add_edge(i,i+4)
        Field.remove_edges_from(obstacles)


    #pathfinding
    def path_planning():
        for tup in sorted(iter.distinct_permutations(gates)):
            paths.append(list(tup))
        for l in paths:
            l.append(end_node)
            l.insert(0, start_node)

        for path in paths:
            print(1)
            counter = 0
            for i in range(0, len(gates)+1):
                counter += nx.shortest_path_length(Field, path[i], path[i+1])
            lens.append(counter)
        final_way = paths[lens.index(min(lens))]

        #print(final_path,min(lens))
        print(final_way)
        for i in range(len(final_way)-1):
            path = list(nx.shortest_path(Field, final_way[i],final_way[i+1]))
            path.pop(0)
            for j in (path):
                final_path.append(j)
        print(final_path)
    #graph it
    def graph():
        nx.draw(Field, with_labels=True, font_weight='bold')
        print(Field)
        plt.show()
    __init__()
    path_planning()
main()