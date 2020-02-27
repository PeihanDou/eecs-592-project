'''
Search Route for route planning problem and TSP problem
'''

import math
import numpy as np
R = 3959
DEBUG = False

class TreeNode:
    '''
    Tree node
    '''
    city = ''
    explored = []
    parent = None
    def __init__(self, input):
        self.city = input
        self.explored = []
        parent = None
    def state(self):
        return [self.city, self.explored]

class MyGraph:
    '''
    graph class
    '''
    node = []
    edge = []
    loc = {}

    def find_short(self):
        d = []
        for i in self.edge:
            d.append(i[-1]['weight'])
        return min(d)

    def whole_distance(self):
        all_way = 0
        for i in self.edge:
            all_way = all_way+i[-1]['weight']
        return all_way

    def find_neighber(self, Node):
        '''
        find all neighbers of a node
        '''
        Neighber = []
        for i in self.edge:
            if (i[0] == Node)and(i[1] not in Neighber):
                Neighber.append(i[1])
            if (i[1] == Node)and(i[0] not in Neighber):
                Neighber.append(i[0])
        return Neighber

    def import_graph_from_files(self, file_name):
        '''
        Input data from txt looks like:
        node1_name, node2_name, distance
        node1_name, [node3_name, distance
        ...
        node3_name, node5_name, distance
        ...

        Note that the distance is united by miles
        '''
        f = open(file_name, "r")
        li = []
        part = []
        for i in f:
            li.append(i)
            part.append(li[-1].split(", "))

        for i in part:
            if i[0] not in self.node:
                self.node.append(i[0])
            if i[1] not in self.node:
                self.node.append(i[1])
            self.connect(i[0], i[1], float(i[2]))
        print("Input finish")
        print("There are", self.node_number(), "nodes in the graph.")
        print("There are", self.edge_number(), "edges in the graph.")

        f.close()

    def import_loc_from_files(self, file_name):
        '''
        Input locations of citys from txt
        '''
        f = open(file_name, 'r')
        li = []
        part = []
        for i in f:
            li.append(i)
            part.append(li[-1].split(", "))
        for i in part:
            self.loc[i[0]] = [float(i[1]), float(i[2])]
        print('Input locations finish')

        f.close()



    def connect(self, a, b, dist):
        '''
        Make connection between a and b
        '''
        if ([a, b, {'weight': dist}] in self.edge)or([b, a, {'weight': dist}] in self.edge):
            print("These two nodes already have connection")
            return
        if a not in self.node:
            self.node.append(a)
        if b not in self.node:
            self.node.append(b)
        self.edge.append([a, b, {'weight': float(dist)}])

    def node_number(self):
        '''
        count the node number
        '''
        if len(self.node) == 0:
            print("There is no node in the graph")
            return None
        else:
            return len(self.node)

    def edge_number(self):
        '''
        count the edges number
        '''
        if len(self.edge) == 0:
            print("There is no edge in the graph")
            return None
        else:
            return len(self.edge)

    def cost(self, a, b):
        '''
        This function return the distance between a and b
        '''
        for i in self.edge:
            if (a in i)and(b in i):
                return i[2]['weight']
        print("There is no connection between these two nodes")
        return None

    def distance(self, a, b):
        '''
        Return the absolute straight line distance between a & b (miles)
        '''
        if (a in self.node)and(b in self.node):
            ax, ay, az = self.convert_to_axis(a)
            bx, by, bz = self.convert_to_axis(b)
            return ((ax-bx)**2 + (ay-by)**2 + (az-bz)**2)**(0.5)
        return None

    def convert_to_axis(self, a):
        '''
        Convert latitude/longtitude to ECEF coordinates
        '''
        if a in self.node:

            a_lat = self.loc[a][0]*(np.pi/180)
            a_long = self.loc[a][1]*(np.pi/180)
            ax = math.cos(a_lat)*math.cos(a_long)
            ay = math.cos(a_lat)*math.sin(a_long)
            az = math.sin(a_lat)
            return ax * R, ay * R, az * R

def find_path_astar_tsp(graph, start, nb, edge):
    '''
    Find the path from start to dest in TSP using A* search. Also calculate the cost
    '''
    pure_path = []
    path = []
    path.append(nb)
    cost = 0
    heu = (graph.node_number() - len(nb.explored))*edge
    format = ''
    while path[-1] != start:
        path.append(path[-1].parent)
    path.reverse()
    for i in range(len(path)-1):
        cost = cost+graph.cost(path[i].city, path[i+1].city)
    for i in path:
        pure_path.append(i.city)
        if i.city != path[0].city:
            format = format+'->'+i.city
        else:
            format = format+i.city
    tot_cost = cost + heu

    return format, tot_cost

def find_path_tsp(graph, start, nb):
    '''
    Find the path from start to dest in TSP. Also calculate the cost
    '''
    pure_path = []
    path = []
    path.append(nb)
    cost = 0
    format = ''
    while path[-1] != start:
        path.append(path[-1].parent)
    path.reverse()
    for i in range(len(path)-1):
        cost = cost+graph.cost(path[i].city, path[i+1].city)
    for i in path:
        pure_path.append(i.city)
        if i.city != path[0].city:
            format = format+'->'+i.city
        else:
            format = format+i.city
    return format, cost

def find_path(graph, pre, start, dest):
    '''
    Find the path from start to dest. Also calculate the cost
    '''
    path = [dest]
    cost = 0
    format = start
    while path[-1] != start:
        path.append(pre[path[-1]])
    path.reverse()
    for i in range(len(path)-1):
        cost = cost+graph.cost(path[i], path[i+1])
        format = format + '->' + path[i+1]
    return format, cost

def find_path_astar(graph, pre, start, node, dest):
    '''
    Find the path from start to dest. Also calculate the A* cost: cost + heuristic cost
    '''
    path = [node]
    cost = 0
    sld_cost = graph.distance(node, dest)
    format = start
    while path[-1] != start:
        path.append(pre[path[-1]])
    path.reverse()
    for i in range(len(path)-1):
        cost = cost+graph.cost(path[i], path[i+1])
        format = format+"->"+path[i+1]
    tot_cost = cost + sld_cost
    return format, tot_cost

def general_search(graph, start, dest, operation='B'):
    '''
    General search. Find the path (and cost if needed) using the way spercific by the operation.
    Default operation is BFS
    You can choose:
    B for BFS;
    D for DFS;
    I for Iterative deepening DFS;
    U for Uniform cost search;
    A for A* search
    '''
    if operation == 'B':
        pre = {}
        queue = []
        queue.append(start)
        explored = []
        expand = 0
        while queue:
            for i in queue:
                if i not in explored:
                    explored.append(i)
            node = queue.pop(0)
            for i in graph.find_neighber(node):
                expand += 1
                if (i not in explored)and(i not in queue):
                    queue.append(i)
                    pre[i] = node
                    if i == dest:
                        return [find_path(graph, pre, start, i), expand]
    if operation == 'D':
        pre = {}
        queue = []
        queue.append(start)
        explored = []
        expand = 0
        while queue:
            for i in queue:
                if i not in explored:
                    explored.append(i)
            node = queue.pop(-1)
            if node == dest:
                return [find_path(graph, pre, start, node), expand]
            for i in graph.find_neighber(node):
                expand += 1
                if (i not in explored)and(i not in queue):
                    queue.append(i)
                    pre[i] = node

    if operation == 'I':
        l = 0
        expand = 0
        while l < 15:
            depth = 0
            pre = {}
            queue = []
            queue.append(start)
            explored = []
            while queue:
                for i in queue:
                    if i not in explored:
                        explored.append(i)
                node = queue.pop(-1)
                back_path, _ = find_path(graph, pre, start, node)
                depth = len(back_path.split("->")) - 1
                if depth < l:
                    for i in graph.find_neighber(node):
                        expand += 1
                        if (i not in explored)and(i not in queue):
                            queue.append(i)
                            pre[i] = node
                            if i == dest:
                                return [find_path(graph, pre, start, i), expand]
            l += 1

    if operation == 'U':
        pre = {}
        queue = []
        cost_queue = [0]
        queue.append(start)
        explored = []
        expand = 0
        while queue:
            node = queue.pop(0)
            waste = cost_queue.pop(0)
            if node not in explored:
                explored.append(node)
            if node == dest:
                return [find_path(graph, pre, start, node), expand]
            for i in graph.find_neighber(node):
                expand += 1
                if (i not in explored)and(i not in queue):
                    pre[i] = node
                    _, cost = find_path(graph, pre, start, i)
                    if (len(cost_queue) == 0)or(cost >= max(cost_queue)):
                        cost_queue.append(cost)
                        queue.append(i)
                    else:
                        for j in range(len(cost_queue)):
                            if cost <= cost_queue[j]:
                                cost_queue.insert(j, cost)
                                queue.insert(j, i)
                                break
                elif i in queue:
                    _, old_cost = find_path(graph, pre, start, i)
                    old_pre = pre[i]
                    pre[i] = node
                    _, new_cost = find_path(graph, pre, start, i)
                    #Just for debug
                    #print("Find another path:",new_cost)
                    if new_cost > old_cost:
                        pre[i] = old_pre

    if operation == 'A':
        pre = {}
        queue = []
        cost_queue = [graph.distance(start, dest)]
        queue.append(start)
        explored = []
        expand = 0
        while queue:
            node = queue.pop(0)
            waste = cost_queue.pop(0)
            if node not in explored:
                explored.append(node)
            if node == dest:
                return [find_path_astar(graph, pre, start, node, dest), expand]
            for i in graph.find_neighber(node):
                expand += 1
                if (i not in explored)and(i not in queue):
                    pre[i] = node
                    _, cost = find_path_astar(graph, pre, start, i, dest)
                    if (len(cost_queue) == 0)or(cost >= max(cost_queue)):
                        cost_queue.append(cost)
                        queue.append(i)
                    else:
                        for j in range(len(cost_queue)):
                            if cost <= cost_queue[j]:
                                cost_queue.insert(j, cost)
                                queue.insert(j, i)
                                break
                elif i in queue:
                    _, old_cost = find_path_astar(graph, pre, start, i, dest)
                    old_pre = pre[i]
                    pre[i] = node
                    _, new_cost = find_path_astar(graph, pre, start, i, dest)
                    #Just for debug
                    #print("Find better path:",new_cost)
                    if new_cost > old_cost:
                        pre[i] = old_pre

def solve_tsp(graph, start, method='B'):
    '''
    Solve the TSP problem
    '''
    if method == 'B':
        queue = []
        queue_state = []
        start_state = TreeNode(start)
        start_state.explored.append(start_state.city)
        queue.append(start_state)
        queue_state.append(start_state.state())
        explored_state = []
        expand = 0
        while queue:
            node = queue.pop(0)
            node_state = queue_state.pop(0)
            explored_state.append([node_state[0], node_state[1]])
            for i in graph.find_neighber(node_state[0]):
                expand += 1
                nb = TreeNode(i)
                nb.explored = [] + node_state[1]
                if nb.city not in nb.explored:
                    nb.explored.append(nb.city)
                nb.parent = node
                if (nb.state() not in explored_state)and(nb.state() not in queue_state):
                    if len(nb.explored) == len(graph.node):
                        return [find_path_tsp(graph, start_state, nb), expand]
                    queue_state.append(nb.state())
                    queue.append(nb)
    if method == 'D':
        queue = []
        queue_state = []
        start_state = TreeNode(start)
        start_state.explored.append(start_state.city)
        queue.append(start_state)
        queue_state.append(start_state.state())
        explored_state = []
        expand = 0
        while queue:
            node = queue.pop(-1)
            node_state = queue_state.pop(-1)
            explored_state.append([node_state[0], node_state[1]])
            for i in graph.find_neighber(node_state[0]):
                expand += 1
                nb = TreeNode(i)
                nb.explored = [] + node_state[1]
                if nb.city not in nb.explored:
                    nb.explored.append(nb.city)
                nb.parent = node
                if (nb.state() not in explored_state)and(nb.state() not in queue_state):
                    if len(nb.explored) == len(graph.node):
                        return [find_path_tsp(graph, start_state, nb), expand]
                    queue_state.append(nb.state())
                    queue.append(nb)
    if method == 'I':
        l = 0
        while l < 15:    
            depth = 0
            queue = []
            queue_state = []
            start_state = TreeNode(start)
            start_state.explored.append(start_state.city)
            queue.append(start_state)
            queue_state.append(start_state.state())
            explored_state = []
            expand = 0
            while queue:
                node = queue.pop(-1)
                node_state = queue_state.pop(-1)
                explored_state.append([node_state[0], node_state[1]])
                back_path, _ = find_path_tsp(graph, start_state, node)
                depth = len(back_path.split("->"))-1
                if depth < l:
                    for i in graph.find_neighber(node_state[0]):
                        expand += 1
                        nb = TreeNode(i)
                        nb.explored = [] + node_state[1]
                        if nb.city not in nb.explored:
                            nb.explored.append(nb.city)
                        nb.parent = node
                        if (nb.state() not in explored_state)and(nb.state() not in queue_state):
                            if len(nb.explored) == len(graph.node):
                                return [find_path_tsp(graph, start_state, nb), expand]
                            queue_state.append(nb.state())
                            queue.append(nb)
            l = l+1
    if method == 'U':
        queue = []
        queue_state = []
        cost_queue = [0]
        start_state = TreeNode(start)
        start_state.explored.append(start_state.city)
        queue.append(start_state)
        queue_state.append(start_state.state())
        explored_state = []
        expand = 0
        while queue:
            node = queue.pop(0)
            node_state = queue_state.pop(0)
            waste = cost_queue.pop(0)
            if len(node.explored) == len(graph.node):
                return [find_path_tsp(graph, start_state, node), expand]
            
            explored_state.append([node_state[0], node_state[1]])
            for i in graph.find_neighber(node_state[0]):
                expand += 1
                nb = TreeNode(i)
                nb.explored = [] + node_state[1]
                if nb.city not in nb.explored:
                    nb.explored.append(nb.city)
                if (nb.state() not in explored_state)and(nb.state() not in queue_state):
                    nb.parent = node
                    _, cost = find_path_tsp(graph, start_state, nb)
                    if (len(cost_queue) == 0)or(cost >= max(cost_queue)):
                        cost_queue.append(cost)
                        queue.append(nb)
                        queue_state.append(nb.state())
                    else:
                        for j in range(len(cost_queue)):
                            if cost <= cost_queue[j]:
                                cost_queue.insert(j, cost)
                                queue.insert(j, nb)
                                queue_state.insert(j, nb.state())
                                break
                elif nb.state() in queue_state:
                    real_nb = queue[queue_state.index(nb.state())]
                    _, old_cost = find_path_tsp(graph, start_state, real_nb)
                    old_prarent = real_nb.parent
                    real_nb.parent = node
                    _, new_cost = find_path_tsp(graph, start_state, real_nb)
                    #Just for debug
                    #print("Find another path:",new_cost)
                    if new_cost > old_cost:
                        real_nb.parent = old_prarent
    if method == 'A':
        min_edge = graph.find_short()
        queue = []
        queue_state = []
        whole = graph.whole_distance()
        cost_queue = [whole]
        start_state = TreeNode(start)
        start_state.explored.append(start_state.city)
        queue.append(start_state)
        queue_state.append(start_state.state())
        explored_state = []
        expand = 0
        while queue:
            node = queue.pop(0)
            node_state = queue_state.pop(0)
            waste = cost_queue.pop(0)
            if len(node.explored) == len(graph.node):
                return [find_path_astar_tsp(graph, start_state, node, min_edge), expand]
            explored_state.append([node_state[0], node_state[1]])
            for i in graph.find_neighber(node_state[0]):
                expand += 1
                nb = TreeNode(i)
                nb.explored = [] + node_state[1]
                if nb.city not in nb.explored:
                    nb.explored.append(nb.city)
                if (nb.state() not in explored_state)and(nb.state() not in queue_state):
                    nb.parent = node
                    _, cost = find_path_astar_tsp(graph, start_state, nb, min_edge)
                    if (len(cost_queue) == 0)or(cost >= max(cost_queue)):
                        cost_queue.append(cost)
                        queue.append(nb)
                        queue_state.append(nb.state())
                    else:
                        for j in range(len(cost_queue)):
                            if cost <= cost_queue[j]:
                                cost_queue.insert(j, cost)
                                queue.insert(j, nb)
                                queue_state.insert(j, nb.state())
                                break
                elif nb.state() in queue_state:
                    real_nb = queue[queue_state.index(nb.state())]
                    _, old_cost = find_path_astar_tsp(graph, start_state, real_nb, min_edge)
                    old_prarent = real_nb.parent
                    real_nb.parent = node
                    _, new_cost = find_path_astar_tsp(graph, start_state, real_nb, min_edge)
                    #Just for debug
                    #print("Find another path:",new_cost)
                    if new_cost > old_cost:
                        real_nb.parent = old_prarent

def main():
    '''
    Main Funciton
    '''
    if not DEBUG:
        '''
        For Grader:
        Uncomment one of them for differet question.
        '''
        #test = 'route.txt'
        test = 'tsp.txt'
        f = open(test, 'r')

    #Import The graph information
    aagraph = MyGraph()
    aagraph.import_graph_from_files('ann_arbor_graph.txt')
    aagraph.import_loc_from_files('City_Axis.txt')
    #Test part
    if not DEBUG:
        if test == 'route.txt':
            print("Route Planing:")
            start = f.readline()[:-1]
            dest = f.readline()[:-1]
            method = f.readline()
            print("-----------------")
            print("Path|Cost")
            print(general_search(aagraph, start, dest, method))
            f.close()

        if test == 'tsp.txt':
            print("Travel Salesman Problem:")
            start = f.readline()[:-1]
            method = f.readline()
            print("Using "+method+" Algorithm")
            print(solve_tsp(aagraph, start, method))
            f.close()
    else:
        print("---------Debug--------")
        start_city = 'Ann Arbor'
        end_city = 'Romeo'

        # print(general_search(aagraph, start_city, end_city, 'B'))
        # print(general_search(aagraph, start_city, end_city, 'D'))
        # print(general_search(aagraph, start_city, end_city, 'I'))
        # print(general_search(aagraph, start_city, end_city, 'U'))
        # print(general_search(aagraph, start_city, end_city, 'A'))

        # print(solve_tsp(aagraph, start_city, 'B'))
        # print(solve_tsp(aagraph, start_city, 'D'))
        # print(solve_tsp(aagraph, start_city, 'I'))
        # print(solve_tsp(aagraph, start_city, 'U'))
        # print(solve_tsp(aagraph, start_city, 'A'))

if __name__ == "__main__":
    main()
