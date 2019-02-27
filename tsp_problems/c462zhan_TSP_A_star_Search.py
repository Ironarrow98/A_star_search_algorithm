import os
import math
import heapq as pq
import matplotlib.pyplot as plt


# define usefull classes for further design
## City class contains information of current city information and path information
class City:
    ## constructor
    def __init__(self, name, cid, path, not_visited, cost, act, tct, state, unvisited):
        self.name = name
        self.cid = cid
        self.path = path
        self.not_visited = not_visited
        self.cost = cost
        self.act = act
        self.tct = tct
        self.state = state
        self.unvisited = unvisited
        return
    ## overwrite ==
    def __eq__(self, other):
        return self.tct == other.tct
    ## overwrite <
    def __lt__(self, other):
        return self.tct > other.tct
    ## overwrite >
    def __gt__(self, other):
        return self.tct < other.tct
    
## TSP class contains information of current TSP problem
class TSP:
    def __init__(self, cities_num, cities_name, cities_coord):
        self.cities_num = cities_num
        self.cities_name = cities_name
        self.cities_coord = cities_coord
        
        
# define usefull constants
cities = [0]
SearchGraph = []
MST = []
MSTAdj = []
PMST = []
CMST = []
MAP = {}
My_Map = {}
Closed = {}
visited = []
search_path = []
maxCost = "x"
total_city = 1
num_expanded = 1
        

# calculate Euclidean distance between 2 unknown cities
def Euclidean_dist(x1, y1, x2, y2):
    return math.sqrt(((x2 - x1) ** 2) + ((y2 - y1) ** 2))


# get distance bwteen 2 known cities
def get_dist(c1, c2):
    return SearchGraph[c1][c2]


# herustic function h(n) = 0
def h0(cities, unvisited_cities, current_city):
    return 0


# herustic function h(n) = cost from current city to closest city + MST of 
#     remaining unvisited cities + cost from A to closet remaining unvisited city
def h1(cities, unvisited_cities, current_city):
    if len(cities) == 0:
        return get_dist(current_city, 0)
    elif len(cities) == 1:
        return get_dist(current_city, cities[0]) + get_dist(cities[0], 0)
    src = []
    dis = []
    minDis = 'x'
    for c in cities:
        src.append(c)
        dis.append('x')
    mst = init_mst(cities, unvisited_cities)
    nearestUnvisitedCityDis = 'x'
    nearestDist2A = 'x'
    nsrc1  = 0
    nsrc2 = 0
    for i in range(len(cities)):
        nsrc1 = get_dist(src[i], current_city)
        nsrc2 = get_dist(src[i], 0)
        if nearestUnvisitedCityDis == 'x' or nsrc1 < nearestUnvisitedCityDis:
            nearestUnvisitedCityDis = nsrc1;
        if nearestDist2A == 'x' or nsrc2 < nearestDist2A:
            nearestDist2A = nsrc2; 
    result = nearestUnvisitedCityDis + mst + nearestDist2A 
    return result


# expand a node to get its successors
def expand_node(name, cid, path, not_visited, cost, act, tct, state, unvisited):
    successor = City(name, cid, path, not_visited, cost, act, tct, state, unvisited)
    return successor
    

# construct the search graph based on the TSP given
def init_sg(TSP):
    global SearchGraph
    global MST
    global MSTAdj
    for i in range(TSP.cities_num):
        SearchGraph.append([])
        MST.append([])
        MSTAdj.append([])        
        for j in range(TSP.cities_num):
            SearchGraph[i].append(0)
            MST[i].append(0)
            MSTAdj[i].append(0)              
    for i in range(TSP.cities_num):
        for j in range(i + 1, TSP.cities_num):
            SearchGraph[i][j] = Euclidean_dist(TSP.cities_coord[i][0], TSP.cities_coord[i][1], TSP.cities_coord[j][0], TSP.cities_coord[j][1])
            SearchGraph[j][i] = SearchGraph[i][j]
            MST[i][j] = 'x'
            MST[j][i] = 'x'
            MSTAdj[i][j] = 0
            MSTAdj[j][i] = 0
    return


# construct the MST graph based on the current city and remaining unvisited cities given
#    return the total weight of MST
def init_mst(cities, unvisited_cities):
    if len(cities) == 0 and len(unvisited_cities) == 1:
        return 0    
    if len(cities) == 1:
        return 0
    unvisited_cities.sort()
    src = []
    dis = []
    minDist = 'x'
    cnames = ""
    for c in cities:
        src.append(c)
        dis.append('x')
    for u in unvisited_cities:
        cnames += (u + ',')
    if cnames in MAP:
        return MAP[cnames]
    nsrc = src[len(cities) - 1]
    minInd = 'x'
    size = 0
    for i in range(len(cities) - 1, 0, -1):
        minDist = 'x'
        for j in range(i):
            dist = get_dist(src[j], nsrc)
            if dis[j] == 'x' or dist < dis[j]:
                dis[j] = dist
            if minDist == 'x' or dis[j] < minDist:
                minDist = dis[j]
                minInd = j
        nsrc = src[minInd]
        size += minDist
        src[minInd] = src[i - 1]
        dis[minInd] = dis[i - 1]
    MAP[cnames] = size
    return size


# Standard A* Search Algorithm (judge determine use h(n) = 0 or not)
def A_star_search(judge):
    global maxCost
    global num_expanded
    global total_city
    rest = ''
    ready = []
    ready_names = []
    while (not search_path == []) and (maxCost == 'x' or pq.heappop(search_path).tct < maxCost):
        if search_path == []:
            break
        curr_city = pq.heappop(search_path)
        state = curr_city.state
        path = curr_city.path
        unvisited = curr_city.unvisited
        not_visited = curr_city.not_visited
        if not_visited == 0:
            total_cost = curr_city.act + get_dist(curr_city.cid, 0)
            if maxCost == 'x' or total_cost < maxCost:
                maxCost = total_cost
                rest = curr_city.path
            continue
        if state in Closed and Closed[state] < curr_city.tct:
            continue
        elif (state in Closed and Closed[state] > curr_city.tct) or state not in Closed:
            Closed[state] = curr_city.tct
        num_expanded += 1
        for u in unvisited:
            ready = []
            ready_names = []  
            for un in unvisited:
                if u == un:
                    continue
                ready.append(un)
            for unv in unvisited:
                if u == un:
                    continue
                ready_names.append(My_Map[unv])
            if judge == 0:
                est = h0(ready, ready_names, u)
            else:
                est = h1(ready, ready_names, u)
            act = curr_city.act + get_dist(curr_city.cid, u)
            tct = act + est
            name = My_Map[u]
            npath = path + ' -> ' + name            
            tmp = npath
            ''.join(sorted(tmp))
            nstate = tmp + ' -> ' + name     
            total_city += 1
            pq.heappush(search_path, expand_node(name, u, npath, curr_city.not_visited - 1, est, act, tct, nstate, ready))
    return rest


# start the program
def main(file, judge):
    global cities 
    global SearchGraph 
    global MST 
    global MSTAdj 
    global PMST 
    global CMST 
    global MAP 
    global My_Map
    global Closed 
    global visited 
    global search_path 
    global maxCost 
    global total_city
    global num_expanded  
    f = open(file, 'r')
    num_cities = int(f.readline())
    line = f.readline()
    prob = TSP(num_cities, [], [])
    l1 = list(range(1, num_cities))
    l2 = []
    for i in range(num_cities):
        (prob.cities_coord).append([0, 0])
    i = 0
    while line != "":
        info = line.split()
        (prob.cities_name).append(info[0])
        l2.append(info[0])
        My_Map[i] = info[0]
        prob.cities_coord[i][0] = int(info[1])
        prob.cities_coord[i][1] = int(info[2])
        i += 1
        line = f.readline()
    f.close()    
    init_sg(prob)
    cities_num = prob.cities_num
    if judge == 0:
        start = h0(l1, l2, 0)
    else:
        start = h1(l1, l2, 0)
    cities[0] = expand_node('A', 0, 'A', cities_num - 1, start, 0, start, 'AA', l1)
    search_path.append(cities[0])
    rest = A_star_search(judge)
    print("the Cost is {0}; The path is {1}\nThe number of expanded nodes is {2}".format(maxCost, rest + ' -> A', num_expanded))
    cities = [0]
    SearchGraph = []
    MST = []
    MSTAdj = []
    PMST = []
    CMST = []
    MAP = {}
    My_Map = {}
    Closed = {}
    visited = []
    search_path = []
    maxCost = "x"
    total_city = 1
    tmp = num_expanded 
    num_expanded = 1
    return tmp


# summarize and plot the result data
def get_plot(judge):
    y = []
    x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
    z = []
    cur_path = os.path.dirname(__file__)
    for i in range(1, 17):
        for j in range(1, 11):
            new_path = os.path.join(cur_path, str(i) + '/instance_' + str(j) +'.txt')
            z.append(main(new_path, judge))
        y.append(sum(z) / 10)
        z = []
    plt.grid(True)    
    plt.plot(x, y)    
    print("x-axis is {0}".format(x))
    print("y-axis is {0}".format(y))
    
    
