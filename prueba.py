import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from scipy.spatial import distance

#PARAMETER
N_RAN = 500  # number of random_points
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length


class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
               str(self.cost) + "," + str(self.parent_index)


#AUXILIAR METHODS
def generate_maze():
    ox = []
    oy = []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)
    for i in range(7):
        ox.append(i)
        oy.append(20)
    for i in range(13, 20):
        ox.append(i + 1)
        oy.append(40)

    return ox, oy

def distance(ax, ay, bx, by):
    dist = []
    a = [ax, ay]
    for (ox, oy) in zip(bx, by):
        b = [ox, oy]
        dist.append(math.dist(a, b)) 
        #dist.append(distance.euclidean(a, b))
    return dist

def order_distances(ix, iy, rx, ry):
    dist = distance(ix, iy, rx, ry)
    #dist = [elem for elem in dist if elem < MAX_EDGE_LEN] #No se pueden borrar los vecinos que tengan distancia mayor a la máxima porque se pierde la referencia de los índices. Habría que mantener el registro de los índices
    #index, sorted_list = zip(*sorted(enumerate(dist)), key=itemgetter(1))
    index = sorted(range(len(dist)), key=lambda k: dist[k])
    return index



#MAIN METHODS
def prm(sx, sy, gx, gy, ox, oy, rs):

    #First step: Generate the random points

    rx, ry = random_points(sx, sy, gx, gy, ox, oy, rs)


    road_map, knn_x, knn_y = generate_road_map(rx, ry, ox, oy, rs)

    plt.plot(rx, ry, ".g") #Prints the random points

    #Prints the road map, the conexions between the random points
    #for (x, y) in zip(knn_x, knn_y):
    #    plt.plot(x, y, "--b")


    res_x, res_y = dijkstra(sx, sy, gx, gy, road_map, rx, ry)

    plt.plot(res_x, res_y, "-y")
    plt.show()

    return res_x, res_y
                                

def random_points(sx, sy, gx, gy, ox, oy, rs):
    
    #The ends of the maze are stored
    max_x = max(ox)
    max_y = max(oy)
    min_x = min(ox)
    min_y = min(oy)

    #List for storing the random points
    rx = [] #random points x
    ry = [] #random points y

    #Random Generator
    rng = np.random.default_rng()
    valid = True

    while len(rx) <= N_RAN:
        #Generate the random points between the max´s
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max_y - min_y)) + min_y

        #Calculate the distance between the generated point and the obstacles
        dist = distance(tx, ty, ox, oy)
        
        #The generated point is valid if the robot can stand on it, so the dist from the point to the obstacles should be less than the robot size
        for i in dist:
            if i <= rs:
                valid = False
        if valid:
            rx.append(tx)
            ry.append(ty)

        valid = True

    rx.append(sx)
    ry.append(sy)
    rx.append(gx)
    ry.append(gy)

    return rx, ry

def generate_road_map(rx, ry, ox, oy, rs):

    road_map = []
    knn_final_x = []
    knn_final_y = []

    for (ix, iy) in zip(rx, ry):
        index = order_distances(ix, iy, rx, ry)
        knn_id = []
        knn_list_x = []
        knn_list_y = []

        for j in range(0, len(index)): #Si no funciona empezar en 1
            knn_x = rx[index[j]]
            knn_y = ry[index[j]]
            #if not is_collision(ix, iy, knn_x, knn_y, ox, oy, rs):
            knn_id.append(index[j])
            knn_list_x.append(knn_x)
            knn_list_y.append(knn_y)
            if len(knn_id) >= N_KNN:
                break
        knn_final_x.append(knn_list_x)
        knn_final_y.append(knn_list_y)        
        road_map.append(knn_id)
    
    return road_map, knn_final_x, knn_final_y

def is_collision(sx, sy, gx, gy, ox, oy, rs):

    anlge = math.atan2(gy - sy, gx - sx)
    d = math.hypot(gx - sx, gy- sy)

    if d >= MAX_EDGE_LEN: return True

    n_moves = round(d/rs) #Number of times the robot will move between the origin point and the destination point.

    for i in range(n_moves):
        dist = distance(sx, sy, ox, oy)
        for j in dist:
            if j <= rs:
                return True
        sx += rs * math.cos(anlge)
        sy += rs * math.sin(anlge)
    
    return False

def dijkstra(sx, sy, gx, gy, road_map, sample_x, sample_y):
    start_node = Node(sx, sy, 0.0, -1)
    goal_node = Node(gx, gy, 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    path_found = True

    while True:
        if not open_set:
            print("Cannot find path")
            path_found = False
            break

        c_id = min(open_set, key=lambda o: open_set[o].cost)
        current = open_set[c_id]

        # show graph
        if len(closed_set.keys()) % 2 == 0:
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current.x, current.y, "xc")
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        # Remove the item from the open set
        del open_set[c_id]
        # Add it to the closed set
        closed_set[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.hypot(dx, dy)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closed_set:
                continue
            # Otherwise if it is already in the open set
            if n_id in open_set:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id].cost = node.cost
                    open_set[n_id].parent_index = c_id
            else:
                open_set[n_id] = node

    if path_found is False:
        return [], []

    # generate final course
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry

def main(rng=None):
    print("START!!")
    sx = 15.0  # start point x
    sy = 15.0  # start point y
    gx = 50.0  # goal point x
    gy = 50.0  # goal point y
    rs = 5.0   # robot size

    ox, oy = generate_maze() #obstacle x, obstacle y  

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "^r")
    plt.plot(gx, gy, "^c")
    #plt.grid(True)
    plt.axis("equal")

    if is_collision(gx, gy, gx-2, gy-2, ox, oy, rs): print('FINAL POINT NO VALID!!!!!!')
    else: 
        prm(sx, sy, gx, gy, ox, oy, rs)
        print('PATH FOUND!')

if __name__ == '__main__':
    main()