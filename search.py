###### Write Your Library Here ###########
from queue import Queue
from queue import PriorityQueue


#########################################


def search(maze, func):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_four_circles": astar_four_circles,
        "astar_many_circles": astar_many_circles
    }.get(func)(maze)


# -------------------- Stage 01: One circle - BFS Algorithm ------------------------ #
def stage1_build_path(maze, dist):
    path = []
    start_point=maze.startPoint()
    end_point = maze.circlePoints()[0]
    row_size, col_size = maze.getDimensions()
    # BackTraking
    dx = [0,1,0,-1]
    dy = [1,0,-1,0]
    point = end_point
    flag = 1
    while flag:
        path.append(point)
        if point == start_point:
            break
        flag = 0
        for i in range(4):
            nx = point[0]+dx[i]
            ny = point[1]+dy[i]
            if nx>=row_size or ny>=col_size or nx<0 or ny<0:
                continue
            if dist[nx][ny] == dist[point[0]][point[1]]-1:
                point = (nx, ny)
                flag = 1
                break
    path.reverse()
    return path

def bfs(maze):
    """
    [문제 01] 제시된 stage1의 맵 세가지를 BFS Algorithm을 통해 최단 경로를 return하시오.(20점)
    """
    start_point=maze.startPoint()

    path=[]

    ####################### Write Your Code Here ################################
    end_point = maze.circlePoints()[0]
    row_size, col_size = maze.getDimensions()
    dist = [[0 for colm in range(col_size)] for row in range(row_size)]
    queue = Queue()
    queue.put(start_point)
    dist[start_point[0]][start_point[1]] = 1
    while not queue.empty():
        point = queue.get()
        neighbors = maze.neighborPoints(point[0],point[1])
        flag = 0
        for next_point in neighbors:
            if dist[next_point[0]][next_point[1]] == 0:
                dist[next_point[0]][next_point[1]] = dist[point[0]][point[1]] + 1
                if next_point != end_point:
                    queue.put(next_point)
    path = stage1_build_path(maze, dist)
    return path
    ############################################################################



class Node:
    def __init__(self,parent,location):
        self.parent=parent
        self.location=location #현재 노드

        self.obj=[]

        # F = G+H
        self.f=0
        self.g=0
        self.h=0

    def __eq__(self, other):
        return self.location==other.location and str(self.obj)==str(other.obj)

    def __le__(self, other):
        return self.g+self.h<=other.g+other.h

    def __lt__(self, other):
        return self.g+self.h<other.g+other.h

    def __gt__(self, other):
        return self.g+self.h>other.g+other.h

    def __ge__(self, other):
        return self.g+self.h>=other.g+other.h


# -------------------- Stage 01: One circle - A* Algorithm ------------------------ #

def manhatten_dist(p1,p2):
    return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

def astar(maze):

    """
    [문제 02] 제시된 stage1의 맵 세가지를 A* Algorithm을 통해 최단경로를 return하시오.(20점)
    (Heuristic Function은 위에서 정의한 manhatten_dist function을 사용할 것.)
    """

    start_point=maze.startPoint()

    end_point=maze.circlePoints()[0]

    path=[]

    ####################### Write Your Code Here ################################
    row_size, col_size = maze.getDimensions()
    dist = [[0 for colm in range(col_size)] for row in range(row_size)]
    pq = PriorityQueue()
    dist[start_point[0]][start_point[1]] = 1
    pq.put((manhatten_dist(start_point, end_point)+dist[start_point[0]][start_point[1]],start_point))
    while not pq.empty():
        d, point = pq.get()
        if point == end_point:
            break
        neighbors = maze.neighborPoints(point[0],point[1])
        flag = 0
        for next_point in neighbors:
            if dist[next_point[0]][next_point[1]] == 0:
                dist[next_point[0]][next_point[1]] = dist[point[0]][point[1]] + 1
                pq.put((manhatten_dist(next_point, end_point)+dist[next_point[0]][next_point[1]],next_point))
    path = stage1_build_path(maze, dist)
    return path

    ############################################################################


# -------------------- Stage 02: Four circles - A* Algorithm  ------------------------ #

def stage2_build_path(maze, dist, end_point):
    path = []
    start_point=maze.startPoint()
    end_points = maze.circlePoints()
    end_points.sort()
    row_size, col_size = maze.getDimensions()
    # BackTraking
    dx = [0,1,0,-1]
    dy = [1,0,-1,0]
    point = end_point
    bit = 15
    flag = 1
    while flag:
        path.append(point)
        if point == start_point and bit == 0:
            break
        nxt_bit = bit
        for i in range(4):
            if point == end_points[i]:
                nxt_bit = bit^(1<<i)
                break
        flag = 0
        for i in range(4):
            nx = point[0]+dx[i]
            ny = point[1]+dy[i]
            if nx>=row_size or ny>=col_size or nx<0 or ny<0:
                continue
            if dist[nx][ny][nxt_bit] == dist[point[0]][point[1]][bit]-1:
                point = (nx, ny)
                bit = nxt_bit
                flag = 1
                break
    path.reverse()
    return path

def stage2_heuristic(point, end_points, visit_bit):
    row_size = manhatten_dist(end_points[0],end_points[2])
    col_size = manhatten_dist(end_points[0],end_points[1])
    remain = []
    for i in range(4):
        if not visit_bit & (1<<i):
            remain.append(end_points[i])
    if len(remain) == 0:
        return 0
    elif len(remain) == 1:
        return manhatten_dist(remain[0],point)
    elif len(remain) == 2:
        return min(manhatten_dist(remain[0],point), manhatten_dist(remain[1],point)) + manhatten_dist(remain[0],remain[1])
    elif len(remain) == 3:
        return min(row_size + col_size + min(manhatten_dist(remain[0],point), manhatten_dist(remain[2],point)), row_size + col_size + min(row_size,col_size) + manhatten_dist(remain[1],point))
    elif len(remain) == 4:
        return row_size + col_size + row_size + col_size - max(row_size,col_size) + min(min(manhatten_dist(remain[0],point), manhatten_dist(remain[1],point)), min(manhatten_dist(remain[2],point), manhatten_dist(remain[3],point)))


def astar_four_circles(maze):
    """
    [문제 03] 제시된 stage2의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage2_heuristic function을 직접 정의하여 사용해야 한다.)
    """

    end_points=maze.circlePoints()
    end_points.sort()

    path=[]

    ####################### Write Your Code Here ################################
    start_point=maze.startPoint()
    row_size, col_size = maze.getDimensions()
    INF = 987654321
    dist = [[[INF for bit in range(16)] for colm in range(col_size)] for row in range(row_size)]
    pq = PriorityQueue()
    dist[start_point[0]][start_point[1]][0] = 0
    pq.put((stage2_heuristic(start_point, end_points, 0)+dist[start_point[0]][start_point[1]][0],start_point,0))
    end_point = None
    while not pq.empty():
        d, point, bit = pq.get()
        if bit == 15: # 1111(2) -> 15
            end_point = point
            break
        neighbors = maze.neighborPoints(point[0],point[1])
        for next_point in neighbors:
            nxt_bit = bit
            for i in range(4):
                if next_point == end_points[i]:
                     nxt_bit = bit|(1<<i)
            if dist[next_point[0]][next_point[1]][nxt_bit] > dist[point[0]][point[1]][bit]+1:
                dist[next_point[0]][next_point[1]][nxt_bit] = dist[point[0]][point[1]][bit] + 1
                pq.put((stage2_heuristic(next_point, end_points, nxt_bit)+dist[next_point[0]][next_point[1]][nxt_bit],next_point,nxt_bit))
    path = stage2_build_path(maze, dist, end_point)
    return path

    ############################################################################



# -------------------- Stage 03: Many circles - A* Algorithm -------------------- #

def mst(objectives, edges):

    cost_sum=0
    ####################### Write Your Code Here ################################
    # Prim algorithm
    INF = 987654321
    visit = [False for i in range(objectives)]
    weight = [INF for i in range(objectives)]
    visit[0] = True
    weight[0] = 0
    minV = 0
    for i in range(objectives-1):
        for v in range(objectives):
            if v == minV or visit[v] == True:
                continue
            if edges[minV][v] < weight[v]:
                weight[v] = edges[minV][v]
        minW = INF
        for v in range(objectives):
            if visit[v] == False and weight[v] < minW:
                minV = v
                minW = weight[v]
        visit[minV] = True
    for v in range(objectives):
        cost_sum += weight[v]
    return cost_sum
    ############################################################################


def stage3_heuristic(mst_data, real_dist, n, point, end_points, visit_bit):
    if visit_bit == (1<<n)-1:
        return 0
    remain = []
    for i in range(n):
        if (visit_bit & (1<<i)) == 0:
            remain.append(end_points[i])
    if mst_data[visit_bit] == -1:
        edges = {}
        for i, a in enumerate(remain):
            edges[i] = {}
            for j, b in enumerate(remain):
                if i != j:
                    edges[i][j] = real_dist[a][b[0]][b[1]]
        mst_data[visit_bit] = mst(len(remain), edges)
    min_x = real_dist[remain[0]][point[0]][point[1]]
    for a in remain:
        min_x = min(min_x, real_dist[a][point[0]][point[1]])
    return min_x + mst_data[visit_bit]

def stage3_build_path(maze, dist, end_point):
    path = []
    start_point=maze.startPoint()
    end_points = maze.circlePoints()
    end_points.sort()
    row_size, col_size = maze.getDimensions()
    # BackTraking
    dx = [0,1,0,-1]
    dy = [1,0,-1,0]
    point = end_point
    n = len(end_points)
    bit = (1<<n)-1
    flag = 1
    while flag:
        path.append(point)
        if point == start_point and bit == 0:
            break
        nxt_bit = bit
        for i in range(n):
            if point == end_points[i]:
                if bit&(1<<i) != 0:
                    nxt_bit = bit^(1<<i)
                break
        flag = 0
        for i in range(4):
            nx = point[0]+dx[i]
            ny = point[1]+dy[i]
            if nx>=row_size or ny>=col_size or nx<0 or ny<0:
                continue
            if dist[nx][ny][nxt_bit] == dist[point[0]][point[1]][bit]-1:
                point = (nx, ny)
                bit = nxt_bit
                flag = 1
                break
        if flag == 0:
            for i in range(4):
                nx = point[0]+dx[i]
                ny = point[1]+dy[i]
                if nx>=row_size or ny>=col_size or nx<0 or ny<0:
                    continue
                if dist[nx][ny][bit] == dist[point[0]][point[1]][bit]-1:
                    point = (nx, ny)
                    bit = bit
                    flag = 1
                    break

    path.reverse()
    return path

def real_dist_preprocessing(maze, end_points):
    real_dist = {}
    row_size, col_size = maze.getDimensions()
    for p in end_points:
        real_dist[p] = [[-1 for col in range(col_size)] for row in range(row_size)]
        queue = Queue()
        queue.put(p)
        real_dist[p][p[0]][p[1]] = 0
        while not queue.empty():
            point = queue.get()
            neighbors = maze.neighborPoints(point[0],point[1])
            for next_point in neighbors:
                if real_dist[p][next_point[0]][next_point[1]] == -1:
                    real_dist[p][next_point[0]][next_point[1]] = real_dist[p][point[0]][point[1]] + 1
                    queue.put(next_point)
    return real_dist


def astar_many_circles(maze):
    """
    [문제 04] 제시된 stage3의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage3_heuristic function을 직접 정의하여 사용해야 하고, minimum spanning tree
    알고리즘을 활용한 heuristic function이어야 한다.)
    """

    end_points= maze.circlePoints()
    end_points.sort()

    path=[]

    ####################### Write Your Code Here ################################
    real_dist = real_dist_preprocessing(maze, end_points)
    start_point=maze.startPoint()
    row_size, col_size = maze.getDimensions()
    n = len(end_points)
    INF = 987654321
    mst_data = [-1 for i in range(1<<n)]
    dist = [[[INF for bit in range(1<<n)] for colm in range(col_size)] for row in range(row_size)]
    pq = PriorityQueue()
    dist[start_point[0]][start_point[1]][0] = 0
    #pq.put((stage3_heuristic(mst_data, n, start_point, end_points, 0)+dist[start_point[0]][start_point[1]][0],start_point,0))
    pq.put((0,start_point,0))
    end_point = None
    while not pq.empty():
        d, point, bit = pq.get()
        if bit == (1<<n)-1:
            end_point = point
            break
        neighbors = maze.neighborPoints(point[0],point[1])
        for next_point in neighbors:
            nxt_bit = bit
            for i in range(n):
                if next_point == end_points[i]:
                    nxt_bit = bit|(1<<i)
            if dist[next_point[0]][next_point[1]][nxt_bit] > dist[point[0]][point[1]][bit] + 1:
                dist[next_point[0]][next_point[1]][nxt_bit] = dist[point[0]][point[1]][bit] + 1
                pq.put((stage3_heuristic(mst_data, real_dist, n, next_point, end_points, nxt_bit)+dist[next_point[0]][next_point[1]][nxt_bit],next_point,nxt_bit))
    path = stage3_build_path(maze, dist, end_point)
    return path
    ############################################################################
