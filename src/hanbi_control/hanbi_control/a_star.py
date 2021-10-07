import rclpy
import numpy as np
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData,Path
from math import pi,cos,sin
from collections import deque
from math import pow, sqrt
import time

# a_star 노드는  OccupancyGrid map을 받아 grid map 기반 최단경로 탐색 알고리즘을 통해 로봇이 목적지까지 가는 경로를 생성하는 노드입니다.
# 로봇의 위치(/pose), 맵(/map), 목표 위치(/goal_pose)를 받아서 전역경로(/global_path)를 만들어 줍니다. 
# goal_pose는 rviz2에서 2D Goal Pose 버튼을 누르고 위치를 찍으면 메시지가 publish 됩니다. 
# 주의할 점 : odom을 받아서 사용하는데 기존 odom 노드는 시작했을 때 로봇의 초기 위치가 x,y,heading(0,0,0) 입니다. 로봇의 초기위치를 맵 상에서 로봇의 위치와 맞춰줘야 합니다. 
# 따라서 sub2의 odom 노드를 수정해줍니다. turtlebot_status 안에는 정답데이터(절대 위치)가 있는데 그 정보를 사용해서 맵과 로봇의 좌표를 맞춰 줍니다.

# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 파라미터 설정
# 3. 맵 데이터 행렬로 바꾸기
# 4. 위치(x,y)를 map의 grid cell로 변환
# 5. map의 grid cell을 위치(x,y)로 변환
# 6. goal_pose 메시지 수신하여 목표 위치 설정
# 7. grid 기반 최단경로 탐색
params = {
    "MAP" : {
        "RESOLUTION": 0.1,
        "OCCUPANCY_UP": 0.02,
        "OCCUPANCY_DOWN": 0.01,
        "CENTER": (0, 0),
        "SIZE": (50, 50),
        "FILENAME": 'test.png',
        "MAPVIS_RESIZE_SCALE": 2.0,
        "PATH" : 'C:\\dev\\ros2_smart_home\\src\\hanbi_control\\map\\map1005.txt'
    },
    "PATH" :{
        "PATH" : 'C:\\dev\\ros2_smart_home\\src\\hanbi_control\\path\\path_kjh.txt'
    }
}
params["MAP"]["origin"] = [params["MAP"]["CENTER"][0]-(params["MAP"]["SIZE"][0]/2), params["MAP"]["CENTER"][1]-(params["MAP"]["SIZE"][1]/2)]
def pose2grid(pose):
    [x,y] = pose
    resol = params["MAP"]["RESOLUTION"]
    origin = params["MAP"]["origin"]
    return  [int((x - origin[0])/resol),int((y - origin[1])/resol)]
    
def grid2pose(grid):
    [x,y] = grid
    resol = params["MAP"]["RESOLUTION"]
    origin = params["MAP"]["origin"]
    return [float(origin[0] + x * resol), float((origin[1] + y * resol))]

class Status():
    def __init__(self, y,x,g,h):
        self.y = y
        self.x = x
        self.f = g+h
        self.g = g
        self.h = h
    def __eq__(self, other):
        return self.y == other.y and self.x == other.x and self.f == other.f and self.g == other.g and self.h == other.h
    def get(self):
        # print("get : ",  self.y, self.x, self.f, self.g, self.h)
        return [self.y, self.x,  self.f, self.g, self.h]
class nodeHeap():
    def __init__(self, size_y, size_x):
        super().__init__()
        self.heap= [Status(-1,-1,-1,-1)]
        self.size = size_y, size_x
        self.memo = [[[size_y * size_x, -1] for x in range(size_y)] for x in range(size_x)]

    def swap(self, a, b):
        tmp = self.heap[a]
        self.heap[a] = self.heap[b]
        self.heap[b] = tmp

    def push(self, status, direction):
        '''
        node를 넣고 가중치가 가장 낮은 node를 root에 두는 메소드
        하단 주석부분을 수정
        '''
        cy,cx,cf,cg,ch = status.get()
        
        
        if self.memo[cy][cx][0] == self.size[0] * self.size[1]:
            # print("ss")
            self.memo[cy][cx] = [cf,direction]

            self.heap.append(status)

            cur_idx = len(self.heap)-1
            while True:
                next_idx = int(cur_idx/2)
                # print(next_idx)
                if next_idx <= 0:
                    break

                # custom 하기위해 이 부분을 수정
                # condition1 = self.heap[cur_idx] < self.heap[next_idx]
                condition1 = self.heap[cur_idx].f < self.heap[next_idx].f or self.heap[next_idx] == Status(-1,-1,-1,-1)

                if condition1:
                    tmp = self.heap[cur_idx]
                    self.heap[cur_idx] = self.heap[next_idx]
                    self.heap[next_idx] = tmp
                    cur_idx = int(cur_idx /2)
                else:
                    break

        elif self.memo[cy][cx][0] > cf:
            # print("??")
            for i in range(1,len(self.heap)):
                if(self.heap[i].y == cy and self.heap[i].x == cx):
                    self.pop(i)
                    break
            self.memo[cy][cx] = [cf,direction]
            self.heap.append(status)
            cur_idx = len(self.heap)-1
            while True:
                next_idx = int(cur_idx/2)
                if next_idx <= 0:
                    break

                # custom 하기위해 이 부분을 수정
                # condition1 = self.heap[cur_idx] < self.heap[next_idx]
                condition1 = self.heap[cur_idx].f < self.heap[next_idx].f or self.heap[next_idx] == Status(-1,-1,-1,-1)

                if condition1:
                    tmp = self.heap[cur_idx]
                    self.heap[cur_idx] = self.heap[next_idx]
                    self.heap[next_idx] = tmp
                    cur_idx = int(cur_idx /2)
                else:
                    break

    def pop(self, init_idx = 1):
        length = len(self.heap)-1
        if length < 1:
            return -1

        cur_idx = init_idx
        answer = self.heap[cur_idx]
        self.heap[cur_idx] = Status(-1,-1,-1,-1)
        while True:
            if cur_idx > length:
                break
            next_idx1 = cur_idx * 2
            next_idx2 = cur_idx * 2 + 1
            noleft = False
            noright = False
            
            if next_idx2 > length or self.heap[next_idx2] == Status(-1,-1,-1,-1):
                noright = True
            
            if next_idx1 > length or self.heap[next_idx1] == Status(-1,-1,-1,-1):
                noleft = True
            
            if noleft and noright :
                break
            elif not noleft and not noright:
                condition1 = self.heap[next_idx1].f < self.heap[next_idx2].f
                next_idx = next_idx1 if condition1 else next_idx2
                # print(self.heap)
                # print(cur_idx, next_idx, self.heap[cur_idx], self.heap[next_idx])
                self.swap(cur_idx, next_idx)
                cur_idx = next_idx

            else:
                next_idx = next_idx1 if noright else next_idx2
                self.swap(cur_idx, next_idx)
                cur_idx = next_idx
        # print("------pop------")
        # print(answer.get())
        return answer        

    def empty(self):
        pass
    def debug(self):
        # f.write("------debug------")
        # for i in range(len(self.heap)):
        #     status = self.heap[i].get()
        #     f.write("%d : " % i)
        #     for e in status:
        #         f.write(str(e))
        #         f.write(" ")
        #     f.write("\n")
        # f.write("\n")
        pass
class a_star(Node):

    def __init__(self):
        super().__init__('a_Star')
        # path_via 기능 추가
        # 로직 1. viapoint를 path로 수신 (그리드 좌표)
        # 로직 2. 맵 정보 수신
        # 로직 3. a_star 수행
        # 로직 4. path_via 퍼블리쉬

        
        # 로직 1. publisher, subscriber 만들기
        self.via_point_sub = self.create_subscription(Path, 'via_point', self.via_callback, 1)
        # self.map_sub = self.create_subscription(OccupancyGrid,'map',self.map_callback,1)
        self.odom_sub = self.create_subscription(Odometry,'odom',self.odom_callback,1)
        self.goal_sub = self.create_subscription(PoseStamped,'goal_pose',self.goal_callback,1)
        self.add_pub= self.create_publisher(Path, 'global_path_add', 1)
        self.dijk_pub= self.create_publisher(OccupancyGrid, 'dijk', 1)
        
        
        # self.map_msg=OccupancyGrid()
        self.dijk_msg=OccupancyGrid()
        self.odom_msg=Odometry()
        self.is_map=False
        self.is_odom=False
        self.is_found_path=False
        self.is_grid_update=False

        # map 파일 읽기
        self.MAP_DATA = []
        f = open(params["MAP"]["PATH"], "r")
        while True:
            line = list(map(int, f.readline().split()))
            if not line: break
            self.MAP_DATA.append(line)
        print("self.MAP_DATA", self.MAP_DATA)

        # 로직 2. 파라미터 설정
        self.goal_grid = [184,224] 
        self.map_size_x=350
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-8-8.75
        self.map_offset_y=-4-8.75
    
        self.GRIDSIZE=350 
 
        self.dx = [-1,0,0,1,-1,-1,1,1]
        self.dy = [0,1,-1,0,-1,1,-1,1]
        self.dCost = [1,1,1,1,1.414,1.414,1.414,1.414]

        # 로직 3. 다익스트라 결과
        self.dijk_msg.header.frame_id="map"
        self.dijk_data = [0 for i in range(self.map_size_x*self.map_size_y)]

        m = MapMetaData()
        m.resolution = self.map_resolution
        m.width = self.map_size_x
        m.height = self.map_size_y
        m.origin = Pose()
        m.origin.position.x = self.map_offset_x
        m.origin.position.y = self.map_offset_y

        self.dijk_msg.info=m
       

    def grid_update(self):
        self.is_grid_update=True
        '''
        로직 3. 맵 데이터 행렬로 바꾸기
        '''
        self.grid=np.array(self.map_msg.data).reshape(350,350)

    def pose_to_grid_cell(self,x,y):
        # x,y 로봇의 현재 좌표위치
        
        '''
        로직 4. 위치(x,y)를 map의 grid cell로 변환 
        (테스트) pose가 (-8,-4)라면 맵의 중앙에 위치하게 된다. 따라서 map_point_x,y 는 map size의 절반인 (175,175)가 된다.
        pose가 (-16.75,12.75) 라면 맵의 시작점에 위치하게 된다. 따라서 map_point_x,y는 (0,0)이 된다.
        '''
        map_point_x = int((x + 16.75) /self.map_resolution)
        map_point_y = int((y + 12.75) / self.map_resolution)
        print(map_point_x, map_point_y)
        return map_point_x,map_point_y


    def grid_cell_to_pose(self,grid_cell):

        x = -16.75 + (grid_cell[0])*self.map_resolution
        y = -12.75 + (grid_cell[1])*self.map_resolution
        '''
        로직 5. map의 grid cell을 위치(x,y)로 변환
        (테스트) grid cell이 (175,175)라면 맵의 중앙에 위치하게 된다. 따라서 pose로 변환하게 되면 맵의 중앙인 (-8,-4)가 된다.
        grid cell이 (350,350)라면 맵의 제일 끝 좌측 상단에 위치하게 된다. 따라서 pose로 변환하게 되면 맵의 좌측 상단인 (0.75,6.25)가 된다
        '''
        return [x,y]
    def getH(self, START, END):
        #맨하튼 거리
        return sqrt(pow(START[0] - END[0],2) + pow(START[1] - END[1],2))


    def odom_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg


    def map_callback(self,msg):
        self.is_map=True
        self.map_msg=msg

    def via_callback(self, msg):
        # 로직 1. global_path에 추가할 두 지점을 수신
        print("find via path")
        start_point = [int(msg.poses[0].pose.position.x), int(msg.poses[0].pose.position.y)]
        end_point = [int(msg.poses[1].pose.position.x), int(msg.poses[1].pose.position.y)]

        print("start, end : {0}, {1}".format(start_point, end_point))

        # 로직 2. a_star 수행
        self.path = self.aStar(start_point,end_point,self.MAP_DATA)
        self.path_ret = self.path.copy()
        self.path_ret.reverse()
        self.path = self.path_ret[:-1] + self.path
        print("path : ", self.path)

        # 로직 3. a_star_pose
        add_pub_msg = Path()
        add_pub_msg.header.frame_id='map'
        for idx, pose in enumerate(self.path) :
            tmp_pose = PoseStamped()
            self.path[idx] = grid2pose(pose)
            tmp_pose.pose.position.x = float(self.path[idx][0])
            tmp_pose.pose.position.y = float(self.path[idx][1])
            tmp_pose.pose.orientation.w=1.0
            add_pub_msg.poses.append(tmp_pose)

        print("path_pose : ", self.path)

        # 로직 4. publish
        self.add_pub.publish(add_pub_msg)
        print("add_pub")

        
        

    def goal_callback(self,msg):
        if msg.header.frame_id=='map':
            '''
            로직 6. goal_pose 메시지 수신하여 목표 위치 설정
            '''             
            #로직 1-1. Grid 정보가 세팅되어 있을 때만 경로 설정
            if self.is_map ==True and self.is_odom==True  :
                if self.is_grid_update==False :
                    print("grid update")
                    self.grid_update()
                else:
                    print("grid not update")


                self.final_path=[]
                self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]
                self.cost = np.array([[self.GRIDSIZE*self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])

                #로직 2-2. start_grid = 현재 로봇기준으로 시작점 설정 (grid 좌표)
                x=self.odom_msg.pose.pose.position.x
                y=self.odom_msg.pose.pose.position.y
                start_grid=self.pose_to_grid_cell(x,y)

                #로직 2-3. 목적지를 설정
                #로직 2-3-1. 목적지를임의로 에어컨 앞 좌표로 고정 설정
                '''
                '''
                #로직 2-3-2. goal_grid = 목적지를 설정 (grid 좌표)
                # self.goal_map = msg.position
                # goal_x=self.goal[0]
                # goal_y=self.goal[1]
                # self.goal_grid= self.pose_to_grid_cell(goal_x, goal_y)
                # print(self.goal_grid)
                
                
                #로직 2-4. 최단경로 탐색
                if self.grid[start_grid[0]][start_grid[1]] ==0  and self.grid[self.goal_grid[0]][self.goal_grid[1]] ==0  and start_grid != self.goal_grid :
                    # self.dijkstra(start_grid_cell)
                    self.final_path = self.aStar(start_grid,self.goal_grid,self.grid)
                print("path : ", self.final_path)

                #로직 2-5. 최단경로를 map 좌표로 변경
                self.global_path_msg=Path()
                self.global_path_msg.header.frame_id='map'
                for grid_cell in reversed(self.final_path) :
                    tmp_pose=PoseStamped()
                    waypoint_x,waypoint_y=self.grid_cell_to_pose(grid_cell)
                    tmp_pose.pose.position.x=waypoint_x
                    tmp_pose.pose.position.y=waypoint_y
                    tmp_pose.pose.orientation.w=1.0
                    self.global_path_msg.poses.append(tmp_pose)
            
                #로직 3-1 dijkstra 결과값을 publishing하는 코드
                # if len(self.final_path)!=0 :
                    # self.a_star_pub.publish(self.global_path_msg)

                # np_cost = self.cost.reshape(1,350*350)
                # list_cost = np_cost.tolist()[0]

                # list_dijk_msg = []

                # for e in list_cost:
                #     value = int((e)/self.dijk_dist_max * 127) if e <= self.dijk_dist_max else 127
                #     list_dijk_msg.append(value)

                # self.dijk_msg.data = list_dijk_msg
            
                # self.dijk_msg.header.stamp =rclpy.clock.Clock().now().to_msg()
                # self.dijk_pub.publish(self.dijk_msg)
                print("done")
                
    def aStar(self, START=[0,0], END=[9,9], MAP_DATA = [
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0]
        ]):
        time_start = time.time()
        '''
        START, END : map좌표계 시작과 목적지 좌표 (X,Y)
        open : 탐색할 노드 heap
        close : 탐색이 완료된 노드 heap
        '''
        START_Y = START[1]
        START_X = START[0]

        END_Y = END[1]
        END_X = END[0]

        MAP = MAP_DATA
        MAP_SIZE_Y = len(MAP_DATA)
        MAP_SIZE_X = len(MAP_DATA[0])

        D = [[-1,0,1],[-1,1,1.414],[0,1,1],[1,1,1.414],[1,0,1],[1,-1,1.414],[0,-1,1],[-1,-1,1.414]]

        open = nodeHeap(MAP_SIZE_Y, MAP_SIZE_X)
        close = [] #nodeStatus의 memo로 역할 대체

        st = Status(START[1], START[0], 0,0)
        open.push(st, -1)

        cnt = 0
        open.debug()
        while len(open.heap)>0:
            cy,cx,cf,cg,ch = open.pop().get()
            
            if [cy,cx] == [END_Y,END_X] :
                break
            for direction,delta in enumerate(D):
                ny = cy + delta[0]
                nx = cx + delta[1]
                ng = cg + delta[2]
                
                
                nh = self.getH([ny,nx],[END_Y,END_X])
                
                if ny < 0 or ny >= MAP_SIZE_Y or nx < 0 or nx >= MAP_SIZE_X:
                    continue

                if MAP[ny][nx] >= 55 :
                    continue

                if open.memo[ny][nx][0] <= ng+nh:
                    continue

                open.push(Status(ny,nx,ng,nh), direction)
                
            cnt+=1

            #debug a* algorithm
            # f.write("------%d--------\n" % cnt)
            # f.write("%d, %d, %f, %f, %f\n" % (cy,cx,cf,cg,ch))
            # open.debug()    
            # f.write("\n")

        #path = minimal distance
        path = []
        print("*------RESULT------")
        if(open.memo[END_Y][END_X][0] == MAP_SIZE_Y * MAP_SIZE_X):
            print("There is no path")
            path = [-1]
        else:
            cv = open.memo[END_Y][END_X][1]
            cy = END_Y
            cx = END_X
            while True:
                # print(cy, cx)
                path.append([cx,cy])
                if  cy < 0 or cx < 0:
                    break
                if START_Y == cy and START_X == cx:
                    break;

                ny = cy - D[cv][0]   
                nx = cx - D[cv][1]

                cv = open.memo[ny][nx][1]
                cy = ny
                cx = nx
                open.memo[ny][nx][1] +=10
            
            # debug map
            # for i in range(MAP_SIZE_X):
            #     for j in range(MAP_SIZE_Y):
            #         f2.write('%5.3d ' % open.memo[i][j][1])
            #     f2.write("\n")
            
        path.reverse()
        print("a* : %6.3f" %(time.time() - time_start))
        return path
        
def main(args=None):
    rclpy.init(args=args)

    global_planner = a_star()

    rclpy.spin(global_planner)


    global_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
