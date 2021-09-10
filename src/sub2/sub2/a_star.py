import rclpy
import numpy as np
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData,Path
from math import pi,cos,sin
from collections import deque

# a_star 노드는  OccupancyGrid map을 받아 grid map 기반 최단경로 탐색 알고리즘을 통해 로봇이 목적지까지 가는 경로를 생성하는 노드입니다.
# 로봇의 위치(/pose), 맵(/map), 목표 위치(/goal_pose)를 받아서 전역경로(/global_path)를 만들어 줍니다. 
# goal_pose는 rviz2에서 2D Goal Pose 버튼을 누르고 위치를 찍으면 메시지가 publish 됩니다. 
# 주의할 점 : odom을 받아서 사용하는데 기존 odom 노드는 시작했을 때 로봇의 초기 위치가 x,y,heading(0,0,0) 입니다. 로봇의 초기위치를 맵 상에서 로봇의 위치와 맞춰줘야 합니다. 
# 따라서 sub2의 odom 노드를 수정해줍니다. turtlebot_status 안에는 정답데이터(절대 위치)가 있는데 그 정보를 사용해서 맵과 로봇의 좌표를 맞춰 줍니다.

# self.odom_msg.pose.pose.position.x
# self.odom_msg.pose.pose.position.y
# 를 보면 0에 가까운 수다. (e-10 이렇게 붙어있다.)

# odom.py에서 self.x와 self.y을 0이라고 했기 때문에
# self.x를 -1로 하니까 base_link와 laser가 격자 1칸 만큼 움직인다.

# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 파라미터 설정
# 3. 맵 데이터 행렬로 바꾸기
# 4. 위치(x,y)를 map의 grid cell로 변환
# 5. map의 grid cell을 위치(x,y)로 변환
# 6. goal_pose 메시지 수신하여 목표 위치 설정
# 7. grid 기반 최단경로 탐색

class a_star(Node):

    def __init__(self):
        super().__init__('a_Star')
        # 로직 1. publisher, subscriber 만들기
        self.map_sub = self.create_subscription(OccupancyGrid,'map',self.map_callback,1)
        self.odom_sub = self.create_subscription(Odometry,'odom',self.odom_callback,1)
        self.goal_sub = self.create_subscription(PoseStamped,'goal_pose',self.goal_callback,1)
        self.a_star_pub= self.create_publisher(Path, 'global_path', 1)
        
        self.map_msg=OccupancyGrid()
        self.odom_msg=Odometry()
        self.is_map=False
        self.is_odom=False
        self.is_found_path=False
        self.is_grid_update=False

        # 로직 2. 파라미터 설정

        # default는 거실에 있는 에어컨 앞
        self.goal = [184,224] 
        self.map_size_x=350
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-8-8.75
        self.map_offset_y=-4-8.75
    
        self.GRIDSIZE=350

        # 가로 세로 1은 비용이 1, 대각선은 1.414
        self.dx = [-1,0,0,1,-1,-1,1,1]
        self.dy = [0,1,-1,0,-1,1,-1,1]
        self.dCost = [1,1,1,1,1.414,1.414,1.414,1.414]
       

    # goal_pose 정보를 받으면 실행되는 goal_callback에서 grid_update가 실행된다.
    def grid_update(self):
        self.is_grid_update=True

        # 로직 3. 맵 데이터 행렬로 바꾸기
        map_to_grid = np.reshape(self.map_msg.data, (350, 350))
        self.grid = map_to_grid

    def pose_to_grid_cell(self,x,y):
        # map_point_x = 0
        # map_point_y = 0

        # 로직 4. 위치(x,y)를 map의 grid cell로 변환
        # (테스트) pose가 (-8,-4)라면 맵의 중앙에 위치하게 된다. 따라서 map_point_x,y 는 map size의 절반인 (175,175)가 된다.
        # pose가 (-16.75,-12.75) 라면 맵의 시작점에 위치하게 된다. 따라서 map_point_x,y는 (0,0)이 된다.
        # 즉 x가 8.75 증가했는데 175만큼 커졌고 (x가 1증가하면 175/8.75 커진다.)
        # y도 마찬가지다.
        
        # cf. int(2.3)은 2다.

        # 명세서를 보니까 self.map_resolution=0.05 self.map_offset_x=-8-8.75 self.map_offset_y=-4-8.75
        # 이 값들을 이용하라고 되어있어서 숫자 대신에 변수를 써도 좋을 것 같다.

        map_point_x = int((x+16.75)*175/8.75)
        map_point_y = int((y+12.75)*175/8.75)
        
        return map_point_x, map_point_y


    def grid_cell_to_pose(self,grid_cell):
        x = 0
        y = 0

        # 로직 5. map의 grid cell을 위치(x,y)로 변환
        # (테스트) grid cell이 (175,175)라면 맵의 중앙에 위치하게 된다. 따라서 pose로 변환하게 되면 맵의 중앙인 (-8,-4)가 된다.
        # grid cell이 (350,350)라면 맵의 제일 끝 좌측 상단에 위치하게 된다. 따라서 pose로 변환하게 되면 맵의 좌측 상단인 (0.75,6.25)가 된다.

        # 최단 경로 탐색 결과는 grid_cell로 나오고
        # 전역 경로로 만들 때는 절대 위치로 변환해서 사용해야 한다.

        x = grid_cell[0]/20-16.75
        y = grid_cell[1]/20-12.75

        return [x,y]


    def odom_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg


    def map_callback(self,msg):
        self.is_map=True
        self.map_msg=msg
        

    def goal_callback(self,msg):
        
        if msg.header.frame_id=='map':
            
            # 로직 6. goal_pose 메시지 수신하여 목표 위치 설정
            goal_x= msg.pose.position.x
            goal_y= msg.pose.position.y
            goal_cell= self.pose_to_grid_cell(goal_x, goal_y)
            # print(self.goal) 찍어보면 (328,223) 이렇게 나온다.
            self.goal = list(goal_cell)

            # 지도 받아왔고 odom 정보도 있는 상태에서 목적지가 정해졌다면
            if self.is_map ==True and self.is_odom==True  :
                
                if self.is_grid_update == False:
                    self.grid_update()

                self.final_path=[]

                # 현재 위치
                x=self.odom_msg.pose.pose.position.x
                y=self.odom_msg.pose.pose.position.y
                # 터틀봇의 절대 위치를 그리드 위치로 변환하기 인 것 같다.
                start_grid_cell = self.pose_to_grid_cell(x,y)
                start_grid_cell = list(start_grid_cell)

                # self.GRIDSIZE는 350이다.
                # 0으로 채워진 350 X 350 행렬 만들기
                self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]

                # 350 * 350 으로 채워진 350 X 350 행렬 만들기
                self.cost = np.array([[self.GRIDSIZE*self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])

                # 다익스트라 알고리즘을 완성하고 주석을 해제 시켜주세요. 
                # 시작지, 목적지가 탐색가능한 영역이고, 시작지와 목적지가 같지 않으면 경로탐색을 합니다.
                # 0은 장애물이 없는 영역을 의미한다.
                if self.grid[start_grid_cell[0]][start_grid_cell[1]] == 0 and self.grid[self.goal[0]][self.goal[1]] == 0  and start_grid_cell != self.goal :
                    # 시작점을 넣어주었다.
                    self.dijkstra(start_grid_cell)

                self.global_path_msg=Path()
                self.global_path_msg.header.frame_id='map'

                # 순서 바꿔주고
                # 각 grid cell에 대해서
                for grid_cell in reversed(self.final_path) :
                    tmp_pose=PoseStamped()
                    waypoint_x,waypoint_y=self.grid_cell_to_pose(grid_cell)
                    
                    tmp_pose.pose.position.x=waypoint_x
                    tmp_pose.pose.position.y=waypoint_y
                    tmp_pose.pose.orientation.w=1.0
                    
                    # 차곡차곡 담기
                    self.global_path_msg.poses.append(tmp_pose)

                # 경로가 존재한다면
                if len(self.final_path)!=0 :
                    self.a_star_pub.publish(self.global_path_msg)

    def dijkstra(self,start):
        # deque는 양쪽에서 삽입 삭제가 가능하다.
        Q = deque()
        Q.append(start)
        self.cost[start[0]][start[1]] = 1
        found = False

        # 로직 7. grid 기반 최단경로 탐색
        while Q:
            # popleft를 통해서 한 쪽 방향으로만 뺀다.
            # 파장이 퍼지듯이 경로를 찾기 때문에
            # 가장 먼저 도착하면 그게 최단경로다.
            if current == self.goal:
                found = True
                break

            current = Q.popleft()

            # 시작점을 기준으로 8방향을 의미하는 것 같다.
            for i in range(8):

                next = [current[0]+self.dx[i], current[1]+self.dy[i]]
                
                # 범위를 벗어나지 않는 좌표 중에서
                if next[0] >= 0 and next[1] >= 0 and next[0] < self.GRIDSIZE and next[1] < self.GRIDSIZE:
                    # 이동가능한 영역이라는 의미인 것 같다.
                    if self.grid[next[0]][next[1]] < 50:

                        # cost 비교
                        if self.cost[current[0]][current[1]] + self.dCost[i] < self.cost[next[0]][next[1]]:
                            
                            # 넥스트를 넣어준다.
                            Q.append(next)
                            # 명세서 : path를 역으로 추적해서 최종 경로를 얻는다.
                            # 이 노드가 어디서 왔는지를 적어주어야 하는 것 같다.
                            self.path[next[0]][next[1]] = current
                            self.cost[next[0]][next[1]] = self.cost[current[0]][current[1]] + self.dCost[i]

        # path를 역으로 추적하는 코드
        # 나중에 reversed로 list 순서 바꿔준다.
        node = self.goal
        # 도착지 넣어주고
        self.final_path.append(node)

        while node != start:
            # node는 nextNode에서 왔다
            nextNode = self.path[node[0]][node[1]]
            # nextNode 넣어주고
            self.final_path.append(nextNode)
            node = nextNode

    def a_star(self,start):
        pass        

        
def main(args=None):
    rclpy.init(args=args)
    global_planner = a_star()
    rclpy.spin(global_planner)
    global_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
