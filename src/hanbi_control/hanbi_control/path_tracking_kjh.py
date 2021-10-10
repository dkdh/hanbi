import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Point32,PoseStamped
# HandControl 추가
from ssafy_msgs.msg import TurtlebotStatus, HandControl
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
# 추가
from sensor_msgs.msg import LaserScan, PointCloud
from math import pi,cos,sin,sqrt,atan2
import numpy as np
# 추가
import threading
import time
from hanvi_interfaces.msg import DetectionList, Detection
from std_msgs.msg import String

params = {
    "MAP" : {
        "RESOLUTION": 0.1,
        "OCCUPANCY_UP": 0.02,
        "OCCUPANCY_DOWN": 0.01,
        "CENTER": (0, 0),
        "SIZE": (50, 50),
        "FILENAME": 'test.png',
        "MAPVIS_RESIZE_SCALE": 2.0,
        "PATH" : 'C:\\dev\\ros2_smart_home\\src\\hanbi_control\\map\\map_final5_.txt'
    },
    "PATH" :{
        "PATH" : 'C:\\dev\\ros2_smart_home\\src\\hanbi_control\\path\\path_final5_.txt'
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

class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.hand_control = self.create_publisher(HandControl, '/hand_control', 10)

        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        self.path_sub = self.create_subscription(Path,'/local_path',self.path_callback,10)
        # 추가
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback,10)
        self.goal_pub = self.create_publisher(PoseStamped,'/goal_pose',10)
        self.a_star_local_sub = self.create_subscription(Path, '/a_star_local_path', self.a_star_local_callback,10)
        self.hanvi_sub = self.create_subscription(DetectionList, '/hanvi_detection', self.hanvi_callback,10)
        self.via_point_pub= self.create_publisher(PoseStamped, 'via_point', 1)
        self.people_check_sub = self.create_subscription(HandControl, '/people_check', self.people_check_callback,10)
        self.detected_local_path_pub= self.create_publisher(Path, 'detected_local_path', 1)
        self.detected_end_sub = self.create_subscription(String, '/detected_end', self.detected_end_callback,10)

        time_period=0.05 
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom = False
        self.is_path = False
        self.is_status = False
        # 추가
        self.is_lidar = False
        self.collision = False
        
        self.is_a_star = False
        self.preview_flag = False
        self.stop_flag = False
        self.a_star_flag = False

        self.odom_msg = Odometry()            
        self.robot_yaw = 0.0
        self.path_msg = Path()
        self.cmd_msg = Twist()
        self.a_star_msg = Path()

        self.lfd = 0.1
        self.min_lfd = 0.5
        self.max_lfd = 0.5

        # 제어 메시지 변수 생성 
        self.hand_control_msg = HandControl()  
        self.goal_pose_msg = PoseStamped()
        self.hanvi_msg = DetectionList()

        self.is_hanvi = False

        self.return_position = [0,0]

        self.thread_flag_1 = False
        self.thread_flag_2 = False
        self.thread_flag_3 = False

        self.is_via = False
        self.detected_point = []
        self.detected_path = []
        self.detected_local_path = []
        self.detected_path_progress = 0

        self.MAP_DATA = []
        self.status_msg = TurtlebotStatus()

        self.check_mans = False
        self.check_trash = False
        self.check_tent = False

        self.action_mod = False

        f = open(params["MAP"]["PATH"], "r")
        while True:
            line = list(map(int, f.readline().split()))
            if not line: break
            self.MAP_DATA.append(line)
    
    def detected_end_callback(self,msg):
        print("detected_end", msg)
        self.action_mod = Tru
        
        if(msg=='mans' and not self.check_mans):
            self.check_mans = True
            start = time.time()
            while(time.time() - start < 3):
                self.cmd_msg.linear.x=0.0
                self.cmd_msg.angular.z=0.0
                self.cmd_pub.publish(self.cmd_msg)
                print("zzz")

            start = time.time()
            while(time.time() - start < 1):
                # self.cmd_msg.linear.x=1.0
                self.cmd_msg.angular.z=1.0
                self.cmd_pub.publish(self.cmd_msg)
                print("turn")

            start = time.time()
            while(time.time() - start < 1):
                self.cmd_msg.linear.x=1.0
                self.cmd_msg.angular.z=1.0
                self.cmd_pub.publish(self.cmd_msg)
                print("go")


        self.is_via=False
        self.action_mod = False
        pass

    def timer_callback(self):
        if(not self.is_path): return
        if not self.is_via:
            print("일반 모드")
            
            # 로봇의 현재 위치를 나타내는 변수
            robot_pose_x=self.odom_msg.pose.pose.position.x
            robot_pose_y=self.odom_msg.pose.pose.position.y

            lateral_error= sqrt(pow(self.path_msg.poses[0].pose.position.x-robot_pose_x,2)+pow(self.path_msg.poses[0].pose.position.y-robot_pose_y,2))
            self.lfd = (self.status_msg.twist.linear.x+lateral_error)*0.5
            
            if self.lfd < self.min_lfd :
                self.lfd = self.min_lfd
            if self.lfd > self.max_lfd:
                self.lfd = self.max_lfd

            min_dis=float('inf')
            self.is_look_forward_point= False
            for num, waypoint in enumerate(self.path_msg.poses):
                self.current_point=waypoint.pose.position
                dis=sqrt(pow(self.path_msg.poses[0].pose.position.x-self.current_point.x,2)+pow(self.path_msg.poses[0].pose.position.y-self.current_point.y,2))
                if  abs(dis-self.lfd) <= min_dis:
                    min_dis=abs(dis-self.lfd)

                    self.forward_point=self.current_point
                    self.is_look_forward_point=True

                    forward_idx = num
            if(self.is_look_forward_point):
                global_forward_point=[self.forward_point.x ,self.forward_point.y,1]

                trans_matrix = np.array([
                    [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                    [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                    [0, 0, 1],
                ])

                # det_tran_matrix는 trans_matrix의 역행렬로, 기준좌표계(Map)에서 로봇좌표계로 좌표변환을 하기위한 변환 행렬입니다.  
                det_trans_matrix=np.linalg.inv(trans_matrix)
                # local_forward_point 는 global_forward_point를 로봇좌표계로 옮겨온 결과를 저장하는 변수입니다.
                local_forward_point=det_trans_matrix.dot(global_forward_point)
                # theta는 로봇과 전방 주시 포인트와의 각도입니다.
                theta=-atan2(local_forward_point[1],local_forward_point[0])

                
                # 로직 7. 선속도, 각속도 정하기
                # 선속도는 1m/s로 고정함

                # theta가 제어할 각 속도에 들어간다.
                # 2를 곱했다. 클수록 더 빠르게 경로에 수렴한다.
                out_vel = 0.9
                out_rad_vel = theta

                self.cmd_msg.linear.x=out_vel
                self.cmd_msg.angular.z=out_rad_vel
            else:
                print("no found forward point")
                self.cmd_msg.linear.x=0.0
                self.cmd_msg.angular.z=0.0

            self.cmd_pub.publish(self.cmd_msg)
        else:
            if(not self.action_mod):
                print("접근 모드")

                # 생성된 path가 없을 때만 접근
                if not self.detected_path  :
                    print("접근모드 경로 없음")
                    return

                
                x=self.odom_msg.pose.pose.position.x
                y=self.odom_msg.pose.pose.position.y

                #
                current_waypoint= -1
                min_dis=float('inf')
                for i,waypoint in enumerate(self.detected_path):
                    distance=sqrt(pow(x-waypoint[0],2)+pow(y-waypoint[1], 2))
                    if distance < min_dis:
                        min_dis=distance
                        current_waypoint=i
                
                '''
                로직 6. local_path 예외 처리
                '''
                local_path_msg = Path()
                if current_waypoint != -1 :
                    for num in range(current_waypoint, len(self.detected_path)):
                        tmp_pose=PoseStamped()
                        tmp_pose.pose.position.x=self.detected_path[num][0]
                        tmp_pose.pose.position.y=self.detected_path[num][1]
                        tmp_pose.pose.orientation.w=1.0
                        local_path_msg.poses.append(tmp_pose)
                local_path_msg.header.frame_id = 'mans'
                self.detected_local_path_pub.publish(local_path_msg)
            else:
                print("액션 모드")

    def odom_callback(self, msg):
        self.is_odom=True
        self.odom_msg=msg
        # 로직 3. Quaternion 을 euler angle 로 변환
        q=Quaternion(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        _,_,self.robot_yaw=q.to_euler()
    
    def path_callback(self, msg):
        self.is_path=True
        self.path_msg=msg

    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg

    def people_check_callback(self, msg):
        #! 콜백함수 등록
        if(self.check_mans): return
        if not self.is_via:
            # 접근 모드 시작
            print("접근 모드 시작")
            self.is_via = True
            x = self.odom_msg.pose.pose.position.x
            y = self.odom_msg.pose.pose.position.y

            # 멈춤
            self.cmd_msg.linear.x=0.0
            self.cmd_msg.angular.z=0.0
            self.cmd_pub.publish(self.cmd_msg)

            # 목표 좌표 msg로부터 추출
            self.detected_point = [msg.put_distance, msg.put_height]
            self.detected_path = []
            self.detected_path_progress=0

            detected_point_msg = PoseStamped()
            detected_point_msg.pose.position.x = msg.put_distance
            detected_point_msg.pose.position.y = msg.put_height
            detected_point_msg.pose.orientation.w=1.0

            s = pose2grid([x,y])
            e = pose2grid(self.detected_point)
            print(s, e)
            grid_path = self.aStar(pose2grid([x,y]),pose2grid(self.detected_point),self.MAP_DATA)


            # 변경
            for e in grid_path:
                self.detected_path.append(grid2pose(e))
            
            print(self.detected_path)
            
    def getH(self, START, END):
        #맨하튼 거리
        return sqrt(pow(START[0] - END[0],2) + pow(START[1] - END[1],2))
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


    # 추가
    def lidar_callback(self, msg):
        self.lidar_msg = msg
        # 충돌 체크를 하려면 경로와 위치를 알고 있어야 한다.
        if self.is_path == True and self.is_odom == True:
            
            # 직교좌표계 형태로 데이터타입을 가지는 포인트 클라우드 생성
            pcd_msg = PointCloud()
            pcd_msg.header.frame_id ='map'

            pose_x = self.odom_msg.pose.pose.position.x
            pose_y = self.odom_msg.pose.pose.position.y
            theta = self.robot_yaw

            t = np.array([
                            [cos(theta), -sin(theta), pose_x],
                            [sin(theta),cos(theta), pose_y],
                            [0,0,1]
                        ])
            for angle, r in enumerate(msg.ranges):
                # angle은 0부터 359까지 있는데
                # 다른 파일에서 -1 인덱스에 0을 집어넣어서 359는 0으로 나오는 걸 주의하자
                global_point = Point32()

                if 0 < r < 12:
                    # 로컬 극좌표계를 직교좌표계로 바꾸고 글로벌로 바꾼다.
                    local_x = r*cos(angle*pi/100)
                    local_y = r*sin(angle*pi/100)
                    local_point = np.array([[local_x],[local_y],[1]])
                    global_result = t.dot(local_point)
                    global_point.x = global_result[0][0]
                    global_point.y = global_result[1][0]
                    pcd_msg.points.append(global_point)

            # 이제 로컬에 있던 라이다 점들을 글로벌로 다 가져온 상태
            # 이제 경로점과 비교해서 충돌체크할 수 있다.            
            
            # 이중 포문을 이용해서 모든 경로점과 모든 라이다 간에 거리를 본다.
            self.collision = False
            for waypoint in self.path_msg.poses[:10]:
                # 우측 전방 30도와 좌측 전방 30도
                for lidar_point in pcd_msg.points:
                    distance = sqrt(pow(waypoint.pose.position.x-lidar_point.x, 2)+pow(waypoint.pose.position.y-lidar_point.y, 2))
                    # 0.1m
                    if distance < 0.4:
                        print("충돌")
                        self.collision = True

            self.is_lidar = True

    def lost_pick_up(self):
        print("분실물 들기")
        self.hand_control_msg.control_mode = 2
        # 잠깐 멈추기 필수
        time.sleep(1.5)
        while (self.status_msg.can_lift):
            print("분실물 들자")
            self.hand_control.publish(self.hand_control_msg)
        print("분실물 들기 완료")
        self.thread_flag_3 = True
        

    def hand_control_pick_up(self):
        print("픽업 함수")
        self.hand_control_msg.control_mode = 2
        # 잠깐 멈추기 필수
        time.sleep(1.5)
        while (self.status_msg.can_lift):
            print("들자")
            self.hand_control.publish(self.hand_control_msg)
        # 바로 preview
        self.hand_control_msg.control_mode = 1
        self.hand_control_msg.put_distance = 1.5
        self.hand_control_msg.put_height = -2.0
        while (not self.status_msg.can_put):
            self.hand_control.publish(self.hand_control_msg)
        self.preview_flag = True
        print("픽업 함수 완료")
        # 원상 복귀
        self.thread_flag_1 = True

        self.goal_pose_msg.header.frame_id = 'map'
        # 쓰레기통 위치
        self.goal_pose_msg.pose.position.x = -6.69
        self.goal_pose_msg.pose.position.y = 5.83
        self.goal_pub.publish(self.goal_pose_msg)
        # 돌아올 곳
        self.return_position = [self.path_msg.poses[-1].pose.position.x, self.path_msg.poses[-1].pose.position.y]

        self.a_star_flag = True


    def hand_control_put_down(self):
        self.hand_control_msg.control_mode = 3
        print("내려놓기")
        while (self.status_msg.can_put):
            self.hand_control.publish(self.hand_control_msg)
        self.preview_flag = False
        # time.sleep(0.5)
        self.stop_flag = False
        self.thread_flag_2 == False
        self.return_position = [0,0]

    def a_star_local_callback(self, msg):
        self.is_a_star = True
        self.a_star_msg = msg

    def hanvi_callback(self,msg):
        self.is_hanvi = True
        self.hanvi_msg = msg
    
def main(args=None):
    rclpy.init(args=args)

    path_tracker = followTheCarrot()
    rclpy.spin(path_tracker)

    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()