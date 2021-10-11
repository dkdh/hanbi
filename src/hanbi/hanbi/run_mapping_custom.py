from typing_extensions import final
from numpy.core.numeric import full
import rclpy
from rclpy.node import Node
import ros2pkg
from geometry_msgs.msg import Twist,PoseStamped,Pose,TransformStamped
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path,OccupancyGrid,MapMetaData
from math import pi,cos,sin,sqrt
import tf2_ros
import os
import sub3.utils as utils
import numpy as np
import cv2
import time
import socketio
import os
import json


# 500*500을 주기적으로 보내니 막힘
# mapping node의 전체 로직 순서
# 1. publisher, subscriber, msg 생성
# 2. mapping 클래스 생성
# 3. 맵의 resolution, 중심좌표, occupancy에 대한 threshold 등의 설정 받기
# 4. laser scan 메시지 안의 ground truth pose 받기
# 5. lidar scan 결과 수신
# 6. map 업데이트 시작
# 7. pose 값을 받아서 좌표변환 행렬로 정의
# 8. laser scan 데이터 좌표 변환
# 9. pose와 laser의 grid map index 변환
# 10. laser scan 공간을 맵에 표시
# 11. 업데이트 중인 map publish
# 12. 맵 저장

sio = socketio.Client(reconnection_delay=1, reconnection_delay_max=4)

params_map = {
    "MAP_RESOLUTION": 0.1,
    "OCCUPANCY_UP": 0.02,
    "OCCUPANCY_DOWN": 1,
    "MAP_CENTER": (-50, -50),
    "MAP_SIZE": (50, 50),
    "MAP_FILENAME": 'test.png',
    "MAPVIS_RESIZE_SCALE": 2.0
}
is_map_exist = False
md = np.zeros((500,500))


import threading



def createLineIterator(P1, P2, img):
    # Bresenham's line algorithm을 구현해서 이미지에 직선을 그리는 메소드입니다.
    
    # 로직 순서
    # 1. 두 점을 있는 백터의 x, y 값과 크기 계산
    # 2. 직선을 그릴 grid map의 픽셀 좌표를 넣을 numpy array 를 predifine
    # 3. 직선 방향 체크
    # 4. 수직선의 픽셀 좌표 계산
    # 5. 수평선의 픽셀 좌표 계산
    # 6. 대각선의 픽셀 좌표 계산
    # 7. 맵 바깥 픽셀 좌표 삭제

   
    imageH = img.shape[0] #height
    imageW = img.shape[1] #width
    P1Y = P1[1] #시작점 y 픽셀 좌표
    P1X = P1[0] #시작점 x 픽셀 좌표
    P2X = P2[0] #끝점 y 픽셀 좌표
    P2Y = P2[1] #끝점 x 픽셀 좌표

    """
    로직 1 : 두 점을 있는 백터의 x, y 값과 크기 계산
    """
    
    dX = P2X - P1X
    dY = P2Y - P1Y
    dXa = np.abs(dX)
    dYa = np.abs(dY)


    """
    # 로직 2 : 직선을 그릴 grid map의 픽셀 좌표를 넣을 numpy array 를 predifine
    """

    itbuffer = np.empty(shape=(np.maximum(dYa,dXa),3),dtype=np.float32)
    itbuffer.fill(np.nan)


    """
    # 로직 3 : 직선  방향 체크                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
    """
    negY =  True if (dY < 0) else False
    negX = True if (dX < 0) else False
 
    
    # 로직 4 : 수직선의 픽셀 좌표 계산   
    if P1X == P2X: #vertical line segment
        itbuffer[:,0] = P1X
        if negY:
            itbuffer[:,1] = np.arange(P1Y - 1,P1Y - dYa - 1,-1)
        else:
            itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)
    # 로직 5 : 수평선의 픽셀 좌표 계산
    elif P1Y == P2Y: #horizontal line segment
        itbuffer[:,1] = P1Y
        if negX:
            itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
        else:
            itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)     
    # 로직 6 : 대각선의 픽셀 좌표 계산  
    else: 
        steepSlope = dYa > dXa
        if steepSlope:
            slope = dX.astype(np.float32)/dY.astype(np.float32)
            if negY:
                itbuffer[:,1] = np.arange(P1Y-1,P1Y-dYa-1,-1)
            else:
                itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)
            itbuffer[:,0] = (slope*(itbuffer[:,1]-P1Y)).astype(np.int32) + P1X
        else:
            slope = dY.astype(np.float32)/dX.astype(np.float32)
            if negX:
                itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
            else:
                itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
            itbuffer[:,1] = (slope*(itbuffer[:,0]-P1X)).astype(np.int32) + P1Y

    
    """
    로직 7 : 맵 바깥 픽셀 좌표 삭제.
    """
    colX = itbuffer[:,0]
    colY = itbuffer[:,1]
    itbuffer = itbuffer[(colX >= 0) & (colY >=0) & (colX<imageW) & (colY<imageH)]
    
    #itbuffer[x][y][img[x][y]]
    itbuffer[:,2] = img[itbuffer[:,1].astype(np.uint),itbuffer[:,0].astype(np.uint)]
    
    return itbuffer


class Mapping:

    # 사용자가 정의한 맵 설정을 받아서 회색의 어레이로 초기화 시키고,
    # 로봇의 pose와 2d 라이다 값들을 받은 다음,
    # 라이다가 나타내는 로봇으로부터 측정된 좌표와의 직선을
    # utils_skeleton.py에 있는 createLineIterator()로
    # 그려가면서 맵을 채워서 저장할 수 있도록 만든 스크립트입니다.

    def __init__(self, params_map):

        # 로직 3. 맵의 resolution, 중심좌표, occupancy에 대한 threshold 등의 설정들을 받습니다
        self.map_resolution = params_map["MAP_RESOLUTION"]
        self.map_size = np.array(params_map["MAP_SIZE"]) / self.map_resolution
        self.map_center = params_map["MAP_CENTER"]
        is_map_exist = True
        self.occu_up = params_map["OCCUPANCY_UP"]
        self.occu_down = params_map["OCCUPANCY_DOWN"]


        self.map_filename = params_map["MAP_FILENAME"]
        self.map_vis_resize_scale = params_map["MAPVIS_RESIZE_SCALE"]

        self.T_r_l = np.array([[0,-1,0],[1,0,0],[0,0,1]])


    def update(self, pose, laser):
        global md
        # do 직교좌표계로 변환된 라이다 정보를 이용해 map에 표시
        # laser = [2][theta] = cos theta와 sin theta. theta는 0~360

        # pose = 라이다 히트포인트 거리값 [x, y, theta]

        # 로직 7. pose 값을 받아서 직선좌표계로 정의
        n_points = laser.shape[1] #(2, 360)[1] = 360

        # 극좌표계를 평면 좌표계로 변환        
        pose_mat = utils.xyh2mat2D(pose)
        # print("pose_mat : ", pose_mat)
            # [[ 0.97656031  0.21524396 -7.45240831]
            #  [-0.21524396  0.97656031 -1.99444008]
            #  [ 0.          0.          1.        ]]


        # 로직 8. laser scan 데이터 좌표 변환
        #pose_mat = [x,y,theta][[0, -1, 0],[1,0,0][0,0,1] = [y, -x, theta]

        # 0,0인 원점을 레이더로 이동하는 매트릭스 생성
        pose_mat = np.matmul(pose_mat,self.T_r_l)
        # print("pose_mat_transformed : ", pose_mat)
        #[[ 0.21524448 -0.9765602  -7.45240784]
        #  [ 0.9765602   0.21524448 -1.9944396 ]
        #  [ 0.          0.          1.        ]]

        laser_mat = np.ones((3, n_points))
        laser_mat[:2, :] = laser
        # [0,1][0~360] = distance (기저 벡터 값)
        # [2][0~360] = theta

        laser_global = np.matmul(pose_mat, laser_mat)

        """
        로직 9. pose와 laser의 grid map index 변환
        """
        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y = (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution
        

        """
        # 로직 10. laser scan 공간을 맵에 표시
        """        
        for i in range(laser_global.shape[1]):
            p1 = np.array([pose_x, pose_y]).reshape(-1).astype(np.int32)
            p2 = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int32)
        
            line_iter = createLineIterator(p1, p2, md)
        
            if (line_iter.shape[0] is 0):
                continue
        
            avail_x = line_iter[:, 0].astype(np.int32)
            avail_y = line_iter[:, 1].astype(np.int32)
        
            ##값이 변동되면 socket으로 전송
            ## Empty
            # for y,x in zip(avail_y[:-1], avail_x[:-1]):
            #     self.map[y, x] = self.map[y, x] + self.occu_down
            #     print(self.map[y,x])
            #     # print("Map2Server", [y,x,self.map[y,x]], " / ",  [int(y), int(x), int(self.pixel(y,x))])
            #     sio.emit("Map2Server", [int(y),int(x),self.pixel(y,x)])

            # # ## Occupied
            # finalY, finalX = avail_y[-1], avail_x[-1]
            # self.map[finalY, finalX] = self.map[finalY, finalX] - self.occu_up
            # sio.emit("Map2Server", [int(finalY), int(finalX),self.pixel(y,x)])
            
            #v1
            # term = 1000
            # for idx, e in enumerate(avail_y[:-1:term]):
            #     cy = avail_y[term * idx:term * idx + term]
            #     cx = avail_x[term * idx:term * idx + term]
                
            #     self.map[cy, cx] = self.map[cy, cx] + self.occu_down
            #     print("map : {0} ~ {1}, {2}, ".format(term * idx, term * idx + term, self.map[cy,cx]))
            #     print("Map2Server", list(zip(cy.tolist(),cx.tolist(),list(map(self.pixel, cy, cx)))))
                # sio.emit("Map2Server", list(zip(cy.tolist(),cx.tolist(),list(map(self.pixel, cy, cx)))))

            #v2
            buf = []
            for y, x in zip(avail_y, avail_x):
                # if(md[y,x] == 1): continue
                md[y,x] = 1
                buf.append((int(y), int(x), md[y,x]))
            print("Map2Server", len(buf))
            sio.emit("Map2Server", buf)

    def __del__(self):
        # 로직 12. 종료 시 map 저장
        ## Ros2의 노드가 종료될 때 만들어진 맵을 저장하도록 def __del__과 save_map이 정의되어 있습니다
        # self.save_map(())
        full_path = os.path.dirname(os.path.abspath(__file__))+'\\..\\map\\map_final5.txt'
        open(full_path, "w")

    # def save_map(self):
    #     map_clone = self.map.copy()
    #     cv2.imwrite(self.map_filename, map_clone*255)




        
class Mapper(Node):

    def __init__(self):
        # print(os.path.realpath(__file__))
        print(os.path.dirname(os.path.abspath(__file__))+'../map/map_final5.txt')
        full_path = os.path.dirname(os.path.abspath(__file__))+'\\..\\map\\map_final5.txt'
        # print(os.path.dirname(os.path.abspath('.')))
        super().__init__('Mapper')
        # full_path = '../map/map_final5.txt'
        f = open(full_path, "w")
        for i in range(500):
            f.writelines(str([0 for x in range(500)]))
            f.writelines('\n')
        
        # 로직 1.  publisher, subscriber, msg 생성
        # 로직 1-1. subscription = laserScan 정보 수신
        self.subscription = self.create_subscription(LaserScan,
        '/scan',self.scan_callback,10)
        # 로직 1-2. map_pub = 장애물을 인식해 반환
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        # 로직 1-3. 터틀봇의 상태 수신 및 송신
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback,10)
        
        # 로직 1-3. 장애물 정보를 맵에서 나타내기 위한 grid_cell 생성 500*500
        self.map_msg=OccupancyGrid()
        self.map_msg.header.frame_id="map"
        self.map_size=int(params_map["MAP_SIZE"][0]\
            /params_map["MAP_RESOLUTION"]*params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])

        self.save_timer = self.create_timer(5, self.save_callback)
        

        # 로직 1-4. map 좌표공간을 설정
        m = MapMetaData()
        m.resolution = params_map["MAP_RESOLUTION"]# 0.05
        m.width = int(params_map["MAP_SIZE"][0]/params_map["MAP_RESOLUTION"])# 500
        m.height = int(params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])# 500
        quat = np.array([0, 0, 0, 1])

        # 로직 1-5. 생성한 map 좌표 공간을 배치할 원점을 설정
        m.origin = Pose()
        m.origin.position.x = params_map["MAP_CENTER"][0]-(params_map["MAP_SIZE"][0]/2)
        m.origin.position.y = params_map["MAP_CENTER"][1]-(params_map["MAP_SIZE"][1]/2)
        self.map_meta_data = m
        self.map_msg.info=self.map_meta_data

        # 로직 2 : mapping 클래스 생성
        self.mapping = Mapping(params_map)
    
    def status_callback(self, msg):
        gridPose =[(msg.twist.angular.y - self.map_meta_data.origin.position.y) / params_map["MAP_RESOLUTION"],(msg.twist.angular.x - self.map_meta_data.origin.position.x) / params_map["MAP_RESOLUTION"]] 
        self.pos=gridPose
        print( self.pos, msg.twist.angular.y, self.map_meta_data.origin.position.y, params_map["MAP_RESOLUTION"])

    def scan_callback(self,msg):
        # 로직 4. lader subscribtion 값으로 ground truth pose를 받는다
        # 속성 명과 관계없이 임의로 보낸 ground truth pose

        pose_x = msg.range_min #[theta]방향의 hitpoint x 벡터값
        pose_y = msg.scan_time #[theta]방향의 hitpoint y 벡터값
        heading = msg.time_increment #theta
        
        # 로직 5 : lidar scan 결과 수신
        Distance= Distance=np.array(msg.ranges) #Distance[theta] = distance from lidar theta방향의 hit point 거리값


        # 라이다로 감지된 hit point의 극좌표계 값을 직교 좌표계 x, y값으로 변환
        x = Distance * np.cos(np.linspace(0, 2 * np.pi, 360))
        y = Distance * np.sin(np.linspace(0, 2 * np.pi, 360))
        laser = np.vstack((x.reshape((1, -1)), y.reshape((1, -1))))

        # 로직 6 : map 업데이트 실행(4,5번이 완성되면 바로 주석처리된 것을 해제하고 쓰시면 됩니다.)
        # pose 로봇의 전체 좌표계 극좌표값
        pose = np.array([[pose_x],[pose_y],[heading]])
        self.mapping.update(pose, laser)            


    def save_callback(self):
        
        full_path = os.path.dirname(os.path.abspath(__file__))+'\\..\\map\\map_final5.txt'
        f=open(full_path,'w')

        for idx,pixel in enumerate(self.map_msg.data) :
            data=''
            data+='{0} '.format(pixel)
            f.write(data) 
            if((idx+1) % 500 ==0):
                f.write("\n")
        f.close()
        



    
def main(args=None):    
    rclpy.init(args=args)
    ip_server = 'http://localhost:3000'
    @sio.event
    def connect():
        print('connection established',sio.sid)
        md = np.zeros((500,500))
        
    @sio.event
    def disconnect():
        print('disconnected from server')
        
    @sio.event
    def connect_error(data):
        print("connect_error")

    @sio.on("MapInit")
    def listen_node():
        global md
        print(md)
        md = np.zeros((500,500))
            

    

    print("connect ", ip_server)
    sio.connect(ip_server)

    try :
        #로직 1. 서버 통신 소켓 설정
        #로직 1-1. 소켓 이벤트 정의
        run_mapping = Mapper()
        rclpy.spin(run_mapping)
        run_mapping.destroy_node()
        rclpy.shutdown()

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()