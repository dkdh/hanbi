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
    "MAP_RESOLUTION": 0.05,
    "OCCUPANCY_UP": 0.02,
    "OCCUPANCY_DOWN": 0.01,
    "MAP_CENTER": (-8.0, -4.0),
    "MAP_SIZE": (17.5, 17.5),
    "MAP_FILENAME": 'test.png',
    "MAPVIS_RESIZE_SCALE": 2.0
}


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
            itbuffer[:,0] = (slope*(itbuffer[:,1]-P1Y)).astype(np.int) + P1X
        else:
            slope = dY.astype(np.float32)/dX.astype(np.float32)
            if negX:
                itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
            else:
                itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
            itbuffer[:,1] = (slope*(itbuffer[:,0]-P1X)).astype(np.int) + P1Y

    
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
        self.map = np.ones((self.map_size[0].astype(np.int), self.map_size[1].astype(np.int)))*0.5
        self.occu_up = params_map["OCCUPANCY_UP"]
        self.occu_down = params_map["OCCUPANCY_DOWN"]

        self.map_filename = params_map["MAP_FILENAME"]
        self.map_vis_resize_scale = params_map["MAPVIS_RESIZE_SCALE"]

        self.T_r_l = np.array([[0,-1,0],[1,0,0],[0,0,1]])

        # self.set_interval(self.func, 1.5)



    def update(self, pose, laser):
        # print("update")
        # do 직교좌표계로 변환된 라이다 정보를 이용해 map에 표시
        # laser = 라이다 센서의 스캔 결과값
        #       [0,1][theta] = 라이다의 theta 방향이 도달한 거리의 0,1 벡터 
        # pose = 라이다 샌서로 얻은 극좌표값 [x, y, theta]

        # 로직 7. pose 값을 받아서 직선좌표계로 정의
        n_points = laser.shape[1] #(2, 360)[1] = 360
        # print(pose)
        pose_mat = utils.xyh2mat2D(pose)
        # print("pose_mat : ", pose_mat)
            # [[ 0.97656031  0.21524396 -7.45240831]
            #  [-0.21524396  0.97656031 -1.99444008]
            #  [ 0.          0.          1.        ]]


        # 로직 8. laser scan 데이터 좌표 변환
        #pose_mat = [x,y,theta][[0, -1, 0],[1,0,0][0,0,1] = [y, -x, theta]
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

        # (-9.9 - (-8) + (350 * 0.05)/2) / 0.05
        # (-1.9 + 8.75) / 0.05
        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        # print(" (%d - %d + ( %d * %f)/2 )/ %f = %f "%(pose[0], self.map_center[0], self.map_size[0], self.map_resolution, self.map_resolution, pose_x))
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y = (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution
        

        """
        # 로직 10. laser scan 공간을 맵에 표시
        """        
        for i in range(laser_global.shape[1]):
            p1 = np.array([pose_x, pose_y]).reshape(-1).astype(np.int)
            p2 = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int)
        
            line_iter = createLineIterator(p1, p2, self.map)
        
            if (line_iter.shape[0] is 0):
                continue
        
            avail_x = line_iter[:, 0].astype(np.int)
            avail_y = line_iter[:, 1].astype(np.int)
        
            ## Empty
            self.map[avail_y[:-1], avail_x[:-1]] = self.map[avail_y[:-1], avail_x[:-1]] + self.occu_down
        
            ## Occupied 
            self.map[avail_y[-1], avail_x[-1]] = self.map[avail_y[-1], avail_x[-1]] - self.occu_up

        self.show_pose_and_points(pose, laser_global)


    def __del__(self):
        # 로직 12. 종료 시 map 저장
        ## Ros2의 노드가 종료될 때 만들어진 맵을 저장하도록 def __del__과 save_map이 정의되어 있습니다
        self.save_map(())

    def save_map(self):
        map_clone = self.map.copy()
        cv2.imwrite(self.map_filename, map_clone*255)


    def show_pose_and_points(self, pose, laser_global):
        tmp_map = self.map.astype(np.float32)
        map_bgr = cv2.cvtColor(tmp_map, cv2.COLOR_GRAY2BGR)

        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y =  (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        for i in range(laser_global.shape[1]):
            (l_x, l_y) = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int)
            center = (l_x, l_y)
            cv2.circle(map_bgr, center, 1, (0,255,0), -1)

        center = (pose_x.astype(np.int32)[0], pose_y.astype(np.int32)[0])
        
        cv2.circle(map_bgr, center, 2, (0,0,255), -1)

        map_bgr = cv2.resize(map_bgr, dsize=(0, 0), fx=self.map_vis_resize_scale, fy=self.map_vis_resize_scale)
        # cv2.imshow('Sample Map', map_bgr)
        # cv2.waitKey(1)

    def set_interval(self, func, sec):
        def func_wrapper():
            self.set_interval(func, sec)
            func()
        t = threading.Timer(sec, func_wrapper)
        t.start()
        return t

    def func(self):
        print("zz")
        if(sio.connected):
             sio.emit('Map2Server', (self.map*255).tolist())

    def func2(self):
        print("ff")

        
class Mapper(Node):

    def __init__(self):
        super().__init__('Mapper')
        full_path = "C:\\dev\\ros2_smart_home\\src\\sub2\\map\\map.txt"
        f = open(full_path, "w")
        for i in range(350):
            f.writelines(str([0 for x in range(350)]))
            f.writelines('\n')
        
        # 로직 1.  publisher, subscriber, msg 생성
        # 로직 1-1. subscription = laserScan 정보 수신
        self.subscription = self.create_subscription(LaserScan,
        '/scan',self.scan_callback,10)
        # 로직 1-2. map_pub = 장애물을 인식해 반환
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        # 로직 1-3. 터틀봇의 상태 수신 및 송신
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback,10)
        self.pos=[0,0] #y,x
        self.set_interval(self.sendRobot, 0.7)
        
        # 로직 1-3. 장애물 정보를 맵에서 나타내기 위한 grid_cell 생성 350*350
        self.map_msg=OccupancyGrid()
        self.map_msg.header.frame_id="map"
        self.map_size=int(params_map["MAP_SIZE"][0]\
            /params_map["MAP_RESOLUTION"]*params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])
        

        # 로직 1-4. map 좌표공간을 설정
        m = MapMetaData()
        m.resolution = params_map["MAP_RESOLUTION"]# 0.05
        m.width = int(params_map["MAP_SIZE"][0]/params_map["MAP_RESOLUTION"])# 350
        m.height = int(params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])# 350
        quat = np.array([0, 0, 0, 1])

        # 로직 1-5. 생성한 map 좌표 공간을 배치할 원점을 설정
        m.origin = Pose()
        m.origin.position.x = params_map["MAP_CENTER"][0]-8.75
        m.origin.position.y = params_map["MAP_CENTER"][1]-8.75
        self.map_meta_data = m

        self.map_msg.info=self.map_meta_data

        # 로직 2 : mapping 클래스 생성
        self.mapping = Mapping(params_map)

    def set_interval(self, func, sec):
        def func_wrapper():
            self.set_interval(func, sec)
            func()
        t = threading.Timer(sec, func_wrapper)
        t.start()
        return t

    def sendRobot(self):
        print("robot")
        if(sio.connected):
            print(self.pos)
            sio.emit('Robot2Server', self.pos)
    
    def status_callback(self, msg):
        gridPose =[(msg.twist.angular.y - self.map_meta_data.origin.position.y) / 0.05,(msg.twist.angular.x - self.map_meta_data.origin.position.x) /0.05] 
        self.pos=gridPose
        # print(self.pos)

    def scan_callback(self,msg):
        # 로직 4. lader subscribtion 값으로 ground truth pose를 받는다
        # 속성 명과 관계없이 임의로 보낸 ground truth pose

        pose_x = msg.range_min
        pose_y = msg.scan_time
        heading = msg.time_increment #theta
        
        # 로직 5 : lidar scan 결과 수신
        Distance= Distance=np.array(msg.ranges) #Distance[theta] = distance from lidar 극좌표계 값

        #np.linspace(start,end,num) Return evenly spaced numbers over a specified interval.
        # np.linspace(0, 2 * np.pi, 360) = [0, ..., 0.01750191, 6.28138,,]

        # 라이다로 감지된 hit point의 극좌표계 값을 직교 좌표계 x, y값으로 변환
        x = Distance * np.cos(np.linspace(0, 2 * np.pi, 360))
        y = Distance * np.sin(np.linspace(0, 2 * np.pi, 360))
        laser = np.vstack((x.reshape((1, -1)), y.reshape((1, -1))))

        # 로직 6 : map 업데이트 실행(4,5번이 완성되면 바로 주석처리된 것을 해제하고 쓰시면 됩니다.)
        # pose 로봇의 전체 좌표계 극좌표값
        pose = np.array([[pose_x],[pose_y],[heading]])
        self.mapping.update(pose, laser)            
        np_map_data=self.mapping.map.reshape(1,self.map_size) 
        list_map_data=np_map_data.tolist()
        for i in range(self.map_size):
            list_map_data[0][i]=100-int(list_map_data[0][i]*100)
            if list_map_data[0][i] >100 :
                list_map_data[0][i]=100
 
            if list_map_data[0][i] <0 :
                list_map_data[0][i]=0

        """
        로직 11 : 업데이트 중인 map publish(#으로 주석처리된 것을 해제하고 쓰시고, 나머지 부분은 직접 완성시켜 실행하십시오)
        """
        self.map_msg.header.stamp =rclpy.clock.Clock().now().to_msg()
        self.map_msg.data=list_map_data[0]
        self.map_pub.publish(self.map_msg)
        


def save_map(node,file_path):

    # 로직 12 : 맵 저장
    pkg_path =os.getcwd()
    back_folder='..'
    folder_name='map'
    file_name=file_path
    # full_path=os.path.join(pkg_path,back_folder,folder_name,file_name)
    full_path = "C:\\dev\\ros2_smart_home\\src\\sub2\\map\\map.txt"
    print(full_path)
    f=open(full_path,'w')
    data=''
    for pixel in node.map_msg.data :
        data+='{0} '.format(pixel)
    f.write(data) 
    f.close()
    
def main(args=None):    
    rclpy.init(args=args)

    ip_server = 'http://localhost:3000/'

    @sio.event
    def connect():
        print('connection established',sio.sid)
        
    @sio.event
    def disconnect():
        print('disconnected from server')
        
    @sio.event
    def connect_error(data):
        print("connect_error")
    

    print("connect ", ip_server)
    sio.connect(ip_server)

    try :
        #로직 1. 서버 통신 소켓 설정
        #로직 1-1. 소켓 이벤트 정의
        run_mapping = Mapper()
        rclpy.spin(run_mapping)
        run_mapping.destroy_node()
        rclpy.shutdown()

    except :
        save_map(run_mapping,'map.txt')


if __name__ == '__main__':
    main()