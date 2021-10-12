import socketio
import rclpy
from rclpy.node import Node
import threading
import numpy as np
import cv2
import base64

from ssafy_msgs.msg import TurtlebotStatus, HandControl
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from hanvi_interfaces.msg import DetectionList, Detection

# client 는 socketio의 기본 API로 구성된 노드입니다. 서버와 연결을 시도해서 서버와 통신을 합니다.

# 각자의 서버 주소에 맞게 connect 함수 안을 바꿔주고, server 스켈레톤코드를 이용해 서비스를 하고 있다면, 연결이 됩니다.
# 버튼을 누르면 해당 키값에 맞는 함수들이 호출이 됩니다. 연결이 된 후에는 emit 함수를 이용해 서버로 키값과 데이터를 보냅니다.
# 이 노드는 AWS EC2에 구축한 서버와 통신만 하는 노드이고, ROS2와 연동하여 사용하면 스마트홈에서 얻은 데이터들을 서버로 보내고, 웹서버로부터의 명령을 ROS2로 전달할 수 있습니다.

people_minimum = 3

class Client(Node):

    def __init__(self):
        super().__init__('client')
        self.status_subscription = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.turtlebot_status_cb,10)
        self.img_subscription = self.create_subscription(CompressedImage,'/image_jpeg/compressed',self.img_callback,10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1
        )

        self.sd_detect_sub = self.create_subscription(
            HandControl,
            '/people_check',
            self.sd_callback,
            1
        )

        self.detect_sub = self.create_subscription(
            DetectionList,
            '/hanvi_detection',
            self.detect_callback,
            1
        )

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.event_list = {'tent': '닫힌 텐트 감지', 'fire': '화재 발생', 'bottle': '심야 음주 감지', 'kickboard': '도로변 킥보드 발견', 'bag': '분실물 발견', 'trash': '쓰레기 발견'}
        self.people_msg = HandControl()
        self.detect_msg = DetectionList()
        self.event_ed = {'tent': False, 'fire': False, 'bottle': False, 'kickboard': False, 'bag': False, 'trash': False, 'people': False}
        self.event_em = {'tent': 0, 'fire': 1, 'bottle': 0, 'kickboard': 0, 'bag': 0, 'trash': 0, 'people': 0}
        self.detected = None
        self.odom_msg = Odometry()

        self.turtlebot_status_msg = TurtlebotStatus()
        self.is_turtlebot_status = False
        self.img_bytes = ""

    def turtlebot_status_cb(self,msg):
        self.is_turtlebot_status=True
        self.turtlebot_status_msg=msg

    def img_callback(self,msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.img_bytes = cv2.imencode('.jpg', img_bgr)[1].tobytes()

    def sd_callback(self,msg):
        self.people_msg = msg
    
    def detect_callback(self, msg):
        self.detect_msg = msg
    
    def odom_callback(self, msg):
        self.odom_msg = msg

    def timer_callback(self):
        emergency = 0
        x = self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y
        pose = (x, y)

        # boolean 배열 같은 거 만들고 이전에 온 신호는 받지 말기
        # detect_msg에서 None이 오거나 people_msg에서 0으로 올 때마다 초기화
        # 위 방법으로 같은 종류의 서로 다른 상황도 감지 가능 기대

        if self.people_msg is not None:
            if self.people_msg.control_mode >= people_minimum and not self.event_ed['people']:
                sd = '사회적 거리두기 위반: {}명'.format(self.people_msg.control_mode)
                emergency = self.event_em['people']
                print('pose', pose)
                sio.emit('History2Server', {'content': sd, 'emergency': emergency, 'pose': pose})
                self.event_ed['people'] = True
            elif self.people_msg.control_mode == 0:
                self.event_ed['people'] = False

        if self.detect_msg is not None:
            content = ''
            for detection in self.detect_msg.detections:
                if detection.name in self.event_list:
                    content = self.event_list[detection.name]
                    emergency = self.event_em[detection.name]
                    self.detected = detection.name
                    break

            if content != '':
                if not self.event_ed[self.detected]:
                    self.event_ed[detection.name] = True
                    print('pose', pose)
                    sio.emit('History2Server', {'content': content, 'emergency': emergency, 'pose': pose})
            elif self.detected in self.event_list:
                self.event_ed[self.detected] = False
                # print(self.detected, self.event_ed[self.detected])

def main_logic(client_node):
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client_node = Client()
    client_thread = threading.Thread(target=main_logic, args=[client_node])
    client_thread.daemon = True
    client_thread.start()

    # 로직 1. 클라이언트 소켓 생성
    global sio
    sio = socketio.Client()

    # connect와 disconnect는 라이브러리에서 미리 만들어놓은 handler다.
    @sio.event
    def connect():
        print('connection established')

    @sio.event
    def disconnect():
        print('disconnected from server')

    # 로직 2. 데이터 수신 콜백함수
    @sio.on('request_stream_from_node')
    def listen_node():
        sio.emit('stream_from_python', client_node.img_bytes)

    # @sio.on('streaming')
    # def streaming_callback():
        # sio.emit('streaming_callback', img_resize)
        
    # 로직 3. 서버 연결
    # sio.connect('http://ec2-3-34-134-166.ap-northeast-2.compute.amazonaws.com:12001/')
    sio.connect('http://localhost:3000/')

    # 로직 4. 데이터 송신
    # 이렇게만 쓰면 client.py를 실행했을 때 딱 한 번만 간다.
    # sio.emit('from_python_to_node', client_node.img_bytes)

    sio.wait()

main()