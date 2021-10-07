import socketio
import rclpy
from rclpy.node import Node
import threading
import numpy as np
import cv2
import base64

from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import CompressedImage

# client 는 socketio의 기본 API로 구성된 노드입니다. 서버와 연결을 시도해서 서버와 통신을 합니다.

# 각자의 서버 주소에 맞게 connect 함수 안을 바꿔주고, server 스켈레톤코드를 이용해 서비스를 하고 있다면, 연결이 됩니다.
# 버튼을 누르면 해당 키값에 맞는 함수들이 호출이 됩니다. 연결이 된 후에는 emit 함수를 이용해 서버로 키값과 데이터를 보냅니다.
# 이 노드는 AWS EC2에 구축한 서버와 통신만 하는 노드이고, ROS2와 연동하여 사용하면 스마트홈에서 얻은 데이터들을 서버로 보내고, 웹서버로부터의 명령을 ROS2로 전달할 수 있습니다.

class Client(Node):

    def __init__(self):
        super().__init__('client')
        self.status_subscription = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.turtlebot_status_cb,10)
        self.img_subscription = self.create_subscription(CompressedImage,'/image_jpeg/compressed',self.img_callback,10)

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
    sio.connect('http://j5a102.p.ssafy.io:3000/')

    # 로직 4. 데이터 송신
    # 이렇게만 쓰면 client.py를 실행했을 때 딱 한 번만 간다.
    # sio.emit('from_python_to_node', client_node.img_bytes)

    sio.wait()

main()