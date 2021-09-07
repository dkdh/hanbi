import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    
    def __init__(self):
        super().__init__('Publisher')
        self.test_pub = self.create_publisher(String,'/test',10)
        self.timer = self.create_timer(0.33, self.timer_callback)
        
        self.test_msg = String()

    # 로직 2. callback 함수 생성
    ## 메시지가 수신될 때 마다 호출되는 함수로 메시지의 내용을 출력하고 있습니다.
    def timer_callback(self):
        self.test_msg.data = "hi"
        self.test_pub.publish(self.test_msg)
        

def main(args=None):
    rclpy.init(args=args)
    minimal_Publisher = MinimalPublisher()
    rclpy.spin(minimal_Publisher)
    minimal_Publisher.destroy_node()
    rclpy.shutdown()