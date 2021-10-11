import rclpy
from rclpy.node import Node


from hanvi_interfaces.msg import DetectionList
from geometry_msgs.msg import PoseStamped

import time
import pygame

class TTS(Node):
    
    def __init__(self):
        super().__init__('tts')

        # sub
        self.signal_sub = self.create_subscription(
            PoseStamped,
            '/tts_signal',
            self.signal_callback,
            1
        )
    
    def signal_callback(self,msg):
        event = msg.header.frame_id
        if event == 'map':
            event = 'people'
        fileName = '..\\sound\\' + event + '.mp3'

        pygame.mixer.init()
        pygame.mixer.music.load(fileName)
        pygame.mixer.music.play()
        time.sleep(8)

def main(args=None):
    rclpy.init(args=args)
    tts = TTS()
    rclpy.spin(tts)
    tts.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

