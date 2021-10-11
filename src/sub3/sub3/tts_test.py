# from gtts import gTTS
# import pygame

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

import time

class TTSTest(Node):
    
    def __init__(self):
        super().__init__('tts')

        self.tts_pub = self.create_publisher(PoseStamped,'/tts_signal',10)
        time_period=0.05 
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.tts_msg = PoseStamped()

    def timer_callback(self):
        self.tts_msg.header.frame_id = 'map'
        self.tts_pub.publish(self.tts_msg)

def main(args=None):
    rclpy.init(args=args)
    tts = TTSTest()
    rclpy.spin(tts)
    tts.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# tts = gTTS(text='한강 공원은 닫힌 텐트 설치를 금지합니다. 텐트를 열어주세요.', lang='ko')
# fileName = '..\\sound\\tent.mp3'
# fileName = 'C:\\Users\\multicampus\\Desktop\\catkin_ws\\src\\sub3\\sound\\test.mp3'

# tts.save(fileName)

# pygame.mixer.init()
# pygame.mixer.music.load(fileName)
# pygame.mixer.music.play()

# fileName = 'C:\\Users\\multicampus\\Desktop\\catkin_ws\\src\\sub3\\sound\\test_wav.wav'
# mySound = pygame.mixer.Sound(fileName)
# mySound.play()

# print('stop...')
# time.sleep(8)
# print('end')