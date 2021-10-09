import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu

from squaternion import Quaternion
from nav_msgs.msg import Odometry
from math import pi,cos,sin

import tf2_ros
import geometry_msgs.msg
import time

# odom 노드는 로봇의 속도(/turtlebot_status), Imu센서(/imu) 메시지를 받아서 로봇의 위치를 추정하는 노드입니다.
# sub1_odom은 imu로 부터 받은 Quaternion을 사용하거나 각속도, 가속도 데이터를 이용해서 로봇의 포즈를 추정 할 것입니다.

# 노드 로직 순서
# 1. publisher, subscriber, broadcaster 만들기
# 2. publish, broadcast 할 메시지 설정
# 3. imu 에서 받은 quaternion을 euler angle로 변환해서 사용
# 4. 로봇 위치 추정
# 5. 추정한 로봇 위치를 메시지에 담아 publish, broadcast


class odom(Node):

    def __init__(self):
        super().__init__('odom')
        
        # 로직 1. publisher, subscriber, broadcaster 만들기
        self.subscription = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.imu_sub = self.create_subscription(Imu,'/imu',self.imu_callback,10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        # 클래스를 통해서 인스턴스를 만드는 것 같다.
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)


        # 로봇의 pose를 저장해 publish 할 메시지 변수 입니다.
        self.odom_msg=Odometry()
        # Map -> base_link 좌표계에 대한 정보를 가지고 있는 변수 입니다.
        self.base_link_transform=geometry_msgs.msg.TransformStamped()
        # base_link -> laser 좌표계에 대한 정보를 가지고 있는 변수 입니다.
        self.laser_transform=geometry_msgs.msg.TransformStamped()
        self.is_status=False
        self.is_imu=False
        self.is_calc_theta=False
        # x,y,theta는 추정한 로봇의 위치를 저장할 변수 입니다.
        # self.x=0.0
        self.x=-9.398
        # self.y=0.0
        self.y=-7.701
        self.theta=0.0

        # imu_offset은 초기 로봇의 orientation을 저장할 변수 입니다.
        self.imu_offset=0
        self.prev_time=0

        # 로직 2. publish, broadcast 할 메시지 설정

        self.odom_msg.header.frame_id = 'map'
        self.odom_msg.child_frame_id = 'base_link'

        self.base_link_transform.header.frame_id = 'map'
        self.base_link_transform.child_frame_id = 'base_link'

        self.laser_transform.header.frame_id = 'base_link'
        self.laser_transform.child_frame_id = 'laser'
        # 명세서
        # translation은 base_link에서 얼마나 떨어져 있는지 의미한다.
        # base_link와 laser간의 translation, rotation은 변하지 않기 때문에
        # 초기에 설정해준다. 
        self.laser_transform.transform.translation.x = 0.0
        self.laser_transform.transform.translation.y = 0.0
        self.laser_transform.transform.translation.z = 1.0
        self.laser_transform.transform.rotation.w = 1.0

    # IMU를 통해서 로봇의 헤딩을 구하는 방법
    def imu_callback(self,msg):
        # 로직 3. IMU 에서 받은 quaternion을 euler angle로 변환해서 사용
        if self.is_imu == False:
            self.is_imu = True
            # 컨설턴트님이 알려주신 방법 참조
            # https://pypi.org/project/squaternion/
            imu_q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
            # degree가 아닌 radian으로
            e = imu_q.to_euler()

            # 연구원님 강의 중 imu_offset 관련 내용
            # imu는 절대적인 헤딩을 계산해준다.
            # 그 헤딩을 0으로 맞추기 위해서 offset을 빼줄 것이다.
            
            # roll pitch yaw 중 yaw에 해당하는 값을 구한다.
            self.imu_offset = e[2]

        else:
            imu_q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
            e = imu_q.to_euler()
            ### 첫 번째 방식 ###
            # self.theta = e[2] - self.imu_offset
            # 왜 imu_offset을 빼주지 않아도 되는 걸까?
            # 생각해보면 맵1이 아예 시작 방향과 딱 떨어지게 나와서 imu_offset을 빼줄 필요가 없는 것 같다.
            self.theta = e[2]

    # 각속도를 누적해서 로봇의 헤딩을 구하는 방법
    def listener_callback(self, msg):
        # print('linear_vel : {}  angular_vel : {}'.format(msg.twist.linear.x,-msg.twist.angular.z))
        if self.is_imu ==True:
            # 처음 터틀봇 정보가 오면 시간만 기록한다.
            if self.is_status == False :
                self.is_status=True
                self.prev_time=rclpy.clock.Clock().now()

            else :
                self.current_time=rclpy.clock.Clock().now()
                # 계산 주기를 저장한 변수 입니다. 단위는 초(s)
                self.period=(self.current_time-self.prev_time).nanoseconds/1000000000
                # 로봇의 선속도, 각속도를 저장하는 변수, 시뮬레이터에서 주는 각 속도는 방향이 반대이므로 (-)를 붙여줍니다.
                linear_x=msg.twist.linear.x
                angular_z=-msg.twist.angular.z

                # 로직 4. 로봇 위치 추정
                # (테스트) linear_x = 1, self.theta = 1.5707(rad), self.period = 1 일 때
                # self.x=0, self.y=1 이 나와야 합니다. 로봇의 헤딩이 90도 돌아가 있는
                # 상태에서 선속도를 가진다는 것은 x축방향이 아니라 y축방향으로 이동한다는 뜻입니다. 

                # 사전학습에 나와있는 코드를 참조함
                self.x += linear_x * cos(self.theta)*self.period
                self.y += linear_x * sin(self.theta)*self.period
                # 각속도를 누적하면 헤딩이 된다.
                
                ### 두 번째 방식 ###
                self.theta += angular_z*self.period

                # 좌표계를 broadcast할 때는 시간을 꼭 넣어줘야 한다.
                self.base_link_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
                self.laser_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
                
                # 로직 5. 추정한 로봇 위치를 메시지에 담아 publish, broadcast

                # 헤딩을 오일러각에서 쿼터니언으로 변환
                # from_eluer(roll, pitch, yaw)
                # default is radians
                q = Quaternion.from_euler(0,0,self.theta)
                
                # map을 기준으로 했을 때 base link의 좌표 x,y를 업데이트
                self.base_link_transform.transform.translation.x = self.x
                self.base_link_transform.transform.translation.y = self.y
                # base_link의 theta를 업데이트한다.
                self.base_link_transform.transform.rotation.x = q.x
                self.base_link_transform.transform.rotation.y = q.y
                self.base_link_transform.transform.rotation.z = q.z
                self.base_link_transform.transform.rotation.w = q.w
                

                self.odom_msg.pose.pose.position.x = self.x
                self.odom_msg.pose.pose.position.y = self.y
                self.odom_msg.pose.pose.orientation.x = q.x
                self.odom_msg.pose.pose.orientation.y = q.y
                self.odom_msg.pose.pose.orientation.z = q.z
                self.odom_msg.pose.pose.orientation.w = q.w
                self.odom_msg.twist.twist.linear.x= linear_x
                self.odom_msg.twist.twist.angular.z= angular_z

                self.broadcaster.sendTransform(self.base_link_transform)
                self.broadcaster.sendTransform(self.laser_transform)
                self.odom_publisher.publish(self.odom_msg)
                self.prev_time=self.current_time

        
def main(args=None):
    rclpy.init(args=args)
    sub1_odom = odom()
    rclpy.spin(sub1_odom)
    sub1_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()