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
        
        # 1-1. subscriber
        self.subscription = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.imu_sub = self.create_subscription(Imu,'/imu',self.imu_callback,10)

        # 1-2. publisher
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # 1-3. broadcaster
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)


        # 2. 변수 
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
        self.robot_x=0.0
        self.robot_y=0.0
                
        self.theta=0.0
        # imu_offset은 초기 로봇의 orientation을 저장할 변수 입니다.
        self.imu_offset=0
        self.prev_time=0
        self.period = 0.0
        self.sum_time = 0.0

        '''
        로직 2. publish, broadcast 할 메시지 설정
        '''
        #글로벌 좌표계를 map으로 명명
        self.odom_msg.header.frame_id='map'
        #map 좌표계에서 사용할 로컬 좌표계를 base_link로 명명
        self.odom_msg.child_frame_id='base_link'

        #로컬 좌표계인 base_link. 로봇의 위치 좌표
        self.base_link_transform.header.frame_id = 'map'
        self.base_link_transform.child_frame_id = 'base_link'

        #로컬 좌표계인 laser_transform. 로봇에 달린 라이다 센서의 위치 좌표
        self.laser_transform.header.frame_id = 'map'
        self.laser_transform.child_frame_id =  'base_link'
        self.laser_transform.transform.translation.x = 0.0
        self.laser_transform.transform.translation.y = 0.0
        self.laser_transform.transform.translation.z = 0.19
        self.laser_transform.transform.rotation.w = 0.0

                      

    def imu_callback(self,msg):
        '''
        로직 3. IMU 에서 받은 quaternion을 euler angle로 변환해서 사용
        '''
        
        if self.is_imu ==False :    
            self.is_imu=True

            #처음 로봇이 바라보는 방향을 저장
            imu_q=msg.orientation

            #방향을 코타니언으로 변환. 이후 뺄것 임
            self.imu_offset=Quaternion.to_euler(imu_q)
            print(self.imu_offset)

            
            #타임 텀을 측정하기 위한 변수
            self.prev_time= rclpy.clock.Clock().now()

            #타임 텀별 평균 속도를 구하기 위한 변수
            self.prev_imu_velocity_x = 0
            self.prev_imu_velocity_y = 0


        else :

            # IMU의 각속도로 self.theta 갱신할 경우 사용

            # 관성 측정장치로 얻은 로봇의 회전각속도

            # 1. 시간 갱신            
            self.current_time= rclpy.clock.Clock().now()
            # 1-1. 측정에 걸린 시간 간격 계산
            self.period = (self.current_time-self.prev_time).nanoseconds/100000000
            # 1-2. 현재 시간을 이전 시간으로 갱신
            
            # 1-3 지난 시간 합계
            self.sum_time+=self.period
            

            # 2. imu로부터 얻은 각속도로 글로벌 좌표계의 theta 계산
            # 2-1 관성 측정장치로 얻은 로봇의 z축 각속도(yaw)
            imu_angular_z = msg.angular_velocity.z
            
            #2-2 z축의 각속도(rad/sec) * 시간(sec) = 이동거리
            #    http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html
            self.theta += (imu_angular_z * self.period);

            #!2-3 각도 360도를 넘었을때 처리를 해줘야할까?
            #    https://www.google.com/search?q=360%EB%8F%84+%EB%9D%BC%EB%94%94%EC%95%88&sxsrf=AOaemvLHvEFx0Qq0GMbTm62Q3ezyEE8NLQ%3A1630564870742&ei=BnIwYZvlLIXx-QaDvpP4Cg&oq=360%EB%8F%84+%EB%9D%BC%EB%94%94%EC%95%88&gs_lcp=Cgdnd3Mtd2l6EAMyCggAEIAEEIcCEBQyBQgAEIAEMgYIABAIEB4yBggAEAgQHjIGCAAQCBAeMgYIABAIEB46BAgjECc6CAgAEIAEELEDOgQIABBDOgsIABCABBCxAxCDAUoECEEYAFCLGlj1KWCvKmgAcAB4AIAB-wKIAb8QkgEHMC43LjMuMZgBAKABAcABAQ&sclient=gws-wiz&ved=0ahUKEwibgJ2_19_yAhWFeN4KHQPfBK8Q4dUDCA4&uact=5
            
            print("imu_angluar = %7.5f, self.period = %7.5f, theta =  %7.5f, sum_time = %7.5f"%(imu_angular_z, self.period, self.theta, self.sum_time))


            #3 x와 y값
            #3-1 x축 y축 가속도
            # imu_accel_x = msg.linear_acceleration.x
            # imu_accel_y = msg.linear_acceleration.y

            #3-2 x축 y축 현재 속도
            # imu_velocity_x = imu_accel_x * self.period
            # imu_velocity_y = imu_accel_y * self.period

            #3-3 현재 속도와 이전 속도의 중간값으로 현재 x, y 좌표 계산
            # self.x+=(imu_velocity_x + self.prev_imu_velocity_x)/ 2
            # self.y+=(imu_velocity_y + self.prev_imu_velocity_y)/ 2

            #3-4 이전 속도 갱신
            # self.prev_imu_velocity_x = imu_velocity_x
            # self.prev_imu_velocity_y = imu_velocity_y
            #self.prev_time= self.current_time


            #ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

            #IMU의 코타니언을 이용


    def listener_callback(self, msg):

        # print('linear_vel : (%5.3f,%5.3f)  global : %5.3f, %5.3f, ang_v : %5.3f / (robot) (%5.3f, %5.3f / %5.3f (period) %8.7f) '%
        #     (msg.twist.linear.x,
        #     msg.twist.linear.z,
        #     msg.twist.angular.x,
        #     msg.twist.angular.y,
        #     msg.twist.angular.z,
        #     #local
        #     self.robot_x,
        #     self.robot_y,
        #     self.theta,
        #     self.period
        #     ))

        if self.is_imu ==True:
            if self.is_status == False :
                self.is_status=True
                self.theta = self.imu_offset[2]
                self.prev_time=rclpy.clock.Clock().now()
            else :
                
                self.current_time=rclpy.clock.Clock().now()
                self.period=(self.current_time-self.prev_time).nanoseconds/1000000000
                # 로봇의 선속도, 각속도를 저장하는 변수, 시뮬레이터에서 주는 각 속도는 방향이 반대이므로 (-)를 붙여줍니다.
                linear_x=msg.twist.linear.x
                angular_z=-msg.twist.angular.z

                '''
                로직 4. 로봇 위치 추정
                (테스트) linear_x = 1, self.theta = 1.5707(rad), self.period = 1 일 때
                self.x=0, self.y=1 이 나와야 합니다. 로봇의 헤딩이 90도 돌아가 있는
                상태에서 선속도를 가진다는 것은 x축방향이 아니라 y축방향으로 이동한다는 뜻입니다. 

                '''
                self.robot_x+=linear_x*cos(self.theta)*self.period
                self.robot_y+=linear_x*sin(self.theta)*self.period
                #print(angular_z, self.period, angular_z * self.period)
                # self.theta+= angular_z*self.period


                self.base_link_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
                self.laser_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
                
                '''
                로직 5. 추정한 로봇 위치를 메시지에 담아 publish, broadcast

                '''
                #theta가 오일러 각이므로 쿼터니언으로 변환
                q = Quaternion.from_euler(0, 0, self.theta)
                
                #좌표계를 broadcast할 때는 시간을 넣어줘야 하기 때문에 넣어줌.
                self.base_link_transform.header.stamp = rclpy.clock.Clock().now().to_msg()
                self.laser_transform.header.stamp = rclpy.clock.Clock().now().to_msg()
                #계산한 x,y가 이동값이 됨
                self.base_link_transform.transform.translation.x = self.robot_x
                self.base_link_transform.transform.translation.y = self.robot_y

                #계산한 q가 회전값이 됨.
                self.base_link_transform.transform.rotation.x = q.x
                self.base_link_transform.transform.rotation.y = q.y
                self.base_link_transform.transform.rotation.z = q.z
                
                #odometry 메세지에 이동, 회전, 제어 값을 채움
                self.odom_msg.pose.pose.position.x=self.robot_x
                self.odom_msg.pose.pose.position.y=self.robot_y
                self.odom_msg.pose.pose.orientation.x=q.x
                self.odom_msg.pose.pose.orientation.y=q.y
                self.odom_msg.pose.pose.orientation.z=q.z
                self.odom_msg.pose.pose.orientation.w=q.w
                self.odom_msg.twist.twist.linear.x = linear_x
                self.odom_msg.twist.twist.angular.x=angular_z


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