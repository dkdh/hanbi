# Git Convention

[접미사] : \[파일]\[동사]

깃 푸쉬 전에 [issue num] 추가

| 접두사   | 내용                     |
| -------- | ------------------------ |
| feat     | 코드를 수정한 경우       |
| fix      | 버그 발생시 고쳣을 경우  |
| docs     | 문서 수정                |
| refactor | 코드 구조 변경           |
| _FE_     |                          |
| design   | 프론트 UI 설계 관한 내용 |
| BE       |                          |
| deploy   | 배포, CI/CD              |

Yolo v5 사전작업.

Yolo v5 GitHub (https://github.com/ultralytics/yolov5)

$ git clone https://github.com/ultralytics/yolov5
$ cd yolov5
$ pip install -r requirements.txt

ROS2 Custom massage type을 사용하기 위해
colcon build로 전체 빌드.
또는, colcon build --packages-select hanvi_interfaces 필요.
