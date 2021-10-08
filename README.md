# 특화 프로젝트

- 팀장: 박성철, 임아연
- 팀원: 김지환, 류한길, 신기호, 오승철
- 기간: 2021.08.30 - 2021.10.08 (6주)
- 역할
  - 김지환: mapping, automative driving, ros-nodejs 소켓 연동, 웹-서버 소켓연동, vue.js FE 개발
  - 류한길: 인지, 다수 사람 체크, 인지 좌표 추정 및 시뮬레이션 좌표 연동
  - 박성철: 데이터 수집 및 전처리, Yolo 모델 학습 및 인지 결과 송출, 발표자.
  - 신기호: 시뮬레이션 맵 제작, UCC 및 발표자료 제작.
  - 오승철: 판단제어 로직 작성, 웹소켓 통신, 데이터베이스, CI/CD
  - 임아연: TF Object Detection API를 이용한 객체 인식.

---

## 프로젝트 기획배경

코로나 시국을 맞이하여 사람들이 한강 공원에 밀집해 있는 상황과, 각종 위험 요소 혹은 단속 요소를 인지하여 이를 관리자에게 알리면 업무 능률을 높이고 한강 공원 방문자 분들에게 보다 안전함을 제공드릴 것이라 생각하여 기획하게 되었습니다.

## 프로젝트 설계

웹

![KakaoTalk_20211007_110023807.png](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/1f98ba5d-d91b-4c3b-a496-91342b17b0f4/KakaoTalk_20211007_110023807.png)

## 기술 스택

- 판단/제어: ROS2
- 인지: PyTorch, Yolo
- 커스텀 맵: Unity
- 웹: Node.js

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/7e62ffa6-d005-4da3-9c1e-6815a5aa0a9e/Untitled.png)
