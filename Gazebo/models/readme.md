🛠️ 실행 방법 (Simulation & Navigation)
본 프로젝트는 Gazebo 시뮬레이터에서 TurtleBot3를 활용하여 물류 박스를 인식하고 지정된 좌표를 자율 주행으로 순회
아래 명령어들을 순서대로 실행
여기서 empty_world는 빈 파일이 아닌 수정한 맵 파일

1️⃣ Gazebo 시뮬레이터 실행
ros2 launch turtlebot3_gazebo empty_world.launch.py use_sim_time:=true

2️⃣ Navigation2 시스템 실행
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true

3️⃣ RViz에서 2D Pose Estimate 버튼을 클릭하여 로봇의 초기 위치를 설정합니다.

4️⃣ 경로 계획 노드 실행
ros2 run path_planner path_node

📦 models.zip 정보
models.zip에는 Gazebo 시뮬레이터에 배치할 색상 박스 모델이 포함되어 있습니다.
압축 해제 후 사용
