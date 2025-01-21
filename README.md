# 재난 구조 로봇 시스템
![Image](https://github.com/user-attachments/assets/6a0be2f9-cec2-4627-86ec-2d7aa08b4c91)

## Overview
- **프로젝트 설명**: 재난 상황 발생시 로봇을 활용하여 재난 현장을 자율적으로 매핑하고 소화 물품과 조난자를 탐지하는 프로젝트 입니다.
- **프로젝트 기간**: 2024.12.24 ~ 2024.12.31
- **팀 구성**: 3명

## Features
| 기능    | 설명 |
|----------|---------------|
| 로봇 자동 매핑 기능 | 로봇을 이용하여 LiDAR 기반 SLAM을 자동으로 수행 |, 
| 실시간 매핑 확인 기능 | SLAM 수행 중 맵 데이터를 실시간으로 시각화 |
| 조난자, 소화기 위치 표시 | 조난자 및 소화기 발견시 맵에 위치 표시 | 

### 로봇 자동 매핑 기능
-	목적: 로봇이 LiDAR SLAM 기술을 사용하여 환경을 자동으로 매핑합니다.  
-	상세 설명:  
    1. 로봇은 LiDAR 센서를 사용하여 자동으로 환경 데이터를 수집합니다.  
    2. 수집된 데이터는 SLAM 알고리즘을 통해 처리되어 2D 맵을 생성합니다.  

### 실시간 매핑 확인 기능
-	목적: SLAM 작업 중 맵 데이터를 실시간으로 확인하며, 로봇의 상태를 모니터링
-	상세 설명:  
    1. SLAM 수행 중 로봇이 수집한 LiDAR 센서 데이터가 맵 형태로 표시됩니다.
    2. 실시간 시각화를 통해 로봇의 현재 위치 및 이동 경로를 확인할 수 있습니다.
    3. 실시간으로 현장이 얼마나 맵핑 됐는지 확인할 수 있습니다.


### 조난자, 소화기 위치 표시
-	목적: 로봇이 탐색 중 조난자와 소화기를 발견하면 해당 위치를 맵에 표시합니다.
-	상세 설명:  
    1. 로봇은 카메라와 사용하여 조난자와 소화기를 감지합니다.
    2. 감지된 객체의 위치를 SLAM 맵에 표시하며, 이를 사용자 GUI에 시각적으로 제공합니다.


## Tech / Skill
-	Ros2 humble  
-	Python  
-	Git     
-	OpenCV  
-	Turtlebot4


## Quick Start

### LiDAR SLAM Auto Mapping
~~~bash
ros2 run slam_pkg explore
~~~

### Show Realtime Map
~~~bash
ros2 run slam_pkg map
~~~

### Get Person And Fire Extinguisher Coordination
~~~bash
ros2 run b4_serv_robot pnp_cam_img
~~~

## References
[Turtlebot4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html)  
[Nav2 docs](https://docs.nav2.org/)  
[OpenCV docs](https://docs.opencv.org/4.x/index.html)
