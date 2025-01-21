# 재난 구조 로봇 시스템
![Image](https://github.com/user-attachments/assets/6a0be2f9-cec2-4627-86ec-2d7aa08b4c91)

## Overview
- **프로젝트 설명**: 재난 상황 발생시 로봇을 활용하여 재난 현장을 자율적으로 매핑하고 소화 물품과 조난자를 탐지하는 프로젝트 입니다.
- **프로젝트 기간**: 2024.12.24 ~ 2024.12.31
- **팀 구성**: 3명

## Feature 
| 기능    | 설명 |
|----------|---------------|
| 로봇 자동 매핑 기능 | 로봇을 이용하여 LiDAR 기반 SLAM을 자동으로 수행 |, 
| 실시간 매핑 확인 기능 | SLAM 수행 중 맵 데이터를 실시간으로 시각화 |
| 조난자, 소화기 위치 표시 | 조난자 및 소화기 발견시 맵에 위치 표시 | 

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
