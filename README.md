# Autonomous_Indoor_Navigation_Robot_Utilizing_YOLO_And_SLAM

> **LiDAR SLAM 및 YOLOv11s 기반의 실내 자율주행 순찰 로봇 플랫폼**

[![Platform](https://img.shields.io/badge/Platform-NVIDIA_Jetson_Orin_Nano-green.svg)]()
[![Platform](https://img.shields.io/badge/Platform-Waveshare_UGV02-green.svg)]()
[![Framework](https://img.shields.io/badge/Framework-ROS_2_Humble-orange.svg)]()

---

## 1. 📖 Project Overview (개요)
* **개발 기간:** 2026.07 ~ 2025.08 (3인 팀 프로젝트)
* **담당 역할:** 비전 파이프라인 최적화
* **목표 & 배경:** 산업 발전 속 노동자 안전 대응은 미흡하고, CCTV 등 수동적 감시는 사각지대가 많으며 공장 내부는 GPS 음영 구역이므로 스스로 위치를 추정하며 주행하는 SLAM 기술이 요구된다. 이에 본 과제는 카메라와 LiDAR 융합 및 SLAM 기반 위치 추정 기술을 적용하여, GPS 음영 환경에서도 능동적으로 사각지대를 탐색하고 위험 요소를 인지·경고하는 실내 자율주행 안전 순찰 로봇을 개발하고자 한다.

---

## 2. 🏗️ System Architecture (시스템 구조)
<!-- 시스템 블록 다이어그램 -->

<img width="1280" height="720" alt="다이어그램" src="https://github.com/user-attachments/assets/d06dea72-d2be-44d2-bb20-ea7864ce5c95" />


### 🛠 Tech Stack & Hardware Specs

| 구분 | 항목 | 사양 / 버전 |
| --- | --- | --- |
| **Hardware** | Main Controller | NVIDIA Jetson Orin Nano |
|  | Sensors & Chassis | RPLiDAR A1, IMX219 Camera, UGV02 Chassis |
| **Software** | OS / Middleware | Ubuntu 22.04 LTS, ROS 2 Humble |
|  | Perception / AI | OpenCV, PyTorch, YOLOv11s, Cartographer |
|  | Tools | Vs Code, Git, Codex, Gemini 3.6 Flash |

---

## 3. ⚙️ Key Features & Implementation (핵심 구현 내용)

### 🔹 1) [핵심 기능 1 : SLAM 맵핑 및 Navigation2 통합]

* Cartographer를 활용한 실내 2D 점유 격자 지도(Occupancy Grid Map) 생성
* Costmap 파라미터 튜닝을 통한 동적 장애물 회피 경로 계획 최적화

<img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/29997910-ae38-48d7-8af3-ae66219a73b5" />


### 🔹 2) [핵심 기능 2 : 엣지 비전 파이프라인]

* Dual IMX219 카메라 입력
* 1280*720의 DATASET을 직접 수집 및 YOLOv11s 모델로 학습
* 학습된 PT파일을 Jetson 환경에 배포하여 실시간 30 FPS 객체 탐지 달성

---

## 4. 🛠️ Troubleshooting & Optimization (트러블슈팅 및 문제 해결)


* **문제 현상:** YOLO 학습 데이터로 ROBOFLOW의  DATASET을 이용했지만, 화각 및 해상도가 설계 환경과 일치하지 않았음
* **해결 방안:** 1280*720의 DATASET을 직접 수집 후 LabelImg를 통해 라벨링과 Bounding Box처리함.
 처리된 데이터를 8:2의 test와 valid 데이터로 무작위 분할 후 그래픽카드 가속기를 사용해 YOLOv11s 모델로 학습 진행

<img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/8a9f813b-3d7d-4f0f-9cb3-3810a967d045" />

---

## 5. 🎬 Demo & Results (구동 영상 / 결과물)
# 객체 탐지 시뮬레이션

<img width="640" height="480" alt="poster_detection_sample" src="https://github.com/user-attachments/assets/b1ad9d2b-29d3-4b8c-8a33-b745fcd9af1a" />

# 구동 시연 영상

[https://youtube.com/shorts/9DksMWg7Gh0?feature=share]

---

## 6. 🚀 Getting Started (실행 방법)

# 1. 저장소 클론
git clone [https://github.com/사용자명/저장소명.git](https://github.com/사용자명/저장소명.git)
cd 저장소명

# 2. 의존성 설치 및 빌드 (ROS 2 예시)
colcon build --symlink-install
source install/setup.bash

# 3. 노드 실행
ros2 launch my_robot_bringup bringup.launch.py

---
