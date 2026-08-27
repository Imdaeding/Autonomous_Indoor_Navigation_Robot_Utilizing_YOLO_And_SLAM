# Autonomous_Indoor_Navigation_Robot_Utilizing_YOLO_And_SLAM

> **LiDAR SLAM 및 YOLOv11s 기반의 실내 자율주행 순찰 로봇 플랫폼**

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-NVIDIA_Jetson_Orin-green.svg)]()
[![Framework](https://img.shields.io/badge/Framework-ROS_2_Humble-orange.svg)]()

---

## 1. 📖 Project Overview (개요)
* **개발 기간:** 2026.07 ~ 2025.08 (3인 팀 프로젝트)
* **담당 역할:** (비전 파이프라인 최적화)
* **목표 & 배경:** (해결하고자 했던 문제나 구현하고자 했던 핵심 시스템 목적 서술)

---

## 2. 🏗️ System Architecture (시스템 구조)
<!-- 시스템 블록 다이어그램, ROS 2 노드 통신 다이어그램, 또는 회로 블록도 삽입 -->

<img width="5440" height="4355" alt="다이어그램" src="https://github.com/user-attachments/assets/d06dea72-d2be-44d2-bb20-ea7864ce5c95" />


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

### 🔹 1) [핵심 기능 1: 예 - SLAM 맵핑 및 Navigation2 통합]

* Cartographer를 활용한 실내 2D 점유 격자 지도(Occupancy Grid Map) 생성
* Costmap 파라미터 튜닝을 통한 동적 장애물 회피 경로 계획 최적화

### 🔹 2) [핵심 기능 2: 예 - 엣지 비전 파이프라인]

* Dual IMX219 카메라 입력
* 1280*720의 DATASET을 직접 수집 및 YOLOv11s 모델로 학습
* 학습된 PT파일을 Jetson 환경에 배포하여 실시간 30 FPS 객체 탐지 달성

---

## 4. 🛠️ Troubleshooting & Optimization (트러블슈팅 및 문제 해결)


* **문제 현상:** YOLO 학습 데이터로 ROBOFLOW의  DATASET을 이용했지만, 화각 및 해상도가 설계 환경과 일치하지 않았음
* **해결 방안:** 1280*720의 DATASET을 직접 수집 후 LabelImg를 통해 라벨링과 Bounding Box처리함
                8:2의 test와 valid 데이터로 무작위 분할 후 그래픽카드 가속기를 사용해 YOLOv11s 모델로 학습 진행

---

## 5. 🎬 Demo & Results (구동 영상 / 결과물)

<img width="640" height="480" alt="poster_detection_sample" src="https://github.com/user-attachments/assets/b1ad9d2b-29d3-4b8c-8a33-b745fcd9af1a" />

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
