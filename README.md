# 🤖 Autonomous Indoor Navigation Robot Utilizing YOLO and SLAM

> **LiDAR SLAM 및 YOLOv11s 기반의 실내 자율주행 안전 순찰 로봇 플랫폼**  
> *Indoor Autonomous Patrol Robot Integrating LiDAR SLAM and Edge AI Vision Pipeline*

[![Platform](https://img.shields.io/badge/Platform-NVIDIA_Jetson_Orin_Nano-76B900.svg?logo=nvidia&logoColor=white)]()
[![Chassis](https://img.shields.io/badge/Chassis-Waveshare_UGV02-blue.svg)]()
[![Framework](https://img.shields.io/badge/Framework-ROS_2_Humble-22314E.svg?logo=ros&logoColor=white)]()
[![Model](https://img.shields.io/badge/AI_Model-YOLOv11s-00FFFF.svg)]()

---

## 1. 📖 Project Overview (개요)

산업 현장 내 작업자 안전사고 예방과 고정형 CCTV의 감시 사각지대 문제를 해결하기 위한 **실내 자율주행 안전 순찰 로봇**입니다.  
GPS 신호가 닿지 않는 실내 공장 환경에서 LiDAR SLAM을 통해 실시간으로 자신의 위치를 파악(Localization)하고, 온디바이스(On-device) 비전 AI 파이프라인을 구동하여 낙하물·케이블 등 작업장 내 위험 요소를 실시간으로 인지·회피합니다.

* **개발 기간:** 2026.07 ~ 2026.08 (3인 팀 프로젝트)
* **담당 역할:** 엣지 비전 파이프라인 구축 및 최적화, 객체 탐지 데이터셋 엔지니어링, ROS 2 통신 연동

---

## 2. 🏗️ System Architecture (시스템 구조)

<div align="center">
  <img width="100%" alt="시스템 아키텍처 다이어그램" src="https://github.com/user-attachments/assets/d06dea72-d2be-44d2-bb20-ea7864ce5c95" />
</div>

### 🛠 Tech Stack & Hardware Specs

| 분류 | 구분 | 세부 사양 및 기술 |
| :--- | :--- | :--- |
| **Hardware** | **Main Controller** | NVIDIA Jetson Orin Nano Developer Kit |
| | **Sensor & Chassis** | RPLiDAR A1 (2D LiDAR), Dual IMX219 Camera, Waveshare UGV02 Rover Chassis |
| **Software** | **OS / Middleware** | Ubuntu 22.04 LTS, ROS 2 Humble (CycloneDDS) |
| | **Perception / AI** | Ultralytics YOLOv11s, PyTorch (CUDA Accelerated), OpenCV |
| | **SLAM / Navigation** | Google Cartographer, Navigation2 (Nav2), Pure Pursuit Controller |
| | **Dev Tools** | VS Code, Git/GitHub |

---

## 3. ⚙️ Key Features & Implementation (핵심 구현 내용)

### 🔹 1) SLAM 기반 2-Pass 자율주행 파이프라인
* **Stage 1 (맵핑 및 경로 기록):** 벽면 추종(Wall Following) 알고리즘으로 미지의 실내 공간을 자율 탐색하며 Google Cartographer 기반 2D 점유 격자 지도(`Occupancy Grid Map`, `.pbstream`) 생성 및 주행 경로(`.csv`) 실시간 기록
* **Stage 2 (위치 추정 및 경로 추종 순찰):** 시작점 복귀 감지 후 순수 위치 추정(Pure Localization) 모드로 자동 전환, 1차 주행 경로를 기반으로 Pure Pursuit 제어기를 통해 정밀 순찰 주행 수행

<div align="center">
  <img width="90%" alt="SLAM 맵핑 및 Navigation 화면" src="https://github.com/user-attachments/assets/29997910-ae38-48d7-8af3-ae66219a73b5" />
</div>

### 🔹 2) 실시간 엣지 비전(Vision) 파이프라인
* **Dual IMX219 카메라 통합:** 시야각 확장을 위한 듀얼 카메라 파이프라인 구축
* **Jetson 온디바이스 추론 가속:** 경량화된 YOLOv11s 모델을 배포하여 실시간 **30 FPS** 고속 객체 탐지 달성
* **장애물 감지 및 비상 정지 연동:** 바닥 케이블, 전도 위험 박스 등 위험 객체 감지 시 ROS 2 Twist 메시지를 제어하여 즉각 비상 정지 및 회피 트리거 동작

---

## 4. 🛠️ Troubleshooting & Optimization (트러블슈팅)

### 📌 실내 환경 특화 커스텀 데이터셋 구축을 통한 탐지 성능 개선
* **문제 상황:**  
  공개 데이터셋(Roboflow) 기반 사전 학습 가중치 사용 시, 로봇의 낮은 화각(Low-angle POV) 및 실제 공장 바닥 조명 조건과의 불일치로 인해 오탐지(False Positive) 및 미탐지 빈번 발생
* **원인 분석:**  
  일반적인 사람 시선(Eye-level) 데이터와 로봇 섀시 장착 카메라(Ground-level) 간의 도메인 갭(Domain Gap) 및 왜곡 현상
* **해결 방안:**  
  1. 실제 주행 환경에서 **1280×720 해상도의 커스텀 이미지 데이터셋을 직접 수집**
  2. `LabelImg`를 활용하여 실내 장애물(박스, 케이블, 구조물 등)에 대한 정밀 바운딩 박스 라벨링 수행
  3. 학습/검증 데이터를 **8:2 비율**로 무작위 분할(Train/Valid Split) 후 GPU 가속을 적용하여 YOLOv11s 재학습 수행
* **결과:**  
  실제 주행 환경에서의 위험 객체 탐지 mAP 대폭 향상 및 오탐지 억제 성공

<div align="center">
  <img width="90%" alt="데이터셋 수집 및 학습 과정" src="https://github.com/user-attachments/assets/8a9f813b-3d7d-4f0f-9cb3-3810a967d045" />
</div>

---

## 5. 🎬 Demo & Results (결과 및 시연)

### 🔹 객체 탐지 시뮬레이션
<div align="center">
  <img width="600" alt="객체 탐지 결과 샘플" src="https://github.com/user-attachments/assets/b1ad9d2b-29d3-4b8c-8a33-b745fcd9af1a" />
</div>

### 🔹 구동 시연 영상
[![자율주행 순찰 시연 영상](https://img.youtube.com/vi/9DksMWg7Gh0/0.jpg)](https://youtube.com/shorts/9DksMWg7Gh0)  
*(클릭 시 유튜브 시연 영상으로 이동합니다)*

---

## 6. 🚀 Getting Started (실행 방법)

### 1. Prerequisites (사전 환경 요구사항)
* **OS:** Ubuntu 22.04 LTS on Jetson Orin Nano
* **ROS 2:** Humble Hawksbill with CycloneDDS (`rmw_cyclonedds_cpp`)
* **SLAM:** `cartographer_ros`
* **Deep Learning:** Python 3.10+, PyTorch (CUDA 지원), `ultralytics`

### 2. Installation (설치)
```bash
# 1) 저장소 클론
git clone [https://github.com/Imdaeding/Autonomous_Indoor_Navigation_Robot_Utilizing_YOLO_And_SLAM.git](https://github.com/Imdaeding/Autonomous_Indoor_Navigation_Robot_Utilizing_YOLO_And_SLAM.git)
cd Autonomous_Indoor_Navigation_Robot_Utilizing_YOLO_And_SLAM

# 2) 실행 스크립트 권한 부여
chmod +x start_autonomous.sh start_patrol_manual.sh
