# RBY1 Isaac ROS (`rby1_isaac_ros`)

Rainbow Robotics의 로봇 플랫폼(RBY1 등)에서 NVIDIA Isaac ROS의 하드웨어 가속 비전 노드(`isaac_ros_apriltag`, `isaac_ros_visual_slam`, `isaac_ros_nvblox` 등)를 컨테이너 환경에서 통합 구동하기 위한 예제 및 애플리케이션 개발 레포지토리입니다.

본 레포지토리는 RBY1 특화 예제 노드, 통합 런치 파일, 설정을 관리하며, 대용량 외부 의존성인 NVIDIA 공식 패키지(`isaac_ros_common`, `isaac_ros_apriltag` 등)는 `.gitignore`로 제외되어 호스트 환경 버전에 맞추어 `src/` 경로에 클론하여 사용합니다.

---

## 1. ⚙️ 사전 호스트 환경 구성 (필수 선행)

본 워크스페이스를 구동하기 전, 사용 중인 호스트 OS 버전에 맞는 사전 환경 설정(NVIDIA 드라이버, Docker, Container Toolkit, Buildx, 디바이스 마운트, `isaac_ros_common` 클론 및 검증)을 먼저 완료해야 합니다.

* 📌 **Ubuntu 22.04 LTS (ROS 2 Humble / release-3.2 권장)**:  
  👉 **[docs/env_setup_ubuntu_22_04.md](docs/env_setup_ubuntu_22_04.md)**
* 📌 **Ubuntu 24.04 LTS (ROS 2 Jazzy / release-4.0+ 권장)**:  
  👉 **[docs/env_setup_ubuntu_24_04.md](docs/env_setup_ubuntu_24_04.md)**

---

## 2. 🧩 기본 개발 및 빌드 워크플로우

호스트 설정이 완료되었다면 아래 단계에 따라 필요한 패키지를 구성하고 빌드합니다.

### Step 1. 필요한 Isaac ROS 패키지 클론 (`src/` 경로)
모든 공식 패키지는 NITROS ABI 호환성을 위해 반드시 **동일한 릴리즈 브랜치**로 통일하여 클론합니다.

#### 🔹 Ubuntu 22.04 LTS 호스트 (ROS 2 Humble / release-3.2)
```bash
cd ~/isaac_ros_ws/src

# 1. 도커 개발 래퍼 (환경 구성 단계에서 클론하지 않은 경우)
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# 2. 비전 및 AprilTag 패키지
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# [Visual SLAM 필요 시]
# git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
```

#### 🔹 Ubuntu 24.04 LTS 호스트 (ROS 2 Jazzy / release-4.0 이상)
```bash
cd ~/isaac_ros_ws/src

# 1. 도커 개발 래퍼 (환경 구성 단계에서 클론하지 않은 경우)
git clone -b release-4.0 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# 2. 비전 및 AprilTag 패키지
git clone -b release-4.0 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
git clone -b release-4.0 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# [Visual SLAM 필요 시]
# git clone -b release-4.0 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
```

---

### Step 2. 컨테이너 구동 및 세션 진입

`isaac_ros_common`은 실행 중인 하드웨어(x86 PC 또는 Jetson ARM64)를 자동으로 감지하여 최적의 GPU 가속 컨테이너를 구동합니다.

#### ⚡ 스마트 단축 커맨드 사용 (`isaac-ros` 권장)
환경 구성 단계에서 `~/.bashrc`에 단축 함수를 등록해 두었으므로, 어느 디렉터리에 있든 터미널에서 아래 한 단어로 컨테이너에 즉시 접속할 수 있습니다:
```bash
isaac-ros
```
> (수동 실행 시: `cd ~/isaac_ros_ws/src/isaac_ros_common && ISAAC_ROS_WS=$HOME/isaac_ros_ws ./scripts/run_dev.sh`)  
> 실행 완료 시 컨테이너 프롬프트(`admin@<hostname>:/workspaces/isaac_ros-dev$`)로 자동 진입합니다.

---

### Step 3. 컨테이너 내부 의존성 설치 (`rosdep`)
컨테이너 내부에서 패키지가 요구하는 모든 NITROS 및 GXF 가속 라이브러리를 공식 패키지 저장소로부터 자동 해결합니다:

```bash
# 컨테이너 내부에서 실행
cd /workspaces/isaac_ros-dev

# 1. rosdep을 통한 Isaac ROS 핵심 의존성(NITROS 등) 일괄 설치
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 2. 센서 드라이버 설치 (RealSense 사용 시)
# Ubuntu 22.04 (Humble):
sudo apt install -y ros-humble-realsense2-camera ros-humble-isaac-ros-realsense
# Ubuntu 24.04 (Jazzy):
# sudo apt install -y ros-jazzy-realsense2-camera
```

---

### Step 4. 패키지 빌드 (`colcon build`)
빌드 시간을 단축하고 테스트 패키지 컴파일 에러를 방지하기 위해 테스트 플래그를 비활성화(`BUILD_TESTING=OFF`)하고 필요한 패키지 단위로 빌드합니다:

```bash
# 컨테이너 내부에서 실행
# 1. AprilTag 패키지 및 관련 의존 노드 빌드 (테스트 빌드 제외)
colcon build --symlink-install --packages-up-to isaac_ros_apriltag --cmake-args -DBUILD_TESTING=OFF

# (참고: 워크스페이스 내 모든 패키지 전체 빌드 시)
# colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

# 2. 빌드 환경 반영 (오버레이 적용)
source /opt/ros/humble/setup.bash             # Jazzy의 경우 /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

---

## 3. 📚 패키지별 구성 및 실행 매뉴얼 (Tutorials & Modules)

본 워크스페이스에서 활용 가능한 NVIDIA Isaac ROS 가속 노드별 상세 가이드 및 튜토리얼 목록입니다. 각 모듈의 상세 런치 설정, 파라미터 튜닝, 카메라 기종별 대응 매뉴얼은 링크된 문서를 참조하십시오.

### 📌 패키지 매뉴얼 목록

* 🎯 **[AprilTag 3D 마커 추적 및 rby1_apriltag 응용 패키지](docs/tutorial_apriltag.md)**:  
  👉 **[docs/tutorial_apriltag.md](docs/tutorial_apriltag.md)**
  * **cuAprilTag** 기반 GPU 가속 6-DoF 마커 포즈 추정 파이프라인
  * **`rby1_apriltag` 응용 패키지**: 타겟 마커 ID 선별, 순방향 행렬 중앙값+SVD 회전 평균화(Jitter 안정화), 표준 `PoseStamped` 및 `/tf` 발행
  * 도커 외부(호스트 일반 환경) 무설치 표준 토픽 연동 가이드
  * RealSense(D405, D435, D455) 720p 및 848x480 60~90 FPS 초고속 설정
  * ZED 카메라 및 일반 USB 웹캠 직결 파이프라인

* 🧭 **[Visual SLAM (cuVSLAM 3D 오도메트리)](docs/tutorial_visual_slam.md)** *(작성 예정)*:
  * 스테레오 카메라 및 IMU 융합 GPU 가속 실시간 6-DoF 로봇 위치 추정

* 🧱 **[nvblox (GPU 실시간 3D 복원 & 매핑)](docs/tutorial_nvblox.md)** *(작성 예정)*:
  * TSDF / ESDF 복셀 기반 실시간 3D 매핑 및 로봇 장애물 회피 연동

---

### ⚡ AprilTag & rby1_apriltag 빠른 시작 요약 (Quickstart)

```bash
# 1. 도커 컨테이너 내부: Isaac ROS cuAprilTag 실행
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag_realsense.launch.py

# 2. 도커 컨테이너 내부 (추가 터미널): 타겟 마커 선별 & 포즈 안정화 노드 실행
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch rby1_apriltag target_tag_filter.launch.py

# 3. 호스트 PC 일반 터미널 (도커 진입 불필요, 추가 설치 불필요):
ros2 topic echo /target_marker/pose
ros2 run tf2_ros tf2_echo camera_color_optical_frame target_marker_7
rviz2
```
> 💡 전체 런치 코드, 파라미터 정의 표, 카메라별 튜닝, 호스트 연동 파이썬 코드는 [docs/tutorial_apriltag.md](docs/tutorial_apriltag.md)에 상세히 수록되어 있습니다.


---

## 4. 🛠️ 컨테이너 세션 관리 치트시트

* **멀티 터미널 추가 접속 (호스트 터미널에서 실행)**:
  컨테이너가 실행 중인 상태에서 새 터미널 창을 열고 `isaac-ros`를 입력하면 자동으로 실행 중인 컨테이너에 attach됩니다:
  ```bash
  isaac-ros
  # 또는 직접 명령어: docker exec -it isaac_ros_dev-x86_64-container bash
  ```
* **세션 종료**: 컨테이너 셸에서 `exit`
* **컨테이너 강제 중지**: `docker stop isaac_ros_dev-x86_64-container`
* **💡 의존성 보존 및 도커 이미지 커밋 (매번 apt 재설치 방지)**:
  컨테이너 내부에서 `apt`나 `rosdep`으로 설치한 패키지는 컨테이너 재생성 시 초기화됩니다. 의존성 설치 후 호스트 터미널에서 현재 컨테이너를 도커 이미지로 커밋해 두면 영구적으로 보존됩니다:
  ```bash
  # 호스트 PC 터미널에서 실행:
  docker commit isaac_ros_dev-x86_64-container isaac_ros_dev-x86_64:latest
  ```
