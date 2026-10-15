# RBY1 Isaac ROS (`rby1_isaac_ros`)

Rainbow Robotics의 로봇 플랫폼(RBY1 등)에서 NVIDIA Isaac ROS의 하드웨어 가속 비전 노드(`isaac_ros_apriltag`, `isaac_ros_visual_slam`, `isaac_ros_nvblox` 등)를 컨테이너 환경에서 통합 구동하기 위한 예제 및 애플리케이션 개발 레포지토리입니다.

본 레포지토리는 RBY1 특화 예제 노드, 통합 런치 파일, 설정을 관리하며, 대용량 외부 의존성인 NVIDIA 공식 패키지(`isaac_ros_common`, `isaac_ros_apriltag` 등)는 `.gitignore`로 제외되어 호스트 환경 버전에 맞추어 `src/` 경로에 클론하여 사용합니다.

---

## 1. ⚙️ 사전 호스트 환경 구성 (필수 선행)

호스트 OS에 맞는 환경 구성 문서를 **끝까지** 따라 합니다. NVIDIA 드라이버, Docker, 저장소 클론, 이미지 만들기, 첫 컨테이너 진입,
컨테이너 단축 명령 `isaac-ros` 등록까지 그 문서에 있습니다.

* 📌 **Ubuntu 22.04 LTS (ROS 2 Humble / release-3.2 권장)** — x86 PC와 Jetson Orin(9장):  
  👉 **[docs/env_setup_ubuntu_22_04.md](docs/env_setup_ubuntu_22_04.md)**
* 📌 **Ubuntu 24.04 LTS (ROS 2 Jazzy / release-4.0+ 권장)**:  
  👉 **[docs/env_setup_ubuntu_24_04.md](docs/env_setup_ubuntu_24_04.md)**

> [!IMPORTANT]
> 드라이버 워크스페이스(`~/ros2_driver_ws`)를 먼저 빌드해 둡니다(드라이버 저장소 README). 이미지에 넣을 로봇 모델을 거기서 가져옵니다.

소스로 받는 NVIDIA 저장소는 **`isaac_ros_common` 하나**입니다. cuMotion, AprilTag(`isaac_ros_apriltag`), 영상 보정
(`isaac_ros_image_proc`)은 NVIDIA apt 패키지로 이 저장소의 이미지 레이어(`docker/Dockerfile.cumotion`)에 들어갑니다.

> [!NOTE]
> 예전 방식대로 `isaac_ros_apriltag`, `isaac_ros_image_pipeline`을 `src/`에 클론해 빌드했다면, 그 빌드 결과(`install/`)가
> apt 설치본보다 우선합니다. 지우거나 `install/`에서 빼 두십시오(소스 폴더는 남아 있어도 `rby1_isaac_ros`만 빌드하면 상관없음).

<details>
<summary>Ubuntu 24.04 호스트에서 추가로 클론하는 것 (확인하지 않은 절차)</summary>

> [!WARNING]
> 이 저장소의 이미지 레이어는 Humble(release-3.2) 기준입니다. 아래 Jazzy 절차는 확인하지 않았습니다.
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
</details>

---

## 2. 🧩 컨테이너에서 패키지 빌드

환경 구성을 끝냈으면 컨테이너에 들어가 이 저장소의 패키지를 빌드합니다. 컨테이너를 끄거나 터미널을 더 여는 방법은
[환경 구성 8.3](docs/env_setup_ubuntu_22_04.md#83-컨테이너-쓰기)에 있습니다.

```bash
isaac-ros                                         # 호스트: 컨테이너에 들어갑니다 (꺼져 있으면 켭니다)
```

### Step 1. 컨테이너 내부 의존성 설치 (`rosdep`)
컨테이너 내부에서 패키지가 요구하는 모든 NITROS 및 GXF 가속 라이브러리를 공식 패키지 저장소로부터 자동 해결합니다:

```bash
# 컨테이너 내부에서 실행
cd /workspaces/isaac_ros-dev

# 1. rosdep을 통한 Isaac ROS 핵심 의존성(NITROS 등) 일괄 설치
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

```

---

### Step 2. 패키지 빌드 (`colcon build`)
빌드 시간을 단축하고 테스트 패키지 컴파일 에러를 방지하기 위해 테스트 플래그를 비활성화(`BUILD_TESTING=OFF`)하고 필요한 패키지 단위로 빌드합니다:

```bash
# 컨테이너 내부에서 실행
# 1. 이 저장소의 패키지만 빌드 (NVIDIA 패키지는 이미지에 apt로 들어 있음)
#    rby1_cumotion, rby1_apriltag, rby1_isaac_ros_examples 가 한 번에 빌드됩니다
colcon build --base-paths src/rby1_isaac_ros --symlink-install --cmake-args -DBUILD_TESTING=OFF

# 2. 빌드 환경 반영 (오버레이 적용)
source /opt/ros/humble/setup.bash             # Jazzy의 경우 /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

> [!TIP]
> 빌드는 여기서 한 번이면 됩니다. 이후 컨테이너에 들어올 때마다 자동으로 source되고, 튜토리얼에서는 빌드 명령을 다시 적지 않습니다.
> `--symlink-install`로 빌드했으므로 설정 파일(`cumotion.yaml`, `target_tags.yaml`)과 런치 파일은 **소스를 고치면 다시 빌드하지 않아도**
> 적용됩니다. C++ 소스(`rby1_apriltag`)를 고쳤을 때만 다시 빌드합니다.

---

## 3. 📚 패키지와 매뉴얼

켜는 방법과 예제는 각 매뉴얼에 있습니다.

* 🦾 **cuMotion 기반 RB-Y1 경로 계획 및 실행 (`rby1_cumotion`)** 👉 **[docs/tutorial_cumotion.md](docs/tutorial_cumotion.md)**
  * 4×4 목표 토픽을 보내면 GPU(cuMotion)가 장애물을 피하는 경로를 계획하고 RB-Y1 드라이버로 실행
  * 움직이는 도중 경로에 나타난 장애물은 멈추지 않고 다시 계획해 돌아감. 움직이는 장애물은 경로 위에서 멈춰 지나가길 기다렸다가 이어 감
  * 시뮬레이터·실기체 동일 절차, 로봇 기종·버전·자세는 기동 시 드라이버에서 자동으로 읽음
  * 개발자용 내용(구조·모델 번들·이미지·충돌 모델·설정·테스트·트러블슈팅) 👉 **[docs/developer_manual.md](docs/developer_manual.md)**

* 🎯 **AprilTag 마커 검출과 머리 추적 (`rby1_apriltag`)** 👉 **[docs/tutorial_apriltag.md](docs/tutorial_apriltag.md)**
  * 카메라는 호스트에서(웹캠·RealSense·이미지 파일, 드라이버 저장소 `rby1_additional_tools`), 영상만 토픽으로 컨테이너에 → GPU 보정·검출(cuAprilTag)
  * 대상 마커 선별, 중앙값+SVD 안정화, 표준 `PoseStamped`·TF 발행
  * 마커 좌표로 로봇 움직이기: 머리가 마커를 따라감(`head_follow`), 마커를 팔 목표로(`marker_target`), 보이는 마커들을 차례로(`marker_shuttle`)

* 🚀 **예제 런치 (`rby1_isaac_ros_examples`)** 👉 **[docs/tutorial_cumotion.md 3.1](docs/tutorial_cumotion.md#31-카메라의-마커를-쓸-때)**
  * 컨테이너에서 켜는 것을 용도별로 런치 하나에 묶은 것. `cumotion_with_markers.launch.py`: cuMotion + 마커 검출

* 🧭 **Visual SLAM (cuVSLAM 3D 오도메트리)** *(작성 예정)*
  * 스테레오 카메라 및 IMU 융합 GPU 가속 실시간 6-DoF 로봇 위치 추정

* 🧱 **nvblox (GPU 실시간 3D 복원 & 매핑)** *(작성 예정)*
  * TSDF / ESDF 복셀 기반 실시간 3D 매핑 및 로봇 장애물 회피 연동
